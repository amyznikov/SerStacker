/*
 * c_fft_autosharp_routine.cc
 *
 *  Created on: Jun 5, 2026
 *      Author: amyznikov
 *
 *      ┌─────────────── SRC_IMAGE (B, G, R) ───────────────┐
 *      │                                                   │
 *      ▼                                                   ▼
 * [ INTENSITY Channel ]                              [ B, G, R Channels ]
 *      │                                                   │
 * Periodic+Smooth Decomposition                   Periodic+Smooth Decomposition
 *      │                                           (for each channel separately)
 *      ▼                                                   │
 *  P_intensity Spectrum                                     ▼
 *      │                                          Split into P_c and S_c
 * [ Generate FILTER ]                                      │
 *      │                                                   │
 *      └───────────────────► Apply FILTER to P_c only ─────┤
 *                                                          │
 *                                                          ▼
 *                                                Assembly: P_c_filtered + S_c
 *                                                          │
 *                                                          ▼
 *                                                  IFFT(P_c_filtered + S_c)
 *
 */

#include "c_fft_autosharp_routine.h"
#include <core/proc/inpaint/average_pyramid_inpaint.h>
#include <core/proc/inpaint/linear_interpolation_inpaint.h>
#include <core/proc/c_line_estimate.h>
#include <core/proc/run-loop.h>
#include <core/io/c_stdio_file.h>
#include <core/proc/fft.h>
#include <core/ssprintf.h>
#include <core/readdir.h>

template<>
const c_enum_member * members_of<c_fft_autosharp_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_fft_autosharp_routine::DISPLAY_SRC_IMAGE, "SRC_IMAGE", },
      { c_fft_autosharp_routine::DISPLAY_RESTORED_IMAGE, "RESTORED_IMAGE", },
      { c_fft_autosharp_routine::DISPLAY_FILTER,"FILTER"},
      { c_fft_autosharp_routine::DISPLAY_P_SPECTRUM, "P_SPECTRUM"},
      { c_fft_autosharp_routine::DISPLAY_S_SPECTRUM, "S_SPECTRUM"},
      { c_fft_autosharp_routine::DISPLAY_V_SPECTRUM, "V_SPECTRUM"},
      { c_fft_autosharp_routine::DISPLAY_RESTORED_SPECTRUM, "RESTORED_SPECTRUM"},
      { c_fft_autosharp_routine::DISPLAY_RESTORED_IMAGE, },
  };

  return members;
}

template<>
const c_enum_member * members_of<c_fft_autosharp_routine::INPAINT_METHOD>()
{
  static const c_enum_member members[] = {
      { c_fft_autosharp_routine::LINEAR_INTERPOLATION_INPAINT, "LINEAR_INTERPOLATION", "" },
      { c_fft_autosharp_routine::AVERAGE_PYRAMID_INPAINT, "AVERAGE_PYRAMID_INPAINT", "" },
      { c_fft_autosharp_routine::INPAINT_DISABLED, "DISABLE", "" },
      { c_fft_autosharp_routine::LINEAR_INTERPOLATION_INPAINT}
  };
  return members;
}

namespace {

class c_radial_spectrum_profile
{
public:
  inline c_radial_spectrum_profile(const cv::Mat1f & mx /* [1][n_bins]*/)
  {
    init(mx);
  }

  inline void init(const cv::Mat1f & mx)
  {
    // Ignore DC component for energy computation
    const int num_bins = mx.cols;
    const double total_energy = cv::norm(mx(cv::Rect(1, 0, num_bins - 1, 1)), cv::NORM_L2SQR);

    // Normalized log of DFT components
    _sp = mx;
    _y0 = float(0.5 * std::log(total_energy));
    cv::log(mx, _lsp);
    cv::subtract(_lsp, _y0, _lsp);

    // Also x bin frequencies in log space
    _xv.resize(num_bins);
    _xv[0] = 0;
    for( int i = 1; i < num_bins; ++i ) {
      _xv[i] = std::log(float(i));
    }
  }

  inline int size() const
  {
    return _sp.cols;
  }

  inline float y0() const
  {
    return _y0;
  }

  inline float xv(int i) const
  {
    return _xv[i]; // precomouted std::log(std::max(1,i));
  }

  inline float yv(int i) const
  {
    return _lsp[0][i];
  }

  inline float sv(int i) const
  {
    return _sp[0][i];
  }

protected:
  std::vector<float> _xv; // [n_bins]
  cv::Mat1f _sp; // [1][n_bins]
  cv::Mat1f _lsp; // cv::log(_sp)
  float _y0 = 0;
};

static inline void resampleAndSmoothRadialProfile(const c_radial_spectrum_profile & sp,
    cv::Mat1f & U /*[1][N_uniform]*/)
{
  constexpr int N_uniform = 100;
  const int n_bins = sp.size();

  // Compute averages skipping DC

  float bin_sums[N_uniform] = { 0.0f };
  int bin_counts[N_uniform] = { 0 };

  const float x_min = sp.xv(1);
  const float x_max = sp.xv(n_bins - 1);
  const float x_range_inv = (x_max > x_min) ? (N_uniform - 1) / (x_max - x_min) : 0.0f;
  for (int i = 1; i < n_bins; ++i) {
    if (sp.sv(i) > 0.0f) {
      const int idx = int((sp.xv(i) - x_min) * x_range_inv);
      const int bin_idx = (idx < 0) ? 0 : ((idx > N_uniform - 1) ? N_uniform - 1 : idx);
      bin_sums[bin_idx] += sp.yv(i);
      bin_counts[bin_idx]++;
    }
  }
  for (int i = 0; i < N_uniform; ++i) {
    if (bin_counts[i] > 1) {
      bin_sums[i] /= static_cast<float>(bin_counts[i]);
    }
  }

  U.create(1, N_uniform);
  float * __restrict up = U[0];

  // Gap filling
  int last_valid_idx = -1;
  for (int j = 0; j < N_uniform; ++j) {
    if (bin_counts[j] > 0) {
      up[j] = bin_sums[j];

      if (last_valid_idx != j - 1) {
        const int left = (last_valid_idx >= 0) ? last_valid_idx : 0;
        const float y_left = (last_valid_idx >= 0) ? up[left] : up[j];
        const float y_right = up[j];
        const float span = float(j - left);
        const float step_delta = (y_right - y_left) / span;
        float current_y = y_left + step_delta;
        for (int k = left + 1; k < j; ++k) {
          up[k] = current_y;
          current_y += step_delta;
        }
      }
      last_valid_idx = j;
    }
  }

  // Extrapolate the right edge if the last cells are empty
  if (last_valid_idx >= 0 && last_valid_idx < N_uniform - 1) {
    std::fill(up + last_valid_idx + 1, up + N_uniform, up[last_valid_idx]);
  }

  // Freeze tail
  const int startCornersBin = int((n_bins - 1) * M_SQRT1_2);
  const int cornerIndex = int((sp.xv(startCornersBin) - x_min) * x_range_inv);
  const int uniformCornerIndex = (cornerIndex < 0) ? 0 : ((cornerIndex > N_uniform - 1) ? N_uniform - 1 : cornerIndex);
  std::fill(up + uniformCornerIndex, up + N_uniform, up[uniformCornerIndex]);

  cv::medianBlur(U, U, 5);
  cv::GaussianBlur(U, U, cv::Size(25, 1), 0, 0, cv::BORDER_REPLICATE);
}

// Linear regression on clean SDCT segment
static bool estimateNature(const c_radial_spectrum_profile & sp, const std::vector<float> & sdct,
    double macroStructSizePx,
    double & S0_nature,
    double & S1_nature,
    bool print_debug_info)
{
  // Frequency start equivalent to frame macro structures (~100 pixels)
  const int n_bins = sp.size();
  const int dctCornerBin = int((n_bins - 1) * M_SQRT1_2);
  const int macroStructStartBin = std::clamp(cvRound(dctCornerBin / macroStructSizePx), 1, dctCornerBin - 15);
  const float x_start = sp.xv(macroStructStartBin);
  const float x_corner = sp.xv(dctCornerBin);
  const float x_midpoint = (x_start + x_corner) / 2;

  // Cumulative linear regression

  c_weighted_line_estimate<float> line;

  float current_shift = S0_nature;
  float current_slope = S1_nature;
  float stdev2 = 0.0;
  float r2 = 1.0f;

  // Accumulate a starting base (at least 8 bins for a reliable initial trend)
  int i = macroStructStartBin;
  for (; i < macroStructStartBin + 8 && i < dctCornerBin; ++i) {
    line.update(sp.xv(i), sdct[i]);
  }

  // Cumulative march from left to right in search of blur
  // Stall tracking parameters
  constexpr int patience = 7;
  int drop_counter = 0;
  int best_valid_bin = i;
  for( ; i < dctCornerBin; ++i ) {
    const float x = sp.xv(i);
    const float y = sdct[i];

    // current line parameters BEFORE adding a new point
    if( line.compute(current_shift, current_slope, stdev2, r2) ) {
      const float y_pred = current_shift + current_slope * x;
      const float delta_y = y - y_pred;

      if( (r2 < 0.95f) || (delta_y < -0.05f) || (x > x_midpoint && delta_y > 0.05f) ) {
        drop_counter++;
      }
      else {
        // False bump, reset the counter
        // Save the last valid linear bin
        drop_counter = 0;
        best_valid_bin = i;
      }
    }

    // Hard stop if blur or noise floor has captured the spectrum irrevocably
    if( drop_counter >= patience ) {
      break;
    }

    // Add a point to the cumulative least squares If all is well
    line.update(x, y);
  }

  // Rollback and fix TARGET: recalculate the line strictly up to the breakdown point
  line.reset();
  for( int k = macroStructStartBin; k <= best_valid_bin; ++k ) {
    line.update(sp.xv(k), sdct[k]);
  }

  const bool fOK = line.compute(current_shift, current_slope);
  if (fOK) { // Successfully update TARGET slope
    S0_nature = current_shift;
    S1_nature = current_slope;
  }

  if( print_debug_info ) {
    CF_DEBUG("\nAUTO_SLOPE: fOK=%d\n"
        "dctCornerBin=%d macroStructSizePx=%g macroStructStartBin=%d (x=%g) "
        "bestValidBin=%d (x=%g) S0_nature = %g S1_nature = %g", fOK,
        dctCornerBin, macroStructSizePx, macroStructStartBin, sp.xv(macroStructStartBin),
        best_valid_bin, sp.xv(best_valid_bin), S0_nature, S1_nature);
  }

  return fOK;
}

static bool computeRadialProfileCorrection(const c_radial_spectrum_profile & sp,
    double macroStructureSizePx,
    double & S0_target,
    double & S1_target,
    bool autoTarget,
    bool print_debug_info,
    std::vector<float> & sspec,
    std::vector<float> & correction)
{
  cv::Mat1f U;

  // Resample sp to uniform log scale, blur and save to U matrx [1][N_uniform]
  resampleAndSmoothRadialProfile(sp, U);

  const int n_bins = sp.size();
  const int N_uniform = U.cols;
  const float * smup = U[0];
  const float x_min = sp.xv(1);
  const float x_max = sp.xv(n_bins - 1);
  const float x_range_inv = (x_max > x_min) ? (N_uniform - 1) / (x_max - x_min) : 0;
  const float dx = (N_uniform - 1 > 0) ? (x_max - x_min) / (N_uniform - 1) : 0;

  sspec.resize(n_bins, 0.0f);
  sspec[0] = sp.yv(0);
  for( int i = 1; i < n_bins; ++i ) {
    const float uniform_idx = (sp.xv(i) - x_min) * x_range_inv;
    const int k = std::clamp(int(uniform_idx), 0, N_uniform - 2);
    const float t = uniform_idx - k;
    const float y = smup[k] * (1 - t) + smup[k + 1] * t ;
    sspec[i] = y;
  }

  if( autoTarget ) {
    // For autoTarget compute linear regression on the limited SDCT segment
    estimateNature(sp, sspec, macroStructureSizePx,
        S0_target, S1_target,
        print_debug_info);
  }

  // Generate array of DFT corrections in uniform grid
  std::vector<float> y_target(N_uniform);
  std::vector<float> uniform_correction(N_uniform, 0.0f);
  const float inv_win_width = 1.f / 2.0f;

  const int dctCornerBin = int((n_bins - 1) * M_SQRT1_2);
  const int regressionStartBin = std::clamp(int(std::round(dctCornerBin / macroStructureSizePx)), 1, dctCornerBin - 15);
  const float x_stable_start = sp.xv(regressionStartBin);

  if ( print_debug_info ) {
    CF_DEBUG("\nCORRECTION: win_width=%g x_min=%g x_stable_start=%g S0_target=%g S1_target=%g",
        1.f / inv_win_width, x_min, x_stable_start, S0_target, S1_target);
  }

  correction.resize(n_bins); // store deltas at this stage

  constexpr float alpha = 0.05f;
  constexpr float inv_2_alpha = 1.0f / (2.0f * alpha);
  const float float_S1_target = float(S1_target);
  const float delta_y_user = float_S1_target * dx;

  // Compute deltas only
  for( int i = 0; i < N_uniform; ++i ) {
    const float x = x_min + i * dx;
    const float delta_y_real = (i > 0) ? (smup[i] - smup[i - 1]) : 0.0f;
    const float t = (x - x_stable_start) * inv_win_width;
    float w_sig = 0.0f;
    if( t >= 1.0f ) {
      w_sig = 1.0f;
    }
    else if( t <= 0.0f ) {
      w_sig = 0.0f;
    }
    else {
      const float w_left = t * t * inv_2_alpha;
      const float w_mid = t;
      const float inv_t = 1.0f - t;
      const float w_right = 1.0f - inv_t * inv_t * inv_2_alpha;
      const float tmp = (t < alpha) ? w_left : w_mid;
      w_sig = (t > (1.0f - alpha)) ? w_right : tmp;
    }
    const float effective_target_delta = (1.0f - w_sig) * delta_y_real + w_sig * delta_y_user;
    correction[i] = std::max(delta_y_real, effective_target_delta);
  }

  // Accumulate deltas into uniform corrections
  for( int i = 0; i < N_uniform; ++i ) {
    const float current_x = x_min + i * dx;
    if( current_x < x_stable_start ) {
      y_target[i] = smup[i];
    }
    else {
      y_target[i] = y_target[i - 1] + correction[i];
    }
    uniform_correction[i] = std::max(0.0f, y_target[i] - smup[i]);
  }

  // Assemble the output array and exponentiate the correction
  correction[0] = 1;
  for (int i = 1; i < n_bins; ++i) {
    const float uniform_idx = (sp.xv(i) - x_min) * x_range_inv;
    const int k = std::clamp(int(uniform_idx), 0, N_uniform - 2);
    const float t = uniform_idx - k;
    const float log_corr = (1 - t) * uniform_correction[k] + t * uniform_correction[k + 1];
    correction[i] = std::exp(log_corr);
  }

  return true;
}

static bool createDFTInverseBlurCorrectionFilter(cv::Mat1f & outputFilter,
    const cv::Mat1f & RadialSpectrumProfile /*[1][n_bins] */,
    const cv::Size & fftSize,
    bool autoTargetSlope,
    double S1_target,
    double macroStructSizePx,
    bool print_debug_info = false,
    const std::string & debug_file_name = "")
{
  INSTRUMENT_REGION("");

  const c_radial_spectrum_profile sp(RadialSpectrumProfile);
  std::vector<float> sspec; // smoothed radial profile [numBins]
  std::vector<float> correction; // corrections to spectrum module [numBins]
  double S0_target = 0;
  const int N = sp.size();

  /*
   * COMPUTE SPECTRUM CORRECTION FOR TARGET SLOPE ADJUSMENT
   */
  computeRadialProfileCorrection(sp,
      macroStructSizePx,
      S0_target,
      S1_target,
      autoTargetSlope,
      print_debug_info,
      sspec,
      correction);

  /*
   * Create the CORRECTION FILTER
   */

  outputFilter.create(fftSize);

  const double cx = fftSize.width / 2.0;
  const double cy = fftSize.height / 2.0;
  const double R = std::sqrt(cx * cx + cy * cy);
  const int numBins = std::max(1, cvRound(R));
  const double scaleX = 1.0 / cx;
  const double scaleY = 1.0 / cy;
  const float binScale = float(numBins * M_SQRT1_2);

  const float * corrections = correction.data();
  uint8_t * filter_base = outputFilter.ptr();
  const size_t filter_stride = outputFilter.step;

  parallel_for(0, fftSize.height, [=](const auto & range) {
    for (int y = rbegin(range); y < rend(range); ++y) {
      float * __restrict fltp = (float * )(filter_base + y * filter_stride);

      // the vertical angular frequency
      const int fy = (y <= fftSize.height / 2) ? y : (fftSize.height - y);
      const float dy = fy * scaleY;
      const float dy2 = dy * dy;

      for (int x = 0; x < fftSize.width; ++x) {
        // the horizontal angular frequency
        const int fx = (x <= fftSize.width / 2) ? x : (fftSize.width - x);
        const float dx = fx * scaleX;
        const float dx2 = dx * dx;
        const float r = std::sqrt(dx2 + dy2);
        const int binIndex = std::clamp(cvRound(r * binScale), 0, N - 1);
        fltp[x] = corrections[binIndex];
      }
    }
  });

  // "/home/projects/temp/analyze_profile.txt"
  if( !debug_file_name.empty() ) {

    c_stdio_file fp;

    const std::string path = get_parent_directory(debug_file_name);
    if ( !create_path(path) ) {
      CF_ERROR("create_path('%s') fails: %s", strerror(errno));
    }
    else if( !fp.open(debug_file_name, "w") ) {
      CF_ERROR("Can not create '%s': %s", fp.cfilename(), strerror(errno));
    }
    else {
      fprintf(fp, "I\tX\tS\tSPEC\tSSPEC\tTARGET\tCORRECTION\tSPEC_RESTORED\n");

      for( int i = 0; i < N; ++i ) {
        const double yraw = sp.sv(i);
        const double x = sp.xv(i); // log of frequency
        const double y = sp.yv(i); // log of spectrum intensity
        const double ys = sspec[i]; // log of smoothed spectrum intensity
        const double ytarget = S0_target + S1_target * x;
        const double corr = std::log(correction[i]);
        const double yrestored = y + corr;

        fprintf(fp, "%4d\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\n",
            i, x, yraw, y, ys, ytarget, corr, yrestored);
      }

      CF_DEBUG("Saved file '%s'", fp.cfilename());
    }
  }

  return true;
}

template<class _Tp>
bool _bgr2YCrCbPlanes(cv::InputArray srcImage, cv::InputArray srcMask, const cv::Size & fftSize,
    std::vector<cv::Mat1f>& outputYCrCbPlanes,
    cv::Rect * outputValidRect)
{
  if ( srcImage.empty() || srcImage.channels() != 3 ) {
    CF_ERROR("BGR image expected on input");
    return false;
  }

  // BT.601
  // Compile-time delta for any type
  constexpr float wR = 0.299000f;
  constexpr float wG = 0.587000f;
  constexpr float wB = 0.114000f;
  constexpr float kCr = 0.713000f;
  constexpr float kCb = 0.564000f;

  const cv::Mat src = srcImage.getMat();
  const cv::Size srcSize = src.size();

  const int border_top = (fftSize.height - srcSize.height) / 2;
  const int border_bottom = (fftSize.height - srcSize.height - border_top);
  const int border_left = (fftSize.width - srcSize.width) / 2;
  const int border_right = (fftSize.width - srcSize.width - border_left);
  if ( outputValidRect ) {
    * outputValidRect = cv::Rect(border_left, border_top, srcSize.width, srcSize.height);
  }

  // Y, Cr, Cb
  outputYCrCbPlanes.resize(3);
  for ( int c = 0; c < 3; ++c ) {
    outputYCrCbPlanes[c].create(fftSize);
  }

  const uint8_t* const src_base = src.ptr();
  const size_t src_stride = src.step;

  uint8_t* const y_base  = outputYCrCbPlanes[0].ptr();
  const size_t y_stride  = outputYCrCbPlanes[0].step;

  uint8_t* const cr_base = outputYCrCbPlanes[1].ptr();
  const size_t cr_stride = outputYCrCbPlanes[1].step;

  uint8_t* const cb_base = outputYCrCbPlanes[2].ptr();
  const size_t cb_stride = outputYCrCbPlanes[2].step;

  if ( srcMask.empty() ) {
    // Little faster path
    parallel_for(0, fftSize.height, [=](const auto & range) {
      for (int y = rbegin(range); y < rend(range); ++y) {
        float* __restrict yp  = (float*)(y_base  + y * y_stride);
        float* __restrict crp = (float*)(cr_base + y * cr_stride);
        float* __restrict cbp = (float*)(cb_base + y * cb_stride);

        if ( y < border_top || y >= srcSize.height + border_top  ) {
          memset(yp, 0, fftSize.width * sizeof(*yp));
          memset(crp, 0, fftSize.width * sizeof(*crp));
          memset(cbp, 0, fftSize.width * sizeof(*cbp));
          continue;
        }

        for (int x = 0; x < border_left; ++x) {
          *yp++ = 0, *crp++ = 0, *cbp++ = 0;
        }

        const _Tp * __restrict srcp = ((const _Tp*)(src_base + (y-border_top) * src_stride));
        const int xmax = border_left + srcSize.width;
        for (int x = border_left; x < xmax; ++x, srcp += 3) {
          const float b = srcp[0], g = srcp[1], r = srcp[2];
          const float Y = r * wR + g * wG + b * wB;
          const float Cr = (r - Y) * kCr;
          const float Cb = (b - Y) * kCb;
          *yp++  = Y;
          *crp++ = Cr;
          *cbp++ = Cb;
        }

        for (int x = xmax; x < fftSize.width; ++x) {
          *yp++ = 0, *crp++ = 0, *cbp++ = 0;
        }
      }
    });
  }
  else {
    // Little slower path
    const cv::Mat1b mask = srcMask.getMat();
    const uint8_t* const mask_base  = mask.ptr();
    const size_t mask_stride  = mask.step;

    parallel_for(0, fftSize.height, [=](const auto & range) {
      for (int y = rbegin(range); y < rend(range); ++y) {
        float* __restrict yp  = (float*)(y_base  + y * y_stride);
        float* __restrict crp = (float*)(cr_base + y * cr_stride);
        float* __restrict cbp = (float*)(cb_base + y * cb_stride);

        if ( y < border_top || y >= border_top + srcSize.height ) {
          memset(yp, 0, fftSize.width * sizeof(*yp));
          memset(crp, 0, fftSize.width * sizeof(*crp));
          memset(cbp, 0, fftSize.width * sizeof(*cbp));
          continue;
        }

        for (int x = 0; x < border_left; ++x) {
          *yp++  = 0, *crp++ = 0, *cbp++ = 0;
        }

        const _Tp * srcp = (const _Tp*)(src_base + (y-border_top) * src_stride);
        const uint8_t * mskp = (const uint8_t*)(mask_base + (y-border_top) * mask_stride);
        const int xmax = border_left + srcSize.width;
        for (int x = border_left; x < xmax; ++x, ++mskp, ++yp, ++crp, ++cbp, srcp += 3 ) {
          if ( !*mskp ) {
            *yp  = 0, *crp = 0, *cbp = 0;
          }
          else {
            const float b = srcp[0], g = srcp[1], r = srcp[2];
            const float Y = r * wR + g * wG + b * wB;
            const float Cr = (r - Y) * kCr;
            const float Cb = (b - Y) * kCb;
            *yp  = Y;
            *crp = Cr;
            *cbp = Cb;
          }
        }

        for (int x = xmax; x < fftSize.width; ++x) {
          *yp++ = 0, *crp++ = 0, *cbp++ = 0;
        }
      }
    });
  }

  return true;
}

bool bgr2YCrCbPlanes(cv::InputArray srcImage, cv::InputArray srcMask, const cv::Size & fftSize,
    std::vector<cv::Mat1f>& outputYCrCbPlanes,
    cv::Rect * outputValidRect)
{
  INSTRUMENT_REGION("");
  if ( srcImage.empty() || srcImage.channels() != 3 ) {
    CF_ERROR("3-channel input BGR image expected");
    return false;
  }
  if ( !srcMask.empty() ) {
    if (srcMask.type() != CV_8UC1) {
      CF_ERROR("Invalid mask type: %d. Single-channel input CV_8UC1 mask expected");
      return false;
    }
    if (srcMask.size() != srcImage.size() ) {
      CF_ERROR("Invalid mask size: %dx%d. Must be %dx%d", srcMask.cols(), srcMask.rows(),
          srcImage.cols(), srcImage.rows());
      return false;
    }
  }

  CV_DISPATCH(srcImage.depth(), _bgr2YCrCbPlanes, srcImage, srcMask, fftSize,
      outputYCrCbPlanes, outputValidRect);

  CF_ERROR("Not supported inpt image depth %d", srcImage.depth());
  return false;
}

bool ycrcbPlanes2BGR(const std::vector<cv::Mat1f> & planes, cv::OutputArray bgrImage)
{
  INSTRUMENT_REGION("");

  if( planes.size() != 3 ) {
    CF_ERROR("Invalid input planes size=%zu. Must be 3", planes.size());
    return false;
  }

  const cv::Size sz = planes[0].size();
  if( sz.empty() ) {
    CF_ERROR("Invalid input image size: %dx%d", sz.width, sz.height);
    return false;
  }

  for( int c = 1; c < 3; ++c ) {
    if( planes[c].size() != sz ) {
      CF_ERROR("Invalid input plane[%d] size=%dx%d. Expected %dx%d",
          c, planes[c].cols, planes[c].rows, sz.width, sz.height);
      return false;
    }
  }

  if( bgrImage.fixedType() && bgrImage.type() != CV_32FC3 ) {
    CF_ERROR("Output BGR image type must be CV_32FC3");
    return false;
  }

  if( bgrImage.fixedSize() && bgrImage.size() != sz ) {
    CF_ERROR("Output BGR image size must be %dx%d", sz.width, sz.height);
    return false;
  }

  // Y = 0.299*R + 0.587*G + 0.114*B
  // Inverse BT.601 coefficients (exact mathematical weights)
  constexpr float iCr = 1.402000f;
  constexpr float iCb = 1.772000f;
  constexpr float gCr = 0.714136f; // (0.299 * iCr) / 0.587
  constexpr float gCb = 0.344136f; // (0.114 * iCb) / 0.587

  const cv::Mat1f Yplane = planes[0];
  const cv::Mat1f Crplane = planes[1];
  const cv::Mat1f Cbplane = planes[2];

  bgrImage.create(sz, CV_32FC3);
  cv::Mat dst = bgrImage.getMatRef();

  const uint8_t * const y_base = Yplane.ptr();
  const size_t y_stride = Yplane.step;

  const uint8_t * const cr_base = Crplane.ptr();
  const size_t cr_stride = Crplane.step;

  const uint8_t * const cb_base = Cbplane.ptr();
  const size_t cb_stride = Cbplane.step;

  uint8_t * const dst_base = dst.ptr();
  const size_t dst_stride = dst.step;

  parallel_for(0, sz.height, [=](const auto & range) {
    for (int y = rbegin(range); y < rend(range); ++y) {
      const float* yp = (const float*)(y_base + y * y_stride);
      const float* crp = (const float*)(cr_base + y * cr_stride);
      const float* cbp = (const float*)(cb_base + y * cb_stride);
      float* __restrict dstp = (float*)(dst_base + y * dst_stride);
      for (int x = 0; x < sz.width; ++x, dstp += 3 ) {
        const float Y = *yp++;
        const float Cr = *crp++;
        const float Cb = *cbp++;
        float r = Y + iCr * Cr;
        float b = Y + iCb * Cb;
        float g = Y - gCr * Cr - gCb * Cb;
        dstp[0] = b;
        dstp[1] = g;
        dstp[2] = r;
      }
    }
  });

  return true;
}

bool ycrcbPlanes2BGR(const std::vector<cv::Mat1f> & planes, const cv::Rect & roi,
    cv::OutputArray bgrImage)
{
  INSTRUMENT_REGION("");

  if( planes.size() != 3 ) {
    CF_ERROR("Invalid input planes size=%zu. Must be 3", planes.size());
    return false;
  }

  const cv::Size srcSize = planes[0].size();
  if( srcSize.empty() ) {
    CF_ERROR("Invalid input image size: %dx%d", srcSize.width, srcSize.height);
    return false;
  }

  for( int c = 1; c < 3; ++c ) {
    if( planes[c].size() != srcSize ) {
      CF_ERROR("Invalid input plane[%d] size=%dx%d. Expected %dx%d",
          c, planes[c].cols, planes[c].rows, srcSize.width, srcSize.height);
      return false;
    }
  }

  if( bgrImage.fixedType() && bgrImage.type() != CV_32FC3 ) {
    CF_ERROR("Output BGR image type must be CV_32FC3");
    return false;
  }

  const cv::Size roiSize = roi.size();

  if( bgrImage.fixedSize() && bgrImage.size() != roiSize ) {
    CF_ERROR("Output BGR image size must be %dx%d", roiSize.width, roiSize.height);
    return false;
  }

  // Y = 0.299*R + 0.587*G + 0.114*B
  // Inverse BT.601 coefficients (exact mathematical weights)
  constexpr float iCr = 1.402000f;
  constexpr float iCb = 1.772000f;
  constexpr float gCr = 0.714136f; // (0.299 * iCr) / 0.587
  constexpr float gCb = 0.344136f; // (0.114 * iCb) / 0.587

  const cv::Mat1f Yplane = planes[0];
  const cv::Mat1f Crplane = planes[1];
  const cv::Mat1f Cbplane = planes[2];

  bgrImage.create(roiSize, CV_32FC3);
  cv::Mat dst = bgrImage.getMatRef();

  const uint8_t * const y_base = Yplane.ptr();
  const size_t y_stride = Yplane.step;

  const uint8_t * const cr_base = Crplane.ptr();
  const size_t cr_stride = Crplane.step;

  const uint8_t * const cb_base = Cbplane.ptr();
  const size_t cb_stride = Cbplane.step;

  uint8_t * const dst_base = dst.ptr();
  const size_t dst_stride = dst.step;

  parallel_for(0, roiSize.height, [=](const auto & range) {
    for (int y = rbegin(range); y < rend(range); ++y) {
      const int src_y = y + roi.y;
      const float* yp = (const float*)(y_base + src_y * y_stride) + roi.x;
      const float* crp = (const float*)(cr_base + src_y * cr_stride) + roi.x;
      const float* cbp = (const float*)(cb_base + src_y * cb_stride) + roi.x;
      float* __restrict dstp = (float*)(dst_base + y * dst_stride);
      for (int x = 0; x < roiSize.width; ++x, dstp += 3 ) {
        const float Y = *yp++;
        const float Cr = *crp++;
        const float Cb = *cbp++;
        float r = Y + iCr * Cr;
        float b = Y + iCb * Cb;
        float g = Y - gCr * Cr - gCb * Cb;
        dstp[0] = b;
        dstp[1] = g;
        dstp[2] = r;
      }
    }
  });

  return true;
}

}


// moon:  /mnt/data/scope/2023-08-04/MOON3/image_stacking1
// mars: /mnt/data/scope/2022-11-13/s7/CapObj/2022-11-13Z/s2
void c_fft_autosharp_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "display", CTL_CONTEXT(ctx, _display), "");

  ctlbind(ctls, "inpaint_missing_pixels:", CTL_CONTEXT(ctx, _mask_inpaint_method),
      "How to fill holes and borders in non-enpty masks");

  ctlbind(ctls, "Auto S1_target: ", CTL_CONTEXT(ctx, _autoS1_target),
      "Try to estimate S1 target automatically based in natural DFT spectrum slope estimation");

  ctlbind(ctls, "S1_target: ", CTL_CONTEXT(ctx, _S1_target),
      "Default Target slope of restored DFT spectrum");

  ctlbind(ctls, "macroStructSizePx: ", CTL_CONTEXT(ctx, _macroStructSizePx),
      "The minimal size in pixels of image macro structures still not much affected by blur");

  ctlbind(ctls, "print_debug_info:", CTL_CONTEXT(ctx, _print_debug_info), "");
  ctlbind(ctls, "write_debug_file:", CTL_CONTEXT(ctx, _write_file), "");
  ctlbind_browse_for_file(ctls, "debug_file ", CTL_CONTEXT(ctx, _debug_file_name), "");
}

bool c_fft_autosharp_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _autoS1_target);
    SERIALIZE_OPTION(settings, save, *this, _S1_target);
    SERIALIZE_OPTION(settings, save, *this, _macroStructSizePx);
    SERIALIZE_OPTION(settings, save, *this, _debug_file_name);
    SERIALIZE_OPTION(settings, save, *this, _mask_inpaint_method);
    return true;
  }
  return false;
}


static bool inpaintMakeBorder(cv::InputArray inputImage, cv::InputArray inputMask,
    const cv::Size & fftSize, c_fft_autosharp_routine::INPAINT_METHOD inpaintMethod,
    cv::OutputArray outputImage,
    cv::Rect * outputValidRoi)
{
  INSTRUMENT_REGION("");
  if( inputMask.empty() || inpaintMethod == c_fft_autosharp_routine::INPAINT_DISABLED ) {
    return fftCopyMakeBorder(inputImage, outputImage, fftSize, outputValidRoi);
  }

  const cv::Size srcSize = inputImage.size();
  if( fftSize.width < srcSize.width || fftSize.height < srcSize.height ) {
    CF_ERROR("Invalid argument: fftSize (%dx%d) must be >= src.size() (%dx%d)",
        fftSize.width, fftSize.height, srcSize.width, srcSize.height);
    return false;
  }

  const cv::Mat src = inputImage.getMat();
  const cv::Mat src_mask = inputMask.getMat();

  outputImage.create(fftSize, inputImage.type());
  outputImage.setTo(cv::Scalar::all(0));
  cv::Mat & dst = outputImage.getMatRef();
  cv::Mat dst_mask(fftSize, src_mask.type(), cv::Scalar::all(0));

  const int border_top = (fftSize.height - srcSize.height) / 2;
  const int border_bottom = (fftSize.height - srcSize.height - border_top);
  const int border_left = (fftSize.width - srcSize.width) / 2;
  const int border_right = (fftSize.width - srcSize.width - border_left);
  const cv::Rect ROI(border_left, border_top, srcSize.width, srcSize.height);
  src.copyTo(dst(ROI));
  src_mask.copyTo(dst_mask(ROI));

  switch (inpaintMethod) {
    case c_fft_autosharp_routine::AVERAGE_PYRAMID_INPAINT:
      average_pyramid_inpaint(dst, dst_mask, dst, cv::noArray(), 9);
      break;
    case c_fft_autosharp_routine::LINEAR_INTERPOLATION_INPAINT:
      linear_interpolation_inpaint(dst, dst_mask);
      break;
  }

  if( outputValidRoi ) {
    *outputValidRoi = ROI;
  }

  return true;
}

bool c_fft_autosharp_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  INSTRUMENT_REGION("dft");

//  CF_DEBUG("enter");

  cv::Rect rc;
  const cv::Size srcSize = image.size();
  const cv::Size psfRadius(7, 7);
  const cv::Size fftSize = fftGetOptimalSize(image.size(), psfRadius, nullptr, true);
  const int cn = image.channels();

  cv::Mat V_SPECTRUM;

  if( VLAP.size() != fftSize ) {
    VLAP = fftGenerateDiscreteLaplacianFilter(fftSize, false);
    CF_DEBUG("Created VLAP: %dx%dx%d", VLAP.cols, VLAP.rows, VLAP.channels());
  }

  if( !mask.empty() ) {
    if( mask.depth() == CV_8U ) {
      SRC_MASK = mask.getMat();
    }
    else {
      cv::compare(mask, 0, SRC_MASK, cv::CMP_GT);
    }
  }



  if ( cn == 1 ) { // Grayscale input

    inpaintMakeBorder(image, mask.empty() ? cv::noArray() : SRC_MASK,
        fftSize, _mask_inpaint_method,
        SRC_IMAGE,
        &rc);

    if ( SRC_P.empty() ) {
      SRC_P.emplace_back();
    }
    if ( SRC_S.empty() ) {
      SRC_S.emplace_back();
    }

    fftPPSDecompositionCCS(SRC_IMAGE, VLAP, SRC_P[0], SRC_S[0],
        _display == DISPLAY_V_SPECTRUM ? V_SPECTRUM :
            cv::noArray());
  }
  else if( cn == 3 ) { // BGR input
    if (mask.empty() || _mask_inpaint_method == INPAINT_DISABLED ) {
      // Standard border BORDER_REFLECT101 still required
      fftCopyMakeBorder(image, SRC_IMAGE, fftSize, &rc);
      if ( !bgr2YCrCbPlanes(SRC_IMAGE, cv::noArray(), fftSize, SRC_PLANES, nullptr) ) {
        CF_ERROR("bgr2YCrCbPlanes() fails");
        return false;
      }
    }
    else {
      // Zero (black) border for mask inpaint (inpaint intensity channel Y only)
      if( !bgr2YCrCbPlanes(image, mask.empty() ? cv::noArray() : SRC_MASK, fftSize, SRC_PLANES, &rc) ) {
        CF_ERROR("bgr2YCrCbPlanes() fails");
        return false;
      }

      cv::Mat1b inpaintMask = cv::Mat1b::zeros(fftSize);
      SRC_MASK.copyTo(inpaintMask(rc));

      switch (_mask_inpaint_method) {
        case c_fft_autosharp_routine::AVERAGE_PYRAMID_INPAINT:
          average_pyramid_inpaint(SRC_PLANES[0], inpaintMask, SRC_PLANES[0], cv::noArray(), 9);
          break;
        case c_fft_autosharp_routine::LINEAR_INTERPOLATION_INPAINT:
          linear_interpolation_inpaint(SRC_PLANES[0], inpaintMask);
          break;
      }
      SRC_IMAGE = SRC_PLANES[0];
    }

    if ( !fftPPSDecompositionCCSPlanes(SRC_PLANES, VLAP, &SRC_P, &SRC_S) ) {
      CF_ERROR("fftPPSDecompositionCCSPlanes() fails");
      return false;
    }
  }
  else {
    CF_ERROR("Grayscale or BGR image is expected on input");
    return false;
  }

  if ( _display == DISPLAY_SRC_IMAGE ) {
    SRC_IMAGE.copyTo(image);
    mask.release();
    return true;
  }
  if ( _display == DISPLAY_P_SPECTRUM ) {
    fftUnpackCCSSpectrum(SRC_P[0], image);
    fftSwapQuadrants(image, image);
    fftSpectrumToPolar(image, image);
    mask.release();
    return true;
  }
  if ( _display == DISPLAY_S_SPECTRUM ) {
    fftUnpackCCSSpectrum(SRC_S[0], image);
    fftSwapQuadrants(image, image);
    fftSpectrumToPolar(image, image);
    mask.release();
    return true;
  }
  if ( _display == DISPLAY_V_SPECTRUM ) {
    if ( V_SPECTRUM.channels() == 1 ) {
      fftUnpackCCSSpectrum(V_SPECTRUM, V_SPECTRUM);
    }
    fftSwapQuadrants(V_SPECTRUM, image);
    fftSpectrumToPolar(image, image);
    mask.release();
    return true;
  }

  fftRadialProfileCCS(SRC_P[0], RadialProfile);

  bool fOK =
      createDFTInverseBlurCorrectionFilter(INVERSE_FILTER, RadialProfile, fftSize,
          _autoS1_target, _S1_target, _macroStructSizePx, _print_debug_info,
          _write_file ? _debug_file_name : "");

  if ( !fOK || INVERSE_FILTER.size() != fftSize ) {
    CF_ERROR("createInverseBlurCorrectionFilter() fails");
    return false;
  }

  if ( _display == DISPLAY_FILTER ) {
    INVERSE_FILTER.copyTo(image);
    mask.release();
    return true;
  }

  if ( _display ==  DISPLAY_RESTORED_SPECTRUM ) {
    fftMulSpectrumCCS(SRC_P[0], INVERSE_FILTER, image);
    fftUnpackCCSSpectrum(image, image);
    fftSpectrumToPolar(image, image);
    fftSwapQuadrants(image);
    mask.release();
    return true;
  }


  if ( true ) {
    INSTRUMENT_REGION("IDFT");
    SRC_CHANNELS_RESTORED.resize(cn);
    for ( int i = 0; i < cn; ++i ) {
      fftMulSpectrumCCS(SRC_P[i], INVERSE_FILTER, SRC_P[i]);
      cv::add(SRC_P[i], SRC_S[i], SRC_P[i]);
      cv::idft(SRC_P[i], SRC_CHANNELS_RESTORED[i], cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
    }
  }

  if (cn == 1 ) {
    SRC_CHANNELS_RESTORED[0](rc).copyTo(image);
  }
  else {
    INSTRUMENT_REGION("MERGE_AND_COPY");
#if 1
    ycrcbPlanes2BGR(SRC_CHANNELS_RESTORED, rc, image);
#else
    ycrcbPlanes2BGR(SRC_CHANNELS_RESTORED, SRC_RESTORED);
    SRC_RESTORED(rc).copyTo(image);
#endif
  }

//  CF_DEBUG("leave");
  return true;
}

