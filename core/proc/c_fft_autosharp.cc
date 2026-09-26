/*
 * c_fft_autosharp.cc
 *
 *  Created on: Sep 25, 2026
 *      Author: amyznikov
 *
            [ INPUT: _srcImage, _srcMask, opts ]
                                    │
                                    ▼
                  Validate Channels (cn == 1 or cn == 3?)
                                    │
            ┌───────────────────────┴───────────────────────┐
            ▼ (cn == 1: Grayscale)                          ▼ (cn == 3: BGR)
    Has Mask & Inpaint Enabled?                     Has Mask & Inpaint Enabled?
      ┌─────┴─────┐                                   ┌─────┴─────┐
   [No]         [Yes]                              [No]         [Yes]
      │           │                                   │           │
      │     Calculate ROI (rc)                        │     Split to YCrCb planes
 Pad Border Copy Mask & Image                   Pad Border  Copy Mask & ROI
(Reflect101) Inpaint (Pyramid/Linear)          (Reflect101) Inpaint Y-plane only
      │           │                                   │           │
      └─────┬─────┘                                   └─────┬─────┘
            │                                               │
     PPS Decomposition                               PPS Decomposition
   (Extract P, S, V components)                    (Extract P, S, V components)
            │                                               │
            └───────────────────────┬───────────────────────┘
                                    │
                                    ▼
                         Compute Radial Profile
                          (fftRadialProfileCCS)
                                    │
                                    ▼
                          Create Inverse Filter
                  (createDFTInverseBlurCorrectionFilter)
                                    │
                                    ▼
                     Apply Filter to P-spectrum (Each Channel)
                        (fftMulSpectrumCCS)
                                    │
                                    ▼
                     Add Smooth Component S Back
                        (cv::add)
                                    │
                                    ▼
                        Inverse Fourier Transform
                        (cv::idft with DFT_SCALE)
                                    │
                                    ▼
                        Reconstruct Final Image:
                      (cn == 3 -> ycrcbPlanes2BGR)
                      (cn == 1 -> crop via ROI rc)
                                    │
                                    ▼
                         [ OUTPUT: _dstImage ]
 */

#include "c_fft_autosharp.h"
#include <core/proc/fft.h>
#include <core/proc/run-loop.h>
#include <core/proc/pixtype.h>
#include <core/proc/inpaint/average_pyramid_inpaint.h>
#include <core/proc/inpaint/linear_interpolation_inpaint.h>
#include <core/proc/c_line_estimate.h>
#include <core/io/c_stdio_file.h>
#include <core/ssprintf.h>
#include <core/readdir.h>
#include <core/debug.h>


template<>
const c_enum_member * members_of<FFT_AUTOSHARP_INPAINT_METHOD>()
{
  static const c_enum_member members[] = {
      { FFT_AUTOSHARP_AVERAGE_PYRAMID_INPAINT, "AVERAGE_PYRAMID_INPAINT", "" },
      { FFT_AUTOSHARP_LINEAR_INTERPOLATION_INPAINT, "LINEAR_INTERPOLATION", "" },
      { FFT_AUTOSHARP_INPAINT_DISABLED, "DISABLE", "" },
      { FFT_AUTOSHARP_INPAINT_DISABLED}
  };
  return members;
}

template<>
const c_enum_member * members_of<FFT_AUTOSHARP_OUTPUT_DISPLAY>()
{
  static const c_enum_member members[] = {
      { FFT_AUTOSHARP_DISPLAY_SRC_IMAGE, "SRC_IMAGE", },
      { FFT_AUTOSHARP_DISPLAY_RESTORED_IMAGE, "RESTORED_IMAGE", },
      { FFT_AUTOSHARP_DISPLAY_INVERSE_FILTER,"INVERSE_FILTER"},
      { FFT_AUTOSHARP_DISPLAY_P_SPECTRUM, "P_SPECTRUM"},
      { FFT_AUTOSHARP_DISPLAY_S_SPECTRUM, "S_SPECTRUM"},
      { FFT_AUTOSHARP_DISPLAY_RESTORED_SPECTRUM, "RESTORED_SPECTRUM"},
      { FFT_AUTOSHARP_DISPLAY_RESTORED_IMAGE, },
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
  INSTRUMENT_REGION("");

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
  INSTRUMENT_REGION("");

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
  INSTRUMENT_REGION("");

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
    const c_fft_autosharp_debug_options * debug_opts)
{
  INSTRUMENT_REGION("");

  const bool print_debug_info =
      debug_opts && debug_opts->print_debug_info;

  const c_radial_spectrum_profile sp(RadialSpectrumProfile);
  std::vector<float> sspec; // smoothed radial profile [numBins]
  std::vector<float> correction; // corrections to spectrum module [numBins]
  double S0_target = 0;
  const int NBins = sp.size();


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

  // For correct bind scaling it is assumed that radial profile was created by fftRadialProfileCCS()
  const int cx = fftSize.width / 2, cy = fftSize.height / 2;
  const float scaleX = float((NBins - 1) * M_SQRT1_2 / cx);
  const float scaleY = float((NBins - 1) * M_SQRT1_2 / cy);

  uint8_t * filter_base = outputFilter.ptr();
  const size_t filter_stride = outputFilter.step;
  const float * corrections = correction.data();

  parallel_for(0, fftSize.height, [=](const auto & range) {
    for (int y = rbegin(range); y < rend(range); ++y) {
      float * __restrict fltp = (float * )(filter_base + y * filter_stride);

      // the vertical angular frequency
      const int fy = (y <= fftSize.height / 2) ? y : (fftSize.height - y);
      const float dy = fy * scaleY;
      const float dy2 = dy * dy;

      for (int x = 0; x < fftSize.width; ++x) {
        // the horizontal angular frequency to bin index
        const int fx = (x <= fftSize.width / 2) ? x : (fftSize.width - x);
        const float dx = fx * scaleX;
        const float dx2 = dx * dx;
        const int r = (int)(std::sqrt(dx2 + dy2));
        fltp[x] = corrections[r];
      }
    }
  });

  // "/home/projects/temp/analyze_profile.txt"
  if( debug_opts && debug_opts->write_file && !debug_opts->debug_file_name.empty() ) {

    c_stdio_file fp;

    const std::string & debug_file_name = debug_opts->debug_file_name;
    const std::string path = get_parent_directory(debug_file_name);
    if ( !create_path(path) ) {
      CF_ERROR("create_path('%s') fails: %s", strerror(errno));
    }
    else if( !fp.open(debug_file_name, "w") ) {
      CF_ERROR("Can not create '%s': %s", fp.cfilename(), strerror(errno));
    }
    else {
      fprintf(fp, "I\tX\tS\tSPEC\tSSPEC\tTARGET\tCORRECTION\tSPEC_RESTORED\n");

      for( int i = 0; i < NBins; ++i ) {
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
    std::vector<cv::Mat>& outputYCrCbPlanes,
    cv::Rect * outputValidRect)
{
  INSTRUMENT_REGION("");

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
  if ( outputYCrCbPlanes.size() != 3 ) {
    outputYCrCbPlanes.resize(3);
  }
  for ( int c = 0; c < 3; ++c ) {
    outputYCrCbPlanes[c].create(fftSize, CV_32FC1);
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

        const _Tp * srcp = (const _Tp*)(src_base + (y - border_top) * src_stride);
        const uint8_t * mskp = (const uint8_t*)(mask_base + (y - border_top) * mask_stride);
        const int xmax = border_left + srcSize.width;
        for (int x = border_left; x < xmax; ++x, ++mskp, ++yp, ++crp, ++cbp, srcp += 3 ) {
          const float fmask = float(!*mskp);
          const float b = srcp[0], g = srcp[1], r = srcp[2];
          const float Y = r * wR + g * wG + b * wB;
          const float Cr = (r - Y) * kCr;
          const float Cb = (b - Y) * kCb;
          *yp  = Y * fmask;
          *crp = Cr * fmask;
          *cbp = Cb * fmask;
        }

        for (int x = xmax; x < fftSize.width; ++x) {
          *yp++ = 0, *crp++ = 0, *cbp++ = 0;
        }
      }
    });
  }

  return true;
}

static bool bgr2YCrCbPlanes(cv::InputArray srcImage, cv::InputArray srcMask, const cv::Size & fftSize,
    std::vector<cv::Mat>& outputYCrCbPlanes,
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

static bool ycrcbPlanes2BGR(const std::vector<cv::Mat1f> & planes, const cv::Rect & roi,
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

void c_fft_autosharp::clearCachedData()
{
  // cleanup cached memory
  _src_mask.release();
  _radial_profile.release();
  _inverse_filter.release();
  _vlap_filter.release();
  _src_planes.clear(), _src_planes.shrink_to_fit();
  _src_p.clear(), _src_p.shrink_to_fit();
  _src_s.clear(), _src_s.shrink_to_fit();
  _src_channels_restored.clear(), _src_channels_restored.shrink_to_fit();
}

extern bool fftPPSDecompositionCCS2(cv::InputArray _src, const cv::Mat1f & VLAP,
    cv::OutputArray P_SPECTRUM, cv::OutputArray S_SPECTRUM);


extern bool fftPPSDecompositionCCSPlanes2(const std::vector<cv::Mat> & planes, const cv::Mat1f & VLAP,
    std::vector<cv::Mat1f> * P_SPECTRUMS, std::vector<cv::Mat1f> * S_SPECTRUMS);

// moon:  /mnt/data/scope/2023-08-04/MOON3/image_stacking1
// mars: /mnt/data/scope/2022-11-13/s7/CapObj/2022-11-13Z/s2
bool c_fft_autosharp::compute(const c_fft_autosharp_options & opts,
    cv::InputArray _srcImage, cv::InputArray _srcMask,
    cv::OutputArray _dstImage, cv::OutputArray _dstMask,
    FFT_AUTOSHARP_OUTPUT_DISPLAY outputDisplay,
    const c_fft_autosharp_debug_options * debugOpts)
{
  INSTRUMENT_REGION("c_fft_autosharp");

  const cv::Size srcSize = _srcImage.size();
  const int cn = _srcImage.channels();
  if ( cn != 1 && cn != 3 ) {
    CF_ERROR("Bad input image: Only 1 channel grayscale or 3 channels BGR inputs are supported");
    return false;
  }

  const cv::Size psfRadius(0, 0);
  const cv::Size fftSize = fftGetOptimalSize(srcSize, psfRadius, nullptr, true);
  if( fftSize.empty() ) {
    CF_ERROR("fftGetOptimalSize() fails for _srcImage.size()=%dx%d", srcSize.width, srcSize.height);
    return false;
  }

  cv::Rect rc(0, 0, srcSize.width, srcSize.height);

  if( _vlap_filter.size() != fftSize ) {
    _vlap_filter = fftGenerateDiscreteLaplacianFilter(fftSize, false);
  }
  if( _src_planes.empty() ) {
    _src_planes.emplace_back();
  }
  if( _src_p.empty() ) {
    _src_p.emplace_back();
  }
  if( _src_s.empty() ) {
    _src_s.emplace_back();
  }
  if( _src_v.empty() ) {
    _src_v.emplace_back();
  }

  cv::Mat srcImage;
  if ( _srcImage.depth() == CV_32F ) {
    srcImage = _srcImage.getMat();
  }
  else {
    _srcImage.getMat().convertTo(srcImage, CV_32F);
  }

  if( cn == 1 ) { // Grayscale input
    if( _srcMask.empty() || opts.mask_inpaint_method == FFT_AUTOSHARP_INPAINT_DISABLED ) {
      if( srcSize == fftSize ) {
        _src_planes[0] = srcImage;
      }
      else { // Standard border BORDER_REFLECT101 still need
        fftCopyMakeBorder(srcImage, _src_planes[0], fftSize, &rc);
      }
    }
    else {
      // Manually copy + InPaint missing pixels
      const cv::Mat srcMask = (_srcMask.depth() == CV_8U) ? _srcMask.getMat() : _srcMask.getMat() > 0;
      const int border_top = (fftSize.height - srcSize.height) / 2;
      const int border_left = (fftSize.width - srcSize.width) / 2;
      rc = cv::Rect(border_left, border_top, srcSize.width, srcSize.height);

      _src_mask.create(fftSize, CV_8UC1), _src_mask.setTo(0);
      srcMask.copyTo(_src_mask(rc));

      _src_planes[0].create(fftSize, srcImage.depth()), _src_planes[0].setTo(0);
      srcImage.copyTo(_src_planes[0](rc), srcMask);

      switch (opts.mask_inpaint_method) {
        case FFT_AUTOSHARP_AVERAGE_PYRAMID_INPAINT:
          average_pyramid_inpaint(_src_planes[0], _src_mask, _src_planes[0], cv::noArray(), 9);
          break;
        case FFT_AUTOSHARP_LINEAR_INTERPOLATION_INPAINT:
          linear_interpolation_inpaint(_src_planes[0], _src_mask);
          break;
      }
    }

    fftPPSDecompositionCCS2(_src_planes[0], _vlap_filter, _src_p[0], _src_s[0]);
  }
  else if( cn == 3 ) { // BGR input
    if( _srcMask.empty() || opts.mask_inpaint_method == FFT_AUTOSHARP_INPAINT_DISABLED ) {
      cv::Mat src;
      if( srcSize == fftSize ) {
        src = srcImage;
      }
      else { // Standard border BORDER_REFLECT101 still need
        fftCopyMakeBorder(srcImage, src, fftSize, &rc, cv::BORDER_REFLECT_101);
      }
      if( !bgr2YCrCbPlanes(src, cv::noArray(), fftSize, _src_planes, nullptr) ) {
        CF_ERROR("bgr2YCrCbPlanes() fails");
        return false;
      }
    }
    else {
      // Zero (black) border for _srcMask inpaint (inpaint intensity channel Y only)
      const cv::Mat srcMask = (_srcMask.depth() == CV_8U) ? _srcMask.getMat() : _srcMask.getMat() > 0;
      if( !bgr2YCrCbPlanes(srcImage, srcMask, fftSize, _src_planes, &rc) ) {
        CF_ERROR("bgr2YCrCbPlanes() fails");
        return false;
      }

      _src_mask.create(fftSize, CV_8UC1), _src_mask.setTo(0);
      srcMask.copyTo(_src_mask(rc));

      switch (opts.mask_inpaint_method) {
        case FFT_AUTOSHARP_AVERAGE_PYRAMID_INPAINT:
          average_pyramid_inpaint(_src_planes[0], _src_mask, _src_planes[0], cv::noArray(), 9);
          break;
        case FFT_AUTOSHARP_LINEAR_INTERPOLATION_INPAINT:
          linear_interpolation_inpaint(_src_planes[0], _src_mask);
          break;
      }
    }

    if( !fftPPSDecompositionCCSPlanes2(_src_planes, _vlap_filter, &_src_p, &_src_s) ) {
      CF_ERROR("fftPPSDecompositionCCSPlanes2() fails");
      return false;
    }
  }
  else {
    CF_ERROR("Grayscale or BGR image is expected on input");
    return false;
  }


  switch( outputDisplay ) {
    case FFT_AUTOSHARP_DISPLAY_SRC_IMAGE:
      _dstImage.assign(_src_planes[0]);
      _dstMask.release();
      return true;
    case FFT_AUTOSHARP_DISPLAY_P_SPECTRUM:
      _dstImage.assign(_src_p[0]);
      _dstMask.release();
      return true;
    case FFT_AUTOSHARP_DISPLAY_S_SPECTRUM:
      _dstImage.assign(_src_s[0]);
      _dstMask.release();
      return true;
    default:
      break;
  }

  fftRadialProfileCCS(_src_p[0], _radial_profile);

  const bool fOK =
      createDFTInverseBlurCorrectionFilter(_inverse_filter, _radial_profile, fftSize,
          opts.autoS1_target, opts.S1_target, opts.macroStructSizePx,
          debugOpts);

  if( !fOK || _inverse_filter.size() != fftSize ) {
    CF_ERROR("createInverseBlurCorrectionFilter() fails");
    return false;
  }

  if( outputDisplay == FFT_AUTOSHARP_DISPLAY_INVERSE_FILTER ) {
    _dstImage.assign(_inverse_filter);
    _dstMask.release();
    return true;
  }

  if ( outputDisplay ==  FFT_AUTOSHARP_DISPLAY_RESTORED_SPECTRUM ) {
    fftMulSpectrumCCS(_src_p[0], _inverse_filter, _dstImage);
    _dstMask.release();
    return true;
  }

  if( _src_channels_restored.empty() || (cn > 1 && _src_channels_restored.size() != cn) ) {
    _src_channels_restored.resize(cn);
  }

  /*
   * Warning: Don't apply filter to S to avoid edge artifacts!
   * */
  if ( true ) {
    INSTRUMENT_REGION("APPLY_FILTER");
    for( int i = 0; i < cn; ++i ) {
      fftMulSpectrumCCS(_src_p[i], _inverse_filter, _src_p[i]);
      cv::add(_src_p[i], _src_s[i], _src_p[i]);
      cv::idft(_src_p[i], _src_channels_restored[i], cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
    }
  }

  if( cn > 1 ) {
    ycrcbPlanes2BGR(_src_channels_restored, rc, _dstImage);
  }
  else if( _src_channels_restored[0].size() == srcSize ) {
    _dstImage.assign(_src_channels_restored[0]);
  }
  else {
    _dstImage.assign(_src_channels_restored[0](rc));
  }

  return true;
}

bool serialize_fft_autosharp_options(c_config_setting settings, bool save,
    c_fft_autosharp_options & opts)
{
  SERIALIZE_OPTION(settings, save, opts, S1_target);
  SERIALIZE_OPTION(settings, save, opts, autoS1_target);
  SERIALIZE_OPTION(settings, save, opts, macroStructSizePx);
  SERIALIZE_OPTION(settings, save, opts, mask_inpaint_method);
  return true;
}
