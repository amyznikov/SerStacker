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
      { c_fft_autosharp_routine::DISPLAY_SRC_SPECTRUM, "SRC_SPECTRUM"},
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

// Returns true if the channel is linear and fills the weights (in order B, G, R)
static bool getLinearIntensityWeights(int channel_type, double & wB, double & wG, double & wR)
{
  switch (channel_type) {
    case color_channel_blue:
      wB = 1.0;
      wG = 0.0;
      wR = 0.0;
      return true;
    case color_channel_green:
      wB = 0.0;
      wG = 1.0;
      wR = 0.0;
      return true;
    case color_channel_red:
      wB = 0.0;
      wG = 0.0;
      wR = 1.0;
      return true;
    case color_channel_gray:
      wB = 0.114;
      wG = 0.587;
      wR = 0.299;
      return true;
    case color_channel_luminance_YCrCb:
      wB = 0.114;
      wG = 0.587;
      wR = 0.299;
      return true;
    default:
      // All others (Lab, Luv, HSV, HLS, MIN/MAX) are nonlinear
      break;
  }
  return false;
}

static void computeLinearIntensityMathitude(cv::Mat1f & INTENSITY_P,
    const cv::Mat1f & SRC_P_B, double _wB,
    const cv::Mat1f & SRC_P_G, double _wG,
    const cv::Mat1f & SRC_P_R, double _wR)
{
  INSTRUMENT_REGION("");

  const cv::Size fftSize = SRC_P_B.size();

  const float wB = float(_wB);
  const float wG = float(_wG);
  const float wR = float(_wR);

  INTENSITY_P.create(fftSize);
  parallel_for(0, fftSize.height, [&, fftSize, wB, wG, wR](const auto & range) {
    for (int y = rbegin(range); y < rend(range); ++y) {
      const float * __restrict bp = (const float*)(SRC_P_B[y]);
      const float * __restrict gp = (const float*)(SRC_P_G[y]);
      const float * __restrict rp = (const float*)(SRC_P_R[y]);
      float * __restrict dstp = INTENSITY_P[y];
      for (int x = 0; x < fftSize.width; ++x, ++bp, ++gp, ++rp) {
        const float v = (*bp) * wB + (*gp) * wG + (*rp) * wR;
        *dstp++ = v;
      }
    }
  });
}

}


// moon:  /mnt/data/scope/2023-08-04/MOON3/image_stacking1
// mars: /mnt/data/scope/2022-11-13/s7/CapObj/2022-11-13Z/s2
void c_fft_autosharp_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "display", CTL_CONTEXT(ctx, _display), "");

  ctlbind(ctls, "Intensity channel: ", CTL_CONTEXT(ctx, _intensity_channel),
      "Select intensity channel for spectrum analysis");

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

  c_anscombe_transform::getcontrols(ctls, ctx(&this_class::_anscombe));
}

bool c_fft_autosharp_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _intensity_channel);
    SERIALIZE_OPTION(settings, save, *this, _autoS1_target);
    SERIALIZE_OPTION(settings, save, *this, _S1_target);
    SERIALIZE_OPTION(settings, save, *this, _macroStructSizePx);
    SERIALIZE_OPTION(settings, save, *this, _debug_file_name);
    SERIALIZE_OPTION(settings, save, *this, _mask_inpaint_method);

    if ( auto group = SERIALIZE_GROUP(settings, save, "anscombe") ) {
      c_anscombe_transform_options & opts = _anscombe.opts();
      SERIALIZE_OPTION(settings, save, opts, method);
      SERIALIZE_OPTION(settings, save, opts.generalized, g);
      SERIALIZE_OPTION(settings, save, opts.generalized, c);
      SERIALIZE_OPTION(settings, save, opts.generalized, auto_estimate);
      SERIALIZE_OPTION(settings, save, opts.generalized, dump_estimated_params);
    }

    return true;
  }
  return false;
}


static bool inpaintMakeBorder(cv::InputArray inputImage, cv::InputArray inputMask,
    const cv::Size & fftSize, c_fft_autosharp_routine::INPAINT_METHOD inpaintMethod,
    cv::OutputArray outputImage,
    cv::Rect * outputValidRoi)
{
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

  cv::Rect rc;
  const cv::Size srcSize = image.size();
  const cv::Size psfRadius(7, 7);
  const cv::Size fftSize = fftGetOptimalSize(image.size(), psfRadius, nullptr, true);
  const int cn = image.channels();

  double wB = 0, wG = 0, wR = 0;

  // CF_DEBUG("srcSize: %dx%d fftSize: %dx%d", srcSize.width, srcSize.height, fftSize.width, fftSize.height);

  image.getMat().convertTo(SRC_IMAGE, CV_32F);
  if( !mask.empty() ) {
    if( mask.depth() == CV_8U ) {
      SRC_MASK = mask.getMat();
    }
    else {
      cv::compare(mask, 0, SRC_MASK, cv::CMP_GT);
    }
  }

  inpaintMakeBorder(SRC_IMAGE, mask.empty() ? cv::noArray() : SRC_MASK,
      fftSize, _mask_inpaint_method,
      SRC_IMAGE,
      &rc);

  if ( _anscombe.method() != anscombe_none ) {
    _anscombe.apply(SRC_IMAGE, SRC_IMAGE);
  }

  if ( _display == DISPLAY_SRC_IMAGE ) {
    SRC_IMAGE(rc).copyTo(image);
    return true;
  }

  if( VLAP.size() != fftSize ) {
    VLAP = fftGenerateDiscreteLaplacianFilter(fftSize, false);
    CF_DEBUG("Created VLAP: %dx%dx%d", VLAP.cols, VLAP.rows, VLAP.channels());
  }

  fftPPSDecompositionCCS(SRC_IMAGE, VLAP, &SRC_P, &SRC_S);

  if ( cn != _prev_cn ) { // Avoid potential cache hysteresis problem for consecutive calls
    INTENSITY_P.release();
    _prev_cn = cn;
  }

  if ( cn == 1 ) {
    INTENSITY_P = SRC_P[0];
  }
  else if ( getLinearIntensityWeights(_intensity_channel, wB, wG, wR) ) {
    computeLinearIntensityMathitude(INTENSITY_P,
        SRC_P[0], wB,
        SRC_P[1], wG,
        SRC_P[2], wR);
  }
  else { // Nonlinear channel: Lab, HSV, etc
    extract_channel(SRC_IMAGE, INTENSITY_CHANNEL, cv::noArray(), cv::noArray(), _intensity_channel);
    fftPPSDecompositionCCS(INTENSITY_CHANNEL, VLAP, INTENSITY_P, INTENSITY_S);
  }

  if ( _display ==  DISPLAY_SRC_SPECTRUM ) {
    fftUnpackCCSSpectrum(INTENSITY_P, image);
    fftSwapQuadrants(image, image);
    fftSpectrumToPolar(image, image);
    mask.release();
    return true;
  }

  fftRadialProfileCCS(INTENSITY_P, INTENSITY_RadialProfile);

  bool fOK =
      createDFTInverseBlurCorrectionFilter(INVERSE_FILTER, INTENSITY_RadialProfile, fftSize,
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
    fftMulSpectrumCCS(INTENSITY_P, INVERSE_FILTER, INTENSITY_P);
    fftUnpackCCSSpectrum(INTENSITY_P, image);
    fftSwapQuadrants(image, image);
    fftSpectrumToPolar(image, image);
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
    SRC_RESTORED = SRC_CHANNELS_RESTORED[0];
  }
  else {
    cv::merge(SRC_CHANNELS_RESTORED, SRC_RESTORED);
  }

  if ( _anscombe.method() != anscombe_none ) {
    _anscombe.inverse(SRC_RESTORED, SRC_RESTORED);
  }

  SRC_RESTORED(rc).copyTo(image);

  return true;
}

