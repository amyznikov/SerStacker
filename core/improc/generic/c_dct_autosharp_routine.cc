/*
 * c_dct_autosharp_routine.cc
 *
 *  Created on: Jul 23, 2026
 *      Author: amyznikov
 */

#include "c_dct_autosharp_routine.h"
#include <core/proc/fft.h>
#include <core/proc/run-loop.h>
#include <core/ssprintf.h>
#include <core/proc/inpaint/average_pyramid_inpaint.h>
#include <core/proc/inpaint/linear_interpolation_inpaint.h>
#include <core/io/c_stdio_file.h>
#include <core/proc/c_line_estimate.h>
#include <core/proc/c_linear_regression.h>
#include <core/readdir.h>
#include <core/debug.h>


template<>
const c_enum_member * members_of<c_dct_autosharp_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_dct_autosharp_routine::DISPLAY_SRC_IMAGE, "SRC_IMAGE", "" },
      { c_dct_autosharp_routine::DISPLAY_RESTORED_IMAGE, "RESTORED_IMAGE", "" },
      { c_dct_autosharp_routine::DISPLAY_FILL_SRC_VOIDS, "FILL_SRC_VOIDS", "" },
      { c_dct_autosharp_routine::DISPLAY_SRC_SPECTRUM, "SRC_SPECTRUM", "" },
      { c_dct_autosharp_routine::DISPLAY_SRC_RADIAL_PROFILE, "SRC_RADIAL_PROFILE", "" },
      { c_dct_autosharp_routine::DISPLAY_SRC_RADIAL_PROFILE_LOG, "SRC_RADIAL_PROFILE_LOG", "" },
      { c_dct_autosharp_routine::DISPLAY_FILTER, "FILTER", "" },
      { c_dct_autosharp_routine::DISPLAY_RESTORED_SPECTRUM, "RESTORED_SPECTRUM", "" },
      { c_dct_autosharp_routine::DISPLAY_RESTORED_PROFILE, "RESTORED_PROFILE", "" },
      { c_dct_autosharp_routine::DISPLAY_RESTORED_IMAGE}
  };
  return members;
}

template<>
const c_enum_member * members_of<c_dct_autosharp_routine::INPAINT_METHOD>()
{
  static const c_enum_member members[] = {
      { c_dct_autosharp_routine::LINEAR_INTERPOLATION_INPAINT, "LINEAR_INTERPOLATION", "" },
      { c_dct_autosharp_routine::AVERAGE_PYRAMID_INPAINT, "AVERAGE_PYRAMID_INPAINT", "" },
      { c_dct_autosharp_routine::INPAINT_DISABLED, "DISABLE", "" },
      { c_dct_autosharp_routine::LINEAR_INTERPOLATION_INPAINT}
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

    // Normalized log of DCT components
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

static inline void resampleSmoothDCT(const c_radial_spectrum_profile & sp,
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

static bool computeCorrectionDCT(const c_radial_spectrum_profile & sp,
    double macroStructureSizePx,
    double & S0_target,
    double & S1_target,
    bool autoTarget,
    bool print_debug_info,
    std::vector<float> & sdct,
    std::vector<float> & correction)
{
  cv::Mat1f U;

  // Resample DCT to uniform log scale, blur and save to U matrx [1][N_uniform]
  resampleSmoothDCT(sp, U);

  const int n_bins = sp.size();
  const int N_uniform = U.cols;
  const float * smup = U[0];
  const float x_min = sp.xv(1);
  const float x_max = sp.xv(n_bins - 1);
  const float x_range_inv = (x_max > x_min) ? (N_uniform - 1) / (x_max - x_min) : 0;
  const float dx = (N_uniform - 1 > 0) ? (x_max - x_min) / (N_uniform - 1) : 0;

  sdct.resize(n_bins, 0.0f);
  sdct[0] = sp.yv(0);
  for( int i = 1; i < n_bins; ++i ) {
    const float uniform_idx = (sp.xv(i) - x_min) * x_range_inv;
    const int k = std::clamp(int(uniform_idx), 0, N_uniform - 2);
    const float t = uniform_idx - k;
    const float y = smup[k] * (1 - t) + smup[k + 1] * t ;
    sdct[i] = y;
  }

  if( autoTarget ) {
    // For autoTarget compute linear regression on the limited SDCT segment
    estimateNature(sp, sdct, macroStructureSizePx,
        S0_target, S1_target,
        print_debug_info);
  }

  // Generate array of DCT corrections in uniform grid
  std::vector<float> y_target(N_uniform);
  std::vector<float> uniform_correction(N_uniform, 0.0f);
  const float inv_win_width = 1.f / 2.0f;

  const int dctCornerBin = int((n_bins - 1) * M_SQRT1_2);
  const int regressionStartBin = std::clamp(int(std::round(dctCornerBin / macroStructureSizePx)), 1, dctCornerBin - 15);
  const float x_stable_start = sp.xv(regressionStartBin);

  if ( print_debug_info ) {
    CF_DEBUG("\nCOREECTION: win_width=%g x_min=%g x_stable_start=%g S0_target=%g S1_target=%g",
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

static cv::Mat1f createInverseBlurCorrectionFilter(const cv::Mat1f & RadialSpectrumProfile, /*[1][n_bins] */
    const cv::Size & dctSize,
    bool autoTargetSlope,
    double S1_target,
    double macroStructSizePx,
    bool print_debug_info = false,
    const std::string & debug_file_name = "")
{
  const c_radial_spectrum_profile sp(RadialSpectrumProfile);
  std::vector<float> sdct, correction;
  double S0_target = 0;
  const int N = sp.size();

  /*
   * COMPUTE DCT SPECTRUM CORRECTION FOR TARGET SLOPE ADJUSMENT
   */
  computeCorrectionDCT(sp,
      macroStructSizePx,
      S0_target,
      S1_target,
      autoTargetSlope,
      print_debug_info,
      sdct,
      correction);

  /*
   * Create the DCT FILTER
   */

  const cv::Size size = dctSize;
  cv::Mat1f FILTER(size);

  const double R = std::sqrt(size.width * size.width + size.height * size.height);
  const int numBins = std::max(1, int(R));
  const double maxNormalizedR = std::sqrt(2.0);
  const double scaleX = 1.0 / size.width;
  const double scaleY = 1.0 / size.height;
  const float * corrections = correction.data();

  parallel_for(0, size.height,
      [=, &FILTER](const auto & range) {
        for (int y = rbegin(range); y < rend(range); ++y) {
          float * __restrict dstp = FILTER[y];

          const float dy = y * scaleY;
          const float dy2 = dy * dy;

          for (int x = 0; x < size.width; ++x) {
            const float dx = x * scaleX;
            const float dx2 = dx * dx;
            const float r = std::sqrt(dx2 + dy2);
            const float continuousBinIdx = r * numBins / maxNormalizedR;
            const int binIndex = std::clamp((int)(continuousBinIdx), 0, N - 1);
            dstp[x] = corrections[binIndex];
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
      fprintf(fp, "I\tX\tS\tDCT\tSDCT\tTARGET\tCORRECTION\tDCT_RESTORED\n");

      for( int i = 0; i < N; ++i ) {
        const double yraw = sp.sv(i);
        const double x = sp.xv(i); // log of frequency
        const double y = sp.yv(i); // log of spectrum intensity
        const double ys = sdct[i]; // log of smoothed spectrum intensity
        const double ytarget = S0_target + S1_target * x;
        const double corr = std::log(correction[i]);
        const double yrestored = y + corr;

        fprintf(fp, "%4d\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\n",
            i, x, yraw, y, ys, ytarget, corr, yrestored);
      }

      CF_DEBUG("Saved file '%s'", fp.cfilename());
    }
  }

  return FILTER;
}

static bool dctRadialProfile2(const cv::Mat1f & dctSpectrum, cv::Mat1f & outputProfile)
{
  if( dctSpectrum.empty() ) {
    return false;
  }

  const int maxW = dctSpectrum.cols;
  const int maxH = dctSpectrum.rows;

  const double R = std::sqrt(maxW * maxW + maxH * maxH);
  const int numBins = std::max(1, int(R));
  const double binScale = numBins * M_SQRT1_2;

  std::vector<double> radialSum(numBins, 0.0);
  std::vector<double> radialCount(numBins, 0.0);

  const double scaleX = 1.0 / maxW;
  const double scaleY = 1.0 / maxH;

  for( int y = 0; y < maxH; ++y ) {
    const double dy = y * scaleY;
    const double dy2 = dy * dy;
    const float * srcp = dctSpectrum[y];

    for( int x = 0; x < maxW; ++x ) {
      const double dx = x * scaleX;
      const double dx2 = dx * dx;
      const double r = std::sqrt(dx2 + dy2);

      const int bin = std::clamp(cvRound(r * binScale), 0, numBins - 1);
      radialSum[bin] += srcp[x];
      //radialSum[bin] += std::abs(srcp[x]);
      radialCount[bin] += 1.0;
    }
  }

  outputProfile.create(1, numBins);
  float * __restrict dstp = outputProfile[0];
  for( int i = 0; i < numBins; ++i ) {
    dstp[i] = (float) (radialCount[i] > 0 ? radialSum[i] / radialCount[i] : 0.0);
  }

  return true;
}

} // namespace

void c_dct_autosharp_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "display", CTL_CONTEXT(ctx, _display), "");

  ctlbind(ctls, "Intensity channel: ", CTL_CONTEXT(ctx, _intensity_channel),
      "Select intensity channel for spectrum analysis");

  ctlbind(ctls, "inpaint_missing_pixels:", CTL_CONTEXT(ctx, _mask_inpaint_method),
      "How to fill holes and borders in non-enpty masks");

  ctlbind(ctls, "Auto S1_target: ", CTL_CONTEXT(ctx, _autoS1_target),
      "Try to estimate S1 target automatically based in natural DCT spectrum slope estimation");

  ctlbind(ctls, "S1_target: ", CTL_CONTEXT(ctx, _S1_target),
      "Default Target slope of restored DCT spectrum");

  ctlbind(ctls, "macroStructSizePx: ", CTL_CONTEXT(ctx, _macroStructSizePx),
      "The minimal size in pixels of image macro structures still not much affected by blur");

  ctlbind(ctls, "print_debug_info:", CTL_CONTEXT(ctx, _print_debug_info), "");
  ctlbind(ctls, "write_debug_file:", CTL_CONTEXT(ctx, _write_file), "");
  ctlbind_browse_for_file(ctls, "debug_file ", CTL_CONTEXT(ctx, _debug_file_name), "");
}

bool c_dct_autosharp_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _intensity_channel);
    SERIALIZE_OPTION(settings, save, *this, _mask_inpaint_method);
    SERIALIZE_OPTION(settings, save, *this, _autoS1_target);
    SERIALIZE_OPTION(settings, save, *this, _S1_target);
    SERIALIZE_OPTION(settings, save, *this, _macroStructSizePx);
    SERIALIZE_OPTION(settings, save, *this, _debug_file_name);
    return true;
  }
  return false;
}

bool c_dct_autosharp_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  INSTRUMENT_REGION("");

  if ( _display == DISPLAY_SRC_IMAGE ) {
    // nothing to process requested
    return true;
  }

  cv::Mat src, msk;
  cv::Mat1f intensity_img;
  cv::Mat1f intensity_dct;
  cv::Mat1f dct_radial_profile;
  std::vector<cv::Mat1f> src_channels;

  const int cn = image.channels();

  image.getMat().convertTo(src, CV_32F);

  if ( !mask.empty() ) {
    if ( mask.depth() == CV_8U ) {
      msk = mask.getMat();
    }
    else {
      cv::compare(mask, 0, msk, cv::CMP_GT);
    }
  }

  if( !msk.empty() ) {
    switch (_mask_inpaint_method) {
      case LINEAR_INTERPOLATION_INPAINT:
        linear_interpolation_inpaint(src, msk);
        break;
      case AVERAGE_PYRAMID_INPAINT:
        average_pyramid_inpaint(src, msk, src, cv::noArray(), 7);
        break;
    }
  }

  if ( _display == DISPLAY_FILL_SRC_VOIDS ) {
    image.move(src);
    return true;
  }

  if ( cn == 1 ) {
     intensity_img = src;
   }
   else {
     extract_channel(src, intensity_img, cv::noArray(), cv::noArray(), _intensity_channel);
   }

   cv::dct(intensity_img, intensity_dct);
   if( _display == DISPLAY_SRC_SPECTRUM ) {
     image.move(intensity_dct);
     mask.release();
     return true;
   }

   if( _display == DISPLAY_SRC_RADIAL_PROFILE_LOG) {
     cv::absdiff(intensity_dct, cv::Scalar::all(0), intensity_dct);
     intensity_dct.setTo(1, intensity_dct == 0);
     cv::log(intensity_dct, intensity_dct);
     dctRadialProfile2(intensity_dct, dct_radial_profile);
     dctRadialProfileToImage(dct_radial_profile, intensity_dct.size(), intensity_dct);
     image.move(intensity_dct);
     mask.release();
     return true;
   }

   dctRadialProfile(intensity_dct, dct_radial_profile);
   if( _display == DISPLAY_SRC_RADIAL_PROFILE) {
     dctRadialProfileToImage(dct_radial_profile, intensity_dct.size(), intensity_dct);
     image.move(intensity_dct);
     mask.release();
     return true;
   }

   cv::Mat1f INVERSE_FILTER =
       createInverseBlurCorrectionFilter(dct_radial_profile, src.size(),
           _autoS1_target, _S1_target, _macroStructSizePx, _print_debug_info,
           _write_file ? _debug_file_name : "");

   if( _display == DISPLAY_FILTER) {
     image.move(INVERSE_FILTER);
     mask.release();
     return true;
   }

   if ( INVERSE_FILTER.empty() ) {
     CF_ERROR("createInverseBlurCorrectionFilter() fails");
     return false;
   }

   if ( cn == 1 ) {
     // Little optimized path for monochrome input image
     cv::multiply(intensity_dct, INVERSE_FILTER, intensity_dct);

     if( _display == DISPLAY_RESTORED_SPECTRUM) {
       image.move(intensity_dct);
     }
     else if( _display == DISPLAY_RESTORED_PROFILE) {
       dctRadialProfile(intensity_dct, dct_radial_profile);
       dctRadialProfileToImage(dct_radial_profile, intensity_dct.size(), intensity_dct);
       image.move(intensity_dct);
     }
     else {
       cv::idct(intensity_dct, image);
     }
   }
   else {
     // Full path for color image image

     std::vector<cv::Mat1f> src_channels;
     cv::split(src, src_channels);

     for ( int i = 0; i < cn; ++i ) {
       cv::dct(src_channels[i], src_channels[i]);
       cv::multiply(src_channels[i], INVERSE_FILTER, src_channels[i]);

       if( _display == DISPLAY_RESTORED_SPECTRUM) {
         continue;
       }

       if( _display == DISPLAY_RESTORED_PROFILE) {
         dctRadialProfile(src_channels[i], dct_radial_profile);
         dctRadialProfileToImage(dct_radial_profile, src_channels[i].size(), src_channels[i]);
         continue;
       }

       cv::idct(src_channels[i], src_channels[i]);
     }

     if( _display == DISPLAY_RESTORED_SPECTRUM || _display == DISPLAY_RESTORED_PROFILE ) {
       cv::merge(src_channels, image);
     }
     else {
       cv::merge(src_channels, image);
     }
   }

   if( _display == DISPLAY_RESTORED_SPECTRUM || _display == DISPLAY_RESTORED_PROFILE ) {
     mask.release();
   }

   return true;
}
