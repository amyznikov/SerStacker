/*
 * c_dct_autosharp_routine.cc
 *
 *  Created on: Jul 23, 2026
 *      Author: amyznikov
 */

#include "c_dct_autosharp_routine.h"
#include <core/proc/fft.h>
#include <core/ssprintf.h>
#include <core/proc/inpaint/average_pyramid_inpaint.h>
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
      { c_dct_autosharp_routine::DISPLAY_FILTER, "FILTER", "" },
      { c_dct_autosharp_routine::DISPLAY_RESTORED_SPECTRUM, "RESTORED_SPECTRUM", "" },
      { c_dct_autosharp_routine::DISPLAY_RESTORED_PROFILE, "RESTORED_PROFILE", "" },
      { c_dct_autosharp_routine::DISPLAY_RESTORED_IMAGE}
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
    const int total_bins = mx.cols;
    const double total_energy = cv::norm(mx(cv::Rect(1, 0, total_bins - 1, 1)), cv::NORM_L2SQR);
    _y0 = 0.5 * std::log(total_energy);
    _m = mx;
  }

  inline int size() const
  {
    return _m.cols;
  }

  inline double y0() const
  {
    return _y0;
  }

  inline double xv(int i) const
  {
    return std::log(std::max(1,i));
  }

  inline double sv(int i) const
  {
    return _m[0][i];
  }

  inline double yv(int i) const
  {
    return std::log(_m[0][i]) - _y0;
  }

protected:
  cv::Mat1f _m; // [1][n_bins]
  double _y0 = 0;
};



static cv::Mat1f smoothDCT(const c_radial_spectrum_profile & p)
{
  constexpr int N_uniform = 100;
  const int n_bins = p.size();

  std::vector<double> bin_sums(N_uniform, 0.0);
  std::vector<int> bin_counts(N_uniform, 0);

  // Initialize the frequency range (skip DC)
  const double x_min = p.xv(1);
  const double x_max = p.xv(n_bins - 1);
  const double x_range_inv = (x_max > x_min) ? (N_uniform - 1) / (x_max - x_min) : 0.0;

  // Uniform accumulation (resampling)
  for( int i = 1; i < n_bins; ++i ) {
    const int bin_idx = std::clamp(int((p.xv(i) - x_min) * x_range_inv), 0, N_uniform - 1);
    const double m = p.sv(i);
    if ( m > 0 ) {
      bin_sums[bin_idx] += p.yv(i);
      bin_counts[bin_idx]++;
    }
  }

  // Average non-empty cells
  for( int i = 0; i < N_uniform; ++i ) {
    if( bin_counts[i] > 1 ) {
      bin_sums[i] /= bin_counts[i];
    }
  }

  // Gap Filling
  cv::Mat1f U(1, N_uniform);
  float * __restrict up = U[0];

  int last_valid_idx = -1;
  for( int j = 0; j < N_uniform; ++j ) {
    if( bin_counts[j] > 0 ) {
      up[j] = static_cast<float>(bin_sums[j]);

      // If there were holes before, interpolate them from the last valid one to the current one
      if (last_valid_idx != j - 1) {
        const int left = std::max(0, last_valid_idx);
        const float y_left = (last_valid_idx >= 0) ? up[left] : up[j];
        const float y_right = up[j];
        const float inv_step = 1.0f / (j - left);

        for (int k = left + 1; k < j; ++k) {
          const float t = (k - left) * inv_step;
          up[k] = (1.0f - t) * y_left + t * y_right;
        }
      }
      last_valid_idx = j;
    }
  }

  // Extrapolate the right edge if the last bins remain empty
  if (last_valid_idx >= 0 && last_valid_idx < N_uniform - 1) {
    std::fill(up + last_valid_idx + 1, up + N_uniform, up[last_valid_idx]);
  }

  // Freezing the tail (Spectrum Matrix Angle Region)
  const int startCornersBin = int((n_bins - 1) * M_SQRT1_2);
  const int uniformStartCornersBin = std::clamp(int((p.xv(startCornersBin) - x_min) * x_range_inv), 0, N_uniform - 1);
  std::fill(up + uniformStartCornersBin, up + N_uniform, up[uniformStartCornersBin]);

  // Smoothing the uniform profile
  cv::medianBlur(U, U, 5);
  cv::GaussianBlur(U, U, cv::Size(31, 1), 0, 0, cv::BORDER_REPLICATE);

  // Back interpolation into the original bin grid
  // DC is kept unchanged
  cv::Mat1f output(1, n_bins);
  float * __restrict dstp = output[0];
  dstp[0] = float(p.yv(0));

  const float * smup = U[0];
  for( int i = 1; i < n_bins; ++i ) {
    // Safe index clamp eliminates crashes and branches in the loop
    const double uniform_idx = (p.xv(i) - x_min) * x_range_inv;
    const int k = std::clamp(int(uniform_idx), 0, N_uniform - 2);
    const double t = uniform_idx - k;
    dstp[i] = float((1.0 - t) * smup[k] + t * smup[k + 1]);
  }

  return output;
}

static int estimateNature2(const c_radial_spectrum_profile &sp,
    const cv::Mat1f & SDCT,
    double & output_S0_nature,
    double & output_S1_nature,
    double & output_S0_maxline,
    double & output_S1_maxline,
    int & output_inflectionPoint,
    cv::Mat1f & kx)
{
  const int N = sp.size();
  const int startCornersBin = int((N - 1) * M_SQRT1_2);
  const double startCornersLog = std::log((N - 1) * M_SQRT1_2);

  kx.create(1, N);
  kx.setTo(0);

  int startSearchBin = 1;
  for ( ; startSearchBin < startCornersBin; ++startSearchBin) {
    const double xrel = startCornersLog - std::log(startSearchBin);
    if ( xrel < 6 ) {
      break;
    }
  }
  if( startSearchBin >= startCornersBin ) {
    CF_ERROR("Bad startSearchBin=%d >= startCornersBin=%d", startSearchBin, startCornersBin);
    return -1;
  }

  CF_DEBUG("Initial startSearchBin=%d (x=%g y=%g)", startSearchBin, sp.xv(startSearchBin), SDCT(0, startSearchBin));

  for ( ; startSearchBin < startCornersBin - 4; ++ startSearchBin) {
    const double x0 = sp.xv(startSearchBin + 0);
    const double y0 = SDCT(0, startSearchBin + 0);
    const double x1 = sp.xv(startSearchBin + 1);
    const double y1 = SDCT(0, startSearchBin + 1);
    const double x2 = sp.xv(startSearchBin + 2);
    const double y2 = SDCT(0, startSearchBin + 2);
    const double x3 = sp.xv(startSearchBin + 3);
    const double y3 = SDCT(0, startSearchBin + 3);
    const double d0 = (y1 - y0) / (x1 - x0);
    const double d1 = (y2 - y1) / (x2 - x1);
    const double d2 = (y3 - y2) / (x3 - x2);
    if ( d0 <= -0.5 && d1 <= -0.5 && d2 <= -0.5 ) {
      break;
    }
  }

  const double startX = sp.xv(startSearchBin);
  const double startY = SDCT(0, startSearchBin);

  int endSearchBin = -1;
  double Kx = DBL_MAX;
  double Kx_best = DBL_MAX;

  CF_DEBUG("startSearchBin=%d (x=%g y=%g)", startSearchBin, startX, startY);

  for( int i = startSearchBin; i < startCornersBin; ++i ) {
    const double x = sp.xv(i) - startX;
    const double y = SDCT(0, i) - startY;
    const double Kx_temp = y / (x + 0.1);
    kx(0, i) = Kx_temp; // save also for debug / visualization
    if( i > startSearchBin + 3 && Kx_temp < Kx_best ) {
      Kx_best = Kx_temp;
      Kx = y / x;
      endSearchBin = i;
    }
  }

  if( endSearchBin <= startSearchBin ) {
    CF_ERROR("ERROR: endSearchBin=%d <= startSearchBin=%d", endSearchBin, startSearchBin);
    return -1;
  }
  CF_DEBUG("endSearchBin=%d (x=%g y=%g) Kx=%g Kx_best=%g", endSearchBin, sp.xv(endSearchBin), SDCT(0, endSearchBin), Kx, Kx_best);


  const double S0_maxline = startY - Kx * startX;
  const double S1_maxline = Kx;
  output_S0_maxline = S0_maxline;
  output_S1_maxline = S1_maxline;

  int inflectionBin = -1;
#if 1
  double max_distance = 0.0;
  const double xMid = 0.5 * (sp.xv(startSearchBin) + sp.xv(endSearchBin));

  // Knee method: looking for the maximum positive vertical distance (curve above the line)
  for (int i = startSearchBin + 3; i < endSearchBin; ++i) {
    const double x = sp.xv(i);
    if ( x > 2.0 ) {
      const double line_val = S1_maxline * x  + S0_maxline;
      const double curve_val = SDCT(0, i);
      const double distance = (curve_val - line_val) / std::sqrt(1.0 + 0.25 * std::abs(x-xMid));
      if (distance > max_distance) {
        max_distance = distance;
        inflectionBin = i;
      }
    }
  }

  if( inflectionBin <= startSearchBin ) {
    CF_ERROR("BAD inflectionBin=%d <= startSearchBin=%d endSearchBin=%d", inflectionBin, startSearchBin, endSearchBin);
    return -1;
  }
#endif
#if 0
  c_line_estimate line;
  output_S0_nature = 0;
  output_S1_nature = -DBL_MAX;
  for (int i = startSearchBin; i < endSearchBin; ++i) {
    const double xrel = startCornersLog - std::log(i);
    if ( xrel < 5.5 ) {
      const double x = sp.xv(i);
      const double y = SDCT(0, i);
      line.update(x, y);
      if ( line.pts() > 15 ) {
        double S0_nature = 0, S1_nature= -DBL_MAX;
        line.compute(S0_nature, S1_nature);
        if ( S1_nature  > output_S1_nature ) {
          output_S0_nature = S0_nature;
          output_S1_nature = S1_nature;
          inflectionBin = i;
        }
      }
    }
  }
#endif

  CF_DEBUG("inflectionBin=%d (x=%g y=%g)", inflectionBin, sp.xv(inflectionBin), SDCT(0, inflectionBin) );

  c_line_estimate line;
  c_linear_regression3 reg3;
  double s2_best = DBL_MAX;
  output_inflectionPoint = -1;

  for ( int i = startSearchBin; i < inflectionBin; ++i ) {
    const double x = sp.xv(i);
    const double y = SDCT(0, i);
    reg3.update(1, x, x * x, y);
    line.update(x, y);
    if ( line.pts() > 15 ) {
      double s0, s1, s2;
      reg3.compute(s0, s1, s2);
      if ( std::abs(s2) < s2_best ) {
        s2_best = std::abs(s2);
        line.compute(output_S0_nature, output_S1_nature);
        output_inflectionPoint = i;
      }
    }
  }

  if ( output_inflectionPoint < 0 ) {
    line.compute(output_S0_nature, output_S1_nature);
    output_inflectionPoint = inflectionBin;
  }

  return startSearchBin;
}

static cv::Mat1f createInverseBlurCorrectionFilter(const cv::Mat1f & RadialSpectrumProfile, /*[1][n_bins] */
    const cv::Size & dctSize,
    double S1_target,
    const std::string & debug_file_name = "")
{
  const c_radial_spectrum_profile sp(RadialSpectrumProfile);
  cv::Mat1f correction(1, sp.size(), 1.0f);
  cv::Mat1f kx(1, sp.size());
  cv::Mat1f FILTER;
  const double wsf = 2;

  const int N = sp.size();
  const cv::Mat1f SDCT = smoothDCT(sp);

  double S0_nature = 0, S1_nature = -500;
  double inflection_x = 0;//, inflection_y = 0;
  int corrStartBin = 0;

  double S0_maxline = 0, S1_maxline = 0;
  int inflectionBin = 0;

  corrStartBin =
      estimateNature2(sp, SDCT,
          S0_nature, S1_nature,
          S0_maxline, S1_maxline,
          inflectionBin,
          kx);

  if ( corrStartBin < 1 ) {
    CF_ERROR("estimateNature2() fails");
    inflectionBin = 0;
  }
  else {

    /**
     * COMPUTE CORRECTION WITH TARGET SLOPE ADJUSMENT:
     * correction  = nature - sdct;
     *  S1_target - entered by user (e.g., -1.5)
     *  S1_nature - calculated in estimateNature2()
     **/

    // Logarithm of frequency at the conjugation point (support point for changing the slope)
    const double x_start = sp.xv(corrStartBin);

    inflection_x = sp.xv(inflectionBin);
    //inflection_y = SDCT(0, inflectionPoint);

    for( int i = corrStartBin + 1; i < sp.size(); ++i ) {
      // Original "natural" trend of the spectrum
      const double x = sp.xv(i);
      const double nature = S0_nature + S1_nature * x;
      const double y = SDCT(0, i);

      // Local whitening additive (pure elimination of subsidence relative to nature)
      const double corr = std::max(0.0, nature - y);

      // Correction for the change in the global slope of the spectrum relative to the x_start point
      // If S1_target > S1_nature (e.g. -2.0 > -2.24), slope_diff will be positive at high frequencies (gain)
      const double slope_diff = (S1_target - S1_nature) * (x - x_start);

      // Total logarithmic correction under the sigmoid smoothing window
      const double total_corr = (corr + slope_diff) / (1.0 + std::exp(-wsf * (x - inflection_x)));
      correction(0, i) = float(std::exp(total_corr));
    }

    /*
     * Create the DCT FILTER
     */

    const cv::Size size = dctSize;
    FILTER.create(size);

    const double R = std::sqrt(size.width * size.width + size.height * size.height);
    const int numBins = std::max(1, int(R));
    const double maxNormalizedR = std::sqrt(2.0);
    const double scaleX = 1.0 / size.width;
    const double scaleY = 1.0 / size.height;

    cv::parallel_for_(cv::Range(0, size.height),
        [=, &FILTER](const cv::Range & range) {
          for (int y = range.start; y < range.end; ++y) {
            float * __restrict dstp = FILTER[y];

            const double dy = y * scaleY;
            const double dy2 = dy * dy;

            for (int x = 0; x < size.width; ++x) {
              const double dx = x * scaleX;
              const double dx2 = dx * dx;
              const double r = std::sqrt(dx2 + dy2);
              const double continuousBinIdx = r * numBins / maxNormalizedR;
              const int binIndex = std::clamp((int)(continuousBinIdx), 0, N - 1);
              dstp[x] = correction(0, binIndex);
            }
          }
        });

  }

  CF_DEBUG("\n"
      "corrStartBin=%d (x = %g y = %g)\n"
      "S0_maxline = %g S1_maxline = %g\n"
      "S0_nature = %g S1_nature = %g\n"
      "inflectionPoint = %d (x = %g y = %g)\n",
      corrStartBin, sp.xv(corrStartBin), sp.yv(corrStartBin),
      S0_maxline, S1_maxline,
      S0_nature, S1_nature,
      inflectionBin, sp.xv(inflectionBin), SDCT(0, inflectionBin));

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
      fprintf(fp, "I\tX\tS\tDCT\tSDCT\tSDCT_LINEAR_APPROXIMATION\tCORRECTION\tDCT_WHITENED\tW\tMAXL\tXIP\tDCT_RESORED\tDYS\tXREL\tKx\n");

      const double startCornersLog = std::log((N - 1) * M_SQRT1_2);
      for( int i = 0; i < N; ++i ) {
        const double src_y = sp.sv(i);
        const double x = sp.xv(i); // log of frequency
        const double y = sp.yv(i); // log of spectrum intensity
        const double ys = SDCT(0, i);
        const double yn = S0_nature + S1_nature * x;
        const double corr = std::log(correction(0, i));
        const double yp = y + corr;
        const double wc = 1.0 / (1.0 + std::exp(-wsf * (x - inflection_x)));
        const double ymaxl = S0_maxline + S1_maxline * x;
        const double xip = sp.xv(inflectionBin);
        const double yip = ys;
        const double dys = i == 0 ? 0 : (ys - SDCT(0, i - 1)) / (x - sp.xv(i - 1));
        const double Kx = kx(0, i);
        const double xrel = -std::log(std::max(1, i)) + startCornersLog;

        fprintf(fp, "%4d\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\n",
            i, x, src_y, y, ys, yn, corr, yp, wc, ymaxl, xip, yip, dys, xrel, Kx);
      }

      CF_DEBUG("Saved file '%s'", fp.cfilename());
    }
  }

  return FILTER;
}
} // namespace

void c_dct_autosharp_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "display", CTL_CONTEXT(ctx, _display), "");
  ctlbind(ctls, "Intensity channel: ", CTL_CONTEXT(ctx, _intensity_channel), "Select intensity channel for spectrum analysis");
  ctlbind(ctls, "inpaint_missing_pixels", CTL_CONTEXT(ctx, _inpaint_missing_pixels), "");
  ctlbind(ctls, "S1_target: ", CTL_CONTEXT(ctx, _S1_target), "");
  ctlbind(ctls, "write_debug_file ", CTL_CONTEXT(ctx, _write_file), "");
  ctlbind_browse_for_file(ctls, "debug_file ", CTL_CONTEXT(ctx, _debug_file_name), "");
}

bool c_dct_autosharp_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _intensity_channel);
    SERIALIZE_OPTION(settings, save, *this, _inpaint_missing_pixels);
    SERIALIZE_OPTION(settings, save, *this, _S1_target);
    SERIALIZE_OPTION(settings, save, *this, _debug_file_name);
    return true;
  }
  return false;
}


bool c_dct_autosharp_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( _display == DISPLAY_SRC_IMAGE ) {
    // nothing to process requested
    return true;
  }

  cv::Mat src;
  cv::Mat1f intensity_img;
  cv::Mat1f intensity_dct;
  cv::Mat1f dct_radial_profile;
  std::vector<cv::Mat1f> src_channels;

  const int cn = image.channels();

  image.getMat().convertTo(src, CV_32F);

  if ( !mask.empty() && _inpaint_missing_pixels ) {
    average_pyramid_inpaint(src, mask, src, cv::noArray(), 7);
  }

  if ( _display == DISPLAY_FILL_SRC_VOIDS ) {
    image.move(src);
    return true;
  }

  if ( cn == 1 ) {
    intensity_img = src;
  }
  else {
    extract_channel(src, intensity_img, cv::noArray(), cv::noArray(),
        _intensity_channel);
  }

  cv::dct(intensity_img, intensity_dct);
  if( _display == DISPLAY_SRC_SPECTRUM ) {
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
      createInverseBlurCorrectionFilter(dct_radial_profile, src.size(), _S1_target,
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

  if( src_channels.empty() ) {
    if ( cn == 1 ) {
      src_channels.emplace_back(src);
    }
    else {
      cv::split(src, src_channels);
    }
  }

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

  if ( cn == 1 ) {
    image.move(src_channels[0]);
  }
  else {
    cv::merge(src_channels, image);
  }

  if( _display == DISPLAY_RESTORED_SPECTRUM || _display == DISPLAY_RESTORED_PROFILE ) {
    mask.release();
  }

  return true;
}

