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
#include <core/proc/c_linear_regression.h>
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
      { c_fft_autosharp_routine::DISPLAY_RESTORED_IMAGE, },
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
    // exclude DC from energy normalization
    const int startCornersBin = int((mx.cols - 1) * M_SQRT1_2);
    _m = mx;
    _y0 = 0.5 * std::log(cv::norm(mx(cv::Rect(1, 0, startCornersBin - 1, 1)), cv::NORM_L2SQR) / (startCornersBin - 1));
    _L0 = 2 * std::log(1. / mx.cols);
  }

  inline int size() const
  {
    return _m.cols;
  }

  // raw linear intensity values from radial spectrum profile
  inline const float * intensity() const
  {
    return _m[0];
  }

  // zero point for log of relative intensity
  inline double y0() const
  {
    return _y0;
  }

  // log of frequency
  inline double xv(int i) const
  {
    return std::log(1.0 + i);
  }

  // log of relative intensity
  inline double yv(int i) const
  {
    return std::log(_m(0, i)) - _y0;
  }

  // log of laplacian operator in log space
  inline double lop(int i) const
  {
    return _L0 + 2 * std::log(1.0 + i);
  }

  // log of laplacian at bin index i
  inline double lv(int i) const
  {
    //return lop(i) + yv(i);
    return _L0 - _y0 + std::log((1.0 + i) * (1.0 + i) * _m(0, i));
  }

protected:
  cv::Mat1f _m; // [1][n_bins]
  double _y0 = 0;
  double _L0 = 0;
};

static cv::Mat1f smoothLaplace(const c_radial_spectrum_profile & p)
{
  const int N_uniform = 100;
  const int n_bins = p.size();

  std::vector<double> bin_sums(N_uniform, 0.0);
  std::vector<int> bin_counts(N_uniform, 0);

  // Skip DC
  const double x_min = p.xv(1);
  const double x_max = p.xv(n_bins - 1);
  const double x_range = x_max - x_min;

  for( int i = 1; i < n_bins; ++i ) {
    const double x = p.xv(i);
    const double l = p.lv(i);
    const int bin_idx = std::clamp(int((x - x_min) * (N_uniform - 1) / x_range), 0, N_uniform - 1);
    bin_sums[bin_idx] += l;
    bin_counts[bin_idx] += 1;
  }
  for( int i = 0; i < N_uniform; ++i ) {
    if( bin_counts[i] > 1 ) {
      bin_sums[i] /= bin_counts[i];
    }
  }

  cv::Mat1f U(1, N_uniform);
  float * __restrict up = U[0];

  // Gap Filling
  for( int j = 0; j < N_uniform; ++j ) {
    if( bin_counts[j] > 0 ) {
      up[j] = bin_sums[j];
    }
    else {
      int left_valid = -1;
      int right_valid = -1;

      for( int k = j - 1; k >= 0; --k ) {
        if( bin_counts[k] > 0 ) {
          left_valid = k;
          break;
        }
      }

      for( int k = j + 1; k < N_uniform; ++k ) {
        if( bin_counts[k] > 0 ) {
          right_valid = k;
          break;
        }
      }

      if( left_valid != -1 && right_valid != -1 ) {
        const float y_left = bin_sums[left_valid];
        const float y_right = bin_sums[right_valid];
        const float t = float(j - left_valid) / (right_valid - left_valid);
        up[j] = (1.0f - t) * y_left + t * y_right;
      }
      else if( left_valid != -1 ) {
        up[j] = float(bin_sums[left_valid]);
      }
      else if( right_valid != -1 ) {
        up[j] = float(bin_sums[right_valid]);
      }
      else {
        up[j] = 0.0f;
      }
    }
  }

  const int startCornersBin = int((n_bins - 1) * M_SQRT1_2);
  const double startCornersX = p.xv(startCornersBin);
  const int uniformStartCornersBin = std::clamp(int((startCornersX - x_min) * (N_uniform - 1) / x_range), 0, N_uniform - 1);
  const double startCornersValue = up[uniformStartCornersBin];
  for ( int i = uniformStartCornersBin; i < N_uniform; ++i ) {
    up[i] = startCornersValue;
  }
  cv::GaussianBlur(U, U, cv::Size(21, 1), 0, 0, cv::BORDER_REPLICATE);

  cv::Mat1f output_lap(1, n_bins);
  float * __restrict dstp = output_lap[0];
  dstp[0] = p.lv(0); //  0.0f; // DC

  const float * smup = U[0];
  for( int i = 1; i < n_bins; ++i ) {
    const double orig_x = p.xv(i);
    const double uniform_idx = (orig_x - x_min) * (N_uniform - 1) / x_range;
    const int k = int(uniform_idx);

    if( k < 0 ) {
      dstp[i] = smup[0];
    }
    else if( k >= N_uniform - 1 ) {
      dstp[i] = smup[N_uniform - 1];
    }
    else {
      double t = uniform_idx - k;
      dstp[i] = (1.0f - t) * smup[k] + t * smup[k + 1];
    }
  }

  return output_lap;
}

static int estimateNature(const c_radial_spectrum_profile sp,
    double S1_nature_gain,
    const cv::Mat1f & LSM,
    double & output_S0_lap,
    double & output_S1_lap,
    double & output_S2_lap,
    double & output_S0_nature,
    double & output_S1_nature,
    double & output_xt,
    double & output_yt )
{
  /*
   * Approximate lap(x) to find start of the blur
   *  lap(x) = S0_lap + S1_lap * x + S2_lap * x  * x
   */

  const int N = sp.size();
  const int startCornersBin = int((N - 1) * M_SQRT1_2);

  int curvatureEndPoint = -1;
  double S0_lap = 0, S1_lap = 0, S2_lap = 0;
  double S2_best = 0;

  c_linear_regression3 reg3;

  for( int i = 1, n = 0; i < startCornersBin; ++i ) {
    const double x = sp.xv(i);
    const double y = sp.yv(i);
    if ( y <= 0 ) {

      const double l = LSM(0, i);
      reg3.update(1, x, x * x, l);

      if ( ++n > 15 ) {
        double S0_tmp = 0, S1_tmp = 0, S2_tmp = 0;
        reg3.compute(S0_tmp, S1_tmp, S2_tmp);
        if( S2_tmp < S2_best ) {
          S2_best = S2_tmp;
          S0_lap = S0_tmp, S1_lap = S1_tmp, S2_lap = S2_tmp;
          curvatureEndPoint = i;
        }
      }
    }
  }

  output_S0_lap = S0_lap;
  output_S1_lap = S1_lap;
  output_S2_lap = S2_lap;

  /*
   * Approximate nature(x) to find nature slope using Gaussian Dome Invariant
   *  nature(x) = S0_nature + S1_nature * x
   */

  // Search for the blur start point (LS separation from LQ) when moving left from curvatureEndPoint
  const double lxmax = -0.5 * S1_lap / S2_lap; // x coordinate of the laplacian extremum
  const double llmax = S0_lap - 0.25 * S1_lap * S1_lap / S2_lap; // (L value at laplacian extremum)
  double xStartMF = 0;
  double yStartMF = 0;
  int indexOfStartMF = -1;
  for ( int i = 1; i < curvatureEndPoint; ++i ) {
    const double x = sp.xv(i);
    const double y = sp.yv(i);
    if ( y <= 0  && sp.yv(i + 1) <= 0 ) {
      indexOfStartMF = i;
      xStartMF = x;
      yStartMF = y;
      break;
    }
  }

  if ( indexOfStartMF < 2 ) {
    CF_ERROR("\nBad input data: indexOfStartMF=%d lxmax=%g S0_lap=%g S1_lap=%g S2_lap=%g",
        indexOfStartMF, lxmax,
        S0_lap, S1_lap, S2_lap);

    return -1;
  }


  if( lxmax < xStartMF ) { // lxmax is in low frequency region

    const double xlt = xStartMF;
    const double ylt = LSM(0, indexOfStartMF);
    const double dx = xStartMF - lxmax;
    const double dy = llmax - (S0_lap + S1_lap * xlt + S2_lap * xlt * xlt);
    output_S1_nature = std::max(0.3, dy / dx) * S1_nature_gain;
    output_S0_nature = ylt - output_S1_nature * xlt;
    output_xt = xlt;
    output_yt = ylt;
    CF_DEBUG("\nLF: "
        "indexOfStartMF=%d xlt=%g ylt=%g dx=%g dy=%g output_S0_nature = %g output_S1_nature = %g",
        indexOfStartMF, xlt, ylt, dx, dy, output_S0_nature, output_S1_nature);
  }
  else {

    const double threshold = 0.25;
    double x_start_blur = sp.xv(1);
    double delta_start_blur = 0;
    int index_start_blur = 1;
    for( int i = indexOfStartMF; i > 0; --i ) {
      const double x = sp.xv(i);
      const double lq = S0_lap + S1_lap * x + S2_lap * x * x;
      const double ls = LSM(0, i);
      if( (ls - lq) >= threshold ) {
        x_start_blur = x;
        delta_start_blur = ls - lq;
        index_start_blur = i;
        break;
      }
    }

    if ( x_start_blur >= lxmax - 0.1 ) {
      CF_DEBUG("\nFixing x_start_blur=%g lxmax=%g", x_start_blur, lxmax);
      x_start_blur = lxmax - 0.5;
    }

    // Radius of the active blur dome and the application of the 0.42 invariant
    const double R = lxmax - x_start_blur;
    const double xlt = lxmax - 0.42 * R;
    // The value of the parabola ordinate at the calculated point of contact xlt
    const double ylt = S0_lap + S1_lap * xlt + S2_lap * xlt * xlt;

    // Coefficients of the LT line according to the requirement of equality of derivatives
    output_S1_nature = std::max(0.3, (2.0 * S2_lap * xlt + S1_lap)) * S1_nature_gain;
    output_S0_nature = ylt - output_S1_nature * xlt;
    output_xt = xlt;
    output_yt = ylt;

    CF_DEBUG("\nMF: "
        "indexOfStartMF=%d x_start_blur=%g delta_start_blur=%g index_start_blur=%d",
        indexOfStartMF, x_start_blur, delta_start_blur, index_start_blur);
  }



  return curvatureEndPoint;
}

cv::Mat1f createInverseBlurCorrectionFilter(const cv::Mat1f & RadialSpectrumProfile, /*[1][n_bins] */
    const cv::Size & fftSize,
    double S1_nature_gain,
    const std::string & debug_file_name = "")
{

  const c_radial_spectrum_profile sp(RadialSpectrumProfile);

  const int N = sp.size();
  const cv::Mat1f LSM = smoothLaplace(sp);

  double S0_lap, S1_lap, S2_lap;
  double S0_nature = 0, S1_nature = -500;
  double xlt = 0, ylt = 0;

  const int curvatureEndPoint =
      estimateNature(sp, S1_nature_gain, LSM, S0_lap, S1_lap, S2_lap,
          S0_nature, S1_nature,
          xlt, ylt);

  if ( curvatureEndPoint < 1 ) {
    CF_ERROR("searchLaplaceExtreme() fails");
    return cv::Mat1f();
  }

  const double lxmax = -0.5 * S1_lap / S2_lap; // (x coordinate of the laplacian extremum)
  const double llmax = S0_lap - 0.25 * S1_lap * S1_lap / S2_lap; // (L value at laplacian extremum)
  if ( S1_nature < 0.05 ) {
    CF_ERROR("BAD S1_nature=%g", S1_nature);
    //S1_nature = 0.05;
  }

//  S1_nature *= S1_nature_gain;
//  const double xlt = 0.5 * (S1_nature - S1_lap) / S2_lap;
//  const double ylt = S0_lap + S1_lap * xlt + S2_lap * xlt * xlt;
//  S0_nature += ylt - S0_nature - S1_nature * xlt;

  /*
   * COMPUTE CORRECTION for x > xlt:
   *   CORRECTION  = NATURE - SMY;
   */
  cv::Mat1f correction(1, sp.size(), 1.0f);
  const int startBin = 4;
  const double startX = std::max(sp.xv(startBin), xlt);

  for( int i = startBin; i < sp.size(); ++i ) {
    const double x = sp.xv(i);
    const double nature = S0_nature + S1_nature * x;
    const double laplace = LSM(0, i); // L_SMOOTH
    const double full_corr = std::max(0., nature - laplace);
    correction(0, i) = float(std::exp(full_corr / (1. + std::exp( -5 * (x - startX)))));
  }

  /*
   * Create FILTER
   */

  const cv::Size size = fftSize;
  cv::Mat1f FILTER(size);

  const double cx = size.width / 2;
  const double cy = size.height / 2;
  const double R = std::sqrt(cx * cx + cy * cy);
  const int numBins = std::max(1, int(R));
  const double maxNormalizedR = std::sqrt(2.0);

  const double scaleX = 1. / cx;
  const double scaleY = 1. / cy;

  cv::parallel_for_(cv::Range(0, size.height),
      [=, &FILTER](const cv::Range & range) {
        for (int y = range.start; y < range.end; ++y) {
          float * __restrict dstp = FILTER[y];

          const double dy = (y - cy) * scaleY;
          const double dy2 = dy * dy;

          for (int x = 0; x < size.width; ++x) {
            const double dx = (x - cx) * scaleX;
            const double dx2 = dx * dx;

            const double r = std::sqrt(dx2 + dy2);
            const double continuousBinIdx = r * numBins / maxNormalizedR; //  - 0.5;
            const int binIndex = std::clamp((int)(continuousBinIdx), 0, N - 1);
            dstp[x] = correction(0, binIndex);
            //              const double corr = correction(0, binIndex);
            //              const double gain = std::exp(corr);
            //              dstp[x] = float(gain);

          }
        }
      });


  CF_DEBUG("\n"
      "S0_lap = %g S1_lap = %g S2_lap = %g\n"
      "S0_nature = %g S1_nature = %g\n"
      "lxmax = %g llmax=%g\n"
      "xlt = %g ylt = %g\n",
      S0_lap, S1_lap, S2_lap,
      S0_nature, S1_nature,
      lxmax, llmax,
      xlt, ylt);

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
      fprintf(fp, "I\tX\tS\tY\tL\tLS\tLQ\tLT\tCORRECTION\tLP\tYP\n");

      const float * src_y = sp.intensity();

      for( int i = 0; i < N; ++i ) {
        const double x = sp.xv(i); // log of frequency
        const double y = sp.yv(i); // log of spectrum intensity
        const double l = sp.lop(i) + y; // laplcace
        const double ls = LSM(0, i);
        const double lq = S0_lap + S1_lap * x + S2_lap * x * x;
        const double ln = S0_nature + S1_nature * x;
        const double corr = std::log(correction(0, i));
        const double yp = y + corr;
        const double lp = l + corr;

        fprintf(fp, "%4d\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\n",
            i, x, src_y[i], y, l, ls, lq, ln, corr, lp, yp);
      }

      CF_DEBUG("Saved file '%s'", fp.cfilename());
    }
  }

  return FILTER;
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

static void computeLinearIntensityMathitude(cv::Mat1f & INTENSITY_Magnitude,
    const cv::Mat2f & SRC_P_B, double wB,
    const cv::Mat2f & SRC_P_G, double wG,
    const cv::Mat2f & SRC_P_R, double wR)
{
  const cv::Size fftSize = SRC_P_B.size();

  INTENSITY_Magnitude.create(fftSize);

  cv::parallel_for_(cv::Range(0, fftSize.height),
      [&, fftSize, wB, wG, wR](const cv::Range & range) {

        const int cx = fftSize.width;

        for (int y = range.start; y < range.end; ++y) {
          const float * __restrict bp = reinterpret_cast<const float*>(SRC_P_B[y]);
          const float * __restrict gp = reinterpret_cast<const float*>(SRC_P_G[y]);
          const float * __restrict rp = reinterpret_cast<const float*>(SRC_P_R[y]);
          float * __restrict dstp = INTENSITY_Magnitude[y];

          for (int x = 0; x < cx; ++x, bp += 2, gp += 2, rp += 2) {
            constexpr int xre = 0;
            constexpr int xim = 1;
            const float re = bp[xre] * wB + gp[xre] * wG + rp[xre] * wR;
            const float im = bp[xim] * wB + gp[xim] * wG + rp[xim] * wR;
            *dstp++ = std::sqrt(re * re + im * im);
          }
        }
      });
}

}


// moon:  /mnt/data/scope/2023-08-04/MOON3/image_stacking1
// mars: /mnt/data/scope/2022-11-13/s7/CapObj/2022-11-13Z/s2
void c_fft_autosharp_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Display: ", CTL_CONTEXT(ctx, _display), "Select image to display");
  ctlbind(ctls, "Intensity channel: ", CTL_CONTEXT(ctx, _intensity_channel), "Select intensity channel for spectrum analysis");
  ctlbind(ctls, "S1_gain: ", CTL_CONTEXT(ctx, _S1_gain), "");
  ctlbind(ctls, "write_debug_file ", CTL_CONTEXT(ctx, _write_file), "");
  ctlbind_browse_for_file(ctls, "debug_file ", CTL_CONTEXT(ctx, _debug_file_name), "");
  c_anscombe_transform::getcontrols(ctls, ctx(&this_class::_anscombe));
}

bool c_fft_autosharp_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _intensity_channel);
    SERIALIZE_OPTION(settings, save, *this, _S1_gain);
    SERIALIZE_OPTION(settings, save, *this, _debug_file_name);

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

bool c_fft_autosharp_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask )
{
  if ( _display == DISPLAY_SRC_IMAGE ) {
    return true;
  }

  cv::Rect rc;
  const cv::Mat src = image.getMat();
  const cv::Size srcSize = src.size();
  const cv::Size psfSize(std::max(15, 2 * (srcSize.width / 32) + 1), std::max(15, 2 * (srcSize.height / 32) + 1));
  const cv::Size fftSize = fftGetOptimalSize(image.size(), psfSize);
  const int cn = image.channels();

  double wB = 0, wG = 0, wR = 0;

  CF_DEBUG("select psfSize=%dx%d", psfSize.width, psfSize.height);

  if ( src.size() == fftSize ) {
    SRC_IMAGE = src, rc = cv::Rect(0, 0, src.cols, src.rows);
  }
  else {
    fftCopyMakeBorder(src, SRC_IMAGE, fftSize, &rc);
  }

  if ( SRC_IMAGE.depth() != CV_32F ) {
    SRC_IMAGE.convertTo(SRC_IMAGE, CV_32F);
  }

  if ( _anscombe.method() != anscombe_none ) {
    _anscombe.apply(SRC_IMAGE, SRC_IMAGE);
  }

  if( VLAP.size() != fftSize ) {
    VLAP = fftGenerateDiscreteLaplacianFilter(fftSize, true);
  }

  fftPPSDecomposition(SRC_IMAGE, VLAP,
      &SRC_P, &SRC_S,
      true);

  if ( cn == 1 ) {
    fftSpectrumModule(SRC_P[0], INTENSITY_Magnitude);
  }
  else if ( getLinearIntensityWeights(_intensity_channel, wB, wG, wR) ) {
    computeLinearIntensityMathitude(INTENSITY_Magnitude,
        SRC_P[0], wB,
        SRC_P[1], wG,
        SRC_P[2], wR);
  }
  else {
    // Nonlinear channel: Lab, HSV, etc
    extract_channel(SRC_IMAGE, INTENSITY_CHANNEL, cv::noArray(), cv::noArray(),
        _intensity_channel);

    fftPPSDecomposition(INTENSITY_CHANNEL, VLAP,
        INTENSITY_P, INTENSITY_S,
        true);

    fftSpectrumModule(INTENSITY_P, INTENSITY_Magnitude);
  }

  fftRadialProfile(INTENSITY_Magnitude, INTENSITY_RadialProfile);

  const cv::Mat1f INVERSE_FILTER =
      createInverseBlurCorrectionFilter(INTENSITY_RadialProfile, fftSize, _S1_gain,
          _write_file ? _debug_file_name : "");

  if ( INVERSE_FILTER.empty() ) {
    CF_ERROR("createInverseBlurCorrectionFilter() fails");
    return false;
  }

  SRC_CHANNELS_RESTORED.resize(cn);
  for ( int i = 0; i < cn; ++i ) {
    fftMulSpectrum(INVERSE_FILTER, SRC_P[i], SRC_P[i]);
    cv::add(SRC_P[i], SRC_S[i], SRC_P[i]);
    fftSwapQuadrants(SRC_P[i]);
    cv::idft(SRC_P[i], SRC_CHANNELS_RESTORED[i], cv::DFT_SCALE |
        cv::DFT_REAL_OUTPUT);
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
