/*
 * c_fft_profile_test1_routine.cc
 *
 *  Created on: Jun 5, 2026
 *      Author: amyznikov
 */

#include "c_fft_profile_test1_routine.h"

#include <core/proc/estimate_noise.h>
#include <core/proc/c_line_estimate.h>
#include <core/proc/c_linear_regression.h>
#include <core/io/c_stdio_file.h>
#include <core/proc/fft.h>
#include <core/ssprintf.h>

template<>
const c_enum_member * members_of<c_fft_profile_test1_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_fft_profile_test1_routine::DISPLAY_SRC_IMAGE, "SRC_IMAGE", },
      { c_fft_profile_test1_routine::DISPLAY_SRC_MODULE, "SRC_MODULE" },
      { c_fft_profile_test1_routine::DISPLAY_SRC_PROFILE, "SRC_PROFILE" },

      { c_fft_profile_test1_routine::DISPLAY_GAUSS_MODULE, "GAUSS_MODULE", },
      { c_fft_profile_test1_routine::DISPLAY_GAUSS_PROFILE, "GAUSS_PROFILE", },
      { c_fft_profile_test1_routine::DISPLAY_GAUSS_FILTERED_MODULE, "GAUSS_FILTERED_MODULE", },
      { c_fft_profile_test1_routine::DISPLAY_GAUSS_FILTERED_PROFILE, "GAUSS_FILTERED_PROFILE", },
      { c_fft_profile_test1_routine::DISPLAY_GAUSS_FILTERED_IMAGE, "GAUSS_FILTERED_IMAGE", },

      { c_fft_profile_test1_routine::DISPLAY_LAPL_MODULE, "LAPL_MODULE", },
      { c_fft_profile_test1_routine::DISPLAY_LAPL_PROFILE, "LAPL_PROFILE", },
      { c_fft_profile_test1_routine::DISPLAY_LAPL_FILTERED_MODULE, "LAPL_FILTERED_MODULE", },
      { c_fft_profile_test1_routine::DISPLAY_LAPL_FILTERED_PROFILE, "LAPL_FILTERED_PROFILE", },
      { c_fft_profile_test1_routine::DISPLAY_LAPL_FILTERED_IMAGE, "LAPL_FILTERED_IMAGE", },

      { c_fft_profile_test1_routine::DISPLAY_GLAP_MODULE, "GLAP_MODULE", },
      { c_fft_profile_test1_routine::DISPLAY_GLAP_PROFILE, "GLAP_PROFILE", },
      { c_fft_profile_test1_routine::DISPLAY_GLAP_FILTERED_MODULE, "GLAP_FILTERED_MODULE", },
      { c_fft_profile_test1_routine::DISPLAY_GLAP_FILTERED_PROFILE, "GLAP_FILTERED_PROFILE", },
      { c_fft_profile_test1_routine::DISPLAY_GLAP_FILTERED_IMAGE, "GLAP_FILTERED_IMAGE", },

      { c_fft_profile_test1_routine::DISPLAY_FILTER, "FILTER", },
      { c_fft_profile_test1_routine::DISPLAY_RESTORED_MODULE, "RESTORED_MODULE", },
      { c_fft_profile_test1_routine::DISPLAY_RESTORED_IMAGE, "RESTORED_IMAGE", },

      { c_fft_profile_test1_routine::DISPLAY_SRC_IMAGE, },
  };

  return members;
}

// Magnitude: sqrt(Re^2 + Im^2)
static bool fftMagnituteDisplay(cv::InputArray _spec, cv::OutputArray _dst, bool swapQuadrants = false)
{
  if( _spec.type() == CV_32FC1 ) {
    _spec.getMat().copyTo(_dst);
    if( swapQuadrants ) {
      fftSwapQuadrants(_dst.getMatRef());
    }
    return true;
  }

  if ( _spec.type() == CV_32FC2 ) {
    cv::Mat1f magnitude;
    fftSpectrumModule(_spec, _dst);
    if ( swapQuadrants )  {
      fftSwapQuadrants(_dst.getMatRef());
    }
    return true;
  }

  CF_ERROR("Invalid argument: Single or Two channel CV_32F complex image is expected on input");
  return false;
}

// Power: Re^2 + Im^2
static bool fftPowerDisplay(cv::InputArray _spec, cv::OutputArray _dst, bool swapQuadrants = false)
{
  if( _spec.type() == CV_32FC1 ) {
    _spec.getMat().copyTo(_dst);
    if( swapQuadrants ) {
      fftSwapQuadrants(_dst.getMatRef());
    }
    return true;
  }

  if ( _spec.type() == CV_32FC2 ) {
    cv::Mat1f pow;
    fftSpectrumPower(_spec, _dst);
    if ( swapQuadrants )  {
      fftSwapQuadrants(_dst.getMatRef());
    }
    return true;
  }

  CF_ERROR("Invalid argument: Single or Two channel CV_32F complex image is expected on input");
  return false;
}

static bool fftMulSpectrum(const cv::Mat1f & filter, cv::Mat2f & complexSpectrum,
    cv::OutputArray dst)
{
  cv::Mat2f F;
  const cv::Mat planes[] {
      filter, filter
  };
  cv::merge(planes, 2, F);
  cv::multiply(F, complexSpectrum, dst);

  return true;
}

static const double blur_model(double x)
{
  return x * x * x;
}

static bool analyzeRadialProfile(const cv::Mat1f & SRC_profile,
    double & output_profile_x0,
    double & output_profile_y0,
    double & output_S_blur,
    double & output_max_blur_value,
    bool writeFile = false)
{
  c_stdio_file fp;

  const int n_bins = SRC_profile.cols;
  const float * src = SRC_profile[0];

  const double x0 = std::log(0.5 / n_bins);
  const double y0 = std::log(src[0]);
  const double L0 = 2 * std::log(CV_PI / n_bins);

  static const auto w_nature = [](int i) -> double {
    return 1;//./(i + 1);
  };

  static const auto w_blur = [](int i) -> double {
    return 1;
  };

  const auto xpos = [x0, n_bins](int i) -> double {
    return std::log(0.5 * (i + 1) / n_bins) - x0;
  };

  //const int T_nature_pixels = 64;
  int imin_nature = 1;
  int imax_nature = 1; // 2 * n_bins / T_nature_pixels;
  int imax_blur = 1;
  double lmin = DBL_MAX;
  double smin = DBL_MAX;

  for( int i = 1; i < n_bins; ++i ) {
    const double x = xpos(i);
    const double y = std::log(src[i]) - y0;
    const double lap = L0 + 2 * std::log(i) + y;
    if ( y >= -1) {
      imin_nature = i;
    }
    if ( y >= -4.5 ) {
      imax_nature = i;
    }
    if( lap < y + 0.25 && lap < lmin ) {
      smin = y;
      lmin = lap;
      imax_blur = i;
    }
  }

  double S0_nature = 0;
  double S1_nature = 0;
  double S_blur = 0;
  double max_blur_value = 0;


  /*
   * y(x) = S0_nature + S1_nature * x
   */
  c_weighted_line_estimate<double> reg_nature;
  reg_nature.update(xpos(0), std::log(src[0]) - y0);
  for( int i = imin_nature; i < imax_nature; ++i ) {
    if( src[i] > 0 ) {
      const double x = xpos(i);
      const double y = std::log(src[i]) - y0;
      reg_nature.update(x, y, w_nature(i));
    }
  }
  reg_nature.compute(S0_nature, S1_nature);

  /*
   * y(x) = y_nature + S_blur * blur_model(x)
   * y(x) - y_nature = S_blur * blur_model(x)
   */

  c_weighted_slope_estimate<double> reg_blur;
  for( int i = 0; i < imax_blur; ++i ) {
    if( src[i] > 0 ) {
      const double x = xpos(i);
      const double y = std::log(src[i]) - y0;
      const double y_nature = S0_nature + S1_nature * x;
      reg_blur.update(blur_model(x), y - y_nature, w_blur(i));
    }
  }
  reg_blur.compute(S_blur);
  max_blur_value = blur_model(std::log(0.5 * (imax_blur + 1) / n_bins) - x0);

  output_profile_x0 = x0;
  output_profile_y0 = y0;
  output_S_blur = S_blur;
  output_max_blur_value = max_blur_value;

  if ( writeFile ) {
    if ( !fp.open("/home/projects/temp/analyze_profile.txt", "w") ) {
      CF_ERROR("Can not create '%s': %s", fp.cfilename(), strerror(errno));
      return false;
    }
    fprintf(fp, "I\tX\tSRC\tLAP\tY_NATURE\tY_TOTAL\tBLUR_CORRECTION\tSRC_CORRECTED\tW1\tW2\n");
  }



  double MAD = 0;
  int NMAD = 0;
  for( int i = 0; i < n_bins; ++i ) {
    if( src[i] > 0 ) {
      const double x = xpos(i);
      const double y = std::log(src[i]) - y0;
      const double lap = L0 +  2 * std::log(i > 0 ? i : 1) + y;
      const double w1 = w_nature(i);
      const double w2 = w_blur(i);

      // Predict: y = S0_nature + S1_nature * x + S_blur * blur_model(x)
      const double y_nature = S0_nature + S1_nature * x;
      const double y_blur = S_blur * std::min(blur_model(x), max_blur_value);
      const double y_total = y_nature + y_blur;
      const double y_corrected = y - y_blur;
      const double dyn = y - y_nature;
      const double dyt = y - y_total;
      if ( i < imax_blur ) {
        MAD += std::abs(dyt);
        NMAD += 1;
      }

      if( fp.is_open() ) {
        fprintf(fp, "%4d\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\n",
            i, x, y, lap, y_nature, y_total, y_blur, y_corrected, w1, w2);
      }
    }
  }


  CF_DEBUG("Saved file: '%s'\n"
      "imin_nature = %d (x=%g) imax_nature = %d (x=%g) imax_blur = %d (%g)\n"
      "lmin = %g smin = %g S0_nature=%g S1_nature=%g S_blur=%g max_blur_value=%g",
      fp.cfilename(),
      imin_nature, xpos(imin_nature), imax_nature, xpos(imax_nature), imax_blur, xpos(imax_blur),
      lmin, smin,
      S0_nature, S1_nature, S_blur, max_blur_value);

  return true;
}

static cv::Mat1f fftCreateRadialCorrectionFilter(const cv::Size & fftSize,
    double x0,
    double S_blur,
    double max_blur_value,
    double k_target,
    double maxGain,
    int butterworthOrder,
    double cutoffRatio)
{
  // Isotropic correction
  // The frequency step is tied to the physical dimensions of the matrix
  // fx = dx / width, fy = dy / height

  const cv::Size size = fftSize;
  const double cx = size.width / 2.0;
  const double cy = size.height / 2.0;
  const double R = std::min(cx, cy);

  // Deformation (stretching) coefficients: convert spectral pixels into dimensionless radial units
  const double scaleX = R / cx;
  const double scaleY = R / cy;

  const double correction = (k_target - S_blur);

  CF_DEBUG("S_blur=%g max_blur_value = %g k_target=%g correction=%g",
      S_blur, max_blur_value, k_target, correction);

  const auto slim = [xmax = max_blur_value, A = 1/(max_blur_value * std::sqrt(std::sqrt(5)))](double x) -> double {
    return x >= xmax ? xmax : 1.25 * x * (1 - std::pow(x * A, 4));
  };

  cv::Mat1f FILTER(size);

  cv::parallel_for_(cv::Range(0, size.height),
      [=, &FILTER](const cv::Range & range) {
        for (int y = range.start; y < range.end; ++y) {

          float * __restrict dstp = FILTER[y];

          const double dy = (y - cy) * scaleY;
          const double dy2 = dy * dy;

          for (int x = 0; x < size.width; ++x) {
            const double dx = (x - cx) * scaleX;
            const double dx2 = dx * dx;

            const double r = sqrt(dx2 + dy2);
            const double xx = std::log(0.5 * (r + 1) / R) - x0;
            const double gain = std::exp(correction * slim(blur_model(xx)));
            dstp[x] = float(gain);
          }
        }
      });

  return FILTER;
}

void c_fft_profile_test1_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Display: ", CTL_CONTEXT(ctx, _display), "Select image to display");
  ctlbind(ctls, "gsigma: ", CTL_CONTEXT(ctx, _gsigma), "Gaussian blur sigma");
  ctlbind(ctls, "lapSQRT: ", CTL_CONTEXT(ctx, _lapSQRT), "");

  ctlbind(ctls, "gamma: ", CTL_CONTEXT(ctx, _gamma), "Noise scale");
  ctlbind(ctls, "k_target: ", CTL_CONTEXT(ctx, _k_target), "");
  ctlbind(ctls, "maxGain: ", CTL_CONTEXT(ctx, _maxGain), "");

  ctlbind(ctls, "bw_cutoff: ", CTL_CONTEXT(ctx, _bw_cutoff), "");
  ctlbind(ctls, "bw_order: ", CTL_CONTEXT(ctx, _bw_order), "");
  ctlbind(ctls, "write_file ", CTL_CONTEXT(ctx, _write_file), "");
}

bool c_fft_profile_test1_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _gsigma);
    SERIALIZE_OPTION(settings, save, *this, _lapSQRT);
    SERIALIZE_OPTION(settings, save, *this, _gamma);
    SERIALIZE_OPTION(settings, save, *this, _k_target);
    SERIALIZE_OPTION(settings, save, *this, _maxGain);
    SERIALIZE_OPTION(settings, save, *this, _bw_cutoff);
    SERIALIZE_OPTION(settings, save, *this, _bw_order);
    return true;
  }
  return false;
}

bool c_fft_profile_test1_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask )
{
  // Single-channel CV_32F image is expected on input
  if ( image.type() != CV_32FC1 ) {
    CF_ERROR("Invalid argument: Single-channel CV_32F image is expected on input");
    return false;
  }

  cv::Rect rc;

  const cv::Mat src = image.getMat();
  const cv::Size srcSize = image.size();
  const cv::Size fftSize = fftGetOptimalSize(srcSize, cv::Size(15,15));

  cv::Mat SRC;
  cv::Mat2f SRC_SPECTRUM; // Complex spectrum of source image
  cv::Mat1f SRC_MODULE; // Spectrum module of source image
  cv::Mat1f SRC_PROFILE;

  cv::Mat1f GAUSS_MODULE;
  cv::Mat1f GAUSS_PROFILE;
  cv::Mat1f GAUSS_FILTERED_MODULE;
  cv::Mat1f GAUSS_FILTERED_PROFILE;

  cv::Mat1f LAPL_MODULE;
  cv::Mat1f LAPL_PROFILE;
  cv::Mat1f LAPL_FILTERED_MODULE;
  cv::Mat1f LAPL_FILTERED_PROFILE;

  cv::Mat1f GLAP_MODULE;
  cv::Mat1f GLAP_PROFILE;
  cv::Mat1f GLAP_FILTERED_MODULE;
  cv::Mat1f GLAP_FILTERED_PROFILE;

  cv::Mat1f SRC_profile;
  cv::Mat1f GAUSS_profile;
  cv::Mat1f LAPL_profile;
  cv::Mat1f GLAP_profile;
  cv::Mat1f GAUSS_FILTERED_profile;
  cv::Mat1f LAPL_FILTERED_profile;
  cv::Mat1f GLAP_FILTERED_profile;

  //double noise_level = 0;
  double profile_x0 = 0;
  double profile_y0 = 0;
  double S0_Nature = 0;
  double S1_Nature = 0;
  double S2_Nature = 0;
  double S_blur = 0;
  double imax_blur = 0;

  if ( src.type() == CV_32FC1 ) {
    if ( src.size() == fftSize ) {
      SRC = src;
      rc = cv::Rect(0, 0, src.cols, src.rows );
    }
    else {
      fftCopyMakeBorder(src, SRC, fftSize, &rc);
    }
  }
  else if ( src.channels() == 1 ) {
    if ( src.size() == fftSize ) {
      SRC = src;
      rc = cv::Rect(0, 0, src.cols, src.rows );
    }
    else {
      fftCopyMakeBorder(src, SRC, fftSize, &rc);
    }
    if ( SRC.depth() != CV_32F ) {
      SRC.convertTo(SRC, CV_32F);
    }
  }
  else if ( src.channels() == 1 ) {
    cv::cvtColor(src, SRC, cv::COLOR_BGR2GRAY);
    if ( SRC.size() == fftSize ) {
      rc = cv::Rect(0, 0, SRC.cols, SRC.rows );
    }
    else {
      fftCopyMakeBorder(SRC, SRC, fftSize, &rc);
    }
    if ( SRC.depth() != CV_32F ) {
      SRC.convertTo(SRC, CV_32F);
    }
  }


  fftImageToSpectrum(SRC, SRC_SPECTRUM, fftSize);
  fftSpectrumModule(SRC_SPECTRUM, SRC_MODULE);
  fftRadialProfile(SRC_MODULE, SRC_profile, false);
  fftRadialProfileToImage(SRC_profile, SRC_MODULE.size(), false, SRC_PROFILE);

  GAUSS_MODULE = fftGenerateGaussianFilter(fftSize, _gsigma);
  fftRadialProfile(GAUSS_MODULE, GAUSS_profile, false);
  fftRadialProfileToImage(GAUSS_profile, GAUSS_MODULE.size(), false, GAUSS_PROFILE);

  cv::multiply(SRC_MODULE, GAUSS_MODULE, GAUSS_FILTERED_MODULE);
  fftRadialProfile(GAUSS_FILTERED_MODULE, GAUSS_FILTERED_profile, false);
  fftRadialProfileToImage(GAUSS_FILTERED_profile, GAUSS_FILTERED_MODULE.size(), false, GAUSS_FILTERED_PROFILE);

  LAPL_MODULE = fftGenerateLaplacianFilter(fftSize, 1., _lapSQRT);
  fftRadialProfile(LAPL_MODULE, LAPL_profile, false);
  fftRadialProfileToImage(LAPL_profile, LAPL_MODULE.size(), false, LAPL_PROFILE);

  cv::multiply(SRC_MODULE, LAPL_MODULE, LAPL_FILTERED_MODULE);
  fftRadialProfile(LAPL_FILTERED_MODULE, LAPL_FILTERED_profile, false);
  fftRadialProfileToImage(LAPL_FILTERED_profile, LAPL_FILTERED_MODULE.size(), false, LAPL_FILTERED_PROFILE);


  cv::multiply(LAPL_MODULE, GAUSS_MODULE, GLAP_MODULE);
  fftRadialProfile(GLAP_MODULE, GLAP_profile, false);
  fftRadialProfileToImage(GLAP_profile, GLAP_MODULE.size(), false, GLAP_PROFILE);

  cv::multiply(SRC_MODULE, GLAP_MODULE, GLAP_FILTERED_MODULE);
  fftRadialProfile(GLAP_FILTERED_MODULE, GLAP_FILTERED_profile, false);
  fftRadialProfileToImage(GLAP_FILTERED_profile, GLAP_FILTERED_MODULE.size(), false, GLAP_FILTERED_PROFILE);


  analyzeRadialProfile(SRC_profile,
      profile_x0,
      profile_y0,
      S_blur,
      imax_blur,
      _write_file);

  if ( _display == DISPLAY_SRC_IMAGE ) {
    return true;
  }
  if ( _display == DISPLAY_SRC_MODULE ) {
    mask.release();
    return fftMagnituteDisplay(SRC_MODULE, image);
  }
  if ( _display == DISPLAY_SRC_PROFILE ) {
    mask.release();
    return fftMagnituteDisplay(SRC_PROFILE, image);
  }

  if ( _display == DISPLAY_GAUSS_MODULE) {
    mask.release();
    return fftMagnituteDisplay(GAUSS_MODULE, image);
  }
  if( _display == DISPLAY_GAUSS_PROFILE ) {
    mask.release();
    return fftMagnituteDisplay(GAUSS_PROFILE, image);
  }
  if ( _display == DISPLAY_GAUSS_FILTERED_MODULE) {
    mask.release();
    return fftMagnituteDisplay(GAUSS_FILTERED_MODULE, image);
  }
  if ( _display == DISPLAY_GAUSS_FILTERED_PROFILE) {
    mask.release();
    return fftMagnituteDisplay(GAUSS_FILTERED_PROFILE, image);
  }
  if ( _display == DISPLAY_GAUSS_FILTERED_IMAGE) {
    cv::Mat SRC_SPECTRUM_FILTERED;
    fftMulSpectrum(GAUSS_MODULE, SRC_SPECTRUM, SRC_SPECTRUM_FILTERED);
    fftSwapQuadrants(SRC_SPECTRUM_FILTERED);
    cv::idft(SRC_SPECTRUM_FILTERED, SRC_SPECTRUM_FILTERED, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
    SRC_SPECTRUM_FILTERED(rc).copyTo(image);
    mask.release();
    return true;
  }

  if ( _display == DISPLAY_LAPL_MODULE) {
    mask.release();
    return fftMagnituteDisplay(LAPL_MODULE, image);
  }
  if( _display == DISPLAY_LAPL_PROFILE ) {
    mask.release();
    return fftMagnituteDisplay(LAPL_PROFILE, image);
  }
  if ( _display == DISPLAY_LAPL_FILTERED_MODULE) {
    mask.release();
    return fftMagnituteDisplay(LAPL_FILTERED_MODULE, image);
  }
  if ( _display == DISPLAY_LAPL_FILTERED_PROFILE) {
    mask.release();
    return fftMagnituteDisplay(LAPL_FILTERED_PROFILE, image);
  }
  if ( _display == DISPLAY_LAPL_FILTERED_IMAGE) {
    cv::Mat SRC_SPECTRUM_FILTERED;
    fftMulSpectrum(LAPL_MODULE, SRC_SPECTRUM, SRC_SPECTRUM_FILTERED);
    fftSwapQuadrants(SRC_SPECTRUM_FILTERED);
    cv::idft(SRC_SPECTRUM_FILTERED, SRC_SPECTRUM_FILTERED, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
    SRC_SPECTRUM_FILTERED(rc).copyTo(image);
    mask.release();
    return true;
  }

  if ( _display == DISPLAY_GLAP_MODULE) {
    mask.release();
    return fftMagnituteDisplay(GLAP_MODULE, image);
  }
  if( _display == DISPLAY_GLAP_PROFILE ) {
    mask.release();
    return fftMagnituteDisplay(GLAP_PROFILE, image);
  }
  if ( _display == DISPLAY_GLAP_FILTERED_MODULE) {
    mask.release();
    return fftMagnituteDisplay(GLAP_FILTERED_MODULE, image);
  }
  if ( _display == DISPLAY_GLAP_FILTERED_PROFILE) {
    mask.release();
    return fftMagnituteDisplay(GLAP_FILTERED_PROFILE, image);
  }
  if ( _display == DISPLAY_GLAP_FILTERED_IMAGE) {
    cv::Mat SRC_SPECTRUM_FILTERED;
    fftMulSpectrum(GLAP_MODULE, SRC_SPECTRUM, SRC_SPECTRUM_FILTERED);
    fftSwapQuadrants(SRC_SPECTRUM_FILTERED);
    cv::idft(SRC_SPECTRUM_FILTERED, SRC_SPECTRUM_FILTERED, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
    SRC_SPECTRUM_FILTERED(rc).copyTo(image);
    mask.release();
    return true;
  }

  const cv::Mat1f FILTER =
      fftCreateRadialCorrectionFilter(fftSize,
          profile_x0,
          S_blur,
          imax_blur,
          _k_target,
          _maxGain,
          _bw_order,
          _bw_cutoff);

  if ( _display == DISPLAY_FILTER) {
    mask.release();
    return fftMagnituteDisplay(FILTER, image);
  }

  if ( _display == DISPLAY_RESTORED_MODULE) {
    mask.release();
    return fftMagnituteDisplay(FILTER.mul(SRC_MODULE), image);
  }

  if ( _display == DISPLAY_RESTORED_IMAGE) {
    cv::Mat SRC_SPECTRUM_FILTERED;
    fftMulSpectrum(FILTER, SRC_SPECTRUM, SRC_SPECTRUM_FILTERED);
    fftSwapQuadrants(SRC_SPECTRUM_FILTERED);
    cv::idft(SRC_SPECTRUM_FILTERED, SRC_SPECTRUM_FILTERED, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
    SRC_SPECTRUM_FILTERED(rc).copyTo(image);
    mask.release();
    return true;
  }



//  approxRadialProfile(S_profile, N_profile,
//      profile_x0, profile_y0,
//      S0_Nature,
//      S1_Nature,
//      S_blur,
//      _write_file);
//
//
//
//
//  if ( _display == DISPLAY_NOISE_POWER ) { // no further computation requested
//    mask.release();
//    return fftMagnituteDisplay(NP, image);
//  }
//
//  if ( _display == DISPLAY_NOISE ) { // no further computation requested
//    cv::Mat panes[2] = {LAPL, LAPL};
//    cv::Mat LAPLC;
//    cv::merge(panes, 2, LAPLC);
//    cv::multiply(LAPLC, NC, NC);
//  fftSwapQuadrants(NC);
//    cv::idft(NC, N, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
//    N(rc).copyTo(image);
//    mask.release();
//    return true;
//  }
//
//
//
//
//  if ( _display == DISPLAY_NOISE_PROFILE ) { // no further computation requested
//    mask.release();
//    return fftMagnituteDisplay(NOISE_PROFILE, image);
//  }
//
//  if ( _display == DISPLAY_LAPL_PROFILE) { // no further computation requested
//    mask.release();
//    return fftMagnituteDisplay(LAPL_PROFILE, image);
//  }
//
//  if ( _display == DISPLAY_NORMALIZED_NOISE_PROFILE ) { // no further computation requested
//    cv::divide(NOISE_PROFILE, LAPL_PROFILE, NORMALIZED_NOISE_PROFILE);
//    NORMALIZED_NOISE_PROFILE.setTo(0, LAPL_PROFILE < 1e-10);
//    mask.release();
//    return fftMagnituteDisplay(NORMALIZED_NOISE_PROFILE, image);
//  }
//
//  if ( _display == DISPLAY_CLEAN_SP ) { // no further computation requested
//    cv::Mat1f CLEAN_SP;
//    cv::subtract(SRC_PROFILE, NOISE_PROFILE, CLEAN_SP);
//    CLEAN_SP.setTo(0, CLEAN_SP < 0);
//    mask.release();
//    return fftMagnituteDisplay(CLEAN_SP, image);
//  }


//  const cv::Mat1f FILTER =
//      fftCreateRadialCorrectionFilter(fftSize,
//          profile_x0,
//          profile_k,
//          _k_target,
//          _maxGain,
//          _bw_order,
//          _bw_cutoff);
//
//  if ( _display == DISPLAY_FILTER ) { // no further computation requested
//    mask.release();
//    return fftMagnituteDisplay(FILTER, image);
//  }

  // DISPLAY_RESTORED_IMAGE

//  cv::Mat1f F_planes[2], F_restored;
//  cv::split(SC, F_planes);
//  cv::multiply(FILTER, F_planes[0], F_planes[0]);
//  cv::multiply(FILTER, F_planes[1], F_planes[1]);
//  cv::merge(F_planes, 2, SC);
//  if ( _display == DISPLAY_RESTORED_PROFILE ) { // no further computation requested
//    mask.release();
//    return fftMagnituteDisplay(SC, image);
//  }
//
//  fftSwapQuadrants(SC);
//  cv::idft(SC, F_restored, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
//
//  F_restored(rc).copyTo(image);

  return true;
}

