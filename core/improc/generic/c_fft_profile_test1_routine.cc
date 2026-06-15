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

//      { c_fft_profile_test1_routine::DISPLAY_GAUSS_MODULE, "GAUSS_MODULE", },
//      { c_fft_profile_test1_routine::DISPLAY_GAUSS_PROFILE, "GAUSS_PROFILE", },
//      { c_fft_profile_test1_routine::DISPLAY_GAUSS_FILTERED_MODULE, "GAUSS_FILTERED_MODULE", },
//      { c_fft_profile_test1_routine::DISPLAY_GAUSS_FILTERED_PROFILE, "GAUSS_FILTERED_PROFILE", },
//      { c_fft_profile_test1_routine::DISPLAY_GAUSS_FILTERED_IMAGE, "GAUSS_FILTERED_IMAGE", },

//      { c_fft_profile_test1_routine::DISPLAY_LAPL_MODULE, "LAPL_MODULE", },
//      { c_fft_profile_test1_routine::DISPLAY_LAPL_PROFILE, "LAPL_PROFILE", },
//      { c_fft_profile_test1_routine::DISPLAY_LAPL_FILTERED_MODULE, "LAPL_FILTERED_MODULE", },
//      { c_fft_profile_test1_routine::DISPLAY_LAPL_FILTERED_PROFILE, "LAPL_FILTERED_PROFILE", },
//      { c_fft_profile_test1_routine::DISPLAY_LAPL_FILTERED_IMAGE, "LAPL_FILTERED_IMAGE", },

//      { c_fft_profile_test1_routine::DISPLAY_GLAP_MODULE, "GLAP_MODULE", },
//      { c_fft_profile_test1_routine::DISPLAY_GLAP_PROFILE, "GLAP_PROFILE", },
//      { c_fft_profile_test1_routine::DISPLAY_GLAP_FILTERED_MODULE, "GLAP_FILTERED_MODULE", },
//      { c_fft_profile_test1_routine::DISPLAY_GLAP_FILTERED_PROFILE, "GLAP_FILTERED_PROFILE", },
//      { c_fft_profile_test1_routine::DISPLAY_GLAP_FILTERED_IMAGE, "GLAP_FILTERED_IMAGE", },

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

namespace {

class c_blur_model
{
public:
  inline void setup(double S0_nature, double S1_nature,
      double S0_lap, double S1_lap, double S2_lap,
      double max_correction,
      bool applyBlurLimit)
  {
    _xlt = 0.5 * (S1_nature - S1_lap) / S2_lap;
    _ylt = S0_lap + S1_lap * _xlt + S2_lap * _xlt * _xlt;
    _S0_nature = S0_nature + _ylt - S0_nature - S1_nature * _xlt;
    _S1_nature = S1_nature;
    _S0_lap = S0_lap;
    _S1_lap = S1_lap;
    _S2_lap = S2_lap;
    _max_correction = max_correction;
  }

  inline double xlt() const
  {
    return _xlt;
  }

  inline double ylt() const
  {
    return _ylt;
  }

  inline double max_correction() const
  {
    return _max_correction;
  }

  inline double slim(double blur, double max_blur) const
  {
    if( blur >= max_blur ) {
      return max_blur;
    }
    const double A = 1.0 / (max_blur * std::sqrt(std::sqrt(5)));
    return 1.25 * blur * (1.0 - std::pow(blur * A, 4));
  }

  inline double lnature(double x) const
  {
    return _S0_nature + _S1_nature * x;
  }

  inline double llap(double x) const
  {
    return _S0_lap + _S1_lap * x + _S2_lap * x * x;
  }

  inline double compute(double x) const
  {
    if ( x <= _xlt ) {
      return 0.0;
    }
    return std::min(_applyBlurLimit ? _max_correction : 1e12,
        lnature(x) - llap(x));

//    if (x <= _lx_nature) {
//      return 0.0;
//    }
//
//    const double L_nature = lnature(x);
//    const double L_blur =  lapprox(x);
//    const double blur = L_nature - L_blur;
//    const double max_blur =  x > _xly ? _max_blur : std::max(_ynoise, L_nature) - lapprox(_xnoise);
//    return slim(blur, _applyBlurLimit ?  max_blur : 1e12 );
  }

//  inline double compute_ulim(double x) const
//  {
//    if (x <= _lx_nature) {
//      return 0.0;
//    }
//
//    const double L_nature = lnature(x);
//    const double L_blur =  lapprox(x);
//    const double blur = L_nature - L_blur;
//    const double max_blur =  x > _xly ? _max_blur : std::max(_ynoise, L_nature) - lapprox(_xnoise);
//    return std::min(blur, _applyBlurLimit ?  max_blur : 1e12 );
//  }

private:
  double _S0_nature = 0, _S1_nature = 0;
  double _S0_lap = 0, _S1_lap = 0, _S2_lap = 0;
  double _xlt = 0, _ylt = 0;
  double _max_correction = 0;
  bool _applyBlurLimit = true;
};

}

static bool analyzeRadialProfile(const cv::Mat1f & SRC_profile,
    double & output_profile_x0,
    double & output_profile_y0,
    c_blur_model & blur_model,
    bool applyBlurLimit,
    bool writeFile)
{
  c_stdio_file fp;

  const int n_bins = SRC_profile.cols;
  const float * src = SRC_profile[0];

  const double x0 = std::log(0.5 / n_bins);
  const double y0 = std::log(src[0]);
  const double L0 = 2 * std::log(CV_PI / n_bins);

//  static const auto wlap = [](int i) -> double {
//    return 1;//./(i + 1);
//  };
//  static const auto w_blur = [](int i) -> double {
//    return 1;
//  };

  static const auto w_nature = [](int i) -> double {
    return 1. / (i + 1);
  };

  const auto xv = [&](int i) -> double {
    return std::log(0.5 * (i + 1) / n_bins) - x0;
  };
  const auto yv = [&](int i) -> double {
    return std::log(src[i]) - y0;
  };
  const auto lop = [&](int i) -> double {
    return L0 + 2 * std::log(i > 0 ? i : 1);
  };

  /*
   * Search for key points
   * */
  int LYIntesectIndex = 2;
  double LMaxValue = -DBL_MAX;
  int LMaxIndex = 2;

  for( int i = 1; i < n_bins; ++i ) {
    if( src[i] > 0 ) {
      const double y = yv(i);
      const double l = lop(i) + y;
      if ( l >= y + 0.01 ) {
        LYIntesectIndex = i;
        break;
      }
      if ( l >= LMaxValue ) {
        LMaxValue = l;
        LMaxIndex = i;
      }
    }
  }


  /*
   * Approximate LAP
   * L(x) = S0_lap + S1_lap * x + S2_lap * x  * x
   * lxmax = -0.5 * S1_lap / S2_lap; (x coordinate of the extremum)
   * llmax = S0_lap - 0.25 * S1_lap * S1_lap / S2_lap; (L value at extremum)
   */

  double S0_lap = 0;
  double S1_lap = 0;
  double S2_lap = 0;
  double lxmax = 0;
  double llmax = 0;
  int S2MaxIndex = 0; // max point in regression which produces max curvature S2_lap

  c_linear_regression3 reg_lap;
  for( int i = 1; i < LYIntesectIndex; ++i ) {
    if( src[i] > 0 ) {
      const double x = xv(i);
      const double y = yv(i);
      const double l = lop(i) + y;

      reg_lap.update(1, x, x * x, l);

      if ( i > LMaxIndex && x > 3 ) {
        double S0_temp = 0, S1_temp = 0, S2_temp = 0;
        reg_lap.compute(S0_temp, S1_temp, S2_temp);
        if( S2_temp < S2_lap ) {
          S0_lap = S0_temp;
          S1_lap = S1_temp;
          S2_lap = S2_temp;
          S2MaxIndex = i;
        }
      }
    }
  }
  lxmax = -0.5 * S1_lap / S2_lap; // (x coordinate of the laplacian extremum)
  llmax = S0_lap - 0.25 * S1_lap * S1_lap / S2_lap; // (L value at laplacian extremum)

  /*
   * Approximate NATURE
   * LNATURE(x) = S0_nature + S1_nature * x
   */
  double S0_nature = 0;
  double S1_nature = 0;
  int NatureMaxPtIndex = 0;
  c_weighted_line_estimate reg_nature;
  for( int i = 1; i < LYIntesectIndex; ++i ) {
    if( src[i] > 0 ) {
      const double x = xv(i);
      if ( x > lxmax + 1 ) { // FIXME: get rid of this hard coded value later
        break;
      }
      const double y = yv(i);
      const double l = lop(i) + y;
      reg_nature.update(x, x > lxmax ? std::max(llmax, l) : l, w_nature(i));
      NatureMaxPtIndex = i;
    }
  }
  reg_nature.compute(S0_nature, S1_nature);

//  const double xlt = 0.5 * (S1_nature - S1_lap) / S2_lap;
//  const double llt = S0_lap + S1_lap * xlt + S2_lap * xlt * xlt;
//  const double dllt = llt - (S0_nature + S1_nature * xlt);
//  S0_nature += dllt;

  double max_corection = 0;
  double x_max_corection = 0;
  for( int i = LMaxIndex; i <= LYIntesectIndex; ++i ) {
    const double x = xv(i);
    const double ln = S0_nature + S1_nature * x; // llmax
    const double la = lop(i) + yv(i); //  S0_lap + S1_lap * x + S2_lap * x * x;
    const double delta = ln - la;
    if ( delta > max_corection ) {
      max_corection = delta;
      x_max_corection = x;
    }
  }

//  const int noisePointInxdex = LYIntesectIndex;
//  const double xNoise = xv(noisePointInxdex);
//  const double natureAtNoisePoint = S0_nature + S1_nature * xNoise;
//  //const double lAtNoisePoint = lop(noisePointInxdex) + yv(noisePointInxdex);
//  const double lAtNoisePoint = S0_lap + S1_lap * xNoise + S2_lap * xNoise * xNoise;
//  const double max_corection = natureAtNoisePoint - lAtNoisePoint;

  blur_model.setup(S0_nature, S1_nature,
      S0_lap, S1_lap, S2_lap,
      max_corection,
      applyBlurLimit);

//
//  blur_model.setup(S0_lap, S1_lap, S2_lap,
//      xv(noiseStartIndex), yv(noiseStartIndex),
//      xv(LYIntesectIndex),
//      applyBlurLimit);

  if ( writeFile ) {
    if ( !fp.open("/home/projects/temp/analyze_profile.txt", "w") ) {
      CF_ERROR("Can not create '%s': %s", fp.cfilename(), strerror(errno));
      return false;
    }
    fprintf(fp, "I\tX\tS\tY\tL\tLA\tLN\tDL\tLP\tYP\n");
  }


  //const int noisePointInxdex =
  //const double maxdl =

  for( int i = 0; i < n_bins; ++i ) {
    //if( src[i] > 0 )
    {
      const double x = xv(i); // log of frequency
      const double y = yv(i); // log of spectrum intensity
      const double l = lop(i) + y; // L for given y at bin index i
      //const double la = S0_lap + S1_lap * x + S2_lap * x * x;
      //const double ln = S0_nature + S1_nature * x;
      //const double dl = x <= xlt ? 0 : std::min(ln - la, max_corection);
      const double la = blur_model.llap(x);
      const double ln = blur_model.lnature(x);
      const double dl = blur_model.compute(x);
      const double lp = l + dl;
      const double yp = y + dl;

      if( fp.is_open() ) {
        fprintf(fp, "%4d\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\n",
            i, x, src[i], y, l, la, ln, dl, lp, yp);
      }
    }
  }

  output_profile_x0 = x0;
  output_profile_y0 = y0;

  CF_DEBUG("Saved file: '%s'\n"
      "x0 = %g y0 = %g\n"
      "LYIntesectIndex = %d (x = %g Y = %g)\n"
      "LMaxIndex = %d (x = %g) LMaxValue = %g\n"
      "S2MaxIndex = %d (x=%g y=%g l=%g)\n"
      "S0_lap = %g S1_lap = %g S2_lap = %g\n"
      "S0_nature = %g S1_nature = %g npts = %d NatureMaxPtIndex=%d\n"
      "lxmax = %g llmax = %g\n"
      "xlt = %g ylt = %g\n"
      "max_corection = %g at x = %g\n",
      fp.cfilename(),
      x0, y0,
      LYIntesectIndex, xv(LYIntesectIndex), yv(LYIntesectIndex),
      LMaxIndex, xv(LMaxIndex),  LMaxValue,
      S2MaxIndex, xv(S2MaxIndex), yv(S2MaxIndex), lop(S2MaxIndex) + yv(S2MaxIndex),
      S0_lap, S1_lap, S2_lap,
      S0_nature, S1_nature, reg_nature.pts(), NatureMaxPtIndex,
      lxmax, llmax,
      blur_model.xlt(), blur_model.ylt(),
      max_corection, x_max_corection);

  return true;
}


static cv::Mat1f fftCreateRadialCorrectionFilter(const cv::Size & fftSize, double x0,
    const c_blur_model & blur_model)
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

  //const double correction = (k_target - S_blur);

//  CF_DEBUG("S_blur=%g max_blur_value = %g k_target=%g correction=%g",
//      S_blur, max_blur_value, k_target, correction);

//  const auto slim = [xmax = max_blur_value, A = 1/(max_blur_value * std::sqrt(std::sqrt(5)))](double x) -> double {
//    return x >= xmax ? xmax : 1.25 * x * (1 - std::pow(x * A, 4));
//  };

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
            const double correction = blur_model.compute(xx);
            const double gain = std::exp(correction);
            dstp[x] = float(gain);
          }
        }
      });

  return FILTER;
}

// moon:  /mnt/data/scope/2023-08-04/MOON3/image_stacking1
// mars: /mnt/data/scope/2022-11-13/s7/CapObj/2022-11-13Z/s2
void c_fft_profile_test1_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Display: ", CTL_CONTEXT(ctx, _display), "Select image to display");
  ctlbind(ctls, "applyBlurLimit: ", CTL_CONTEXT(ctx, _applyBlurLimit), "");

//  ctlbind(ctls, "gsigma: ", CTL_CONTEXT(ctx, _gsigma), "Gaussian blur sigma");
//  ctlbind(ctls, "lapSQRT: ", CTL_CONTEXT(ctx, _lapSQRT), "");
//
//  ctlbind(ctls, "gamma: ", CTL_CONTEXT(ctx, _gamma), "Noise scale");
//  ctlbind(ctls, "k_target: ", CTL_CONTEXT(ctx, _k_target), "");
//  ctlbind(ctls, "maxGain: ", CTL_CONTEXT(ctx, _maxGain), "");
//
//  ctlbind(ctls, "bw_cutoff: ", CTL_CONTEXT(ctx, _bw_cutoff), "");
//  ctlbind(ctls, "bw_order: ", CTL_CONTEXT(ctx, _bw_order), "");
  ctlbind(ctls, "write_debug_file ", CTL_CONTEXT(ctx, _write_file), "");
}

bool c_fft_profile_test1_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _applyBlurLimit);
//
//    SERIALIZE_OPTION(settings, save, *this, _gsigma);
//    SERIALIZE_OPTION(settings, save, *this, _lapSQRT);
//    SERIALIZE_OPTION(settings, save, *this, _gamma);
//    SERIALIZE_OPTION(settings, save, *this, _k_target);
//    SERIALIZE_OPTION(settings, save, *this, _maxGain);
//    SERIALIZE_OPTION(settings, save, *this, _bw_cutoff);
//    SERIALIZE_OPTION(settings, save, *this, _bw_order);
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

//  cv::Mat1f GAUSS_MODULE;
//  cv::Mat1f GAUSS_PROFILE;
//  cv::Mat1f GAUSS_FILTERED_MODULE;
//  cv::Mat1f GAUSS_FILTERED_PROFILE;

//  cv::Mat1f LAPL_MODULE;
//  cv::Mat1f LAPL_PROFILE;
//  cv::Mat1f LAPL_FILTERED_MODULE;
//  cv::Mat1f LAPL_FILTERED_PROFILE;

//  cv::Mat1f GLAP_MODULE;
//  cv::Mat1f GLAP_PROFILE;
//  cv::Mat1f GLAP_FILTERED_MODULE;
//  cv::Mat1f GLAP_FILTERED_PROFILE;

  cv::Mat1f SRC_profile;
//  cv::Mat1f GAUSS_profile;
//  cv::Mat1f LAPL_profile;
//  cv::Mat1f GLAP_profile;
//  cv::Mat1f GAUSS_FILTERED_profile;
//  cv::Mat1f LAPL_FILTERED_profile;
//  cv::Mat1f GLAP_FILTERED_profile;

  //double noise_level = 0;
  double profile_x0 = 0;
  double profile_y0 = 0;
//  double S0_Nature = 0;
//  double S1_Nature = 0;
//  double S2_Nature = 0;
//  double S_blur = 0;
  double imax_blur = 0;
  c_blur_model blur_model;

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

//  GAUSS_MODULE = fftGenerateGaussianFilter(fftSize, _gsigma);
//  fftRadialProfile(GAUSS_MODULE, GAUSS_profile, false);
//  fftRadialProfileToImage(GAUSS_profile, GAUSS_MODULE.size(), false, GAUSS_PROFILE);

//  cv::multiply(SRC_MODULE, GAUSS_MODULE, GAUSS_FILTERED_MODULE);
//  fftRadialProfile(GAUSS_FILTERED_MODULE, GAUSS_FILTERED_profile, false);
//  fftRadialProfileToImage(GAUSS_FILTERED_profile, GAUSS_FILTERED_MODULE.size(), false, GAUSS_FILTERED_PROFILE);

//  LAPL_MODULE = fftGenerateLaplacianFilter(fftSize, 1., _lapSQRT);
//  fftRadialProfile(LAPL_MODULE, LAPL_profile, false);
//  fftRadialProfileToImage(LAPL_profile, LAPL_MODULE.size(), false, LAPL_PROFILE);

//  cv::multiply(SRC_MODULE, LAPL_MODULE, LAPL_FILTERED_MODULE);
//  fftRadialProfile(LAPL_FILTERED_MODULE, LAPL_FILTERED_profile, false);
//  fftRadialProfileToImage(LAPL_FILTERED_profile, LAPL_FILTERED_MODULE.size(), false, LAPL_FILTERED_PROFILE);


//  cv::multiply(LAPL_MODULE, GAUSS_MODULE, GLAP_MODULE);
//  fftRadialProfile(GLAP_MODULE, GLAP_profile, false);
//  fftRadialProfileToImage(GLAP_profile, GLAP_MODULE.size(), false, GLAP_PROFILE);

//  cv::multiply(SRC_MODULE, GLAP_MODULE, GLAP_FILTERED_MODULE);
//  fftRadialProfile(GLAP_FILTERED_MODULE, GLAP_FILTERED_profile, false);
//  fftRadialProfileToImage(GLAP_FILTERED_profile, GLAP_FILTERED_MODULE.size(), false, GLAP_FILTERED_PROFILE);


  analyzeRadialProfile(SRC_profile,
      profile_x0,
      profile_y0,
      blur_model,
      _applyBlurLimit,
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

//  if ( _display == DISPLAY_GAUSS_MODULE) {
//    mask.release();
//    return fftMagnituteDisplay(GAUSS_MODULE, image);
//  }
//  if( _display == DISPLAY_GAUSS_PROFILE ) {
//    mask.release();
//    return fftMagnituteDisplay(GAUSS_PROFILE, image);
//  }
//  if ( _display == DISPLAY_GAUSS_FILTERED_MODULE) {
//    mask.release();
//    return fftMagnituteDisplay(GAUSS_FILTERED_MODULE, image);
//  }
//  if ( _display == DISPLAY_GAUSS_FILTERED_PROFILE) {
//    mask.release();
//    return fftMagnituteDisplay(GAUSS_FILTERED_PROFILE, image);
//  }
//  if ( _display == DISPLAY_GAUSS_FILTERED_IMAGE) {
//    cv::Mat SRC_SPECTRUM_FILTERED;
//    fftMulSpectrum(GAUSS_MODULE, SRC_SPECTRUM, SRC_SPECTRUM_FILTERED);
//    fftSwapQuadrants(SRC_SPECTRUM_FILTERED);
//    cv::idft(SRC_SPECTRUM_FILTERED, SRC_SPECTRUM_FILTERED, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
//    SRC_SPECTRUM_FILTERED(rc).copyTo(image);
//    mask.release();
//    return true;
//  }

//  if ( _display == DISPLAY_LAPL_MODULE) {
//    mask.release();
//    return fftMagnituteDisplay(LAPL_MODULE, image);
//  }
//  if( _display == DISPLAY_LAPL_PROFILE ) {
//    mask.release();
//    return fftMagnituteDisplay(LAPL_PROFILE, image);
//  }
//  if ( _display == DISPLAY_LAPL_FILTERED_MODULE) {
//    mask.release();
//    return fftMagnituteDisplay(LAPL_FILTERED_MODULE, image);
//  }
//  if ( _display == DISPLAY_LAPL_FILTERED_PROFILE) {
//    mask.release();
//    return fftMagnituteDisplay(LAPL_FILTERED_PROFILE, image);
//  }
//  if ( _display == DISPLAY_LAPL_FILTERED_IMAGE) {
//    cv::Mat SRC_SPECTRUM_FILTERED;
//    fftMulSpectrum(LAPL_MODULE, SRC_SPECTRUM, SRC_SPECTRUM_FILTERED);
//    fftSwapQuadrants(SRC_SPECTRUM_FILTERED);
//    cv::idft(SRC_SPECTRUM_FILTERED, SRC_SPECTRUM_FILTERED, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
//    SRC_SPECTRUM_FILTERED(rc).copyTo(image);
//    mask.release();
//    return true;
//  }

//  if ( _display == DISPLAY_GLAP_MODULE) {
//    mask.release();
//    return fftMagnituteDisplay(GLAP_MODULE, image);
//  }
//  if( _display == DISPLAY_GLAP_PROFILE ) {
//    mask.release();
//    return fftMagnituteDisplay(GLAP_PROFILE, image);
//  }
//  if ( _display == DISPLAY_GLAP_FILTERED_MODULE) {
//    mask.release();
//    return fftMagnituteDisplay(GLAP_FILTERED_MODULE, image);
//  }
//  if ( _display == DISPLAY_GLAP_FILTERED_PROFILE) {
//    mask.release();
//    return fftMagnituteDisplay(GLAP_FILTERED_PROFILE, image);
//  }
//  if ( _display == DISPLAY_GLAP_FILTERED_IMAGE) {
//    cv::Mat SRC_SPECTRUM_FILTERED;
//    fftMulSpectrum(GLAP_MODULE, SRC_SPECTRUM, SRC_SPECTRUM_FILTERED);
//    fftSwapQuadrants(SRC_SPECTRUM_FILTERED);
//    cv::idft(SRC_SPECTRUM_FILTERED, SRC_SPECTRUM_FILTERED, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
//    SRC_SPECTRUM_FILTERED(rc).copyTo(image);
//    mask.release();
//    return true;
//  }

  const cv::Mat1f FILTER =
      fftCreateRadialCorrectionFilter(fftSize,
          profile_x0,
          blur_model);

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

  return true;
}

