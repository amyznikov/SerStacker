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

      { c_fft_profile_test1_routine::DISPLAY_V_MATRIX, "V_MATRIX" },
      { c_fft_profile_test1_routine::DISPLAY_V_MODULE, "V_MODULE" },
      { c_fft_profile_test1_routine::DISPLAY_VLAP_FILTER, "V_FILTER" },

      { c_fft_profile_test1_routine::DISPLAY_S_MATRIX, "S_MATRIX" },
      { c_fft_profile_test1_routine::DISPLAY_S_MODULE, "S_MODULE" },


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

class c_radial_spectrum_profile
{
public:
  inline c_radial_spectrum_profile(const cv::Mat1f & mx)
  {
    init(mx);
  }

  inline void init(const cv::Mat1f & mx)
  {
    _m = mx;
    _v = _m[0];
    _x0 = std::log(0.5 / mx.cols);
    _y0 = std::log(mx[0][0]);
    _L0 = 2 * std::log(CV_PI / mx.cols);
  }

  inline int size() const
  {
    return _m.cols;
  }

  inline const float * values() const
  {
    return _v;
  }

  inline const cv::Mat1f & mx() const
  {
    return _m;
  }

  inline double x0() const
  {
    return _x0;
  }

  inline double y0() const
  {
    return _y0;
  }

  inline double xv(int i) const
  {
    return std::log(0.5 * (i + 1) / _m.cols) - _x0;
  }

  inline double yv(int i) const
  {
    return std::log(_v[i]) - _y0;
  };

  inline double lop(int i) const
  {
    return _L0 + 2 * std::log(i > 0 ? i : 1);
  }

  inline double lv(int i) const
  {
    return lop(i) + yv(i);
  }

protected:
  cv::Mat1f _m; // [1][n_bins]
  const float * _v = nullptr;
  double _x0 = 0;
  double _y0 = 0;
  double _L0 = 0;
};

static cv::Mat1f smooth_laplace(const c_radial_spectrum_profile & p)
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
  cv::GaussianBlur(U, U, cv::Size(15, 1), 0, 0, cv::BORDER_REPLICATE);

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

cv::Mat1f createRadialBlurCorrectionFilter(const cv::Mat1f & SRC_profile, const cv::Size & fftSize,
    bool write_debug_file = false)
{


  const c_radial_spectrum_profile sp(SRC_profile);
  const cv::Mat1f LSM = smooth_laplace(sp);

  const int n_bins = sp.size();
  const float * src = sp.values();

  /*
   * Approximate LAP
   * L(x) = S0_lap + S1_lap * x + S2_lap * x  * x
   * lxmax = -0.5 * S1_lap / S2_lap; (x coordinate of the extremum)
   * llmax = S0_lap - 0.25 * S1_lap * S1_lap / S2_lap; (L value at extremum)
   */

  double S0_lap = 0, S1_lap = 0, S2_lap = 0;
  double lxmax = 0, llmax = 0;

  c_linear_regression3 reg_lap;
  for( int i = 3; i < n_bins; ++i ) {
    if( src[i] > 0 ) {

      const double x = sp.xv(i);
      const double y = sp.yv(i);
      const double l = sp.lop(i) + y;

      if ( l >= y + 0.01 ) {
        break;
      }

      reg_lap.update(1, x, x * x, l);

      if ( x > 3 ) { // i > LMaxIndex &&
        double S0_temp = 0, S1_temp = 0, S2_temp = 0;
        reg_lap.compute(S0_temp, S1_temp, S2_temp);
        if( S2_temp < S2_lap ) {
          S0_lap = S0_temp;
          S1_lap = S1_temp;
          S2_lap = S2_temp;
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
  double S0_nature = 0, S1_nature = 0;
  int NatureMaxPtIndex = 0;
  c_line_estimate reg_nature;
  for( int i = 2; i < n_bins; ++i ) {
    if( src[i] > 0 ) {
      const double x = sp.xv(i);
      if ( x > lxmax + 0.5 ) { // FIXME: get rid of this hard coded value later
        break;
      }
      const double l = sp.lv(i);
      reg_nature.update(x, x > lxmax ? std::max(llmax, l) : l);
      NatureMaxPtIndex = i;
    }
  }
  reg_nature.compute(S0_nature, S1_nature);
  if ( S1_nature < 0.15 ) {
    // FIXME: negative S1_nature may happen, consider better how to deal with
    CF_DEBUG("Fixing negative S1_nature=%g", S1_nature);
    S1_nature = 0.15;
  }
  const double xlt = 0.5 * (S1_nature - S1_lap) / S2_lap;
  const double ylt = S0_lap + S1_lap * xlt + S2_lap * xlt * xlt;
  S0_nature += ylt - S0_nature - S1_nature * xlt;

  /*
   * COMPUTE CORRECTION for x > xlt:
   *   CORRECTION  = NATURE - LSM;
   */
  cv::Mat1f correction(1, n_bins, 0.f);

//  const int anti_aliasing_start_bin = static_cast<int>(0.88 * n_bins);
//  const double drop_speed = 0.5;


  for( int i = 2; i < n_bins; ++i ) {
    const double x = sp.xv(i);
    if ( x > xlt ) {
      const double y = sp.yv(i); // Y
      const double nature = S0_nature + S1_nature * x; // L_NATURE
      const double laplace = LSM(0, i); // L_SMOOTH
      const double corr = nature - laplace;
//      const double snr = (y - laplace);
//      if (i > anti_aliasing_start_bin) {
//        double delta_bin = i - anti_aliasing_start_bin;
//        // Вычитаем квадратичную параболу из ЛОГАРИФМА коррекции.
//        // В спектре это эквивалентно умножению на Гауссиану (идеальный Blur для углов)
//        corr = corr - (drop_speed * delta_bin * delta_bin);
//      }
      correction(0, i) = corr;
    }
  }


  if( write_debug_file ) {

    c_stdio_file fp;

    if( !fp.open("/home/projects/temp/analyze_profile.txt", "w") ) {
      CF_ERROR("Can not create '%s': %s", fp.cfilename(), strerror(errno));
    }
    else {
      fprintf(fp, "I\tX\tS\tY\tL\tL_SMOOTH\tL_QUAD\tL_NATURE\tCORRECTION\tL_RESTORED\tY_RESTORED\n");

      for( int i = 0; i < n_bins; ++i ) {
        const double x = sp.xv(i); // log of frequency
        const double y = sp.yv(i); // log of spectrum intensity
        const double l = sp.lop(i) + y; // laplacian for given y at bin index i
        const double ls = LSM(0, i);
        const double la = S0_lap + S1_lap * x + S2_lap * x * x;
        const double ln = S0_nature + S1_nature * x;
        const double corr = correction(0, i);
        const double lp = l + corr;
        const double yp = y + corr;

        fprintf(fp, "%4d\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\n",
            i, x, src[i], y, l, ls, la, ln, corr, lp, yp);
      }

      CF_DEBUG("Saved file '%s'", fp.cfilename());
    }
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
            const int binIndex = std::clamp((int)(continuousBinIdx), 0, n_bins - 1);
            const double corr = correction(0, binIndex);
            const double gain = std::exp(corr);
            dstp[x] = float(gain);
          }
        }
      });


  CF_DEBUG("\n"
      "x0 = %g y0 = %g\n"
      "S0_lap = %g S1_lap = %g S2_lap = %g\n"
      "S0_nature = %g S1_nature = %g npts = %d NatureMaxPtIndex=%d\n"
      "lxmax = %g llmax = %g\n"
      "xlt = %g ylt = %g\n",
      sp.x0(), sp.y0(),
      S0_lap, S1_lap, S2_lap,
      S0_nature, S1_nature, reg_nature.pts(), NatureMaxPtIndex,
      lxmax, llmax,
      xlt, ylt);

  return FILTER;
}

}


// moon:  /mnt/data/scope/2023-08-04/MOON3/image_stacking1
// mars: /mnt/data/scope/2022-11-13/s7/CapObj/2022-11-13Z/s2
void c_fft_profile_test1_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Display: ", CTL_CONTEXT(ctx, _display), "Select image to display");
  ctlbind(ctls, "cleanSpectrum: ", CTL_CONTEXT(ctx, _cleanSpectrum), "Set checked to clean spectrum from spikes");
  ctlbind(ctls, "write_debug_file ", CTL_CONTEXT(ctx, _write_file), "");
}

bool c_fft_profile_test1_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _cleanSpectrum);
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
  const cv::Size fftSize = fftGetOptimalSize(srcSize, cv::Size(63,63));

  cv::Mat SRC;
  cv::Mat2f SRC_SPECTRUM; // Complex spectrum of source image
  cv::Mat1f SRC_MODULE; // Spectrum module of source image
  cv::Mat1f SRC_RadialProfile;
  cv::Mat1f SRC_PROFILE;
  cv::Mat2f S;
  cv::Mat1f V;
  cv::Mat2f V_SPECTRUM;
  cv::Mat2f SRC_CLEAN_SPECTRUM; // Complex spectrum of source image after Periodic+Smooth Decomposition
  cv::Mat1f FILTER;

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

  if ( !_cleanSpectrum ) {
    SRC_CLEAN_SPECTRUM = SRC_SPECTRUM;
  }
  else {
    // Periodic + Smooth Decomposition

    fftCreateVMatrix(SRC, V);
    if ( _display == DISPLAY_V_MATRIX ) {
      mask.release();
      V.copyTo(image);
      return true;
    }

    fftImageToSpectrum(V, V_SPECTRUM, fftSize);
    if ( _display == DISPLAY_V_MODULE ) {
      mask.release();
      return fftMagnituteDisplay(V_SPECTRUM, image);
    }

    const cv::Mat1f VLAP = fftGenerateDiscreteLaplacianFilter(V.size(), true);
    if ( _display == DISPLAY_VLAP_FILTER ) {
      mask.release();
      VLAP.copyTo(image);
      return true;
    }

    fftMulSpectrum(VLAP, V_SPECTRUM, S);
    if ( _display == DISPLAY_S_MODULE ) {
      mask.release();
      return fftMagnituteDisplay(S, image);
    }

    if ( _display == DISPLAY_S_MATRIX) {
      cv::Mat1f S_RESTORED;
      fftSwapQuadrants(S);
      cv::idft(S, S_RESTORED, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
      S_RESTORED.copyTo(image);
      mask.release();
      return true;
    }

    cv::subtract(SRC_SPECTRUM, S, SRC_CLEAN_SPECTRUM);
  }

  fftSpectrumModule(SRC_CLEAN_SPECTRUM, SRC_MODULE);
  fftRadialProfile(SRC_MODULE, SRC_RadialProfile);
  fftRadialProfileToImage(SRC_RadialProfile, SRC_MODULE.size(), SRC_PROFILE);

  FILTER =
      createRadialBlurCorrectionFilter(SRC_RadialProfile, fftSize,
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

  if ( _display == DISPLAY_FILTER) {
    mask.release();
    return fftMagnituteDisplay(FILTER, image);
  }

  if ( _display == DISPLAY_RESTORED_MODULE) {
    mask.release();
    return fftMagnituteDisplay(FILTER.mul(SRC_MODULE), image);
  }

  if ( _display == DISPLAY_RESTORED_IMAGE) {
    cv::Mat SRC_SPECTRUM_RESTORED;

    if ( !_cleanSpectrum ) {
      fftMulSpectrum(FILTER, SRC_SPECTRUM, SRC_SPECTRUM_RESTORED);
     }
     else {
       // Filter only the clean spectrum without spikes
       // to not over-amplify the crosshairs in the original frame
       // Returning the smooth part: add the S spectrum back as is, without amplification.
       // This will preserve the overall macro-energy and remove the "ghost stripes" from the craters.
       cv::Mat P_RESTORED;
       fftMulSpectrum(FILTER, SRC_CLEAN_SPECTRUM, P_RESTORED);
       cv::add(P_RESTORED, S, SRC_SPECTRUM_RESTORED);
     }

    fftSwapQuadrants(SRC_SPECTRUM_RESTORED);
    cv::idft(SRC_SPECTRUM_RESTORED, SRC_SPECTRUM_RESTORED, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
    SRC_SPECTRUM_RESTORED(rc).copyTo(image);
    mask.release();
    return true;
  }

  return true;
}
