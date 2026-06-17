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
      double x_max_correction,
      bool applyBlurLimit)
  {
    _xlt = 0.5 * (S1_nature - S1_lap) / S2_lap;
    _ylt = S0_lap + S1_lap * _xlt + S2_lap * _xlt * _xlt;
    _S0_nature = S0_nature + _ylt - S0_nature - S1_nature * _xlt;
    _S1_nature = S1_nature;
    CF_DEBUG("in setup: xlt=%g ylt=%g S0_nature=%g _S0_nature=%g", _xlt, _ylt, S0_nature, _S0_nature);

    _S0_lap = S0_lap;
    _S1_lap = S1_lap;
    _S2_lap = S2_lap;
    _max_correction = max_correction;
    _x_max_correction = x_max_correction;
    _applyBlurLimit = applyBlurLimit;
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

    const double max_correction = x <= _x_max_correction ?  _max_correction :
        _max_correction + _S1_nature * (x - _x_max_correction);

    return std::min(_applyBlurLimit ? max_correction : 1e12,
        lnature(x) - llap(x));
  }

private:
  double _S0_nature = 0, _S1_nature = 0;
  double _S0_lap = 0, _S1_lap = 0, _S2_lap = 0;
  double _xlt = 0, _ylt = 0;
  double _max_correction = 0;
  double _x_max_correction = 0;
  bool _applyBlurLimit = true;
};


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
    //const int bin_idx = std::max(0, std::min(N_uniform - 1, int((x - x_min) * (N_uniform - 1) / x_range)));
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

}


static bool analyzeRadialProfile(const cv::Mat1f & SRC_profile,
    double & output_profile_x0,
    double & output_profile_y0,
    c_blur_model & blur_model,
    bool applyBlurLimit,
    bool writeFile)
{
  c_stdio_file fp;
  const c_radial_spectrum_profile sp(SRC_profile);

  const int n_bins = sp.size();
  const float * src = sp.values();


//  const int n_bins = SRC_profile.cols;
//  const float * src = SRC_profile[0];

//  const double x0 = std::log(0.5 / n_bins);
//  const double y0 = std::log(src[0]);
//  const double L0 = 2 * std::log(CV_PI / n_bins);

//  static const auto wlap = [](int i) -> double {
//    return 1;//./(i + 1);
//  };
//  static const auto w_blur = [](int i) -> double {
//    return 1;
//  };

  static const auto w_nature = [](int i) -> double {
    return 1. / (i + 1);
  };

//  const auto xv = [&](int i) -> double {
//    return std::log(0.5 * (i + 1) / n_bins) - x0;
//  };
//  const auto yv = [&](int i) -> double {
//    return std::log(src[i]) - y0;
//  };
//  const auto lop = [&](int i) -> double {
//    return L0 + 2 * std::log(i > 0 ? i : 1);
//  };



  /*
   * Search for key points
   * */
  int LYIntesectIndex = 2;
  double LMaxValue = -DBL_MAX;
  int LMaxIndex = 2;

  for( int i = 1; i < n_bins; ++i ) {
    if( src[i] > 0 ) {
      const double y = sp.yv(i);
      const double l = sp.lop(i) + y;
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
  for( int i = 3; i < LYIntesectIndex; ++i ) {
    if( src[i] > 0 ) {
      const double x = sp.xv(i);
      const double l = sp.lv(i);

      reg_lap.update(1, x, x * x, l);

      if ( x > 3 ) { // i > LMaxIndex &&
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
  for( int i = 2; i < LYIntesectIndex; ++i ) {
    if( src[i] > 0 ) {
      const double x = sp.xv(i);
      if ( x > lxmax + 0.5 ) { // FIXME: get rid of this hard coded value later
        break;
      }
      //const double y = yv(i);
      const double l = sp.lv(i);
      reg_nature.update(x, x > lxmax ? std::max(llmax, l) : l, w_nature(i));
      NatureMaxPtIndex = i;
    }
  }
  reg_nature.compute(S0_nature, S1_nature);
  // FIXME: negative S1_nature, may happen, used below before setup()
  if ( S1_nature < 0.2 ) {
    CF_DEBUG("Fixing negative S1_nature=%g", S1_nature);
    S1_nature = 0.2;
  }
  const double xlt = 0.5 * (S1_nature - S1_lap) / S2_lap;
  const double ylt = S0_lap + S1_lap * xlt + S2_lap * xlt * xlt;
  S0_nature += ylt - S0_nature - S1_nature * xlt;
  CF_DEBUG("initial : xlt=%g ylt=%g S0_nature=%g", xlt, ylt, S0_nature);




  double max_corection = 0;
  double x_max_corection = 0;
  for( int i = S2MaxIndex; i <= LYIntesectIndex; ++i ) {
    const double x = sp.xv(i);
    // FIXME: not shifted S0_nature used
    const double ln = S0_nature + S1_nature * x; // llmax
    const double la = sp.lv(i); //  S0_lap + S1_lap * x + S2_lap * x * x;
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

  const double LAtS2MaxIndex = sp.lv(S2MaxIndex);
  double total_noise_energy = 0;
  int n_total_noise_energy = 0;
  for( int i = S2MaxIndex; i < n_bins; ++i ) {
    const double delta = sp.lv(i) - LAtS2MaxIndex;
    total_noise_energy += delta;
    n_total_noise_energy += 1;
  }

  total_noise_energy /= n_total_noise_energy;

  blur_model.setup(S0_nature, S1_nature,
      S0_lap, S1_lap, S2_lap,
      max_corection,
      x_max_corection,
      applyBlurLimit);

  if ( writeFile ) {
    if ( !fp.open("/home/projects/temp/analyze_profile.txt", "w") ) {
      CF_ERROR("Can not create '%s': %s", fp.cfilename(), strerror(errno));
      return false;
    }
    fprintf(fp, "I\tX\tS\tY\tL\tL_QUAD\tL_NATURE\tCORRECTION\tL_RESORED\tY_RESORED\t_LSMOOTH\n");
  }

//  cv::Mat1f LSM;

//  const auto smooth_lap = [&]() {
//    LSM.create(1, n_bins);
//
//
//    // Skip DC
//    const int N_uniform = 100;
//    const double x_min = xv(1);
//    const double x_max = xv(n_bins - 1);
//    const double x_range = x_max - x_min;
//
//    std::vector<double> bin_sums(N_uniform, 0.0);
//    std::vector<int> bin_counts(N_uniform, 0);
//
//    for (int i = 1; i < n_bins; ++i) {
//      const double x = xv(i);
//      const double l = lop(i) + yv(i);
//      const int bin_idx = std::max(0, std::min(N_uniform - 1,
//          int((x - x_min) * (N_uniform - 1) / x_range)));
//      bin_sums[bin_idx] += l;
//      bin_counts[bin_idx]+= 1;
//    }
//    for (int i = 0; i < N_uniform; ++i) {
//      if ( bin_counts[i] > 1 ) {
//        bin_sums[i] /= bin_counts[i];
//      }
//    }
//
//    cv::Mat1f uniform_L(1, N_uniform);
//    float* src_uniform = uniform_L[0];
//
//    // Gap Filling
//    for (int j = 0; j < N_uniform; ++j) {
//      if (bin_counts[j] > 0) {
//        src_uniform[j] = bin_sums[j];
//      }
//      else {
//        int left_valid = -1;
//        int right_valid = -1;
//
//        for (int k = j - 1; k >= 0; --k) {
//          if (bin_counts[k] > 0) {
//            left_valid = k;
//            break;
//          }
//        }
//
//        for (int k = j + 1; k < N_uniform; ++k) {
//          if (bin_counts[k] > 0) {
//            right_valid = k;
//            break;
//          }
//        }
//
//        if (left_valid != -1 && right_valid != -1) {
//          const float y_left = bin_sums[left_valid];
//          const float y_right = bin_sums[right_valid];
//          const float t = float(j - left_valid) / (right_valid - left_valid);
//          src_uniform[j] = (1.0f - t) * y_left + t * y_right;
//        }
//        else if (left_valid != -1) {
//          src_uniform[j] = float(bin_sums[left_valid]);
//        }
//        else if (right_valid != -1) {
//          src_uniform[j] = float(bin_sums[right_valid]);
//        }
//        else {
//          src_uniform[j] = 0.0f;
//        }
//      }
//    }
//
//    cv::GaussianBlur(uniform_L, uniform_L, cv::Size(15, 1), 0, 0, cv::BORDER_REPLICATE);
//
//    float* dst_profile = LSM[0];
//    dst_profile[0] = lop(0) + yv(0); //  0.0f; // DC
//
//    const float* smoothed_uniform_ptr = uniform_L[0];
//    for (int i = 1; i < n_bins; ++i) {
//      const double orig_x = xv(i);
//      const double uniform_idx = (orig_x - x_min) * (N_uniform - 1) / x_range;
//      //const int k = int(std::floor(uniform_idx));
//      const int k = int(uniform_idx);
//
//      if (k < 0) {
//        dst_profile[i] = smoothed_uniform_ptr[0];
//      }
//      else if (k >= N_uniform - 1) {
//        dst_profile[i] = smoothed_uniform_ptr[N_uniform - 1];
//      }
//      else {
//        double t = uniform_idx - k;
//        dst_profile[i] = (1.0f - t) * smoothed_uniform_ptr[k] + t * smoothed_uniform_ptr[k + 1];
//      }
//    }
//  };

  const cv::Mat1f LSM = smooth_laplace(sp);

  for( int i = 0; i < n_bins; ++i ) {
    //if( src[i] > 0 )
    {
      const double x = sp.xv(i); // log of frequency
      const double y = sp.yv(i); // log of spectrum intensity
      const double l = sp.lop(i) + y; // L for given y at bin index i
      const double la = blur_model.llap(x);
      const double ln = blur_model.lnature(x);
      const double dl = blur_model.compute(x);
      const double lp = l + dl;
      const double yp = y + dl;

      if( fp.is_open() ) {
        fprintf(fp, "%4d\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\n",
            i, x, src[i], y, l, la, ln, dl, lp, yp, LSM[0][i]);
      }
    }
  }

  output_profile_x0 = sp.x0();
  output_profile_y0 = sp.y0();

  CF_DEBUG("Saved file: '%s'\n"
      "x0 = %g y0 = %g\n"
      "LYIntesectIndex = %d (x = %g Y = %g)\n"
      "LMaxIndex = %d (x = %g) LMaxValue = %g\n"
      "S2MaxIndex = %d (x=%g y=%g l=%g)\n"
      "S0_lap = %g S1_lap = %g S2_lap = %g\n"
      "S0_nature = %g S1_nature = %g npts = %d NatureMaxPtIndex=%d\n"
      "lxmax = %g llmax = %g\n"
      "xlt = %g ylt = %g\n"
      "max_corection = %g at x = %g\n"
      "total_noise_energy = %g n_total_noise_energy = %d\n",
      fp.cfilename(),
      sp.x0(), sp.y0(),
      LYIntesectIndex, sp.xv(LYIntesectIndex), sp.yv(LYIntesectIndex),
      LMaxIndex, sp.xv(LMaxIndex),  LMaxValue,
      S2MaxIndex, sp.xv(S2MaxIndex), sp.yv(S2MaxIndex), sp.lv(S2MaxIndex),
      S0_lap, S1_lap, S2_lap,
      S0_nature, S1_nature, reg_nature.pts(), NatureMaxPtIndex,
      lxmax, llmax,
      blur_model.xlt(), blur_model.ylt(),
      max_corection, x_max_corection,
      total_noise_energy, n_total_noise_energy);

  return true;
}


static cv::Mat1f fftCreateRadialCorrectionFilter(const cv::Size & fftSize, double x0,
    const c_blur_model & blur_model, bool cornersIncluded)
{
  // Isotropic correction
  // The frequency step is tied to the physical dimensions of the matrix
  // fx = dx / width, fy = dy / height

  const cv::Size size = fftSize;
  const double cx = size.width / 2.0;
  const double cy = size.height / 2.0;

//  const double R = std::min(cx, cy);
//  // Deformation (stretching) coefficients: convert spectral pixels into dimensionless radial units
//  const double scaleX = R / cx;
//  const double scaleY = R / cy;

  cv::Mat1f FILTER(size);

  const double R = cornersIncluded ? std::sqrt(cx * cx + cy * cy) : std::min(cx, cy);
  const double scaleX = R / cx;
  const double scaleY = R / cy;
  //const int numBins = radialProfile.cols;
  //CF_DEBUG("cornersIncluded =%d R=%g numBins=%d", cornersIncluded, R, numBins);

  // Deformation (stretching) coefficients: convert spectral pixels into dimensionless radial units
  //const double scaleX = 1; //R / cx;
  //const double scaleY = 1; // R / cy;

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
  ctlbind(ctls, "cleanSpectrum: ", CTL_CONTEXT(ctx, _cleanSpectrum), "Set checked to clean spectrum from spikes");
  ctlbind(ctls, "includeCorners: ", CTL_CONTEXT(ctx, _includeCorners), "");
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
    SERIALIZE_OPTION(settings, save, *this, _cleanSpectrum);
    SERIALIZE_OPTION(settings, save, *this, _includeCorners);
    SERIALIZE_OPTION(settings, save, *this, _applyBlurLimit);
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
  cv::Mat1f V;
  cv::Mat2f V_SPECTRUM;

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

  if ( _cleanSpectrum ) {

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

    cv::Mat2f S;
    fftMulSpectrum(VLAP, V_SPECTRUM, S);
    if ( _display == DISPLAY_S_MODULE ) {
      mask.release();
      return fftMagnituteDisplay(S, image);
    }

    cv::subtract(SRC_SPECTRUM, S, SRC_SPECTRUM);
  }

  fftSpectrumModule(SRC_SPECTRUM, SRC_MODULE);
  fftRadialProfile(SRC_MODULE, SRC_profile, _includeCorners);
  fftRadialProfileToImage(SRC_profile, SRC_MODULE.size(), _includeCorners, SRC_PROFILE);

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
          blur_model,
          _includeCorners);

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

#if 0
smooth_lap
void smoothLaplacianViaBinnedAveraging(const std::vector<double>& log_X, const std::vector<double>& L, cv::Mat1f& LAP_profile)
{
    const int N = static_cast<int>(L.size());

    const int N_uniform = 100;
    const double x_min = log_X[1]; // Пропускаем DC
    const double x_max = log_X[N - 1];
    const double x_range = x_max - x_min;

    std::vector<double> bin_sums(N_uniform, 0.0);
    std::vector<int> bin_counts(N_uniform, 0);

    // 1. Распределяем все исходные точки по корзинам
    for (int i = 1; i < N; ++i) {
        double current_x = log_X[i];
        int bin_idx = static_cast<int>((current_x - x_min) * (N_uniform - 1) / x_range);
        bin_idx = std::max(0, std::min(N_uniform - 1, bin_idx));

        bin_sums[bin_idx] += L[i];
        bin_counts[bin_idx]++;
    }

    cv::Mat1f uniform_L(1, N_uniform);
    float* src_uniform = uniform_L;

    // 2. ИНТЕРПОЛЯЦИЯ ПУСТЫХ БИНОВ (Gap Filling)
    for (int j = 0; j < N_uniform; ++j) {
        if (bin_counts[j] > 0) {
            // Если в корзине есть точки, пишем честное среднее
            src_uniform[j] = static_cast<float>(bin_sums[j] / bin_counts[j]);
        } else {
            // Если корзина пустая (типично для НЧ), ищем границы интерполяции
            int left_valid = -1;
            for (int k = j - 1; k >= 0; --k) {
                if (bin_counts[k] > 0) { left_valid = k; break; }
            }

            int right_valid = -1;
            for (int k = j + 1; k < N_uniform; ++k) {
                if (bin_counts[k] > 0) { right_valid = k; break; }
            }

            // Математически строгий расчет значения в пустой корзине
            if (left_valid != -1 && right_valid != -1) {
                // Мы внутри диапазона — делаем плавную линейную интерполяцию между узлами
                float y_left = bin_sums[left_valid] / bin_counts[left_valid];
                float y_right = bin_sums[right_valid] / bin_counts[right_valid];
                float t = static_cast<float>(j - left_valid) / (right_valid - left_valid);
                src_uniform[j] = (1.0f - t) * y_left + t * y_right;
            } else if (left_valid != -1) {
                // Краевой случай (справа пусто до самого конца Найквиста)
                src_uniform[j] = static_cast<float>(bin_sums[left_valid] / bin_counts[left_valid]);
            } else if (right_valid != -1) {
                // Краевой случай (слева пусто до самого старта)
                src_uniform[j] = static_cast<float>(bin_sums[right_valid] / bin_counts[right_valid]);
            } else {
                src_uniform[j] = 0.0f; // Дефолтная заглушка на случай полной пустоты
            }
        }
    }

    // 3. СГЛАЖИВАНИЕ И ВОЗВРАТ
    cv::Mat1f uniform_L_smoothed;
    cv::GaussianBlur(uniform_L, uniform_L_smoothed, cv::Size(7, 1), 0);

    LAP_profile.create(1, N);
    float* dst_profile = LAP_profile[0];
    dst_profile[0] = 0.0f; // DC строго 0

    const float* smoothed_uniform_ptr = uniform_L_smoothed;

    for (int i = 1; i < N; ++i) {
        double orig_x = log_X[i];
        double uniform_idx = (orig_x - x_min) * (N_uniform - 1) / x_range;
        int k = static_cast<int>(std::floor(uniform_idx));

        if (k < 0) {
            dst_profile[i] = smoothed_uniform_ptr[0];
        } else if (k >= N_uniform - 1) {
            dst_profile[i] = smoothed_uniform_ptr[N_uniform - 1];
        } else {
            double t = uniform_idx - k;
            dst_profile[i] = (1.0f - t) * smoothed_uniform_ptr[k] + t * smoothed_uniform_ptr[k + 1];
        }
    }
}


// Оптимизированный линейный Gap-Filling O(N) без вложенных циклов
std::vector<int> left_nodes(N_uniform, -1);
std::vector<int> right_nodes(N_uniform, -1);

// Проход 1: фиксируем ближайший заполненный бин слева
int last_valid = -1;
for (int j = 0; j < N_uniform; ++j) {
    if (bin_counts[j] > 0) last_valid = j;
    left_nodes[j] = last_valid;
}

// Проход 2: фиксируем ближайший заполненный бин справа
last_valid = -1;
for (int j = N_uniform - 1; j >= 0; --j) {
    if (bin_counts[j] > 0) last_valid = j;
    right_nodes[j] = last_valid;
}

// Теперь заполнение пустых ячеек происходит за один проход без поиска:
for (int j = 0; j < N_uniform; ++j) {
    if (bin_counts[j] == 0) {
        int l = left_nodes[j];
        int r = right_nodes[j];

        if (l != -1 && r != -1) {
            float y_l = static_cast<float>(bin_sums[l] / bin_counts[l]);
            float y_r = static_cast<float>(bin_sums[r] / bin_counts[r]);
            float t = static_cast<float>(j - l) / (r - l);
            src_uniform[j] = (1.0f - t) * y_l + t * y_r;
        } else if (l != -1) {
            src_uniform[j] = static_cast<float>(bin_sums[l] / bin_counts[l]);
        } else if (r != -1) {
            src_uniform[j] = static_cast<float>(bin_sums[r] / bin_counts[r]);
        }
    }
}

#endif
