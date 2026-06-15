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

//static const double blur_model(double x)
//{
//  return x * x * x;
//}

namespace {

//class c_blur_model
//{
//public:
//  inline void setup(double S0, double S1, double S2, double xnoise, double ynoise)
//  {
//    _S0 = S0;
//    _S1 = S1;
//    _S2 = S2;
//    _lxmax = -0.5 * S1 / S2;
//    _llmax = S0 - 0.25 * S1 * S1 / S2;
//    if ( ynoise < _llmax + 0.1 ) {
//      ynoise = _llmax + 0.1;
//    }
//
//    const double inside_sqrt = xnoise * xnoise - (ynoise - _S0 - _S1 * xnoise) / _S2;
//    if (inside_sqrt >= 0.0) {
//      _lx_nature = xnoise - std::sqrt(inside_sqrt);
//    }
//    else {
//      _lx_nature = _lxmax * 0.85;
//    }
//
//    _ll_nature = lapprox(_lx_nature);
//    _lslope_nature = _S1 + 2.0 * _S2 * _lx_nature;
//    _max_blur = 1e12; // no limit
//    //_max_blur = ynoise - lapprox(xnoise);
//    A = 1.0 / (_max_blur * std::sqrt(std::sqrt(5)));
//  }
//
//  inline double lxmax() const { return _lxmax; }
//  inline double llmax() const { return _llmax; }
//  inline double lapprox(double x) const { return _S0 + _S1 * x + _S2 * x * x; }
//  inline double max_blur() const { return _max_blur; }
//
//  inline double slim(double blur) const
//  {
//    return blur >= _max_blur ? _max_blur : 1.25 * blur * (1.0 - std::pow(blur * A, 4));
//  }
//
//  inline double lnature(double x) const
//  {
//    return _ll_nature + _lslope_nature * (x - _lx_nature);
//  }
//
//  inline double compute(double x) const
//  {
//    if (x <= _lx_nature) {
//      return 0.0;
//    }
//
//    const double L_target = lnature(x);
//    const double L_blur = lapprox(x);
//    const double blur = L_target - L_blur;
//    return slim(blur);
//  }
//
//private:
//  double _lxmax = 0, _llmax = 0, _S0 = 0, _S1 = 0, _S2 = 0, _max_blur = 0, A = 0;
//  double _lx_nature = 0, _ll_nature = 0, _lslope_nature = 0;
//};

//class c_blur_model
//{
//public:
//  inline void setup(double S0, double S1, double S2, double xnoise, double ynoise)
//  {
//    _S0 = S0;
//    _S1 = S1;
//    _S2 = S2;
//    _xnoise = xnoise;
//    _ynoise = ynoise;
//    _lxmax = -0.5 * S1 / S2;
//    _llmax = S0 - 0.25 * S1 * S1 / S2;
//
//    if ( _ynoise < _llmax + 0.1 ) {
//      _ynoise = _llmax + 0.1;
//    }
//
//    const double inside_sqrt = _xnoise * _xnoise - (_ynoise - _S0 - _S1 * _xnoise) / _S2;
//    if (inside_sqrt >= 0.0) {
//      _lx_nature = _xnoise - std::sqrt(inside_sqrt);
//    }
//    else {
//      _lx_nature = _lxmax * 0.85;
//    }
//
//    _ll_nature = lapprox(_lx_nature);
//    _lslope_nature = _S1 + 2.0 * _S2 * _lx_nature;
//  }
//
//  inline double lxmax() const { return _lxmax; }
//  inline double llmax() const { return _llmax; }
//  inline double lapprox(double x) const { return _S0 + _S1 * x + _S2 * x * x; }
//
//  inline double lnature(double x) const
//  {
//    return _ll_nature + _lslope_nature * (x - _lx_nature);
//  }
//
//  inline double compute(double x) const
//  {
//    if (x <= _lx_nature) {
//      return 0.0;
//    }
//
//    const double L_target = lnature(x);
//    const double L_blur = lapprox(x);
//    const double blur = L_target - L_blur;
//
//    // ДИНАМИЧЕСКИЙ НАКЛОННЫЙ ЛИМИТ (Защита от взрыва параболы)
//    // Истинный предел блура на частоте x — это расстояние от нашей наклонной
//    // природной линии до горизонтального уровня шума _ynoise.
//    double dynamic_max_blur = L_target - _ynoise;
//
////    // Предохранитель на случай сверхнизких частот у самого старта
////    if (dynamic_max_blur <= 1e-5) dynamic_max_blur = 1e-5;
//
//    // Используем ваш полином 4-й степени (slim), но с динамическим радиусом насыщения!
//    double local_A = 1.0 / (dynamic_max_blur * std::sqrt(std::sqrt(5)));
//
//    if (blur >= dynamic_max_blur) {
//      return dynamic_max_blur;
//    }
//
//    return 1.25 * blur * (1.0 - std::pow(blur * local_A, 4));
//  }
//
//private:
//  double _lxmax = 0, _llmax = 0, _S0 = 0, _S1 = 0, _S2 = 0;
//  double _lx_nature = 0, _ll_nature = 0, _lslope_nature = 0;
//  double _xnoise = 0, _ynoise = 0; // Сохраняем координаты шума
//};
//

//class c_blur_model
//{
//public:
//  inline void setup(double S0, double S1, double S2, double xnoise, double ynoise, double xly,
//      bool applyBlurLimit)
//  {
//    _S0 = S0;
//    _S1 = S1;
//    _S2 = S2;
//    _lxmax = -0.5 * S1 / S2;
//    _llmax = S0 - 0.25 * S1 * S1 / S2;
//    if ( ynoise < _llmax + 0.2 ) {
//      ynoise = _llmax + 0.2;
//    }
//    _xnoise = xnoise;
//    _ynoise = ynoise;
//    _xly = xly;
//
//    const double inside_sqrt = xnoise * xnoise - (ynoise - _S0 - _S1 * xnoise) / _S2;
//    if (inside_sqrt >= 0.0) {
//      _lx_nature = xnoise - std::sqrt(inside_sqrt);
//    }
//    else {
//      CF_DEBUG("bad _lx_nature: inside_sqrt=%g", inside_sqrt);
//      _lx_nature = _lxmax * 0.85;
//    }
//
//    _applyBlurLimit = applyBlurLimit;
//    _ll_nature = lapprox(_lx_nature);
//    _lslope_nature = _S1 + 2.0 * _S2 * _lx_nature;
//    _max_blur = std::max(_ynoise, lnature(_xly)) - lapprox(_xnoise);
//  }
//
//  inline double lxmax() const { return _lxmax; }
//  inline double llmax() const { return _llmax; }
//  inline double lapprox(double x) const { return _S0 + _S1 * x + _S2 * x * x; }
//  inline double max_blur() const { return _max_blur; }
//
//  inline double slim(double blur, double max_blur) const
//  {
//    if( blur >= max_blur ) {
//      return max_blur;
//    }
//    const double A = 1.0 / (max_blur * std::sqrt(std::sqrt(5)));
//    return 1.25 * blur * (1.0 - std::pow(blur * A, 4));
//  }
//
//  inline double lnature(double x) const
//  {
//    return _ll_nature + _lslope_nature * (x - _lx_nature);
//  }
//
//  inline double compute(double x) const
//  {
//    if (x <= _lx_nature) {
//      return 0.0;
//    }
//
//    const double L_nature = lnature(x);
//    const double L_blur =  lapprox(x);
//    const double blur = L_nature - L_blur;
//    const double max_blur =  x > _xly ? _max_blur : std::max(_ynoise, L_nature) - lapprox(_xnoise);
//    return slim(blur, _applyBlurLimit ?  max_blur : 1e12 );
//  }
//
//private:
//  double _lxmax = 0, _llmax = 0, _S0 = 0, _S1 = 0, _S2 = 0, _max_blur = 0;//, A = 0;
//  double _lx_nature = 0, _ll_nature = 0, _lslope_nature = 0, _xly = 0;
//  double _xnoise = 0, _ynoise = 0;
//  bool _applyBlurLimit = true;
//};


//class c_blur_model
//{
//public:
//  void setup(double S0, double S1, double S2, double xnoise, double ynoise,
//      double noise_power_factor, bool applyBlurLimit)
//  {
//    _S0 = S0;
//    _S1 = S1;
//    _S2 = S2;
//    _lxmax = -0.5 * S1 / S2;
//    _llmax = S0 - 0.25 * S1 * S1 / S2;
//    if ( ynoise < _llmax + 0.2 ) {
//      ynoise = _llmax + 0.2;
//    }
//
//    _xnoise = xnoise;
//    _ynoise = ynoise;
//    _applyBlurLimit = applyBlurLimit;
//
//    const double inside_sqrt = _xnoise * _xnoise - (_ynoise - _S0 - _S1 * _xnoise) / _S2;
//    if ( inside_sqrt < 0 ) {
//      CF_DEBUG("bad _lx_nature: inside_sqrt=%g", inside_sqrt);
//    }
//
//    _lx_nature = _xnoise - std::sqrt(inside_sqrt);
//    _ll_nature = lapprox(_lx_nature);
//    _lslope_nature = _S1 + 2.0 * _S2 * _lx_nature;
//
//    const double base_max_blur = lnature(_xnoise) - lapprox(_xnoise);
//    const double noise_to_signal_ratio = std::sqrt(noise_power_factor) / base_max_blur;
//    _max_blur = base_max_blur * std::exp(-noise_to_signal_ratio);
//    A = 1.0 / (_max_blur * std::sqrt(std::sqrt(5)));
//  }
//
//
//  inline double lxmax() const { return _lxmax; }
//  inline double llmax() const { return _llmax; }
//  inline double lapprox(double x) const { return _S0 + _S1 * x + _S2 * x * x; }
//
//  inline double slim(double blur) const
//  {
//    if (blur >= _max_blur) {
//      return _max_blur;
//    }
//    return 1.25 * blur * (1.0 - std::pow(blur * A, 4));
//  }
//
//  inline double lnature(double x) const
//  {
//    return _ll_nature + _lslope_nature * (x - _lx_nature);
//  }
//
//  inline double compute(double x) const
//  {
//    if (x <= _lx_nature) {
//      return 0.0;
//    }
//
//    const double L_target = lnature(x);
//    const double L_blur = lapprox(x);
//    const double blur = L_target - L_blur;
//
//    return _applyBlurLimit ? slim(blur) : blur;
//  }
//
//private:
//  double _lxmax = 0, _llmax = 0, _S0 = 0, _S1 = 0, _S2 = 0, _max_blur = 0, A = 0;
//  double _lx_nature = 0, _ll_nature = 0, _lslope_nature = 0;
//  double _xnoise = 0, _ynoise = 0;
//  bool _applyBlurLimit = true;
//};

class c_blur_model
{
public:
  inline void setup(double S0, double S1, double S2, double xnoise, double ynoise, double xly,
      bool applyBlurLimit)
  {
    _S0 = S0;
    _S1 = S1;
    _S2 = S2;
    _lxmax = -0.5 * S1 / S2;
    _llmax = S0 - 0.25 * S1 * S1 / S2;
    if ( ynoise < _llmax + 0.01 ) {
      ynoise = _llmax + 0.01;
    }
    _xnoise = xnoise;
    _ynoise = ynoise;
    _xly = xly;

    const double inside_sqrt = xnoise * xnoise - (ynoise - _S0 - _S1 * xnoise) / _S2;
    if (inside_sqrt >= 0.0) {
      _lx_nature = xnoise - std::sqrt(inside_sqrt);
    }
    else {
      CF_DEBUG("bad _lx_nature: inside_sqrt=%g", inside_sqrt);
      _lx_nature = _lxmax * 0.85;
    }

    _applyBlurLimit = applyBlurLimit;
    _ll_nature = lapprox(_lx_nature);
    _lslope_nature = _S1 + 2.0 * _S2 * _lx_nature;
    _max_blur = std::max(_ynoise, lnature(_xly)) - lapprox(_xnoise);
  }

  inline double lxmax() const { return _lxmax; }
  inline double llmax() const { return _llmax; }
  inline double lapprox(double x) const { return _S0 + _S1 * x + _S2 * x * x; }
  inline double max_blur() const { return _max_blur; }

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
    return _ll_nature + _lslope_nature * (x - _lx_nature);
  }

  inline double compute(double x) const
  {
    if (x <= _lx_nature) {
      return 0.0;
    }

    const double L_nature = lnature(x);
    const double L_blur =  lapprox(x);
    const double blur = L_nature - L_blur;
    const double max_blur =  x > _xly ? _max_blur : std::max(_ynoise, L_nature) - lapprox(_xnoise);
    return slim(blur, _applyBlurLimit ?  max_blur : 1e12 );
  }

  inline double compute_ulim(double x) const
  {
    if (x <= _lx_nature) {
      return 0.0;
    }

    const double L_nature = lnature(x);
    const double L_blur =  lapprox(x);
    const double blur = L_nature - L_blur;
    const double max_blur =  x > _xly ? _max_blur : std::max(_ynoise, L_nature) - lapprox(_xnoise);
    return std::min(blur, _applyBlurLimit ?  max_blur : 1e12 );
  }

private:
  double _lxmax = 0, _llmax = 0, _S0 = 0, _S1 = 0, _S2 = 0, _max_blur = 0;//, A = 0;
  double _lx_nature = 0, _ll_nature = 0, _lslope_nature = 0, _xly = 0;
  double _xnoise = 0, _ynoise = 0;
  bool _applyBlurLimit = true;
};

}

//static int searchNoiseStartIndex(const cv::Mat1f & profile_kneedle_distances,
//    int KDXBeg, int KDmaxIndex)
//{
//  // --- SECOND KNEEDLE PASS (Finding the left knee / plateau entry) ---
//  // Draw the chord on the KD chart from the start to the plateau peak
//
//  const cv::Mat1f & KD = profile_kneedle_distances;
//  const double KDx0 = double(KDXBeg);
//  const double KDy0 = double(KD[0][KDXBeg]);
//  const double KDx1 = double(KDmaxIndex);
//  const double KDy1 = double(KD[0][KDmaxIndex]);
//
//  double secondKDmax = -DBL_MAX;
//  int subNoiseIndex = KDXBeg;
//
//  for (int i = KDXBeg; i <= KDmaxIndex; ++i) {
//    // Normalized X and Y for the second pass inside the left slope
//    const double tx = (i - KDx0) / (KDx1 - KDx0);
//    const double ty = (double(KD[0][i]) - KDy0) / (KDy1 - KDy0);
//    // maximum deviation of the actual KD graph upwards above the ascent chord
//    const double secondKD = ty - tx;
//    if (secondKD > secondKDmax) {
//      secondKDmax = secondKD;
//      subNoiseIndex = i;
//    }
//  }
//
//  return subNoiseIndex;
//}



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

  static const auto wlap = [](int i) -> double {
    return 1;//./(i + 1);
  };

  static const auto w_blur = [](int i) -> double {
    return 1;
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

  // Kneedle distances to Y[i] curve

  const int KDXbeg = 2;
  const int KDXend = n_bins - 1;
  const double KDYbeg = yv(KDXbeg);
  const double KDYend = yv(KDXend);
  double KDmaxValue = 0;
  int KDmaxIndex = 0;

  cv::Mat1f kneedle_distances(1, n_bins, 0.f);

  for( int i = KDXbeg; i < n_bins; ++i ) {
    const double y = yv(i);
    const double Kx = double(i - KDXbeg) / (KDXend - KDXbeg);
    const double Ky = (y - KDYbeg) / (KDYend - KDYbeg);
    const double KD = Ky - Kx;
    if( KD > KDmaxValue ) {
      KDmaxValue = KD;
      KDmaxIndex = i;
    }
    kneedle_distances[0][i] = float(KD);
  }

  int noiseStartIndex = KDmaxIndex;
  for( int i = KDXbeg; i <= KDmaxIndex; ++i ) {
    const double KD = kneedle_distances[0][i];
    if ( KD >= 0.98 * KDmaxValue ) {
      noiseStartIndex = i;
      break;
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
  int LYIntesectIndex = 2;

  double LMaxValue = -DBL_MAX;
  int LMaxIndex = 2;

  for( int i = 1; i < n_bins; ++i ) {
    if( src[i] > 0 ) {
      const double y = yv(i);
      const double l = lop(i) + y;
      if ( l > y + 0.01 ) {
        // don't allow to intersect Y line anyway independently on noiseStartIndex
        LYIntesectIndex = i;
        break;
      }
      if ( l >= LMaxValue ) {
        LMaxValue = l;
        LMaxIndex = i;
      }
    }
  }

  c_weighted_linear_regression3 reg_lap;
  for( int i = 1; i < std::min(KDmaxIndex, LYIntesectIndex); ++i ) {
    if( src[i] > 0 ) {

      const double x = xv(i);
      const double y = yv(i);
      const double l = lop(i) + y;

      reg_lap.update(1, x, x * x, l, wlap(i));

      if ( i > LMaxIndex && x > 3 ) {
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

  /*
   * NOISE ENERGY STATISTICS FROM THE LAPLACIAN TAIL
   */
  double total_noise_energy = 0.0;
  int noise_bins_count = 0;
  for (int i = noiseStartIndex; i < n_bins; ++i) {
    if (src[i] > 0) {
      const double x = xv(i);
      const double l = lop(i) + yv(i);
      const double la =  S0_lap + S1_lap * x + S2_lap * x * x;
      if (l > la) {
        const double delta_noise = l - la;
        total_noise_energy += delta_noise * delta_noise;
      }
      // TODO: double check this way of counting
      ++noise_bins_count;
    }
  }
  const double noise_power_factor =
      (noise_bins_count > 0) ? (total_noise_energy / noise_bins_count) : 0.0;

//  blur_model.setup(S0_lap, S1_lap, S2_lap,
//                   xv(noiseStartIndex), yv(noiseStartIndex),
//                   noise_power_factor, applyBlurLimit);

  blur_model.setup(S0_lap, S1_lap, S2_lap,
      xv(noiseStartIndex), yv(noiseStartIndex),
      xv(LYIntesectIndex),
      applyBlurLimit);

  if ( writeFile ) {
    if ( !fp.open("/home/projects/temp/analyze_profile.txt", "w") ) {
      CF_ERROR("Can not create '%s': %s", fp.cfilename(), strerror(errno));
      return false;
    }
    fprintf(fp, "I\tX\tS\tY\tL\tLA\tKD\tYP\tLP\tLL\tBC\tBU\n");
  }


  for( int i = 0; i < n_bins; ++i ) {
    //if( src[i] > 0 )
    {
      const double x = xv(i); // log of frequency
      const double y = yv(i); // log of spectrum intensity
      const double l = lop(i) + y; // L for given y at bin index i
      //const double ll = ll_nature + lslope_nature * (x - lx_nature);
      const double ll = blur_model.lnature(x);
      const double la = blur_model.lapprox(x); // Predict L for given x using model from above
      const double bc = blur_model.compute(x);
      const double bu = blur_model.compute_ulim(x);
      const double yp = y + bu;
      const double lp = l + bu;
      const double KD = kneedle_distances[0][i];

      if( fp.is_open() ) {
        fprintf(fp, "%4d\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\n",
            i, x, src[i], y, l, la, KD, yp, lp, ll, bc, bu);
      }
    }
  }

  output_profile_x0 = x0;
  output_profile_y0 = y0;

  CF_DEBUG("Saved file: '%s'\n"
      "x0 = %g y0 = %g\n"
      "KDmaxIndex = %d (x=%g y=%g KD=%g)\n"
      "noiseStartIndex = %d (x = %g Y = %g L = %g)\n"
      "LYIntesectIndex = %d (x = %g Y = %g)\n"
      "LMaxIndex = %d (x = %g) LMaxValue = %g\n"
      "S0_lap = %g S1_lap = %g S2_lap = %g\n"
      "lxmax = %g llmax = %g\n"
      "noise_power_factor=%g noise_bins_count=%d\n",
      fp.cfilename(),
      x0, y0,
      KDmaxIndex, xv(KDmaxIndex), yv(KDmaxIndex), kneedle_distances[0][KDmaxIndex],
      noiseStartIndex, xv(noiseStartIndex), yv(noiseStartIndex), lop(noiseStartIndex) + yv(noiseStartIndex),
      LYIntesectIndex, xv(LYIntesectIndex), yv(LYIntesectIndex),
      LMaxIndex, xv(LMaxIndex),  LMaxValue,
      S0_lap, S1_lap, S2_lap,
      blur_model.lxmax(), blur_model.llmax(),
      noise_power_factor, noise_bins_count);

  return true;
}

//static bool analyzeRadialProfile(const cv::Mat1f & SRC_profile,
//    double & output_profile_x0,
//    double & output_profile_y0,
//    double & output_S_blur,
//    double & output_max_blur_value,
//    bool writeFile = false)
//{
//  c_stdio_file fp;
//
//  const int n_bins = SRC_profile.cols;
//  const float * src = SRC_profile[0];
//
//  const double x0 = std::log(0.5 / n_bins);
//  const double y0 = std::log(src[0]);
//  const double L0 = 2 * std::log(CV_PI / n_bins);
//
//  static const auto w_nature = [](int i) -> double {
//    return 1;//./(i + 1);
//  };
//
//  static const auto w_blur = [](int i) -> double {
//    return 1;
//  };
//
//  const auto xpos = [x0, n_bins](int i) -> double {
//    return std::log(0.5 * (i + 1) / n_bins) - x0;
//  };
//
//  //const int T_nature_pixels = 64;
//  int imin_nature = 1;
//  int imax_nature = 1; // 2 * n_bins / T_nature_pixels;
//  int imax_blur = 1;
//  double lmin = DBL_MAX;
//  double smin = DBL_MAX;
//
//  for( int i = 1; i < n_bins; ++i ) {
//    const double x = xpos(i);
//    const double y = std::log(src[i]) - y0;
//    const double lap = L0 + 2 * std::log(i) + y;
//    // y = lap - L0 - 2 * log(i)
//
//    if ( y >= -1) {
//      imin_nature = i;
//    }
//    if ( y >= -4.5 ) {
//      imax_nature = i;
//    }
//    if( lap < y + 0.25 && lap < lmin ) {
//      smin = y;
//      lmin = lap;
//      imax_blur = i;
//    }
//  }
//
//  double S0_nature = 0;
//  double S1_nature = 0;
//  double S_blur = 0;
//  double max_blur_value = 0;
//
//
//  /*
//   * y(x) = S0_nature + S1_nature * x
//   */
//  c_weighted_line_estimate<double> reg_nature;
//  reg_nature.update(xpos(0), std::log(src[0]) - y0);
//  for( int i = imin_nature; i < imax_nature; ++i ) {
//    if( src[i] > 0 ) {
//      const double x = xpos(i);
//      const double y = std::log(src[i]) - y0;
//      reg_nature.update(x, y, w_nature(i));
//    }
//  }
//  reg_nature.compute(S0_nature, S1_nature);
//
//  /*
//   * y(x) = y_nature + S_blur * blur_model(x)
//   * y(x) - y_nature = S_blur * blur_model(x)
//   */
//
//  c_weighted_slope_estimate<double> reg_blur;
//  for( int i = 0; i < imax_blur; ++i ) {
//    if( src[i] > 0 ) {
//      const double x = xpos(i);
//      const double y = std::log(src[i]) - y0;
//      const double y_nature = S0_nature + S1_nature * x;
//      reg_blur.update(blur_model(x), y - y_nature, w_blur(i));
//    }
//  }
//  reg_blur.compute(S_blur);
//  max_blur_value = blur_model(std::log(0.5 * (imax_blur + 1) / n_bins) - x0);
//
//  output_profile_x0 = x0;
//  output_profile_y0 = y0;
//  output_S_blur = S_blur;
//  output_max_blur_value = max_blur_value;
//
//  if ( writeFile ) {
//    if ( !fp.open("/home/projects/temp/analyze_profile.txt", "w") ) {
//      CF_ERROR("Can not create '%s': %s", fp.cfilename(), strerror(errno));
//      return false;
//    }
//    fprintf(fp, "I\tX\tSRC\tLAP\tY_NATURE\tY_TOTAL\tBLUR_CORRECTION\tSRC_CORRECTED\tW1\tW2\n");
//  }
//
//
//
//  double MAD = 0;
//  int NMAD = 0;
//  for( int i = 0; i < n_bins; ++i ) {
//    if( src[i] > 0 ) {
//      const double x = xpos(i);
//      const double y = std::log(src[i]) - y0;
//      const double lap = L0 +  2 * std::log(i > 0 ? i : 1) + y;
//      const double w1 = w_nature(i);
//      const double w2 = w_blur(i);
//
//      // Predict: y = S0_nature + S1_nature * x + S_blur * blur_model(x)
//      const double y_nature = S0_nature + S1_nature * x;
//      const double y_blur = S_blur * std::min(blur_model(x), max_blur_value);
//      const double y_total = y_nature + y_blur;
//      const double y_corrected = y - y_blur;
//      const double dyn = y - y_nature;
//      const double dyt = y - y_total;
//      if ( i < imax_blur ) {
//        MAD += std::abs(dyt);
//        NMAD += 1;
//      }
//
//      if( fp.is_open() ) {
//        fprintf(fp, "%4d\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\t%9.5f\n",
//            i, x, y, lap, y_nature, y_total, y_blur, y_corrected, w1, w2);
//      }
//    }
//  }
//
//
//  CF_DEBUG("Saved file: '%s'\n"
//      "imin_nature = %d (x=%g) imax_nature = %d (x=%g) imax_blur = %d (%g)\n"
//      "lmin = %g smin = %g S0_nature=%g S1_nature=%g S_blur=%g max_blur_value=%g",
//      fp.cfilename(),
//      imin_nature, xpos(imin_nature), imax_nature, xpos(imax_nature), imax_blur, xpos(imax_blur),
//      lmin, smin,
//      S0_nature, S1_nature, S_blur, max_blur_value);
//
//  return true;
//}

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
            const double correction = blur_model.compute_ulim(xx);
            const double gain = std::exp(correction);
            dstp[x] = float(gain);
          }
        }
      });

  return FILTER;
}

// /mnt/data/scope/2022-11-13/s7/CapObj/2022-11-13Z/s2
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

