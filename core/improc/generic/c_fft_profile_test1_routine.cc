/*
 * c_fft_profile_test1_routine.cc
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
      { c_fft_profile_test1_routine::DISPLAY_RESTORED_IMAGE, "RESTORED_IMAGE", },
      { c_fft_profile_test1_routine::DISPLAY_RESTORED_IMAGE, },
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

  inline const cv::Mat1f & mx() const
  {
    return _m;
  }

  // raw linear intensity values from radial spectrum profile
  inline const float * values() const
  {
    return _v;
  }

  // raw linear intensity value from radial spectrum profile at bin i
  inline double y(int i) const
  {
    return _v[i];
  }

  // zero point for x (log of frequency)
  inline double x0() const
  {
    return _x0;
  }

  // zero point for log of relative intensity
  inline double y0() const
  {
    return _y0;
  }

  // log of frequency
  inline double xv(int i) const
  {
    return std::log(0.5 * (i + 1) / _m.cols) - _x0;
  }

  // log of relative intensity
  inline double yv(int i) const
  {
    return std::log(_v[i]) - _y0;
  };

  // log of laplacian operator in log space
  inline double lop(int i) const
  {
    return _L0 + 2 * std::log(i > 0 ? i : 1);
  }

  // log of laplacian at bin index i
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

cv::Mat1f createInverseBlurCorrectionFilter(const cv::Mat1f & RadialSpectrumProfile, /*[1][n_bins] */
    const cv::Size & fftSize,
    double S1_nature_gain,
    bool write_debug_file = false)
{

  const c_radial_spectrum_profile sp(RadialSpectrumProfile);
  const cv::Mat1f LSM = smoothLaplace(sp);

  const int n_bins = sp.size();
  const float * src = sp.values();

  /**
   * Search the point of intersection  of L and Y curves,
   * as well as the index of of Maximum of Laplacian
   */
  double MaxLValue = -DBL_MAX;
  int LYIntersectionIndex = 0;
  int YKneedleIndex = 0;
  for ( int i = 1; i < n_bins; ++i ) {
    const double y = sp.yv(i);
    const double l = sp.lop(i) + y;
    if ( l > y + 0.05 ) {
      LYIntersectionIndex = i;
      break;
    }

    if ( l > MaxLValue * 1.05 ) {
      YKneedleIndex = i;
    }

    if ( l > MaxLValue ) {
      MaxLValue = l;
    }
  }

  CF_DEBUG("\nLYIntersectionIndex=%d (x=%g y=%g) yKneedleIndex=%d (x=%g y=%g)",
      LYIntersectionIndex, sp.xv(LYIntersectionIndex), sp.yv(LYIntersectionIndex),
      YKneedleIndex, sp.xv(YKneedleIndex), sp.yv(YKneedleIndex));

  /*
   * Approximate LAP to find start of the blur
   * L(x) = S0_lap + S1_lap * x + S2_lap * x  * x
   * lxmax = -0.5 * S1_lap / S2_lap; (x coordinate of the extremum)
   * llmax = S0_lap - 0.25 * S1_lap * S1_lap / S2_lap; (L value at extremum)
   */

  double S0_lap = 0, S1_lap = 0, S2_lap = 0;
  double lxmax = 0, llmax = 0;

  c_linear_regression3 reg_lap;
  for( int i = 1; i < n_bins; ++i ) {
    if( src[i] > 0 ) {

      const double x = sp.xv(i);
      const double y = sp.yv(i);
      const double l = sp.lop(i) + y;

      if ( l >= y + 0.01 ) {
        break;
      }

      reg_lap.update(1, x, x * x, l);

      if ( i > YKneedleIndex ) {
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

  // Not robust stuff temporary disabled for manual experimentation
  double S0_nature = 0, S1_nature = 0;
  int NatureMaxPtIndex = 0;
  const int startCornersBin = int((n_bins - 1) * M_SQRT1_2);

  c_line_estimate reg_nature;
  for( int i = 1; i < n_bins; ++i ) {
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

  if ( S1_nature < 0.1 ) {
    // FIXME: negative S1_nature may happen, consider better how to deal with
    CF_DEBUG("Fixing negative S1_nature=%g", S1_nature);
    S1_nature = 0.1;
  }

  S1_nature *= S1_nature_gain;

  const double xlt = 0.5 * (S1_nature - S1_lap) / S2_lap;
  const double ylt = S0_lap + S1_lap * xlt + S2_lap * xlt * xlt;
  S0_nature += ylt - S0_nature - S1_nature * xlt;

  /*
   * COMPUTE CORRECTION for x > xlt:
   *   CORRECTION  = NATURE - LSM;
   */
  cv::Mat1f correction(1, n_bins, 0.f);

  for( int i = 2; i < n_bins; ++i ) {
    const double x = sp.xv(i);
    if ( x > xlt ) {
      const double nature = S0_nature + S1_nature * x; // L_NATURE
      const double laplace = LSM(0, i); // L_SMOOTH
      const double corr = nature - laplace;
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
      S0_nature, S1_nature, 0/*, reg_nature.pts()*/, NatureMaxPtIndex,
      lxmax, llmax,
      xlt, ylt);

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
void c_fft_profile_test1_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Display: ", CTL_CONTEXT(ctx, _display), "Select image to display");
  ctlbind(ctls, "Intensity channel: ", CTL_CONTEXT(ctx, _intensity_channel), "Select intensity channel for spectrum analysis");
  ctlbind(ctls, "S1_nature: ", CTL_CONTEXT(ctx, _S1_nature), "");
  c_anscombe_transform::getcontrols(ctls, ctx(&this_class::_anscombe));
  ctlbind(ctls, "write_debug_file ", CTL_CONTEXT(ctx, _write_file), "");
}

bool c_fft_profile_test1_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _intensity_channel);
    SERIALIZE_OPTION(settings, save, *this, _S1_nature);

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

bool c_fft_profile_test1_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask )
{
  if ( _display == DISPLAY_SRC_IMAGE ) {
    return true;
  }

  cv::Rect rc;
  const cv::Mat src = image.getMat();
  const cv::Size fftSize = fftGetOptimalSize(image.size(), cv::Size(63,63));
  const int cn = image.channels();
  const double fftArea = fftSize.area();

  double wB = 0, wG = 0, wR = 0;

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
      createInverseBlurCorrectionFilter(INTENSITY_RadialProfile, fftSize,
          _S1_nature,
          _write_file);

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


#if 0
Расчет меры разреженности (Kurtosis)
через теорему ПарсеваляНам нужно посчитать Эксцесс полученного сигнала градиентов.
Классическая формула эксцесса в пространстве изображения.

// ... [Ваш код аппроксимации LAP и NATURE до расчета S1_nature] ...
reg_nature.compute(S0_nature, S1_nature);

if ( S1_nature < 0.1 ) {
  CF_DEBUG("Fixing negative S1_nature=%g", S1_nature);
  S1_nature = 0.1;
}

// --- НАЧАЛО БЛОКА АВТОМАТИЧЕСКОГО ПОИСКА ОПТИМАЛЬНОГО НАКЛОНА ---

// Лямбда для вычисления энтропии восстановленного СЧ/ВЧ профиля при заданном gain
auto compute_profile_entropy = [&](double test_gain) -> double {
    double test_S1_nature = S1_nature * test_gain;

    // Пересчитываем точку стыковки и смещение для текущего наклона
    double test_xlt = 0.5 * (test_S1_nature - S1_lap) / S2_lap;
    double test_ylt = S0_lap + S1_lap * test_xlt + S2_lap * test_xlt * test_xlt;
    double test_S0_nature = S0_nature + test_ylt - S0_nature - test_S1_nature * test_xlt;

    // Массив энергий для информативных колец (от точки излома xlt до начала углов corners)
    std::vector<double> energies;
    double total_energy = 0.0;

    for (int i = 2; i < startCornersBin; ++i) {
        const double x = sp.xv(i);
        if (x > test_xlt) {
            const double nature = test_S0_nature + test_S1_nature * x;
            // Восстановленный логарифм интенсивности: Y_RESTORED
            const double y_restored = sp.yv(i) + (nature - LSM(0, i));

            // Переходим из логарифмического масштаба в реальную энергию полосы
            double energy = std::exp(y_restored);

            energies.push_back(energy);
            total_energy += energy;
        }
    }

    if (total_energy <= 0.0 || energies.empty()) return DBL_MAX;

    // Считаем Шенноновскую энтропию нормированного распределения энергий
    double entropy = 0.0;
    for (double e : energies) {
        double p = e / total_energy;
        if (p > 1e-12) {
            entropy -= p * std::log(p);
        }
    }
    return entropy;
};

// Одномерный поиск минимума энтропии (Метод золотого сечения)
// Ищем оптимальный S1_nature_gain в разумных физических пределах [0.5, 3.0]
double a = 0.5;
double b = 3.0;
const double tau = 0.6180339887498949; // Золотое сечение

double k1 = b - tau * (b - a);
double k2 = a + tau * (b - a);

double f1 = compute_profile_entropy(k1);
double f2 = compute_profile_entropy(k2);

// Итерируемся до желаемой точности (например, 20 шагов хватит за глаза)
for (int iter = 0; iter < 20; ++iter) {
    if (f1 < f2) {
        b = k2;
        k2 = k1;
        f2 = f1;
        k1 = b - tau * (b - a);
        f1 = compute_profile_entropy(k1);
    } else {
        a = k1;
        k1 = k2;
        f1 = f2;
        k2 = a + tau * (b - a);
        f2 = compute_profile_entropy(k2);
    }
}

// Фиксируем оптимальный коэффициент усиления наклона
S1_nature_gain = 0.5 * (a + b);
CF_DEBUG("Automatically selected S1_nature_gain = %g", S1_nature_gain);

// Применяем найденный оптимальный наклон
S1_nature *= S1_nature_gain;

// Заново вычисляем финальные геометрические параметры для построения фильтра
const double xlt = 0.5 * (S1_nature - S1_lap) / S2_lap;
const double ylt = S0_lap + S1_lap * xlt + S2_lap * xlt * xlt;
S0_nature += ylt - S0_nature - S1_nature * xlt;

// --- КОНЕЦ БЛОКА АВТОМАТИЧЕСКОГО ПОИСКА ---

/*
 * COMPUTE CORRECTION for x > xlt:
 *   CORRECTION  = NATURE - LSM;
 */
// ... [Далее ваш оригинальный код без изменений] ...

#endif
