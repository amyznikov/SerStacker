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
    const int total_bins = mx.cols;
    const double total_energy = cv::norm(mx(cv::Rect(1, 0, total_bins - 1, 1)), cv::NORM_L2SQR);
    _y0 = 0.5 * std::log(total_energy);
    _sp = mx;
    cv::log(mx, _lsp);
    cv::subtract(_lsp, _y0, _lsp);
  }

  inline int size() const
  {
    return _sp.cols;
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
    return _sp[0][i];
  }

  inline double yv(int i) const
  {
    return _lsp[0][i];
  }

protected:
  cv::Mat1f _sp; // [1][n_bins]
  cv::Mat1f _lsp; // log(_sp)
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
    if ( p.sv(i) > 0 ) {
      const int bin_idx = std::clamp(int((p.xv(i) - x_min) * x_range_inv), 0, N_uniform - 1);
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
      up[j] = float(bin_sums[j]);

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
  dstp[0] = float(p.sv(0) > 0 ? p.yv(0) : 0);

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

//static cv::Mat1f smoothDCT(const c_radial_spectrum_profile & p,
//  double S1_target_slope,
//  double & out_x0,
//  double & out_y0,
//  double & out_S1_slope)
//{
//  constexpr int N_uniform = 100;
//  const int n_bins = p.size();
//
//  std::vector<double> bin_sums(N_uniform, 0.0);
//  std::vector<int> bin_counts(N_uniform, 0);
//
//  // Initialize the frequency range (skip DC)
//  const double x_min = p.xv(1);
//  const double x_max = p.xv(n_bins - 1);
//  const double x_range_inv = (x_max > x_min) ? (N_uniform - 1) / (x_max - x_min) : 0.0;
//
//  // Uniform accumulation (resampling)
//  for( int i = 1; i < n_bins; ++i ) {
//    if ( p.sv(i) > 0 ) {
//      const int bin_idx = std::clamp(int((p.xv(i) - x_min) * x_range_inv), 0, N_uniform - 1);
//      bin_sums[bin_idx] += p.yv(i);
//      bin_counts[bin_idx]++;
//    }
//  }
//
//  // Average non-empty cells
//  for( int i = 0; i < N_uniform; ++i ) {
//    if( bin_counts[i] > 1 ) {
//      bin_sums[i] /= bin_counts[i];
//    }
//  }
//
//  // Gap Filling
//  cv::Mat1f U(1, N_uniform);
//  float * __restrict up = U[0];
//
//  int last_valid_idx = -1;
//  for( int j = 0; j < N_uniform; ++j ) {
//    if( bin_counts[j] > 0 ) {
//      up[j] = float(bin_sums[j]);
//
//      // If there were holes before, interpolate them from the last valid one to the current one
//      if (last_valid_idx != j - 1) {
//        const int left = std::max(0, last_valid_idx);
//        const float y_left = (last_valid_idx >= 0) ? up[left] : up[j];
//        const float y_right = up[j];
//        const float inv_step = 1.0f / (j - left);
//
//        for (int k = left + 1; k < j; ++k) {
//          const float t = (k - left) * inv_step;
//          up[k] = (1.0f - t) * y_left + t * y_right;
//        }
//      }
//      last_valid_idx = j;
//    }
//  }
//
//  // Extrapolate the right edge if the last bins remain empty
//  if (last_valid_idx >= 0 && last_valid_idx < N_uniform - 1) {
//    std::fill(up + last_valid_idx + 1, up + N_uniform, up[last_valid_idx]);
//  }
//
//  // Freezing the tail (Spectrum Matrix Angle Region)
//  const int startCornersBin = int((n_bins - 1) * M_SQRT1_2);
//  const int uniformStartCornersBin = std::clamp(int((p.xv(startCornersBin) - x_min) * x_range_inv), 0, N_uniform - 1);
//  std::fill(up + uniformStartCornersBin, up + N_uniform, up[uniformStartCornersBin]);
//
//  // Smoothing the uniform profile
//  cv::medianBlur(U, U, 5);
//  cv::GaussianBlur(U, U, cv::Size(25, 1), 0, 0, cv::BORDER_REPLICATE);
//  const float * smup = U[0];
//
//  //
//  // Compute also sliding linear regression and search point x0
//  // with local slope closest to target slope
//  //
//  // Set window width to 15% of scale
//  const double dx = (N_uniform - 1 > 0) ? (x_max - x_min) / (N_uniform - 1) : 0.0;
//  //const double target_win_width_log = (x_max - x_min) * 0.15;
//  const double sliding_win_size = 7; // (dx > 0) ? target_win_width_log / dx : 15.0;
//  c_sliding_line_estimate<double> estimator(sliding_win_size);
//
//  const double x_stable_start = x_min + (x_max - x_min) * 0.15;
//  double min_diff = DBL_MAX;
//  double best_x0 = x_min, best_y0 = 0;
//  double best_slope = 0.0;
//  bool found_pivot = false;
//
//  for( int j = 1; j < N_uniform; ++j ) {
//    const double current_x = x_min + j * dx;
//    const double current_y = smup[j];
//
//    estimator.update(current_x, current_y, 1.0);
//
//    if( current_x >= x_stable_start && estimator.pts() > 6 ) {
//
//      double current_shift = 0.0;
//      double current_slope = 0.0;
//
//      if( estimator.compute(current_shift, current_slope) ) {
//        const double diff = std::abs(current_slope - S1_target_slope);
//        if( diff < min_diff ) {
//          min_diff = diff;
//          best_x0 = current_x;
//          best_y0 = current_shift + current_slope * current_x;
//          best_slope = current_slope;
//          found_pivot = true;
//        }
//      }
//    }
//  }
//  if( found_pivot ) {
//    out_x0 = best_x0;
//    out_y0 = best_y0;
//    out_S1_slope = best_slope;
//  }
//  else {
//    // Force fallback if regression failed to initialize
//    out_x0 = x_stable_start;
//    const int fallback_idx = std::clamp(int((out_x0 - x_min) * x_range_inv), 0, N_uniform - 1);
//    out_y0 = double(smup[fallback_idx]);
//    out_S1_slope = 0.0;
//  }
//
//
//  //
//  // Back interpolation into the original bin grid
//  // DC is kept unchanged
//  cv::Mat1f output(1, n_bins);
//  float * __restrict dstp = output[0];
//  dstp[0] = float(p.sv(0) > 0 ? p.yv(0) : 0);
//
//  for( int i = 1; i < n_bins; ++i ) {
//    // Safe index clamp eliminates crashes and branches in the loop
//    const double uniform_idx = (p.xv(i) - x_min) * x_range_inv;
//    const int k = std::clamp(int(uniform_idx), 0, N_uniform - 2);
//    const double t = uniform_idx - k;
//    dstp[i] = float((1.0 - t) * smup[k] + t * smup[k + 1]);
//  }
//
//  return output;
//}
//
//static cv::Mat1f smoothDCT(const c_radial_spectrum_profile & p,
//  double S1_target_slope,
//  double & out_x0,
//  double & out_y0,
//  double & out_S1_slope)
//{
//  constexpr int N_uniform = 100;
//  const int n_bins = p.size();
//
//  std::vector<double> bin_sums(N_uniform, 0.0);
//  std::vector<int> bin_counts(N_uniform, 0);
//
//  // Initialize the frequency range (skip DC)
//  const double x_min = p.xv(1);
//  const double x_max = p.xv(n_bins - 1);
//  const double x_range_inv = (x_max > x_min) ? (N_uniform - 1) / (x_max - x_min) : 0.0;
//
//  // Uniform accumulation (resampling)
//  for( int i = 1; i < n_bins; ++i ) {
//    if ( p.sv(i) > 0 ) {
//      const int bin_idx = std::clamp(int((p.xv(i) - x_min) * x_range_inv), 0, N_uniform - 1);
//      bin_sums[bin_idx] += p.yv(i);
//      bin_counts[bin_idx]++;
//    }
//  }
//
//  // Average non-empty cells
//  for( int i = 0; i < N_uniform; ++i ) {
//    if( bin_counts[i] > 1 ) {
//      bin_sums[i] /= bin_counts[i];
//    }
//  }
//
//  // Gap Filling
//  cv::Mat1f U(1, N_uniform);
//  float * __restrict up = U[0];
//
//  int last_valid_idx = -1;
//  for( int j = 0; j < N_uniform; ++j ) {
//    if( bin_counts[j] > 0 ) {
//      up[j] = float(bin_sums[j]);
//
//      // If there were holes before, interpolate them from the last valid one to the current one
//      if (last_valid_idx != j - 1) {
//        const int left = std::max(0, last_valid_idx);
//        const float y_left = (last_valid_idx >= 0) ? up[left] : up[j];
//        const float y_right = up[j];
//        const float inv_step = 1.0f / (j - left);
//
//        for (int k = left + 1; k < j; ++k) {
//          const float t = (k - left) * inv_step;
//          up[k] = (1.0f - t) * y_left + t * y_right;
//        }
//      }
//      last_valid_idx = j;
//    }
//  }
//
//  // Extrapolate the right edge if the last bins remain empty
//  if (last_valid_idx >= 0 && last_valid_idx < N_uniform - 1) {
//    std::fill(up + last_valid_idx + 1, up + N_uniform, up[last_valid_idx]);
//  }
//
//  // Freezing the tail (Spectrum Matrix Angle Region)
//  const int startCornersBin = int((n_bins - 1) * M_SQRT1_2);
//  const int uniformStartCornersBin = std::clamp(int((p.xv(startCornersBin) - x_min) * x_range_inv), 0, N_uniform - 1);
//  std::fill(up + uniformStartCornersBin, up + N_uniform, up[uniformStartCornersBin]);
//
//  // Smoothing the uniform profile
//  cv::medianBlur(U, U, 5);
//  cv::GaussianBlur(U, U, cv::Size(25, 1), 0, 0, cv::BORDER_REPLICATE);
//
//  // =========================================================================
//  // НАЧАЛО БЛОКА: Инвариантный расчет шарнира (x0, y0) через баланс энергии НЧ
//  // =========================================================================
//  const float * smup = U[0];
//  const double dx = (N_uniform - 1 > 0) ? (x_max - x_min) / (N_uniform - 1) : 0.0;
//
//  // 1. Определение зоны замера макро-энергии (первые 15% логарифмической шкалы)
//  // Там геометрия сцены стабильна, а оптический блюр физически неспособен повлиять на сигнал.
//  const double x_macro_end = x_min + (x_max - x_min) * 0.15;
//
//  double sum_x = 0.0;
//  double sum_y = 0.0;
//  int macro_pts = 0;
//
//  for (int j = 0; j < N_uniform; ++j) {
//    const double current_x = x_min + j * dx;
//    if (current_x <= x_macro_end) {
//      sum_x += current_x;
//      sum_y += double(smup[j]);
//      macro_pts++;
//    }
//  }
//
//  // Защитный фолбэк, если в массив не попало достаточно точек
//  double avg_x = (macro_pts > 0) ? (sum_x / macro_pts) : (x_min + 0.075 * (x_max - x_min));
//  double avg_y = (macro_pts > 0) ? (sum_y / macro_pts) : double(smup[0]);
//
//  // В ручном режиме жестко берем пользовательский наклон.
//  // Если передан 0.0 (автомат) — временно подставляем эталонный физический закон 1/f (-1.0).
//  double target_slope = (S1_target_slope <= -0.1) ? S1_target_slope : -1.0;
//
//  // 2. Нахождение истинной энергетической оси: y = shift + target_slope * x
//  // Рассчитываем shift из интегрального равенства масс: avg_y = shift + target_slope * avg_x
//  double true_axis_shift = avg_y - target_slope * avg_x;
//
//  // 3. Расчет точки сшивания x0 (строго 4% от линейного частотного диапазона ДКП)
//  // Это геометрический инвариант разрешения матрицы, защищающий макроструктуру сцены.
//  //out_x0 = x_min + (x_max - x_min) * 0.04;
//  const double Fmax =  n_bins - 1;
//  const double Fcut = 0.04 * Fmax;
//  out_x0 = p.xv(int(Fcut));
//
//  // 4. Расчет высоты шарнира y0 из уравнения нашей неискаженной энергетической оси
//  out_y0 = true_axis_shift + target_slope * out_x0;
//
//  // Фактическим наклоном в точке сшивания объявляем целевой наклон
//  out_S1_slope = target_slope;
//  // =========================================================================
//  // КОНЕЦ БЛОКА
//  // =========================================================================
//
//  CF_DEBUG("\ntarget_slope=%g x_min=%g x_max=%g x_macro_end=%g\n"
//      "macro_pts=%d avg_x=%g avg_y=%g true_axis_shift=%g\n"
//      "Fmax = %g Fcut=%g out_x0=%g out_y0=%g out_S1_slope=%g\n",
//      target_slope, x_min, x_max, x_macro_end,
//      macro_pts, avg_x, avg_y, true_axis_shift,
//      Fmax, Fcut, out_x0, out_y0, out_S1_slope);
//
//  // Back interpolation into the original bin grid
//  // DC is kept unchanged
//  cv::Mat1f output(1, n_bins);
//  float * __restrict dstp = output[0];
//  dstp[0] = float(p.sv(0) > 0 ? p.yv(0) : 0);
//
//  for( int i = 1; i < n_bins; ++i ) {
//    const double uniform_idx = (p.xv(i) - x_min) * x_range_inv;
//    const int k = std::clamp(int(uniform_idx), 0, N_uniform - 2);
//    const double t = uniform_idx - k;
//    dstp[i] = float((1.0 - t) * smup[k] + t * smup[k + 1]);
//  }
//
//  return output;
//}

//static cv::Mat1f smoothDCT(const c_radial_spectrum_profile & p,
//  double S1_target_slope, cv::Mat1f & out_correction)
//{
//  constexpr int N_uniform = 100;
//  const int n_bins = p.size();
//
//  std::vector<double> bin_sums(N_uniform, 0.0);
//  std::vector<int> bin_counts(N_uniform, 0);
//
//  // Initialize the frequency range (skip DC)
//  const double x_min = p.xv(1);
//  const double x_max = p.xv(n_bins - 1);
//  const double x_range_inv = (x_max > x_min) ? (N_uniform - 1) / (x_max - x_min) : 0.0;
//
//  // Uniform accumulation (resampling)
//  for( int i = 1; i < n_bins; ++i ) {
//    if ( p.sv(i) > 0 ) {
//      const int bin_idx = std::clamp(int((p.xv(i) - x_min) * x_range_inv), 0, N_uniform - 1);
//      bin_sums[bin_idx] += p.yv(i);
//      bin_counts[bin_idx]++;
//    }
//  }
//
//  // Average non-empty cells
//  for( int i = 0; i < N_uniform; ++i ) {
//    if( bin_counts[i] > 1 ) {
//      bin_sums[i] /= bin_counts[i];
//    }
//  }
//
//  // Gap Filling
//  cv::Mat1f U(1, N_uniform);
//  float * __restrict up = U[0];
//
//  int last_valid_idx = -1;
//  for( int j = 0; j < N_uniform; ++j ) {
//    if( bin_counts[j] > 0 ) {
//      up[j] = float(bin_sums[j]);
//
//      if (last_valid_idx != j - 1) {
//        const int left = std::max(0, last_valid_idx);
//        const float y_left = (last_valid_idx >= 0) ? up[left] : up[j];
//        const float y_right = up[j];
//        const float inv_step = 1.0f / (j - left);
//
//        for (int k = left + 1; k < j; ++k) {
//          const float t = (k - left) * inv_step;
//          up[k] = (1.0f - t) * y_left + t * y_right;
//        }
//      }
//      last_valid_idx = j;
//    }
//  }
//
//  if (last_valid_idx >= 0 && last_valid_idx < N_uniform - 1) {
//    std::fill(up + last_valid_idx + 1, up + N_uniform, up[last_valid_idx]);
//  }
//
//  const int startCornersBin = int((n_bins - 1) * M_SQRT1_2);
//  const int uniformStartCornersBin = std::clamp(int((p.xv(startCornersBin) - x_min) * x_range_inv), 0, N_uniform - 1);
//  std::fill(up + uniformStartCornersBin, up + N_uniform, up[uniformStartCornersBin]);
//
//  // Smoothing the uniform profile
//  cv::medianBlur(U, U, 5);
//  cv::GaussianBlur(U, U, cv::Size(25, 1), 0, 0, cv::BORDER_REPLICATE);
//
//  // =========================================================================
//  // НАЧАЛО БЛОКА: Дифференциально-кумулятивный расчет целевой траектории и коррекции
//  // =========================================================================
//  const float * smup = U[0];
//  const double dx = (N_uniform - 1 > 0) ? (x_max - x_min) / (N_uniform - 1) : 0.0;
//
//  // Инициализируем буферы для равномерной сетки
//  std::vector<double> y_target(N_uniform, 0.0);
//  std::vector<float> uniform_correction(N_uniform, 0.0f);
//
//  // Базовая безопасная граница макроструктуры НЧ (первые 20% логарифмической шкалы)
//  const double x_stable_start = x_min + (x_max - x_min) * 0.20;
//
//  // Целевой шаг, который требует пользователь
//  const double delta_y_user = S1_target_slope * dx;
//
//  // Начальная точка
//  y_target[0] = double(smup[0]);
//  uniform_correction[0] = 0.0f;
//
//  for (int j = 1; j < N_uniform; ++j) {
//    const double current_x = x_min + j * dx;
//
//    if (current_x < x_stable_start) {
//      // В глубоких НЧ принудительно идем строго по реальному спектру кадра
//      y_target[j] = double(smup[j]);
//    }
//    else {
//      // Вычисляем фактический локальный шаг реального спектра
//      const double delta_y_real = double(smup[j]) - double(smup[j - 1]);
//
//      // КРИТИЧЕСКОЕ ПРАВИЛО: Выбираем максимальный (наиболее пологий) шаг.
//      // Если реальный спектр падает круче пользовательского лимита, замещаем его на delta_y_user.
//      // Если идет положе (как на плато), оставляем реальный шаг нетронутым.
//      const double chosen_delta = std::max(delta_y_real, delta_y_user);
//
//      // Строим траекторию кумулятивно от предыдущей точки
//      y_target[j] = y_target[j - 1] + chosen_delta;
//    }
//
//    // Логарифмическая разность между идеалом и реальностью
//    // Защита std::max(0.0) гарантирует, что мы только восстанавливаем просевшие частоты
//    const double diff = std::max(0.0, y_target[j] - double(smup[j]));
//    uniform_correction[j] = float(diff);
//  }
//  // =========================================================================
//  // КОНЕЦ БЛОКА
//  // =========================================================================
//
//  // Инициализируем выходную матрицу коррекции дефолтными единицами
//  out_correction = cv::Mat1f(1, n_bins, 1.0f);
//  float * __restrict corr_dst = out_correction[0];
//
//  // Интерполяция сглаженного профиля И матрицы коррекции обратно в исходную сетку бинов
//  cv::Mat1f output(1, n_bins);
//  float * __restrict dstp = output[0];
//  dstp[0] = float(p.sv(0) > 0 ? p.yv(0) : 0);
//  corr_dst[0] = 1.0f; // DC не трогаем
//
//  for( int i = 1; i < n_bins; ++i ) {
//    const double uniform_idx = (p.xv(i) - x_min) * x_range_inv;
//    const int k = std::clamp(int(uniform_idx), 0, N_uniform - 2);
//    const double t = uniform_idx - k;
//
//    // 1. Возвращаем сглаженный спектр для ваших графиков
//    dstp[i] = float((1.0 - t) * smup[k] + t * smup[k + 1]);
//
//    // 2. Возвращаем линейный коэффициент коррекции для фильтра (экспоненцируем прямо здесь!)
//    const double log_corr = (1.0 - t) * uniform_correction[k] + t * uniform_correction[k + 1];
//    corr_dst[i] = float(std::exp(log_corr));
//  }
//
//  return output;
//}

//static cv::Mat1f smoothDCT(const c_radial_spectrum_profile & p,
//  double S1_target_slope, cv::Mat1f & out_correction)
//{
//  constexpr int N_uniform = 100;
//  const int n_bins = p.size();
//
//  std::vector<double> bin_sums(N_uniform, 0.0);
//  std::vector<int> bin_counts(N_uniform, 0);
//
//  // Initialize the frequency range (skip DC)
//  const double x_min = p.xv(1);
//  const double x_max = p.xv(n_bins - 1);
//  const double x_range_inv = (x_max > x_min) ? (N_uniform - 1) / (x_max - x_min) : 0.0;
//
//  // Uniform accumulation (resampling)
//  for( int i = 1; i < n_bins; ++i ) {
//    if ( p.sv(i) > 0 ) {
//      const int bin_idx = std::clamp(int((p.xv(i) - x_min) * x_range_inv), 0, N_uniform - 1);
//      bin_sums[bin_idx] += p.yv(i);
//      bin_counts[bin_idx]++;
//    }
//  }
//
//  // Average non-empty cells
//  for( int i = 0; i < N_uniform; ++i ) {
//    if( bin_counts[i] > 1 ) {
//      bin_sums[i] /= bin_counts[i];
//    }
//  }
//
//  // Gap Filling
//  cv::Mat1f U(1, N_uniform);
//  float * __restrict up = U[0];
//
//  int last_valid_idx = -1;
//  for( int j = 0; j < N_uniform; ++j ) {
//    if( bin_counts[j] > 0 ) {
//      up[j] = float(bin_sums[j]);
//
//      if (last_valid_idx != j - 1) {
//        const int left = std::max(0, last_valid_idx);
//        const float y_left = (last_valid_idx >= 0) ? up[left] : up[j];
//        const float y_right = up[j];
//        const float inv_step = 1.0f / (j - left);
//
//        for (int k = left + 1; k < j; ++k) {
//          const float t = (k - left) * inv_step;
//          up[k] = (1.0f - t) * y_left + t * y_right;
//        }
//      }
//      last_valid_idx = j;
//    }
//  }
//
//  if (last_valid_idx >= 0 && last_valid_idx < N_uniform - 1) {
//    std::fill(up + last_valid_idx + 1, up + N_uniform, up[last_valid_idx]);
//  }
//
//  const int startCornersBin = int((n_bins - 1) * M_SQRT1_2);
//  const int uniformStartCornersBin = std::clamp(int((p.xv(startCornersBin) - x_min) * x_range_inv), 0, N_uniform - 1);
//  std::fill(up + uniformStartCornersBin, up + N_uniform, up[uniformStartCornersBin]);
//
//  // Smoothing the uniform profile
//  cv::medianBlur(U, U, 5);
//  cv::GaussianBlur(U, U, cv::Size(25, 1), 0, 0, cv::BORDER_REPLICATE);
//
//  // =========================================================================
//  // НАЧАЛО БЛОКА: Дифференциально-кумулятивный расчет целевой траектории и коррекции
//  // =========================================================================
//  const float * smup = U[0];
//  const double dx = (N_uniform - 1 > 0) ? (x_max - x_min) / (N_uniform - 1) : 0.0;
//
//  // Инициализируем буферы для равномерной сетки
//  std::vector<double> y_target(N_uniform, 0.0);
//  std::vector<float> uniform_correction(N_uniform, 0.0f);
//
//  // Базовая безопасная граница макроструктуры НЧ (первые 20% логарифмической шкалы)
//  const double x_stable_start = x_min + (x_max - x_min) * 0.20;
//
//  // Целевой шаг, который требует пользователь
//  const double delta_y_user = S1_target_slope * dx;
//
//  // Начальная точка
//  y_target[0] = double(smup[0]);
//  uniform_correction[0] = 0.0f;
//
//  // Координата центра перелома, где СЧ должны начать плавно переходить в ВЧ-деблур
//  // Логарифм частоты около 3.5 — идеальная граница затухания крупных поясов
//  const double x_transition_center = x_min + (x_max - x_min) * 0.45; // ~3.4 - 3.6
//  const double wsf_diff = 5.0; // Крутизна перехода дифференциала
//
//  for (int j = 1; j < N_uniform; ++j) {
//    const double current_x = x_min + j * dx;
//
//    if (current_x < x_stable_start) {
//      y_target[j] = double(smup[j]);
//    }
//    else {
//      const double delta_y_real = double(smup[j]) - double(smup[j - 1]);
//      const double delta_y_user = S1_target_slope * dx;
//
//      // Наш кумулятивный максимум
//      const double delta_y_max = std::max(delta_y_real, delta_y_user);
//
//      // Плавный вес сигмоиды для смешивания ДИФФЕРЕНЦИАЛА
//      const double w_mix = 1.0 / (1.0 + std::exp(-wsf_diff * (current_x - x_transition_center)));
//
//      // Мягко смешиваем шаги: в СЧ доминирует реальный спад кадра, в ВЧ — воля пользователя
//      const double chosen_delta = (1.0 - w_mix) * delta_y_real + w_mix * delta_y_max;
//
//      y_target[j] = y_target[j - 1] + chosen_delta;
//    }
//
//    const double diff = std::max(0.0, y_target[j] - double(smup[j]));
//    uniform_correction[j] = float(diff);
//  }
//
//  // =========================================================================
//  // КОНЕЦ БЛОКА
//  // =========================================================================
//
//  // Инициализируем выходную матрицу коррекции дефолтными единицами
//  out_correction = cv::Mat1f(1, n_bins, 1.0f);
//  float * __restrict corr_dst = out_correction[0];
//
//  // Интерполяция сглаженного профиля И матрицы коррекции обратно в исходную сетку бинов
//  cv::Mat1f output(1, n_bins);
//  float * __restrict dstp = output[0];
//  dstp[0] = float(p.sv(0) > 0 ? p.yv(0) : 0);
//  corr_dst[0] = 1.0f; // DC не трогаем
//
//  for( int i = 1; i < n_bins; ++i ) {
//    const double uniform_idx = (p.xv(i) - x_min) * x_range_inv;
//    const int k = std::clamp(int(uniform_idx), 0, N_uniform - 2);
//    const double t = uniform_idx - k;
//
//    // 1. Возвращаем сглаженный спектр для ваших графиков
//    dstp[i] = float((1.0 - t) * smup[k] + t * smup[k + 1]);
//
//    // 2. Возвращаем линейный коэффициент коррекции для фильтра (экспоненцируем прямо здесь!)
//    const double log_corr = (1.0 - t) * uniform_correction[k] + t * uniform_correction[k + 1];
//    corr_dst[i] = float(std::exp(log_corr));
//  }
//
//  return output;
//}

//static cv::Mat1f smoothDCT(const c_radial_spectrum_profile & p,
//  double S1_target_slope, cv::Mat1f & out_correction)
//{
//  constexpr int N_uniform = 100;
//  const int n_bins = p.size();
//
//  std::vector<double> bin_sums(N_uniform, 0.0);
//  std::vector<int> bin_counts(N_uniform, 0);
//
//  // Initialize the frequency range (skip DC)
//  const double x_min = p.xv(1);
//  const double x_max = p.xv(n_bins - 1);
//  const double x_range_inv = (x_max > x_min) ? (N_uniform - 1) / (x_max - x_min) : 0.0;
//
//  // Uniform accumulation (resampling)
//  for( int i = 1; i < n_bins; ++i ) {
//    if ( p.sv(i) > 0 ) {
//      const int bin_idx = std::clamp(int((p.xv(i) - x_min) * x_range_inv), 0, N_uniform - 1);
//      bin_sums[bin_idx] += p.yv(i);
//      bin_counts[bin_idx]++;
//    }
//  }
//
//  // Average non-empty cells
//  for( int i = 0; i < N_uniform; ++i ) {
//    if( bin_counts[i] > 1 ) {
//      bin_sums[i] /= bin_counts[i];
//    }
//  }
//
//  // Gap Filling
//  cv::Mat1f U(1, N_uniform);
//  float * __restrict up = U[0];
//
//  int last_valid_idx = -1;
//  for( int j = 0; j < N_uniform; ++j ) {
//    if( bin_counts[j] > 0 ) {
//      up[j] = float(bin_sums[j]);
//
//      if (last_valid_idx != j - 1) {
//        const int left = std::max(0, last_valid_idx);
//        const float y_left = (last_valid_idx >= 0) ? up[left] : up[j];
//        const float y_right = up[j];
//        const float inv_step = 1.0f / (j - left);
//
//        for (int k = left + 1; k < j; ++k) {
//          const float t = (k - left) * inv_step;
//          up[k] = (1.0f - t) * y_left + t * y_right;
//        }
//      }
//      last_valid_idx = j;
//    }
//  }
//
//  if (last_valid_idx >= 0 && last_valid_idx < N_uniform - 1) {
//    std::fill(up + last_valid_idx + 1, up + N_uniform, up[last_valid_idx]);
//  }
//
//  const int startCornersBin = int((n_bins - 1) * M_SQRT1_2);
//  const int uniformStartCornersBin = std::clamp(int((p.xv(startCornersBin) - x_min) * x_range_inv), 0, N_uniform - 1);
//  std::fill(up + uniformStartCornersBin, up + N_uniform, up[uniformStartCornersBin]);
//
//  // Smoothing the uniform profile
//  cv::medianBlur(U, U, 5);
//  cv::GaussianBlur(U, U, cv::Size(25, 1), 0, 0, cv::BORDER_REPLICATE);
//
//  // =========================================================================
//  // НАЧАЛО БЛОКА: Дифференциально-кумулятивный расчет целевой траектории и коррекции
//  // =========================================================================
//  const float * smup = U[0];
//  const double dx = (N_uniform - 1 > 0) ? (x_max - x_min) / (N_uniform - 1) : 0.0;
//
//  // Инициализируем буферы для равномерной сетки
//  std::vector<double> y_target(N_uniform, 0.0);
//  std::vector<float> uniform_correction(N_uniform, 0.0f);
//
//  // Базовая безопасная граница макроструктуры НЧ (первые 20% логарифмической шкалы)
//  const double x_stable_start = x_min + (x_max - x_min) * 0.20;
//
//  // Целевой шаг, который требует пользователь
//  const double delta_y_user = S1_target_slope * dx;
//
//  // Начальная точка
//  y_target[0] = double(smup[0]);
//  uniform_correction[0] = 0.0f;
//
//  // Координата центра перелома, где СЧ должны начать плавно переходить в ВЧ-деблур
//  // Логарифм частоты около 3.5 — идеальная граница затухания крупных поясов
//  const double x_transition_center = x_min + (x_max - x_min) * 0.45; // ~3.4 - 3.6
//  const double wsf_diff = 3.0; // Крутизна перехода дифференциала
//
//  for (int j = 1; j < N_uniform; ++j) {
//    const double current_x = x_min + j * dx;
//
//    if (current_x < x_stable_start) {
//      y_target[j] = double(smup[j]);
//    }
//    else {
//      const double delta_y_real = double(smup[j]) - double(smup[j - 1]);
//      const double delta_y_user = S1_target_slope * dx;
//
//      // Наш кумулятивный максимум
//      const double delta_y_max = std::max(delta_y_real, delta_y_user);
//
//      // Плавный вес сигмоиды для смешивания ДИФФЕРЕНЦИАЛА
//      const double w_mix = 1.0 / (1.0 + std::exp(-wsf_diff * (current_x - x_transition_center)));
//
//      // Мягко смешиваем шаги: в СЧ доминирует реальный спад кадра, в ВЧ — воля пользователя
//      const double chosen_delta = (1.0 - w_mix) * delta_y_real + w_mix * delta_y_max;
//
//      y_target[j] = y_target[j - 1] + chosen_delta;
//    }
//
//    const double diff = std::max(0.0, y_target[j] - double(smup[j]));
//    uniform_correction[j] = float(diff);
//  }
//
//  // =========================================================================
//  // КОНЕЦ БЛОКА
//  // =========================================================================
//
//  // Инициализируем выходную матрицу коррекции дефолтными единицами
//  out_correction = cv::Mat1f(1, n_bins, 1.0f);
//  float * __restrict corr_dst = out_correction[0];
//
//  // Интерполяция сглаженного профиля И матрицы коррекции обратно в исходную сетку бинов
//  cv::Mat1f output(1, n_bins);
//  float * __restrict dstp = output[0];
//  dstp[0] = float(p.sv(0) > 0 ? p.yv(0) : 0);
//  corr_dst[0] = 1.0f; // DC не трогаем
//
//  for( int i = 1; i < n_bins; ++i ) {
//    const double uniform_idx = (p.xv(i) - x_min) * x_range_inv;
//    const int k = std::clamp(int(uniform_idx), 0, N_uniform - 2);
//    const double t = uniform_idx - k;
//
//    // 1. Возвращаем сглаженный спектр для ваших графиков
//    dstp[i] = float((1.0 - t) * smup[k] + t * smup[k + 1]);
//
//    // 2. Возвращаем линейный коэффициент коррекции для фильтра (экспоненцируем прямо здесь!)
//    const double log_corr = (1.0 - t) * uniform_correction[k] + t * uniform_correction[k + 1];
//    corr_dst[i] = float(std::exp(log_corr));
//  }
//
//  return output;
//}

//static cv::Mat1f smoothDCT(const c_radial_spectrum_profile & p,
//  double S1_target_slope, cv::Mat1f & out_correction)
//{
//  constexpr int N_uniform = 100;
//  const int n_bins = p.size();
//
//  std::vector<double> bin_sums(N_uniform, 0.0);
//  std::vector<int> bin_counts(N_uniform, 0);
//
//  // Initialize the frequency range (skip DC)
//  const double x_min = p.xv(1);
//  const double x_max = p.xv(n_bins - 1);
//  const double x_range_inv = (x_max > x_min) ? (N_uniform - 1) / (x_max - x_min) : 0.0;
//
//  // Uniform accumulation (resampling)
//  for( int i = 1; i < n_bins; ++i ) {
//    if ( p.sv(i) > 0 ) {
//      const int bin_idx = std::clamp(int((p.xv(i) - x_min) * x_range_inv), 0, N_uniform - 1);
//      bin_sums[bin_idx] += p.yv(i);
//      bin_counts[bin_idx]++;
//    }
//  }
//
//  // Average non-empty cells
//  for( int i = 0; i < N_uniform; ++i ) {
//    if( bin_counts[i] > 1 ) {
//      bin_sums[i] /= bin_counts[i];
//    }
//  }
//
//  // Gap Filling
//  cv::Mat1f U(1, N_uniform);
//  float * __restrict up = U[0];
//
//  int last_valid_idx = -1;
//  for( int j = 0; j < N_uniform; ++j ) {
//    if( bin_counts[j] > 0 ) {
//      up[j] = float(bin_sums[j]);
//
//      if (last_valid_idx != j - 1) {
//        const int left = std::max(0, last_valid_idx);
//        const float y_left = (last_valid_idx >= 0) ? up[left] : up[j];
//        const float y_right = up[j];
//        const float inv_step = 1.0f / (j - left);
//
//        for (int k = left + 1; k < j; ++k) {
//          const float t = (k - left) * inv_step;
//          up[k] = (1.0f - t) * y_left + t * y_right;
//        }
//      }
//      last_valid_idx = j;
//    }
//  }
//
//  if (last_valid_idx >= 0 && last_valid_idx < N_uniform - 1) {
//    std::fill(up + last_valid_idx + 1, up + N_uniform, up[last_valid_idx]);
//  }
//
//  const int startCornersBin = int((n_bins - 1) * M_SQRT1_2);
//  const int uniformStartCornersBin = std::clamp(int((p.xv(startCornersBin) - x_min) * x_range_inv), 0, N_uniform - 1);
//  std::fill(up + uniformStartCornersBin, up + N_uniform, up[uniformStartCornersBin]);
//
//  // Smoothing the uniform profile
//  cv::medianBlur(U, U, 5);
//  cv::GaussianBlur(U, U, cv::Size(25, 1), 0, 0, cv::BORDER_REPLICATE);
//
//  // =========================================================================
//  // НАЧАЛО БЛОКА: Дифференциально-кумулятивный расчет целевой траектории и коррекции
//  // =========================================================================
//  const float * smup = U[0];
//  const double dx = (N_uniform - 1 > 0) ? (x_max - x_min) / (N_uniform - 1) : 0.0;
//
//  // Инициализируем буферы для равномерной сетки
//  std::vector<double> y_target(N_uniform, 0.0);
//  std::vector<float> uniform_correction(N_uniform, 0.0f);
//
//  // Базовая безопасная граница макроструктуры НЧ (первые 20% шкалы)
//  const double x_stable_start = x_min + (x_max - x_min) * 0.20;
//  const double delta_y_user = S1_target_slope * dx;
//
//  y_target[0] = double(smup[0]);
//  uniform_correction[0] = 0.0f;
//
//  // Геометрическая ширина окна раскрытия в логарифмических единицах.
//  // Вы можете вынести этот параметр в аргументы функции для эмпирического подбора.
//  const double win_width = 2.0;
//
//  for( int j = 1; j < N_uniform; ++j ) {
//    const double current_x = x_min + j * dx;
//
//    if( current_x <= x_stable_start ) {
//      y_target[j] = double(smup[j]);
//    }
//    else {
//      const double delta_y_real = double(smup[j]) - double(smup[j - 1]);
//      double w_sig = 0.0;
//      double t = (current_x - x_stable_start) / win_width;
//
//      if( t >= 1.0 ) {
//        w_sig = 1.0;
//      }
//      else {
//        // alpha = 0.05 задает скругление краев на 5% от ширины окна.
//        // Центральные 90% диапазона идут как идеально прямая линия.
//        constexpr double alpha = 0.05;
//
//        if( t < alpha ) {
//          // Плавный квадратичный разгон из нуля
//          w_sig = (t * t) / (2.0 * alpha);
//        }
//        else if( t <= (1.0 - alpha) ) {
//          // Идеальный честный линейный участок (90% от ширины окна)
//          w_sig = t;
//        }
//        else {
//          // Плавное квадратичный выход на плато 1.0
//          const double inv_t = 1.0 - t;
//          w_sig = 1.0 - (inv_t * inv_t) / (2.0 * alpha);
//        }
//      }
//
//      // Применяем полученный вес к смешиванию шагов в кумулятивном цикле
//      const double delta_y_user = S1_target_slope * dx;
//      const double effective_target_delta = (1.0 - w_sig) * delta_y_real + w_sig * delta_y_user;
//
//      y_target[j] = y_target[j - 1] + effective_target_delta;
//    }
//
//  // Исходный физический замок по разности траекторий
//  const double diff = std::max(0.0, y_target[j] - double(smup[j]));
//  uniform_correction[j] = float(diff);
//}
//
//  // =========================================================================
//  // КОНЕЦ БЛОКА
//  // =========================================================================
//
//  // Инициализируем выходную матрицу коррекции дефолтными единицами
//  out_correction = cv::Mat1f(1, n_bins, 1.0f);
//  float * __restrict corr_dst = out_correction[0];
//
//  // Интерполяция сглаженного профиля И матрицы коррекции обратно в исходную сетку бинов
//  cv::Mat1f output(1, n_bins);
//  float * __restrict dstp = output[0];
//  dstp[0] = float(p.sv(0) > 0 ? p.yv(0) : 0);
//  corr_dst[0] = 1.0f; // DC не трогаем
//
//  for( int i = 1; i < n_bins; ++i ) {
//    const double uniform_idx = (p.xv(i) - x_min) * x_range_inv;
//    const int k = std::clamp(int(uniform_idx), 0, N_uniform - 2);
//    const double t = uniform_idx - k;
//
//    // 1. Возвращаем сглаженный спектр для ваших графиков
//    dstp[i] = float((1.0 - t) * smup[k] + t * smup[k + 1]);
//
//    // 2. Возвращаем линейный коэффициент коррекции для фильтра (экспоненцируем прямо здесь!)
//    const double log_corr = (1.0 - t) * uniform_correction[k] + t * uniform_correction[k + 1];
//    corr_dst[i] = float(std::exp(log_corr));
//  }
//
//  return output;
//}

static cv::Mat1f smoothDCT(const c_radial_spectrum_profile & p,
  double S1_target_slope, cv::Mat1f & out_correction)
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
    if ( p.sv(i) > 0 ) {
      const int bin_idx = std::clamp(int((p.xv(i) - x_min) * x_range_inv), 0, N_uniform - 1);
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
      up[j] = float(bin_sums[j]);

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

  if (last_valid_idx >= 0 && last_valid_idx < N_uniform - 1) {
    std::fill(up + last_valid_idx + 1, up + N_uniform, up[last_valid_idx]);
  }

  const int startCornersBin = int((n_bins - 1) * M_SQRT1_2);
  const int uniformStartCornersBin = std::clamp(int((p.xv(startCornersBin) - x_min) * x_range_inv), 0, N_uniform - 1);
  std::fill(up + uniformStartCornersBin, up + N_uniform, up[uniformStartCornersBin]);

  // Smoothing the uniform profile
  cv::medianBlur(U, U, 5);
  cv::GaussianBlur(U, U, cv::Size(25, 1), 0, 0, cv::BORDER_REPLICATE);

  const float * smup = U[0];
  const double dx = (N_uniform - 1 > 0) ? (x_max - x_min) / (N_uniform - 1) : 0.0;

  std::vector<double> y_target(N_uniform, 0.0);
  std::vector<float> uniform_correction(N_uniform, 0.0f);

  const double x_stable_start = x_min + (x_max - x_min) * 0.20;
  const double win_width = 2.0;
  const double delta_y_user = S1_target_slope * dx;

  y_target[0] = double(smup[0]);
  uniform_correction[0] = 0.0f;

  for( int j = 1; j < N_uniform; ++j ) {
    const double current_x = x_min + j * dx;

    if( current_x < x_stable_start ) {
      y_target[j] = double(smup[j]);
    }
    else {
      const double delta_y_real = double(smup[j]) - double(smup[j - 1]);
      const double delta_y_user = S1_target_slope * dx;

      double w_sig = 0.0;
      double t = (current_x - x_stable_start) / win_width;
      if( t >= 1.0 ) {
        w_sig = 1.0;
      }
      else {
        constexpr double alpha = 0.05;
        if( t < alpha )
          w_sig = (t * t) / (2.0 * alpha);
        else if( t <= (1.0 - alpha) )
          w_sig = t;
        else {
          const double inv_t = 1.0 - t;
          w_sig = 1.0 - (inv_t * inv_t) / (2.0 * alpha);
        }
      }

      const double effective_target_delta = (1.0 - w_sig) * delta_y_real + w_sig * delta_y_user;
      const double chosen_delta = std::max(delta_y_real, effective_target_delta);
      y_target[j] = y_target[j - 1] + chosen_delta;
    }

    const double diff = std::max(0.0, y_target[j] - double(smup[j]));
    uniform_correction[j] = float(diff);
  }

  out_correction = cv::Mat1f(1, n_bins, 1.0f);
  float * __restrict corr_dst = out_correction[0];

  cv::Mat1f output(1, n_bins);
  float * __restrict dstp = output[0];
  dstp[0] = float(p.sv(0) > 0 ? p.yv(0) : 0);
  corr_dst[0] = 1.0f; // DC не трогаем

  for( int i = 1; i < n_bins; ++i ) {
    const double uniform_idx = (p.xv(i) - x_min) * x_range_inv;
    const int k = std::clamp(int(uniform_idx), 0, N_uniform - 2);
    const double t = uniform_idx - k;
    dstp[i] = float((1.0 - t) * smup[k] + t * smup[k + 1]);
    const double log_corr = (1.0 - t) * uniform_correction[k] + t * uniform_correction[k + 1];
    corr_dst[i] = float(std::exp(log_corr));
  }

  return output;
}

static std::vector<float> computeCorrection(const c_radial_spectrum_profile & sp, double S1_target_slope,
    std::vector<float> * sdct = nullptr)
{
  constexpr int N_uniform = 100;
  const int n_bins = sp.size();

  std::vector<double> bin_sums(N_uniform, 0.0);
  std::vector<int> bin_counts(N_uniform, 0);

  // Logarithmic frequency range skipping DC
  const double x_min = sp.xv(1);
  const double x_max = sp.xv(n_bins - 1);
  const double x_range_inv = (x_max > x_min) ? (N_uniform - 1) / (x_max - x_min) : 0.0;

  // Uniform resampling of the original spectrum profile
  for( int i = 1; i < n_bins; ++i ) {
    if( sp.sv(i) > 0 ) {
      const int bin_idx = std::clamp(int((sp.xv(i) - x_min) * x_range_inv), 0, N_uniform - 1);
      bin_sums[bin_idx] += sp.yv(i);
      bin_counts[bin_idx]++;
    }
  }
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
      up[j] = float(bin_sums[j]);

      if( last_valid_idx != j - 1 ) {
        const int left = std::max(0, last_valid_idx);
        const float y_left = (last_valid_idx >= 0) ? up[left] : up[j];
        const float y_right = up[j];
        const float inv_step = 1.0f / (j - left);

        for( int k = left + 1; k < j; ++k ) {
          const float t = (k - left) * inv_step;
          up[k] = (1.0f - t) * y_left + t * y_right;
        }
      }
      last_valid_idx = j;
    }
  }

  // Extrapolate the right edge if the last cells are empty
  if( last_valid_idx >= 0 && last_valid_idx < N_uniform - 1 ) {
    std::fill(up + last_valid_idx + 1, up + N_uniform, up[last_valid_idx]);
  }

  // Freeze the tail of the spectrum matrix (corner region of the frame)
  const int startCornersBin = int((n_bins - 1) * M_SQRT1_2);
  const int uniformStartCornersBin = std::clamp(int((sp.xv(startCornersBin) - x_min) * x_range_inv), 0, N_uniform - 1);
  std::fill(up + uniformStartCornersBin, up + N_uniform, up[uniformStartCornersBin]);

  // Filtering and smoothing the resampled profile
  cv::medianBlur(U, U, 5);
  cv::GaussianBlur(U, U, cv::Size(25, 1), 0, 0, cv::BORDER_REPLICATE);
  const float * smup = U[0];

  // Differential-cumulative calculation of the target trajectory
  const double dx = (N_uniform - 1 > 0) ? (x_max - x_min) / (N_uniform - 1) : 0.0;

  std::vector<double> y_target(N_uniform, 0.0);
  std::vector<float> uniform_correction(N_uniform, 0.0f);

  const double x_stable_start = x_min + (x_max - x_min) * 0.20;
  const double win_width = 2.0;

  y_target[0] = double(smup[0]);
  uniform_correction[0] = 0.0f;

  for( int j = 1; j < N_uniform; ++j ) {
    const double current_x = x_min + j * dx;

    if( current_x < x_stable_start ) {
      y_target[j] = double(smup[j]);
    }
    else {
      const double delta_y_real = double(smup[j]) - double(smup[j - 1]);
      const double delta_y_user = S1_target_slope * dx;

      // The weight of a quasi-linear opening window with chamfered edges (alpha = 5%)
      double w_sig = 0.0;
      const double t = (current_x - x_stable_start) / win_width;
      if( t >= 1.0 ) {
        w_sig = 1.0;
      }
      else {
        constexpr double alpha = 0.05;
        if( t < alpha ) {
          w_sig = (t * t) / (2.0 * alpha);
        }
        else if( t <= (1.0 - alpha) ) {
          w_sig = t;
        }
        else {
          const double inv_t = 1.0 - t;
          w_sig = 1.0 - (inv_t * inv_t) / (2.0 * alpha);
        }
      }

      // Linear mixing and hard differential locking
      const double effective_target_delta = (1.0 - w_sig) * delta_y_real + w_sig * delta_y_user;
      const double chosen_delta = std::max(delta_y_real, effective_target_delta);
      y_target[j] = y_target[j - 1] + chosen_delta;
    }

    // Physical lock based on the difference of logarithmic trajectories
    const double diff = std::max(0.0, y_target[j] - double(smup[j]));
    uniform_correction[j] = float(diff);
  }

  // Back interpolation into the original frame bin grid
  std::vector<float> out_correction(n_bins, 1.0f);
  out_correction[0] = 1.0f; // Don't touch DC

  if( sdct != nullptr ) {
    sdct->resize(n_bins, 0.0f);
    (*sdct)[0] = float(sp.sv(0) > 0 ? sp.yv(0) : 0);
  }

  for( int i = 1; i < n_bins; ++i ) {
    const double uniform_idx = (sp.xv(i) - x_min) * x_range_inv;
    const int k = std::clamp(int(uniform_idx), 0, N_uniform - 2);
    const double t = uniform_idx - k;

    // Smoothed spectrum profile if explicitly requested by the application
    if( sdct != nullptr ) {
      (*sdct)[i] = float((1.0 - t) * smup[k] + t * smup[k + 1]);
    }

    // Linear coefficients of the DCT filter using the delta exponential
    const double log_corr = (1.0 - t) * uniform_correction[k] + t * uniform_correction[k + 1];
    out_correction[i] = float(std::exp(log_corr));
  }

  return out_correction;
}


static int estimateNature2(const c_radial_spectrum_profile &sp,
    const cv::Mat1f & SDCT,
    double & output_S0_nature,
    double & output_S1_nature,
    double & output_S0_maxline,
    double & output_S1_maxline,
    int & output_kneeIndex,
    int & output_curvatureStartIndex,
    cv::Mat1f & kx,
    bool print_debug_info = false)
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

  if ( print_debug_info ) {
    CF_DEBUG("Initial startSearchBin=%d (x=%g y=%g)", startSearchBin, sp.xv(startSearchBin), SDCT(0, startSearchBin));
  }

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


  //
  // Search for max slope line originated from startSearchBin
  //

  const double startX = sp.xv(startSearchBin);
  const double startY = SDCT(0, startSearchBin);
  if ( print_debug_info ) {
    CF_DEBUG("startSearchBin=%d (x=%g y=%g)", startSearchBin, startX, startY);
  }

  int endSearchBin = -1;
  double Kx = DBL_MAX;
  double Kx_best = DBL_MAX;

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
  if ( print_debug_info ) {
    CF_DEBUG("endSearchBin=%d (x=%g y=%g) Kx=%g Kx_best=%g", endSearchBin, sp.xv(endSearchBin), SDCT(0, endSearchBin), Kx, Kx_best);
  }

  const double S0_maxline = startY - Kx * startX;
  const double S1_maxline = Kx;
  output_S0_maxline = S0_maxline;
  output_S1_maxline = S1_maxline;


  //
  // Knee method: looking for the maximum positive vertical distance (curve above the line)
  //
  const double xMid = 0.5 * (sp.xv(startSearchBin) + sp.xv(endSearchBin));
  double max_distance = 0.0;
  int kneeIndex = 0;

  for (int i = startSearchBin + 3; i < endSearchBin; ++i) {
    const double x = sp.xv(i);
    if ( x > 2.0 ) {
      const double line_val = S1_maxline * x  + S0_maxline;
      const double curve_val = SDCT(0, i);
      const double distance = (curve_val - line_val) / std::sqrt(1.0 + 0.25 * std::abs(x - xMid));
      if (distance > max_distance) {
        max_distance = distance;
        kneeIndex = i;
      }
    }
  }

  if( kneeIndex <= startSearchBin ) {
    CF_ERROR("BAD kneeIndex=%d <= startSearchBin=%d endSearchBin=%d", kneeIndex, startSearchBin, endSearchBin);
    return -1;
  }

  output_kneeIndex = kneeIndex;
  if ( print_debug_info ) {
    CF_DEBUG("kneeIndex=%d (x=%g y=%g)", kneeIndex, sp.xv(kneeIndex), SDCT(0, kneeIndex));
  }

  //
  // Compute linear regression from startSearchBin to kneeIndex.
  // Search also max linear interval and save to curvatureStartIndex
  //

  c_line_estimate line;
  c_linear_regression3 reg3;
  double s2_best = DBL_MAX;
  int curvatureStartIndex = kneeIndex;

  for ( int i = startSearchBin; i < kneeIndex; ++i ) {
    const double x = sp.xv(i);
    const double y = SDCT(0, i);
    reg3.update(1, x, x * x, y);
    line.update(x, y);
    if ( line.pts() > 10 ) {
      double s0, s1, s2;
      reg3.compute(s0, s1, s2);
      if ( std::abs(s2) < s2_best ) {
        s2_best = std::abs(s2);
        curvatureStartIndex = i;
      }
    }
  }

  line.compute(output_S0_nature, output_S1_nature);
  output_curvatureStartIndex = curvatureStartIndex;

  return startSearchBin;
}

static cv::Mat1f createInverseBlurCorrectionFilter(const cv::Mat1f & RadialSpectrumProfile, /*[1][n_bins] */
    const cv::Size & dctSize,
    bool useS1_target,
    double S1_target,
    bool print_debug_info = false,
    const std::string & debug_file_name = "")
{
  const c_radial_spectrum_profile sp(RadialSpectrumProfile);
  std::vector<float> SDCT;
  const int N = sp.size();

  /*
   * COMPUTE DCT SPECTRUM CORRECTION FOR TARGET SLOPE ADJUSMENT
   */
  const std::vector<float> correction =
      computeCorrection(sp, S1_target,
          debug_file_name.empty() ? nullptr :
              &SDCT);

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
        const double ys = SDCT[i];
        const double ytarget = S1_target * x;
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
  ctlbind(ctls, "Intensity channel: ", CTL_CONTEXT(ctx, _intensity_channel), "Select intensity channel for spectrum analysis");
  ctlbind(ctls, "inpaint_missing_pixels:", CTL_CONTEXT(ctx, _mask_inpaint_method), "");
  ctlbind(ctls, "S1_target: ", CTL_CONTEXT(ctx, _useS1_target), "");
  ctlbind(ctls, "S1_target: ", CTL_CONTEXT(ctx, _S1_target), "");
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
    SERIALIZE_OPTION(settings, save, *this, _useS1_target);
    SERIALIZE_OPTION(settings, save, *this, _S1_target);
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

  cv::Mat src;
  cv::Mat1f intensity_img;
  cv::Mat1f intensity_dct;
  cv::Mat1f dct_radial_profile;
  std::vector<cv::Mat1f> src_channels;

  const int cn = image.channels();

  image.getMat().convertTo(src, CV_32F);

  if( !mask.empty() ) {
    switch (_mask_inpaint_method) {
      case LINEAR_INTERPOLATION_INPAINT:
        linear_interpolation_inpaint(src, mask);
        break;
      case AVERAGE_PYRAMID_INPAINT:
        average_pyramid_inpaint(src, mask, src, cv::noArray(), 7);
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
           _useS1_target, _S1_target, _print_debug_info,
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
