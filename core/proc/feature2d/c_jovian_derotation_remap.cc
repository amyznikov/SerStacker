/*
 * c_jovian_derotation_remap.cc
 *
 *  Created on: May 25, 2026
 *      Author: amyznikov
 */

#include "c_jovian_derotation_remap.h"

void c_jovian_derotation_remap::set_reference_pose(const cv::Size & image_size,
    const cv::Point2d & center,
    const cv::Vec3d & axes,
    const cv::Vec3d & pose)
{
  _image_size = image_size;
  _center = center;
  _axes = axes;
  _current_pose = _target_pose = pose;
  _Rcurrent = _Rtarget = build_ellipsoid_rotation(_target_pose);
}

void c_jovian_derotation_remap::compute_derotation_for_time(double deltat_msec)
{
  // Jupiter daily rotation period is 9h 55m 30s.
  static constexpr double rotation_period_sec = 9. * 3600 + 55. * 60 + 30.;
  const double rotation_angle_deg = 0.360 * deltat_msec / rotation_period_sec;
  const double rotation_angle_radians = rotation_angle_deg * CV_PI / 180;
  return compute_derotation_for_angle(rotation_angle_radians);
}

void c_jovian_derotation_remap::compute_derotation_for_angle(double longitude_rotation_radians)
{
  _current_pose = cv::Vec3d(_target_pose(0) + longitude_rotation_radians, _target_pose(1), _target_pose(2));
  _Rcurrent = build_ellipsoid_rotation(_current_pose);

  compute_ellipsoid_zrotation_remap(_image_size,
      _center,
      _axes,
      _Rcurrent,
      _Rtarget,
      _rmap,
      _wmap,
      _rmask,
      nullptr);
}


//#include <opencv2/opencv.hpp>
//#include <vector>
//
//class c_ellipsoid_derotation_stacker {
//public:
//  // Инициализация аккумуляторов при старте серии
//  void init(const cv::Size& size) {
//    _sum_image = cv::Mat3f::zeros(size);
//    _sum_weights = cv::Mat1f::zeros(size);
//    _master_mask = cv::Mat1b::zeros(size);
//  }
//
//  /**
//   * Добавление очередного деротированного кадра в стек
//   * @param derotated_frame - кадр после cv::remap (BGR, тип CV_8UC3 или CV_16UC3)
//   * @param wmap             - карта весов из compute_ellipsoid_zrotation_remap
//   * @param mask             - маска валидности из compute_ellipsoid_zrotation_remap
//   */
//  void add_frame(const cv::Mat& derotated_frame, const cv::Mat1f& wmap, const cv::Mat1b& mask) {
//    // Приводим исходный кадр к плавающей точке (0.0 ... 255.0 или 0.0 ... 65535.0)
//    cv::Mat3f frame_f;
//    derotated_frame.convertTo(frame_f, CV_32F);
//
//    const int rows = _sum_image.rows;
//    const int cols = _sum_image.cols;
//
//    // Попиксельное взвешенное накопление
//    for (int y = 0; y < rows; ++y) {
//      const cv::Vec3f* src_ptr = frame_f.ptr<cv::Vec3f>(y);
//      const float* w_ptr = wmap.ptr<float>(y);
//      const uchar* m_ptr = mask.ptr<uchar>(y);
//
//      cv::Vec3f* sum_img_ptr = _sum_image.ptr<cv::Vec3f>(y);
//      float* sum_w_ptr = _sum_weights.ptr<float>(y);
//      uchar* dst_mask_ptr = _master_mask.ptr<uchar>(y);
//
//      for (int x = 0; x < cols; ++x) {
//        // Добавляем только те пиксели, которые валидны (внутри диска и не вылезли из-за изнанки)
//        if (m_ptr[x] > 0 && w_ptr[x] > 0.0f) {
//          float w = w_ptr[x];
//
//          sum_img_ptr[x][0] += src_ptr[x][0] * w; // Канал B
//          sum_img_ptr[x][1] += src_ptr[x][1] * w; // Канал G
//          sum_img_ptr[x][2] += src_ptr[x][2] * w; // Канал R
//
//          sum_w_ptr[x] += w;
//          dst_mask_ptr[x] = 255; // Накапливаем общую маску покрытия
//        }
//      }
//    }
//  }
//
//  /**
//   * Финальный расчет усредненного изображения
//   * @param result_image       - выходной BGR кадр (тип соответствует исходному)
//   * @param out_mask           - объединенная маска диска планеты
//   * @param output_type        - CV_8UC3 или CV_16UC3
//   * @param background_source  - (Опционально) Исходный мастер-кадр для восстановления фона неба
//   */
//  void finalize(cv::OutputArray result_image, cv::OutputArray out_mask, int output_type = CV_8UC3, const cv::Mat& background_source = cv::Mat()) {
//    cv::Size size = _sum_image.size();
//    cv::Mat3f final_f = cv::Mat3f::zeros(size);
//
//    // Если передан кадр-источник фона, инициализируем им выходную матрицу
//    if (!background_source.empty()) {
//      background_source.convertTo(final_f, CV_32F);
//    }
//
//    for (int y = 0; y < size.height; ++y) {
//      const cv::Vec3f* sum_img_ptr = _sum_image.ptr<cv::Vec3f>(y);
//      const float* sum_w_ptr = _sum_weights.ptr<float>(y);
//      const uchar* mask_ptr = _master_mask.ptr<uchar>(y);
//      cv::Vec3f* dst_ptr = final_f.ptr<cv::Vec3f>(y);
//
//      for (int x = 0; x < size.width; ++x) {
//        if (mask_ptr[x] > 0 && sum_w_ptr[x] > 0.0f) {
//          // Вычисляем честное среднее взвешенное значение
//          dst_ptr[x][0] = sum_img_ptr[x][0] / sum_w_ptr[x];
//          dst_ptr[x][1] = sum_img_ptr[x][1] / sum_w_ptr[x];
//          dst_ptr[x][2] = sum_img_ptr[x][2] / sum_w_ptr[x];
//        }
//      }
//    }
//
//    // Конвертируем обратно в целевой формат данных для отображения/сохранения
//    final_f.convertTo(result_image, output_type);
//    _master_mask.copyTo(out_mask);
//  }
//
//private:
//  cv::Mat3f _sum_image;
//  cv::Mat1f _sum_weights;
//  cv::Mat1b _master_mask;
//};
