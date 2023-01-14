/*
 * c_fit_jovian_ellipse_routine.cc
 *
 *  Created on: Aug 12, 2022
 *      Author: amyznikov
 */

#include "c_fit_jovian_ellipse_routine.h"
#include <core/proc/cdiffs.h>
#include <core/ssprintf.h>

c_fit_jovian_ellipse_routine::c_class_factory c_fit_jovian_ellipse_routine::class_factory;

template<>
const c_enum_member * members_of<c_fit_jovian_ellipse_routine::display_type>()
{
  static constexpr c_enum_member members[] = {
      { c_fit_jovian_ellipse_routine::display_detected_planetary_disk_mask, "detected_planetary_disk_mask", },
      { c_fit_jovian_ellipse_routine::display_detected_planetary_disk_edge, "detected_planetary_disk_edge", },
      { c_fit_jovian_ellipse_routine::display_detected_ellipseAMS, "detected_ellipseAMS", },
      { c_fit_jovian_ellipse_routine::display_planetary_disk_ellipse_edge, "planetary_disk_ellipse_edge", },
      { c_fit_jovian_ellipse_routine::display_initial_artifial_ellipse_edge, "initial_artifial_ellipse_edge", },
      { c_fit_jovian_ellipse_routine::display_remapped_artifial_ellipse_edge, "remapped_artifial_ellipse_edge", },
      { c_fit_jovian_ellipse_routine::display_aligned_artifial_ellipse_edge,"aligned_artifial_ellipse_edge"},
      { c_fit_jovian_ellipse_routine::display_aligned_artifial_ellipse_mask,"aligned_artifial_ellipse_mask"},

      { c_fit_jovian_ellipse_routine::display_planetary_disk_ellipseAMS2, "planetary_disk_ellipseAMS2", },
      { c_fit_jovian_ellipse_routine::display_gray_image, "gray_image", },
      { c_fit_jovian_ellipse_routine::display_normalized_image, "normalized_image", },
      { c_fit_jovian_ellipse_routine::display_final_ellipse_fit, "final_ellipse_fit", },
      { c_fit_jovian_ellipse_routine::display_final_ellipse_fit, nullptr, },
  };

  return members;
}

c_fit_jovian_ellipse_routine::c_fit_jovian_ellipse_routine(bool enabled)
  : base(&class_factory, enabled)
{
}

c_fit_jovian_ellipse_routine::ptr c_fit_jovian_ellipse_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

void c_fit_jovian_ellipse_routine::set_display(display_type v)
{
  display_type_ = v;
}

c_fit_jovian_ellipse_routine::display_type c_fit_jovian_ellipse_routine::display() const
{
  return display_type_;
}

void c_fit_jovian_ellipse_routine::set_hlines(const std::vector<float> & hlines)
{
  detector_.set_hlines(hlines);
}

const std::vector<float> & c_fit_jovian_ellipse_routine::hlines() const
{
  return detector_.hlines();
}

void c_fit_jovian_ellipse_routine::set_normalization_scale(int v)
{
  return detector_.set_normalization_scale(v);
}

int c_fit_jovian_ellipse_routine::normalization_scale() const
{
  return detector_.normalization_scale();
}

void c_fit_jovian_ellipse_routine::set_stdev_factor(double v)
{
  detector_.set_stdev_factor(v);
}

double c_fit_jovian_ellipse_routine::stdev_factor() const
{
  return detector_.stdev_factor();
}

void c_fit_jovian_ellipse_routine::set_normalization_blur(double v)
{
  return detector_.set_normalization_blur(v);
}

double c_fit_jovian_ellipse_routine::normalization_blur() const
{
  return detector_.normalization_blur();
}

void c_fit_jovian_ellipse_routine::set_gradient_blur(double v)
{
  return detector_.set_gradient_blur(v);
}

double c_fit_jovian_ellipse_routine::gradient_blur() const
{
  return detector_.gradient_blur();
}

c_jovian_ellipse_detector * c_fit_jovian_ellipse_routine::detector()
{
  return &detector_;
}

const c_jovian_ellipse_detector * c_fit_jovian_ellipse_routine::detector() const
{
  return &detector_;
}

bool c_fit_jovian_ellipse_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  SAVE_PROPERTY(settings, *this, hlines);
  SAVE_PROPERTY(settings, *this, normalization_scale);
  SAVE_PROPERTY(settings, *this, normalization_blur);
  SAVE_PROPERTY(settings, *this, gradient_blur);
  SAVE_PROPERTY(settings, *this, stdev_factor);
  SAVE_PROPERTY(settings, *this, display);

  return true;
}

bool c_fit_jovian_ellipse_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  LOAD_PROPERTY(settings, *this, hlines);
  LOAD_PROPERTY(settings, *this, normalization_scale);
  LOAD_PROPERTY(settings, *this, normalization_blur);
  LOAD_PROPERTY(settings, *this, gradient_blur);
  LOAD_PROPERTY(settings, *this, stdev_factor);
  LOAD_PROPERTY(settings, *this, display);

  return true;
}

//
///*
// * Five-point approximation to first order image derivative.
// *  <https://en.wikipedia.org/wiki/Numerical_differentiation>
// * */
//static void differentiate(cv::InputArray _src, cv::Mat & gx, cv::Mat & gy, int ddepth)
//{
//  static thread_local const cv::Matx<float, 1, 5> K(
//      (+1.f / 12),
//      (-8.f / 12),
//        0.f,
//      (+8.f / 12),
//      (-1.f / 12));
//
//  if ( ddepth < 0 ) {
//    ddepth = std::max(_src.depth(), CV_32F);
//  }
//
//  const cv::Mat & src = _src.getMat();
//  cv::filter2D(src, gx, ddepth, K, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
//  cv::filter2D(src, gy, ddepth, K.t(), cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
//}

static void rotatedRectange(cv::InputOutputArray image, const cv::RotatedRect & rc,
    const cv::Scalar color, int thickness = 1, int lineType = cv::LINE_8, int shift = 0)
{
  cv::Point2f pts[4];
  rc.points(pts);

  for (int i = 0; i < 4; i++) {
   cv::line(image, pts[i], pts[(i+1)%4], color, thickness, lineType, shift);
  }

  cv::line(image, (pts[0]+pts[1])*0.5, (pts[2]+pts[3])*0.5, color, thickness, lineType, shift);
  cv::line(image, (pts[1]+pts[2])*0.5, (pts[0]+pts[3])*0.5, color, thickness, lineType, shift);
}

bool c_fit_jovian_ellipse_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{

  static const auto atan2 =
      [](const cv::Mat1f & Nx, const cv::Mat1f & Ny, cv::Mat1f & angle) {

        angle.create(Nx.size());
        for ( int y = 0; y < Nx.rows; ++y ) {
          for ( int x = 0; x < Nx.cols; ++x ) {
            angle[y][x] = std::atan2 (Ny[y][x], Nx[y][x]) * 180 / CV_PI;
          }
        }

      };


  cv::Mat tmp;

  detector_.detect_planetary_disk(image, mask);

  switch (display_type_) {

    case display_detected_planetary_disk_mask:
    image.setTo(cv::Scalar::all(1), detector_.detected_planetary_disk_mask());
    break;

  case display_detected_planetary_disk_edge:
    image.setTo(cv::Scalar::all(1), detector_.detected_planetary_disk_edge());
    break;

  case display_detected_ellipseAMS:
    rotatedRectange(image, detector_.ellipseAMS(), CV_RGB(0, 1, 0), 1);
    cv::ellipse(image, detector_.ellipseAMS(), CV_RGB(0, 0, 1), 1);
    break;

  case display_initial_artifial_ellipse_edge:
    cv::cvtColor(detector_.initial_artifial_ellipse_edge(), tmp, cv::COLOR_GRAY2BGR);
    tmp.copyTo(image, detector_.initial_artifial_ellipse_edge() > 0.01);
    break;

  case display_remapped_artifial_ellipse_edge:
    cv::cvtColor(detector_.remapped_artifial_ellipse_edge(), tmp, cv::COLOR_GRAY2BGR);
    tmp.copyTo(image, detector_.remapped_artifial_ellipse_edge() > 0.01);
    break;

  case display_aligned_artifial_ellipse_edge:
    image.setTo(cv::Scalar::all(1), detector_.aligned_artifial_ellipse_edge_mask());
    break;

  case display_aligned_artifial_ellipse_mask:
    cv::cvtColor(detector_.aligned_artifial_ellipse_mask(), tmp, cv::COLOR_GRAY2BGR);
    image.setTo(cv::Scalar::all(1), detector_.aligned_artifial_ellipse_mask());
    break;

  case display_gray_image:
    cv::cvtColor(detector_.gray_image(), image, cv::COLOR_GRAY2BGR);
    rotatedRectange(image, detector_.ellipseAMS2(), CV_RGB(0, 1, 0), 1);
    cv::ellipse(image, detector_.ellipseAMS2(), CV_RGB(0, 0, 1), 1);
    break;

  case display_normalized_image:
    cv::cvtColor(detector_.normalized_image(), image, cv::COLOR_GRAY2BGR);
    rotatedRectange(image, detector_.ellipseAMS2(), CV_RGB(0, 1, 0), 1);
    cv::ellipse(image, detector_.ellipseAMS2(), CV_RGB(0, 0, 1), 1);
    break;

  case display_final_ellipse_fit:
  case display_planetary_disk_ellipseAMS2:
    rotatedRectange(image, detector_.ellipseAMS2(), CV_RGB(0, 1, 0), 1);
    cv::ellipse(image, detector_.ellipseAMS2(), CV_RGB(0, 0, 1), 1);
    break;
  }

  if( mask.needed() ) {
    mask.release();
  }

  return true;
}
