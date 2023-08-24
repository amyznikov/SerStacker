/*
 * c_fit_jovian_ellipse_routine.cc
 *
 *  Created on: Aug 12, 2022
 *      Author: amyznikov
 */

#include "c_fit_jovian_ellipse_routine.h"
#include <core/proc/cdiffs.h>
#include <core/ssprintf.h>

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
      { c_fit_jovian_ellipse_routine::display_final_ellipse_fit, "final_ellipse_fit", },

      { c_fit_jovian_ellipse_routine::display_gradient_test_image, "gradient_test", },


      { c_fit_jovian_ellipse_routine::display_final_ellipse_fit, nullptr, },
  };

  return members;
}

void c_fit_jovian_ellipse_routine::set_display(display_type v)
{
  display_type_ = v;
}

c_fit_jovian_ellipse_routine::display_type c_fit_jovian_ellipse_routine::display() const
{
  return display_type_;
}

void c_fit_jovian_ellipse_routine::set_stdev_factor(double v)
{
  detector_.set_stdev_factor(v);
}

double c_fit_jovian_ellipse_routine::stdev_factor() const
{
  return detector_.stdev_factor();
}

c_jovian_ellipse_detector * c_fit_jovian_ellipse_routine::detector()
{
  return &detector_;
}

const c_jovian_ellipse_detector * c_fit_jovian_ellipse_routine::detector() const
{
  return &detector_;
}

bool c_fit_jovian_ellipse_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {

    SERIALIZE_PROPERTY(settings, save, *this, stdev_factor);
    SERIALIZE_PROPERTY(settings, save, *this, display);

    return true;
  }
  return false;
}

static void rotatedRectange(cv::InputOutputArray image, const cv::RotatedRect & rc,
    const cv::Scalar color, int thickness = 1, int lineType = cv::LINE_8, int shift = 0)
{

  if ( 1 ) {
    cv::rectangle(image, compute_ellipse_bounding_box(rc),
        cv::Scalar(0, 0, 1),
        3);
  }

  cv::Point2f pts[4];
  rc.points(pts);


  for( int i = 0; i < 4; i++ ) {
    cv::line(image, pts[i], pts[(i + 1) % 4], color, thickness, lineType, shift);
  }

  cv::line(image, (pts[0] + pts[1]) * 0.5, (pts[2] + pts[3]) * 0.5, color, thickness, lineType, shift);
  cv::line(image, (pts[1] + pts[2]) * 0.5, (pts[0] + pts[3]) * 0.5, color, thickness, lineType, shift);
}

bool c_fit_jovian_ellipse_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{

//  static const auto atan2 =
//      [](const cv::Mat1f & Nx, const cv::Mat1f & Ny, cv::Mat1f & angle) {
//
//        angle.create(Nx.size());
//        for ( int y = 0; y < Nx.rows; ++y ) {
//          for ( int x = 0; x < Nx.cols; ++x ) {
//            angle[y][x] = std::atan2 (Ny[y][x], Nx[y][x]) * 180 / CV_PI;
//          }
//        }
//
//      };

  cv::Mat tmp;

  detector_.set_enable_debug_images(true);
  detector_.detect_jovian_disk(image, mask);

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

  case display_final_ellipse_fit:
  case display_planetary_disk_ellipseAMS2:
    rotatedRectange(image, detector_.ellipseAMS2(), CV_RGB(0, 1, 0), 1);
    cv::ellipse(image, detector_.ellipseAMS2(), CV_RGB(0, 0, 1), 1);
    break;

  case display_gradient_test_image:
    detector_.gradient_test_image().copyTo(image);
    break;
  }

  if( mask.needed() ) {
    mask.release();
  }

  return true;
}
