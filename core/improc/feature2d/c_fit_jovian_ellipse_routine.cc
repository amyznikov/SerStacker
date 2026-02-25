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
  static const c_enum_member members[] = {
      { c_fit_jovian_ellipse_routine::display_gray_image, "gray_image", },

      { c_fit_jovian_ellipse_routine::display_detected_planetary_disk_mask, "detected_planetary_disk_mask", },
      { c_fit_jovian_ellipse_routine::display_detected_planetary_disk_edge, "detected_planetary_disk_edge", },

//      { c_fit_jovian_ellipse_routine::display_detected_ellipseAMS, "detected_ellipseAMS", },

      { c_fit_jovian_ellipse_routine::display_pca_gx, "pca_gx", },
      { c_fit_jovian_ellipse_routine::display_pca_gy, "pca_gy", },

      // { c_fit_jovian_ellipse_routine::display_final_planetary_disk_ellipse, "final_planetary_disk_ellipse", },
      { c_fit_jovian_ellipse_routine::display_final_planetary_disk_mask, "final_planetary_disk_mask", },

      { c_fit_jovian_ellipse_routine::display_final_ellipse_fit, "final_ellipse_fit", },

      { c_fit_jovian_ellipse_routine::display_final_ellipse_fit, },
  };

  return members;
}

void c_fit_jovian_ellipse_routine::set_display(display_type v)
{
  _display_type = v;
}

c_fit_jovian_ellipse_routine::display_type c_fit_jovian_ellipse_routine::display() const
{
  return _display_type;
}

void c_fit_jovian_ellipse_routine::set_stdev_factor(double v)
{
  _detector.set_stdev_factor(v);
}

double c_fit_jovian_ellipse_routine::stdev_factor() const
{
  return _detector.stdev_factor();
}

void c_fit_jovian_ellipse_routine::set_pca_blur(double v)
{
  _detector.set_pca_blur(v);
}

double c_fit_jovian_ellipse_routine::pca_blur() const
{
  return _detector.pca_blur();
}

void c_fit_jovian_ellipse_routine::set_offset(const cv::Point2f & v)
{
  _detector.set_offset(v);
}

const cv::Point2f & c_fit_jovian_ellipse_routine::offset() const
{
  return _detector.offset();
}

c_jovian_ellipse_detector * c_fit_jovian_ellipse_routine::detector()
{
  return &_detector;
}

const c_jovian_ellipse_detector * c_fit_jovian_ellipse_routine::detector() const
{
  return &_detector;
}

void c_fit_jovian_ellipse_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "stdev_factor", ctx, &this_class::stdev_factor, &this_class::set_stdev_factor, "stdev_factor");
  ctlbind(ctls, "pca_blur", ctx, &this_class::pca_blur, &this_class::set_pca_blur, "pca_blur");
  ctlbind(ctls, "offset", ctx, &this_class::offset, &this_class::set_offset, "offset");
  ctlbind(ctls, "display", ctx, &this_class::display, &this_class::set_display, "display image type");
}


bool c_fit_jovian_ellipse_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, stdev_factor);
    SERIALIZE_PROPERTY(settings, save, *this, pca_blur);
    SERIALIZE_PROPERTY(settings, save, *this, offset);
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
  _detector.set_enable_debug_images(true);
  _detector.detect_jovian_disk(image, mask);

  switch (_display_type) {

    case display_gray_image:
      _detector.gray_image().copyTo(image);
      break;

    case display_detected_planetary_disk_mask:
      image.setTo(cv::Scalar::all(1), _detector.detected_planetary_disk_mask());
      break;

    case display_final_planetary_disk_mask:
      image.setTo(cv::Scalar::all(1), _detector.final_planetary_disk_mask());
      break;

    case display_detected_planetary_disk_edge:
      image.setTo(cv::Scalar::all(1), _detector.detected_planetary_disk_edge());
      break;

//    case display_detected_ellipseAMS:
//      rotatedRectange(image, detector_.ellipseAMS(), CV_RGB(0, 1, 0), 1);
//      cv::ellipse(image, detector_.ellipseAMS(), CV_RGB(0, 0, 1), 1);
//      break;

    case display_pca_gx:
      _detector.pca_gx().copyTo(image);
      break;

    case display_pca_gy:
      _detector.pca_gy().copyTo(image);
      break;

//    case display_final_planetary_disk_ellipse:
//      if( image.channels() == 1 ) {
//        cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
//      }
//      image.setTo(cv::Scalar::all(1), detector_.final_planetary_disk_mask());
//      rotatedRectange(image, detector_.final_planetary_disk_ellipse(), CV_RGB(0, 1, 0), 1);
//      cv::ellipse(image, detector_.final_planetary_disk_ellipse(), CV_RGB(0, 0, 1), 1);
//      break;


    case display_final_ellipse_fit:
      if( image.channels() == 1 ) {
        cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
      }
      rotatedRectange(image, _detector.final_planetary_disk_ellipse(), CV_RGB(0, 1, 0), 1);
      image.setTo(cv::Scalar::all(1), _detector.detected_planetary_disk_edge());
      cv::ellipse(image, _detector.final_planetary_disk_ellipse(), CV_RGB(0, 0, 1), 1);
      break;
  }

  if( mask.needed() ) {
    mask.release();
  }

  return true;
}
