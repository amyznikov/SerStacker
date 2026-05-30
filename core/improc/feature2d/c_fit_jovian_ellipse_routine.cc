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
      { c_fit_jovian_ellipse_routine::display_final_ellipse_fit, "final_ellipse_fit", },
      { c_fit_jovian_ellipse_routine::display_gray_image, "gray_image", },
      { c_fit_jovian_ellipse_routine::display_normalized_image, "normalized_image", },
      { c_fit_jovian_ellipse_routine::display_g, "g", },
      { c_fit_jovian_ellipse_routine::display_gx, "gx", },
      { c_fit_jovian_ellipse_routine::display_gy, "gy", },
      { c_fit_jovian_ellipse_routine::display_detected_planetary_disk_mask, "detected_planetary_disk_mask", },
      { c_fit_jovian_ellipse_routine::display_detected_planetary_disk_edge, "detected_planetary_disk_edge", },
      { c_fit_jovian_ellipse_routine::display_final_planetary_disk_mask, "final_planetary_disk_mask", },
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

void c_fit_jovian_ellipse_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "display", ctx, &this_class::display, &this_class::set_display, "display image type");
  ctlbind(ctls, ctx(&this_class::_opts));
}

bool c_fit_jovian_ellipse_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, display);
    serialize_base_jovian_ellipse_detector_options(settings, save, _opts);
    return true;
  }
  return false;
}

static void drawRotatedRect(cv::InputOutputArray image, const cv::RotatedRect & rc,
    const cv::Scalar color, int thickness = 1, int lineType = cv::LINE_8, int shift = 0)
{

  if ( 1 ) {
    cv::rectangle(image, ellipse_bounding_box(rc),
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
  _detector.set_options(_opts);
  _detector.detect_jovian_ellipse(image, mask);

  switch (_display_type) {

    case display_gray_image:
      _detector.grayscale_image().copyTo(image);
      _detector.detected_planetary_disk_mask().copyTo(mask);
      break;

    case display_detected_planetary_disk_mask:
      image.setTo(cv::Scalar::all(1), _detector.detected_planetary_disk_mask());
      mask.release();
      break;

    case display_final_planetary_disk_mask:
      image.setTo(cv::Scalar::all(1), _detector.final_planetary_disk_mask());
      mask.release();
      break;

    case display_detected_planetary_disk_edge:
      image.setTo(cv::Scalar::all(1), _detector.detected_planetary_disk_edge());
      mask.release();
      break;

    case display_normalized_image:
      _detector.normalized_image().copyTo(image);
      _detector.gradient_mask().copyTo(mask);
      break;

    case display_gx:
      _detector.gx_image().copyTo(image);
      _detector.gradient_mask().copyTo(mask);
      break;

    case display_gy:
      _detector.gy_image().copyTo(image);
      _detector.gradient_mask().copyTo(mask);
      break;

    case display_g:
      _detector.g_image().copyTo(image);
      _detector.gradient_mask().copyTo(mask);
      break;

    case display_final_ellipse_fit:
      if( image.channels() == 1 ) {
        cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
      }
      drawRotatedRect(image, _detector.final_planetary_disk_ellipse(), CV_RGB(0, 1, 0), 1);
      image.setTo(cv::Scalar::all(1), _detector.detected_planetary_disk_edge());
      cv::ellipse(image, _detector.final_planetary_disk_ellipse(), CV_RGB(0, 0, 1), 1);
      mask.release();
      break;
  }

  return true;
}
