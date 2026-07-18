/*
 * c_star_extractor_routine.cc
 *
 *  Created on: May 16, 2025
 *      Author: amyznikov
 */

#include "c_star_extractor_routine.h"

template<>
const c_enum_member * members_of<c_star_extractor_routine::DisplayType>()
{
  static const c_enum_member members[] = {
      { c_star_extractor_routine::DisplaySourceImage, "SourceImage", "Display Source Image" },
      { c_star_extractor_routine::DisplayDogImage, "DOG", "Display DoG" },
      { c_star_extractor_routine::DisplayDogImage }
  };

  return members;
}

void c_star_extractor_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "display", ctx(&this_class::_display_type), "");
  ctlbind(ctls, "show blobs", ctx(&this_class::_display_blobs), "");
  ctlbind(ctls, ctx(&this_class::_opts));
}

bool c_star_extractor_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display_type);
    SERIALIZE_OPTION(settings, save, *this, _display_blobs);
    serialize_simple_star_detector_options(settings, save, _opts);
    return true;
  }
  return false;
}

bool c_star_extractor_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  _detector.set_options(_opts);
  _detector.detect(image, mask);

  switch (_display_type)
  {
    case DisplaySourceImage:
      _detector.get_cc().copyTo(mask);
      break;
    case DisplayDogImage:
      _detector.get_dog().copyTo(image);
      _detector.get_cc().copyTo(mask);
      break;
  }

  if ( _display_blobs ) {

    double minv = 0, maxv = 1;
    cv::minMaxLoc(image, &minv, &maxv);
    const cv::Scalar color1 = CV_RGB(maxv * 1.01, 0, 0);

    cv::Mat display;
    if ( image.channels() == 3 ) {
      display = image.getMatRef();
    }
    else {
      cv::cvtColor(image, display, cv::COLOR_GRAY2BGR);
    }

    const auto & blobs = _detector.detected_blobs();
    for ( const auto & b : blobs ) {
      const cv::RotatedRect rrc(cv::Point2f(b.x, b.y), cv::Size2f(4 * b.a, 4 * b.b), b.theta * 180 / CV_PI);
      cv::ellipse(display, rrc, color1, 1, cv::LINE_8);
    }

    if ( image.channels() != display.channels() ) {
      image.move(display);
    }
  }

  return true;
}

