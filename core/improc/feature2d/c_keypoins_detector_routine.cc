/*
 * c_keypoins_detector_routine.cc
 *
 *  Created on: Aug 9, 2023
 *      Author: amyznikov
 */

#include "c_keypoins_detector_routine.h"
#include <core/feature2d/feature2d_settings.h>
#include <core/proc/pose.h>
#include <core/proc/pixtype.h>
#include <core/proc/normalize.h>
#include <core/debug.h>

template<>
const c_enum_member * members_of<c_keypoins_detector_routine::DisplayType>()
{
  static const c_enum_member members[] = {
    {c_keypoins_detector_routine::DisplayRichKeypoints, "RichKeypoints", "Display Rich Keypoints"},
    {c_keypoins_detector_routine::DisplayWhiteCircles, "WhiteCircles", "Display White Circles"},
    {c_keypoins_detector_routine::DisplayRichKeypoints}
  };

  return members;
}

void c_keypoins_detector_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "display", ctx(&this_class::_display_type), "");
  ctlbind(ctls, "normalize_display", ctx(&this_class::_normalize_display), "");
  ctlbind(ctls, "black_background", ctx(&this_class::_black_background), "");
  ctlbind(ctls, "octave", ctx(&this_class::_octave), "Draw key points from selected octave only");
  ctlbind(ctls, "Options", ctx(&this_class:: _opts), "Options for feature2D detector");
}

bool c_keypoins_detector_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _opts);
    SERIALIZE_OPTION(settings, save, *this, _octave);
    SERIALIZE_OPTION(settings, save, *this, _display_type);
    SERIALIZE_OPTION(settings, save, *this, _black_background);
    SERIALIZE_OPTION(settings, save, *this, _normalize_display);
    return true;
  }
  return false;
}

void c_keypoins_detector_routine::parameter_changed()
{
  _keypoints_detector.reset();
}

bool c_keypoins_detector_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  try {

    if( image.needed() && !image.empty() ) {

      if( !_keypoints_detector && !(_keypoints_detector = create_sparse_feature_detector(_opts)) ) {
        CF_ERROR("create_sparse_feature_detector() fails");
        return false;
      }

      _keypoints.clear();
      _keypoints_detector->detect(image, _keypoints, mask);

      if( _opts.max_keypoints > 0 && _keypoints.size() > _opts.max_keypoints ) {

        std::sort(_keypoints.begin(), _keypoints.end(),
            [](const cv::KeyPoint & prev, const cv::KeyPoint & next) -> bool {
              return prev.response > next.response;
            });

        _keypoints.erase(_keypoints.begin() + _opts.max_keypoints, _keypoints.end());
      }

      if( _octave >= 0 ) {

        std::vector<cv::KeyPoint> tmp;

        std::copy_if(_keypoints.begin(), _keypoints.end(),  std::back_inserter(tmp),
            [octave = this->_octave](const cv::KeyPoint & p) {
              return p.octave == octave;
            });

        _keypoints = std::move(tmp);
      }



      if( _black_background ) {
        _display.create(image.size(), CV_MAKETYPE(image.depth(), 3));
        _display.setTo(cv::Scalar::all(0));
      }
      else if( image.channels() == 3 ) {
        image.copyTo(_display);
      }
      else {
        cv::cvtColor(image, _display, cv::COLOR_GRAY2BGR);
      }

      if( _display.depth() != CV_8U ) {
        if( _normalize_display ) {
          normalize_meanStdDev(_display, 7, 0, 255, cv::noArray(), CV_8U);
        }
        else {
          double scale = 1, offset = 0;
          getScaleOffset(_display.depth(), CV_8U, &scale, &offset);
          _display.convertTo(_display, CV_8U, scale, offset);
        }
      }

      switch (_display_type) {
        case DisplayWhiteCircles: {
          const cv::Scalar color = cv::Scalar::all(255);
          for ( const auto & kp : _keypoints ) {
            cv::circle(_display, kp.pt, kp.size, color, 1, cv::LINE_8);
          }
          break;
        }

        case DisplayRichKeypoints:
        default:
          cv::drawKeypoints(_display, _keypoints, _display,
              cv::Scalar::all(-1),
              cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
          break;
      }

      image.move(_display);
    }

    return true;
  }

  catch( const std::exception & e ) {
    CF_ERROR("keypoints_detector_->detect() fails: %s", e.what());
  }
  catch( ... ) {
    CF_ERROR("UNknown exception in c_keypoins_detector_routine::process()");
  }

  return false;
}
