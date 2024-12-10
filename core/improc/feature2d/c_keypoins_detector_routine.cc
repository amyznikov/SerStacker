/*
 * c_keypoins_detector_routine.cc
 *
 *  Created on: Aug 9, 2023
 *      Author: amyznikov
 */

#include "c_keypoins_detector_routine.h"
#include <core/feature2d/feature2d_settings.h>

void c_keypoins_detector_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_SPARSE_FEATURE_DETECTOR_CTRL(ctls, options, "Options", "Options for feature2D detector");
  BIND_PCTRL(ctls, octave, "Draw key points from selected octave only");
  BIND_PCTRL(ctls, black_background, "");
}

bool c_keypoins_detector_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _options);
    SERIALIZE_PROPERTY(settings, save, *this, octave);
    SERIALIZE_PROPERTY(settings, save, *this, black_background);
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

      if( !_keypoints_detector && !(_keypoints_detector = create_sparse_feature_detector(_options)) ) {
        CF_ERROR("create_sparse_feature_detector() fails");
        return false;
      }

      _keypoints.clear();

      _keypoints_detector->detect(image, _keypoints, mask);

      if( _options.max_keypoints > 0 && _keypoints.size() > _options.max_keypoints ) {

        std::sort(_keypoints.begin(), _keypoints.end(),
            [](const cv::KeyPoint & prev, const cv::KeyPoint & next) -> bool {
              return prev.response > next.response;
            });

        _keypoints.erase(_keypoints.begin() + _options.max_keypoints,
            _keypoints.end());
      }

      if( _octave >= 0 ) {

        std::vector<cv::KeyPoint> tmp;

        const int octave =
            this->_octave;

        std::copy_if(_keypoints.begin(), _keypoints.end(),  std::back_inserter(tmp),
            [octave](const cv::KeyPoint & p) {
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


      cv::drawKeypoints(_display, _keypoints, _display,
          cv::Scalar::all(-1),
          cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

      _display.copyTo(image);
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
