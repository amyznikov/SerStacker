/*
 * c_find_chessboard_corners_routine.cc
 *
 *  Created on: Feb 25, 2023
 *      Author: amyznikov
 */

#include "c_find_chessboard_corners_routine.h"
#include <core/ssprintf.h>

template<>
const c_enum_member* members_of<c_find_chessboard_corners_routine::DisplayType>()
{
  static const c_enum_member members[] = {
      { c_find_chessboard_corners_routine::DisplayCorners, "Corners", "" },
      { c_find_chessboard_corners_routine::DisplayOtsuImage, "OtsuImage", "" },
      { c_find_chessboard_corners_routine::DisplayHarrisImage, "HarrisImage", "" },
      { c_find_chessboard_corners_routine::DisplayHarrisThresholdImage, "HarrisThresholdImage", "" },
      { c_find_chessboard_corners_routine::DisplayHHarrisImage, "HHarrisImage", "" },
      { c_find_chessboard_corners_routine::DisplayVHarrisImage, "VHarrisImage", "" },
      { c_find_chessboard_corners_routine::DisplayCorners }
  };

  return members;
}


bool c_find_chessboard_corners_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display_type);
    SERIALIZE_OPTION(settings, save, *this, _stereo);
    SERIALIZE_OPTION(settings, save, *this, _options);
    return true;
  }

  return false;
}

void c_find_chessboard_corners_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "stereo", ctx(&this_class::_stereo), "Set to true for horizontal layout stereo frame");
  ctlbind(ctls, "display_type", ctx(&this_class::_display_type), "Display type");
  ctlbind(ctls, ctx(&this_class::_options));

  ctlbind_menu_button(ctls, "Options...", ctx);
    ctlbind_command_button(ctls, "Copy config to clipboard", ctx,
        std::function([](this_class * _ths) {
          ctlbind_copy_config_to_clipboard("c_chessboard_corners_detection_options", _ths->_options);
          return false;
        }));
    ctlbind_command_button(ctls, "Paste config from clipboard", ctx,
        std::function([](this_class * _ths) {
          return ctlbind_paste_config_from_clipboard("c_chessboard_corners_detection_options", &_ths->_options);
        }));
}

bool c_find_chessboard_corners_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat displayImage;
  std::vector<cv::Point2f> corners;
  cv::Size size;

  if( (size = _options.chessboard_size).empty() || _display_type != DisplayCorners ) {

    cv::Mat * otsu_image = nullptr;
    cv::Mat * harris_image = nullptr;
    cv::Mat * harris_threshold_image = nullptr;
    cv::Mat * hharris_image = nullptr;
    cv::Mat * vharris_image = nullptr;

    switch (_display_type) {
      case DisplayOtsuImage:
        otsu_image = &displayImage;
        break;
      case DisplayHarrisImage:
        harris_image = &displayImage;
        break;
      case DisplayHarrisThresholdImage:
        harris_threshold_image = &displayImage;
        break;
      case DisplayHHarrisImage:
        hharris_image = &displayImage;
        break;
      case DisplayVHarrisImage:
        vharris_image = &displayImage;
        break;
      default:
        break;
    }

    size =
        estimate_chessboard_size(image,
            otsu_image,
            harris_image,
            harris_threshold_image,
            hharris_image,
            vharris_image);

    CF_DEBUG("estimate_chessboard_size: %dx%d", size.width, size.height);

    if ( size.empty() ) {
      CF_ERROR("estimate_chessboard_size() fails");
    }
  }

  if( _display_type != DisplayCorners ) {
    displayImage.copyTo(image);
    return true;
  }

  if ( size.width < 2 || size.height < 2 ) {
    return true;
  }

  if( !_stereo ) {

    if( find_chessboard_corners(image, size, corners, _options) ) {

      if( image.channels() == 1 ) {
        cv::cvtColor(image, image,
            cv::COLOR_GRAY2BGR);
      }

      cv::drawChessboardCorners(image,
          size,
          corners,
          true);
    }

  }
  else {

    if( image.channels() == 1 ) {
      cv::cvtColor(image, image,
          cv::COLOR_GRAY2BGR);
    }

    cv::Mat img =
        image.getMat();

    const cv::Rect rc[2] = {
        cv::Rect(0, 0, img.cols / 2, img.rows),
        cv::Rect(img.cols / 2, 0, img.cols / 2, img.rows)
    };

    for ( int i = 0; i < 2; ++i ) {

      if( find_chessboard_corners(img(rc[i]), size, corners, _options) ) {

        cv::drawChessboardCorners(img(rc[i]),
            size,
            corners,
            true);
      }
    }
  }

  return true;
}

