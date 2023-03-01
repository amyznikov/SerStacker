/*
 * c_find_chessboard_corners_routine.h
 *
 *  Created on: Feb 25, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_find_chessboard_corners_routine_h__
#define __c_find_chessboard_corners_routine_h__

#include "c_image_processor.h"
#include <core/proc/chessboard/chessboard_detection.h>

class c_find_chessboard_corners_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_find_chessboard_corners_routine,
      "find_chessboard_corners",
      "Detect chessboard corners on image");

  enum DisplayType {
    DisplayCorners = 0,
    DisplayOtsuImage,
    DisplayHarrisImage,
    DisplayHarrisThresholdImage,
    DisplayHHarrisImage,
    DisplayVHarrisImage
  };

  void set_boardSize(const cv::Size & v)
  {
    boardSize_ = v;
  }

  const cv::Size & boardSize() const
  {
    return boardSize_;
  }

  void set_method(enum FindChessboardCornersMethod v)
  {
    options_.method = v;
  }

  enum FindChessboardCornersMethod method() const
  {
    return options_.method;
  }

  void set_findChessboardCorners_max_scales(int v)
  {
    options_.findChessboardCorners.max_scales = v;
  }

  int findChessboardCorners_max_scales() const
  {
    return options_.findChessboardCorners.max_scales;
  }

  void set_findChessboardCorners_flags(int v)
  {
    options_.findChessboardCorners.flags = v;
  }

  int findChessboardCorners_flags() const
  {
    return options_.findChessboardCorners.flags;
  }


  void set_findChessboardCornersSB_max_scales(int v)
  {
    options_.findChessboardCornersSB.max_scales = v;
  }

  int findChessboardCornersSB_max_scales() const
  {
    return options_.findChessboardCornersSB.max_scales;
  }

  void set_findChessboardCornersSB_flags(int v)
  {
    options_.findChessboardCornersSB.flags = v;
  }

  int findChessboardCornersSB_flags() const
  {
    return options_.findChessboardCornersSB.flags;
  }

  void set_flags(int v)
  {
    options_.findChessboardCorners.flags = v;
    options_.findChessboardCornersSB.flags = v;
  }

  int flags() const
  {
    return options_.findChessboardCorners.flags;
  }

  void set_winSize(const cv::Size &v)
  {
    options_.cornerSubPix.winSize = v;
  }

  const cv::Size & winSize() const
  {
    return options_.cornerSubPix.winSize;
  }

  void set_zeroZone(const cv::Size & v)
  {
    options_.cornerSubPix.zeroZone = v;
  }

  const cv::Size & zeroZone() const
  {
    return options_.cornerSubPix.zeroZone;
  }

  void set_max_iterations(int v)
  {
    if( (options_.cornerSubPix.termCriteria.maxCount = v) > 0 ) {
      options_.cornerSubPix.termCriteria.type |= cv::TermCriteria::COUNT;
    }
    else {
      options_.cornerSubPix.termCriteria.type &= ~cv::TermCriteria::COUNT;
    }
  }

  int max_iterations() const
  {
    return options_.cornerSubPix.termCriteria.maxCount;
  }

  void set_eps(double v)
  {
    if( (options_.cornerSubPix.termCriteria.epsilon = v) >= 0 ) {
      options_.cornerSubPix.termCriteria.type |= cv::TermCriteria::EPS;
    }
    else {
      options_.cornerSubPix.termCriteria.type &= ~cv::TermCriteria::EPS;
    }
  }

  double eps() const
  {
    return options_.cornerSubPix.termCriteria.epsilon;
  }

  void set_bilateralFilter_d(int v)
  {
    options_.bilateralFilter.d = v;
  }

  int bilateralFilter_d() const
  {
    return options_.bilateralFilter.d;
  }

  void set_bilateralFilter_sigmaColor(double v)
  {
    options_.bilateralFilter.sigmaColor = v;
  }

  double bilateralFilter_sigmaColor() const
  {
    return options_.bilateralFilter.sigmaColor;
  }

  void set_bilateralFilter_sigmaSpace(double v)
  {
    options_.bilateralFilter.sigmaSpace = v;
  }

  double bilateralFilter_sigmaSpace() const
  {
    return options_.bilateralFilter.sigmaSpace;
  }

  void set_display_type(enum DisplayType v)
  {
    display_type_ = v;
  }

  enum DisplayType display_type() const
  {
    return display_type_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL2(ctls, display_type, Display, "");

    ADD_IMAGE_PROCESSOR_CTRL(ctls, boardSize, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, method, "");

    ADD_IMAGE_PROCESSOR_CTRL_GROUP(ctls, "findChessboardCorners", "");
      ADD_IMAGE_PROCESSOR_CTRL2(ctls, findChessboardCorners_max_scales, max_scales, "");
      //ADD_IMAGE_PROCESSOR_CTRL2(ctls, findChessboardCorners_flags, flags, "");
      ADD_IMAGE_PROCESSOR_FLAGS_CTRL(ctls, findChessboardCorners_flags, flags, FindChessboardCornersFlags, "")
    END_IMAGE_PROCESSOR_CTRL_GROUP(ctls);

    ADD_IMAGE_PROCESSOR_CTRL_GROUP(ctls, "findChessboardCornersSB", "");
      ADD_IMAGE_PROCESSOR_CTRL2(ctls, findChessboardCornersSB_max_scales, max_scales, "");
      //ADD_IMAGE_PROCESSOR_CTRL2(ctls, findChessboardCornersSB_flags, flags, "");
      ADD_IMAGE_PROCESSOR_FLAGS_CTRL(ctls, findChessboardCornersSB_flags, flags, FindChessboardCornersSBFlags, "")
    END_IMAGE_PROCESSOR_CTRL_GROUP(ctls);

    ADD_IMAGE_PROCESSOR_CTRL_GROUP(ctls, "cornerSubPix", "");
      ADD_IMAGE_PROCESSOR_CTRL(ctls, winSize, "");
      ADD_IMAGE_PROCESSOR_CTRL(ctls, zeroZone, "");
      ADD_IMAGE_PROCESSOR_CTRL(ctls, max_iterations, "");
      ADD_IMAGE_PROCESSOR_CTRL(ctls, eps, "");
    END_IMAGE_PROCESSOR_CTRL_GROUP(ctls);

    ADD_IMAGE_PROCESSOR_CTRL_GROUP(ctls, "bilateralFilter", "");
      ADD_IMAGE_PROCESSOR_CTRL2(ctls, bilateralFilter_d, d, "");
      ADD_IMAGE_PROCESSOR_CTRL2(ctls, bilateralFilter_sigmaColor, sigmaColor, "");
      ADD_IMAGE_PROCESSOR_CTRL2(ctls, bilateralFilter_sigmaSpace, sigmaSpace, "");
    END_IMAGE_PROCESSOR_CTRL_GROUP(ctls);

  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {

      c_config_setting section;

      SERIALIZE_PROPERTY(settings, save, *this, display_type);
      SERIALIZE_PROPERTY(settings, save, *this, boardSize);

      SERIALIZE_PROPERTY(settings, save, *this, method);

      if ( (section = SERIALIZE_GROUP(settings, save, "findChessboardCorners")) ) {
        SERIALIZE_OPTION(section, save, options_.findChessboardCorners, max_scales);
        SERIALIZE_OPTION(section, save, options_.findChessboardCorners, flags);
      }

      if ( (section = SERIALIZE_GROUP(settings, save, "findChessboardCornersSB")) ) {
        SERIALIZE_OPTION(section, save, options_.findChessboardCornersSB, max_scales);
        SERIALIZE_OPTION(section, save, options_.findChessboardCornersSB, flags);
      }

      if ( (section = SERIALIZE_GROUP(settings, save, "cornerSubPix")) ) {

        SERIALIZE_OPTION(section, save, options_.cornerSubPix, winSize);
        SERIALIZE_OPTION(section, save, options_.cornerSubPix, zeroZone);
        SERIALIZE_OPTION(section, save, options_.cornerSubPix, termCriteria);
      }

      if ( (section = SERIALIZE_GROUP(settings, save, "bilateralFilter")) ) {

        SERIALIZE_OPTION(section, save, options_.bilateralFilter, d);
        SERIALIZE_OPTION(section, save, options_.bilateralFilter, sigmaColor);
        SERIALIZE_OPTION(section, save, options_.bilateralFilter, sigmaSpace);
      }

      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask)
  {
    cv::Mat displayImage;
    std::vector<cv::Point2f> corners;
    cv::Size size;

    if( (size = boardSize_).empty() || display_type_ != DisplayCorners ) {

      cv::Mat * otsu_image = nullptr;
      cv::Mat * harris_image = nullptr;
      cv::Mat * harris_threshold_image = nullptr;
      cv::Mat * hharris_image = nullptr;
      cv::Mat * vharris_image = nullptr;

      switch (display_type_) {
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

    if( display_type_ != DisplayCorners ) {
      displayImage.copyTo(image);
      return true;
    }

    if ( size.width < 2 || size.height < 2 ) {
      return true;
    }

    if( !find_chessboard_corners(image, size, corners, options_) ) {
      return true;
    }


    if ( image.channels() == 1 ) {
      cv::cvtColor(image, image,
          cv::COLOR_GRAY2BGR);
    }

    cv::drawChessboardCorners(image,
        size,
        corners,
        true);

    return true;
  }

protected:
  cv::Size boardSize_;
  c_chessboard_corners_detection_options options_;
  enum DisplayType display_type_ = DisplayCorners;
};

#endif /* __c_find_chessboard_corners_routine_h__ */
