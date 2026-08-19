/*
 * c_lunar_birdview3_routine.h
 *
 *  Created on: Aug 19, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_lunar_birdview3_routine_h__
#define __c_lunar_birdview3_routine_h__

#include <core/improc/c_image_processor.h>

class c_lunar_birdview3_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_lunar_birdview3_routine,
      "lunar_birdview3", "birdview transform to moon closeup image");

  enum MapProjection {
    MapOrthographic,   // True "Birdview" (top view), maintains parallelism of rays
    MapStereographic   // Stereographic projection (preserves the angles and shape of craters)
  };

  enum ResizeMode {
    ResizeModeKeep,
    ResizeModeAdjust,
    ResizeModeCropVisible,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  cv::Point2d rpx0, rpx1, rpx2; // 3 reference points in pixel coordinates
  cv::Point2d rse0, rse1, rse2; // 3 reference points in selenographic coordinates (lon,lat) in [deg]
  double _l = 0;             // Libration in longitude [deg]
  double _b = 0;             // Libration in latitude [deg]
  MapProjection _projection = MapStereographic;
  cv::InterpolationFlags _interpolation = cv::INTER_LINEAR;
  cv::BorderTypes _borderMode = cv::BORDER_CONSTANT;
  cv::Scalar _borderValue;
};

#endif /* __c_lunar_birdview3_routine_h__ */
