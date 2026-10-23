/*
 * image_transform.h
 *
 *  Created on: Feb 12, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __image_transform_h__
#define __image_transform_h__

#include "c_image_transform.h"
#include "estimate_image_transform.h"

enum IMAGE_MOTION_TYPE {
  IMAGE_MOTION_UNKNOWN = -1,
  IMAGE_MOTION_TRANSLATION = 0,
  IMAGE_MOTION_EUCLIDEAN,
  IMAGE_MOTION_SCALED_EUCLIDEAN,
  IMAGE_MOTION_AFFINE,
  IMAGE_MOTION_HOMOGRAPHY,
  IMAGE_MOTION_SEMI_QUADRATIC,
  IMAGE_MOTION_QUADRATIC,
  IMAGE_MOTION_EPIPOLAR_DEROTATION,
  IMAGE_MOTION_PRH,
};

struct c_image_transform_params
{
  enum IMAGE_MOTION_TYPE motionType = IMAGE_MOTION_UNKNOWN;
  cv::Vec2f Cr, Cc;
  std::vector<float> params;
};

c_image_transform::sptr create_image_transform(IMAGE_MOTION_TYPE type);
c_image_transform::sptr create_image_transform(const c_image_transform_params & opts);
bool get_image_transform_params(const c_image_transform::sptr & transform, c_image_transform_params & opts);
bool serialize_image_transform_opts(c_config_setting settings, bool save, c_image_transform_params & opts);



#endif /* __image_transform_h__ */
