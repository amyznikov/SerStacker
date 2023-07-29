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
};

c_image_transform::sptr create_image_transform(IMAGE_MOTION_TYPE type);

#endif /* __image_transform_h__ */
