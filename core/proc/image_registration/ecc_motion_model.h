/*
 * ecc_motion_model.h
 *
 *  Created on: Feb 12, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __ecc_motion_model_h__
#define __ecc_motion_model_h__

#include "c_translation_ecc_motion_model.h"
#include "c_euclidean_ecc_motion_model.h"
#include "c_affine_ecc_motion_model.h"
#include "c_homography_ecc_motion_model.h"
#include "c_quadratic_ecc_motion_model.h"

c_ecc_motion_model::sptr create_ecc_motion_model(c_image_transform * transform);


inline c_ecc_motion_model::sptr create_ecc_motion_model(const c_image_transform::sptr & transform)
{
  return create_ecc_motion_model(transform.get());
}


#endif /* __ecc_motion_model_h__ */
