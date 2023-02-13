/*
 * ecc_motion_model.cc
 *
 *  Created on: Feb 12, 2023
 *      Author: amyznikov
 */

#include "ecc_motion_model.h"
#include <core/debug.h>


c_ecc_motion_model::sptr create_ecc_motion_model(c_image_transform * transform)
{
  if( !transform ) {
    CF_ERROR("No pointer to image transform model specified");
  }
  else {

    if( c_euclidean_image_transform *t = dynamic_cast<c_euclidean_image_transform*>(transform) ) {
      return c_ecc_motion_model::sptr(new c_euclidean_ecc_motion_model(t));
    }

    if( c_affine_image_transform *t = dynamic_cast<c_affine_image_transform*>(transform) ) {
      return c_ecc_motion_model::sptr(new c_affine_ecc_motion_model(t));
    }

    if( c_homography_image_transform *t = dynamic_cast<c_homography_image_transform*>(transform) ) {
      return c_ecc_motion_model::sptr(new c_homography_ecc_motion_model(t));
    }

    if( c_quadratic_image_transform *t = dynamic_cast<c_quadratic_image_transform*>(transform) ) {
      return c_ecc_motion_model::sptr(new c_quadratic_ecc_motion_model(t));
    }

    CF_ERROR("Unknow c_image_transform type specified");
  }

  return nullptr;
}
