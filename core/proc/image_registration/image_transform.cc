/*
 * image_transform.cc
 *
 *  Created on: Feb 12, 2023
 *      Author: amyznikov
 */
#include "image_transform.h"
#include <core/ssprintf.h>
#include <core/debug.h>


template<>
const c_enum_member * members_of<IMAGE_MOTION_TYPE>()
{
  static const c_enum_member members[] = {
      { IMAGE_MOTION_TRANSLATION, "TRANSLATION", "translation" },
      { IMAGE_MOTION_EUCLIDEAN, "EUCLIDEAN", "Rotation + Translation" },
      { IMAGE_MOTION_SCALED_EUCLIDEAN, "SCALED_EUCLIDEAN", "Scale * Rotation + Translation" },
      { IMAGE_MOTION_AFFINE, "AFFINE", "AFFINE" },
      { IMAGE_MOTION_HOMOGRAPHY, "HOMOGRAPHY", "HOMOGRAPHY" },
      { IMAGE_MOTION_SEMI_QUADRATIC, "SEMI_QUADRATIC", "SEMI_QUADRATIC" },
      { IMAGE_MOTION_QUADRATIC, "QUADRATIC", "QUADRATIC" },
      { IMAGE_MOTION_EPIPOLAR_DEROTATION, "EPIPOLAR_DEROTATION", "EPIPOLAR_DEROTATION" },
      { IMAGE_MOTION_PRH, "PRH", "Pure Rotation Homography" },

      { IMAGE_MOTION_UNKNOWN},
  };

  return members;
}

c_image_transform::sptr create_image_transform(IMAGE_MOTION_TYPE type)
{
  switch (type) {
    case IMAGE_MOTION_EUCLIDEAN: {
      c_euclidean_image_transform::sptr t(new c_euclidean_image_transform());
      t->set_fix_scale(true);
      return t;
    }
    case IMAGE_MOTION_SCALED_EUCLIDEAN:
      return c_image_transform::sptr(new c_euclidean_image_transform());
    case IMAGE_MOTION_TRANSLATION:
      return c_image_transform::sptr(new c_translation_image_transform());
    case IMAGE_MOTION_AFFINE:
      return c_image_transform::sptr(new c_affine_image_transform());
    case IMAGE_MOTION_HOMOGRAPHY:
      return c_image_transform::sptr(new c_homography_image_transform());
    case IMAGE_MOTION_SEMI_QUADRATIC:
      return c_image_transform::sptr(new c_semi_quadratic_image_transform());
    case IMAGE_MOTION_QUADRATIC:
      return c_image_transform::sptr(new c_quadratic_image_transform());
    case IMAGE_MOTION_EPIPOLAR_DEROTATION:
      return c_image_transform::sptr(new c_epipolar_derotation_image_transform());
    default:
      CF_ERROR("Unknown image transform type specified: %d", type);
      break;
  }

  return nullptr;
}

