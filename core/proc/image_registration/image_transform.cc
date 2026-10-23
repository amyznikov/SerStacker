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

c_image_transform::sptr create_image_transform(const c_image_transform_params & opts)
{
  c_image_transform::sptr transform = create_image_transform(opts.motionType);
  if( transform ) {
    transform->set_reference_points(opts.Cr, opts.Cc);
    if( !opts.params.empty() ) {
      transform->set_parameters(opts.params);
    }
  }
  return transform;
}

bool get_image_transform_params(const c_image_transform::sptr & transform, c_image_transform_params & opts)
{
  if ( transform ) {
    opts.motionType = IMAGE_MOTION_UNKNOWN;
    opts.Cr = transform->destination_reference_point();
    opts.Cc = transform->source_reference_point();
    transform->parameters().copyTo(opts.params);

    if( dynamic_cast<const c_translation_image_transform*>(transform.get()) ) {
      opts.motionType = IMAGE_MOTION_TRANSLATION;
    }
    else if( dynamic_cast<const c_euclidean_image_transform*>(transform.get()) ) {
      const c_euclidean_image_transform * t = dynamic_cast<const c_euclidean_image_transform*>(transform.get());
      if( t->fix_scale() ) {
        opts.motionType = IMAGE_MOTION_EUCLIDEAN;
      }
      else {
        opts.motionType = IMAGE_MOTION_SCALED_EUCLIDEAN;
      }
    }
    else if( dynamic_cast<const c_affine_image_transform*>(transform.get()) ) {
      opts.motionType = IMAGE_MOTION_AFFINE;
    }
    else if( dynamic_cast<const c_epipolar_derotation_image_transform*>(transform.get()) ) {
      opts.motionType = IMAGE_MOTION_EPIPOLAR_DEROTATION;
    }
    else if( dynamic_cast<const c_homography_image_transform*>(transform.get()) ) {
      opts.motionType = IMAGE_MOTION_HOMOGRAPHY;
    }
    else if( dynamic_cast<const c_semi_quadratic_image_transform*>(transform.get()) ) {
      opts.motionType = IMAGE_MOTION_SEMI_QUADRATIC;
    }
    else if( dynamic_cast<const c_quadratic_image_transform*>(transform.get()) ) {
      opts.motionType = IMAGE_MOTION_QUADRATIC;
    }
    else {
      CF_ERROR("Unknown c_image_transform type specified (%s)", typeid(*transform).name());
    }

    return true;
  }

  return false;
}

bool serialize_image_transform_opts(c_config_setting settings, bool save, c_image_transform_params & opts)
{
  SERIALIZE_OPTION(settings, save, opts, motionType);
  SERIALIZE_OPTION(settings, save, opts, Cr);
  SERIALIZE_OPTION(settings, save, opts, Cc);
  SERIALIZE_OPTION(settings, save, opts, params);
  return true;
}
