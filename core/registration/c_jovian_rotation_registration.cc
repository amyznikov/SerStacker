/*
 * c_jovian_derotation_registration.cc
 *
 *  Created on: Sep 12, 2021
 *      Author: amyznikov
 */

#include "c_jovian_rotation_registration.h"
#include <core/debug.h>

c_jovian_rotation_registration::c_jovian_rotation_registration()
{
}

c_jovian_rotation_registration::c_jovian_rotation_registration(const c_jovian_derotation_options & opts)
  : jovian_derotation_options_(opts)
{
}

c_jovian_rotation_registration::c_jovian_rotation_registration(const c_frame_registration_base_options & base_opts,
    const c_planetary_disk_registration_options & planetary_disk_options)
  : base(base_opts, planetary_disk_options)
{
}

c_jovian_rotation_registration::c_jovian_rotation_registration(const c_frame_registration_base_options & base_opts,
    const c_planetary_disk_registration_options & planetary_disk_options,
    const c_jovian_derotation_options & jovian_derotation_options)
  : base(base_opts, planetary_disk_options), jovian_derotation_options_(jovian_derotation_options)
{
}


c_jovian_rotation_registration::ptr c_jovian_rotation_registration::create()
{
  return ptr(new c_jovian_rotation_registration());
}

// hmmm... it is questionable if these overloads are really usefull though...
c_jovian_rotation_registration::ptr c_jovian_rotation_registration::create(const c_jovian_derotation_options & opts)
{
  return ptr(new c_jovian_rotation_registration(opts));
}

c_jovian_rotation_registration::ptr c_jovian_rotation_registration::create(const c_frame_registration_base_options & base_opts,
    const c_planetary_disk_registration_options & planetary_disk_opts,
    const c_jovian_derotation_options & jovian_derotation_options)
{
  return ptr(new c_jovian_rotation_registration(base_opts, planetary_disk_opts, jovian_derotation_options));
}


const c_jovian_derotation_options & c_jovian_rotation_registration::jovian_derotation_options() const
{
  return jovian_derotation_options_;
}

c_jovian_derotation_options & c_jovian_rotation_registration::jovian_derotation_options()
{
  return jovian_derotation_options_;
}

const c_jovian_derotation & c_jovian_rotation_registration::jovian_derotation() const
{
  return derotation_;
}

bool c_jovian_rotation_registration::setup_referece_frame(cv::InputArray image, cv::InputArray mask)
{
  if ( !base::setup_referece_frame(image, mask) ) {
    CF_ERROR("base::setup_referece_frame() fails");
    return false;
  }

  if ( !derotation_.setup_reference_image(image, mask) ) {
    CF_ERROR("derotation_.setup_reference_image() fails");
    return false;
  }

  return true;
}

bool c_jovian_rotation_registration::register_frame(cv::InputArray src, cv::InputArray srcmask,
    cv::OutputArray dst, cv::OutputArray dstmask)
{
  if ( !base::register_frame(src, srcmask) ) {
    CF_ERROR("base::register_frame() fails");
    return false;
  }

  // call derotation_.compute() and update the current_remap_ here.
  // the problem to solve is that dstmask need to be 32FC1, but currently it is CV_8UC1

  return false;
}
