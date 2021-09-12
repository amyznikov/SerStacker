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

  if ( !derotation_.compute(src, srcmask) ) {
    CF_ERROR("derotation_.compute() fails");
    return false;
  }

  const cv::Mat2f & rotation_remap =
      derotation_.current_rotation_remap();

  rotation_remap.copyTo(base::current_remap_(derotation_.reference_boundig_box()),
      derotation_.current_binary_rotation_mask());

  if ( (dst.needed() || dstmask.needed()) && !remap(src, dst, srcmask, dstmask) ) {
    CF_ERROR(" c_jovian_rotation_registration::remap() fails");
    return false;
  }

  return true;
}


bool c_jovian_rotation_registration::custom_remap(const cv::Mat2f & rmap,
    cv::InputArray src, cv::OutputArray dst,
    cv::InputArray src_mask, cv::OutputArray dst_mask,
    int interpolation_flags, int border_mode,
    const cv::Scalar & border_value) const
{
  bool fOk = base::custom_remap(rmap,
      src, dst,
      src_mask, dst_mask,
      interpolation_flags,
      border_mode,
      border_value);

  if ( !fOk ) {
    CF_ERROR(" c_jovian_rotation_registration:  base::custom_remap() fails fails");
    return false;
  }

  if ( dst_mask.needed() ) {

    const cv::Mat & dstmask = dst_mask.getMatRef();

    cv::Mat1f combined_mask(dstmask.size(), 1.0);

    combined_mask(derotation_.reference_boundig_box()).setTo(0,
      derotation_.reference_ellipse_mask());

    derotation_.current_rotation_mask().copyTo(
        combined_mask(derotation_.reference_boundig_box()),
        derotation_.current_binary_rotation_mask());

    cv::GaussianBlur(combined_mask, combined_mask, cv::Size(), 3);

    if ( dstmask.depth() == CV_8U ) {
      combined_mask.setTo(0, ~dstmask);
    }
    else {
      cv::multiply(dstmask, combined_mask, combined_mask, 1.0, combined_mask.depth());
    }

    dst_mask.move(combined_mask);
  }

  return true;
}
