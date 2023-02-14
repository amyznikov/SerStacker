/*
 * c_align_color_channels.cc
 *
 *  Created on: Jul 11, 2021
 *      Author: amyznikov
 *
 *  Use of ECC for color channels alignment in multi-channel images
 */

#include "c_align_color_channels.h"
#include "ecc_motion_model.h"
#include <core/debug.h>

void c_align_color_channels::set_motion_type(IMAGE_MOTION_TYPE motion_type)
{
  motion_type_ = motion_type;
}

IMAGE_MOTION_TYPE  c_align_color_channels::motion_type() const
{
  return motion_type_;
}

void c_align_color_channels::set_interpolation(enum ECC_INTERPOLATION_METHOD flags)
{
  interpolation_ = flags;
}

enum ECC_INTERPOLATION_METHOD c_align_color_channels::interpolation() const
{
  return interpolation_;
}

void c_align_color_channels::set_border_mode(enum ECC_BORDER_MODE border_mode)
{
  border_mode_ = border_mode;
}

enum ECC_BORDER_MODE c_align_color_channels::border_mode() const
{
  return border_mode_;
}

void c_align_color_channels::set_border_value(const cv::Scalar & v)
{
  border_value_ = v;
}

const cv::Scalar & c_align_color_channels::border_value() const
{
  return border_value_;
}


void c_align_color_channels::set_max_iterations(int v)
{
  max_iterations_ = v;
}

int c_align_color_channels::max_iterations() const
{
  return max_iterations_;
}

void c_align_color_channels::set_smooth_sigma(double v)
{
//  ecc_.set_reference_smooth_sigma(v);
//  ecc_.set_input_smooth_sigma(v);
  smooth_sigma_ = v;
}

double c_align_color_channels::smooth_sigma() const
{
  //return ecc_.reference_smooth_sigma();
  return smooth_sigma_;
}

void c_align_color_channels::set_update_step_scale(double v)
{
  update_step_scale_ = v;
}

double c_align_color_channels::update_step_scale() const
{
  return update_step_scale_;
}

void c_align_color_channels::set_eps(double v)
{
  eps_ = v;
}

double c_align_color_channels::eps() const
{
  return eps_;
}

double c_align_color_channels::computed_rho(int channel_index) const
{
  return computed_rhos_[channel_index];
}

double c_align_color_channels::computed_eps(int channel_index) const
{
  return computed_eps_[channel_index];
}

int c_align_color_channels::computed_iterations(int channel_index) const
{
  return computed_iterations_[channel_index];
}

const c_image_transform::sptr & c_align_color_channels::computed_transform(int channel_index) const
{
  return computed_transforms_[channel_index];
}

//c_ecc_forward_additive & c_align_color_channels::ecc()
//{
//  return ecc_;
//}
//
//const c_ecc_forward_additive & c_align_color_channels::ecc() const
//{
//  return ecc_;
//}
//

bool c_align_color_channels::align(int reference_channel,
    cv::InputArray src, cv::OutputArray dst,
    cv::InputArray srcmask, cv::OutputArray dstmask)
{

  computed_eps_.clear();
  computed_rhos_.clear();
  computed_transforms_.clear();
  computed_iterations_.clear();

  const int cn = src.channels();

  if ( reference_channel < 0 || reference_channel >= cn ) {
    CF_ERROR("Invalid argument: reference_channel = %d is out if range 0..%d",
        reference_channel, cn - 1);
    return false;
  }

  if ( cn < 2 ) {

    if ( dst.needed() ) {
      src.copyTo(dst);
    }

    if ( dstmask.needed() ) {
      srcmask.copyTo(dstmask);
    }

    computed_eps_.emplace_back(0);
    computed_rhos_.emplace_back(1);
    computed_iterations_.emplace_back(0);
    computed_transforms_.emplace_back(nullptr);

    return true;
  }



  if ( !srcmask.empty() ) {

    if ( srcmask.size() != src.size()  ) {
      CF_ERROR("Invalid argument: input image (%dx%d) and mask (%dx%d) sizes not match",
          src.cols(), src.rows(), srcmask.cols(), srcmask.rows());
      return false;
    }

    if ( srcmask.channels() > 1 && srcmask.channels() != src.channels()  ) {
      CF_ERROR("Invalid argument: number of input image (%d) and mask (%d) channels not match",
          src.channels(), srcmask.channels());
      return false;
    }
  }


  cv::Mat channels[cn];
  cv::Mat masks[cn];
  cv::Mat cumulative_mask;
  cv::Mat cumulative_image;

  cv::split(src.getMat(), channels);

  if ( !srcmask.empty() ) {
    if ( srcmask.channels() > 1 ) {
      cv::split(srcmask.getMat(), masks);
    }
    else {
      for ( int i = 0; i < cn; ++i ) {
        srcmask.copyTo(masks[i]);
      }
    }
  }

  computed_eps_.resize(cn);
  computed_rhos_.resize(cn);
  computed_transforms_.resize(cn);
  computed_iterations_.resize(cn);

  c_ecc_forward_additive ecc;

  ecc.set_interpolation(interpolation_);
  ecc.set_update_step_scale(update_step_scale_);
  ecc.set_reference_smooth_sigma(smooth_sigma_);
  ecc.set_input_smooth_sigma(smooth_sigma_);
  ecc.set_max_iterations(max_iterations_);
  ecc.set_max_eps(eps_);

  if( !ecc.set_reference_image(channels[reference_channel], masks[reference_channel]) ) {
    CF_ERROR("ecc.set_reference_image(reference_channel=%d) fails");
    return false;
  }

  for ( int i = 0; i < cn; ++i ) {

    computed_transforms_[i] =
        create_image_transform(motion_type_);

    if ( i == reference_channel ) {
      computed_eps_[i] = 0;
      computed_rhos_[i] = 1;
      computed_iterations_[i] = 0;

      if ( dst.needed() || dstmask.needed() ) {
        if ( masks[i].empty() ) {
          masks[i] = cv::Mat1b(src.size(), 255);
        }
      }

    }
    else {

      c_ecc_motion_model::sptr model =
          create_ecc_motion_model(computed_transforms_[i]);

      ecc.set_model(model.get());

      if( !ecc.align_to_reference(channels[i], masks[i]) ) {
        CF_ERROR("ecc.align_to_reference() %d-> %d fails: failed=%d iterations=%d rho=%g eps=%g",
            i,
            reference_channel,
            ecc.failed(),
            ecc.num_iterations(),
            ecc.rho(),
            ecc.eps());
      }

      computed_eps_[i] = ecc.eps();
      computed_rhos_[i] = ecc.rho();
      computed_iterations_[i] = ecc.num_iterations();

      if ( dst.needed() ) {

        cv::remap(channels[i], channels[i],
            ecc.current_remap(),
            cv::noArray(),
            interpolation_,
            border_mode_,
            border_value_[i]);

      }

      if( dstmask.needed() ) {

        cv::remap(masks[i].empty() ? cv::Mat1b(src.size(), 255) : masks[i],
            masks[i],
            ecc.current_remap(),
            cv::noArray(),
            cv::INTER_AREA,
            cv::BORDER_REPLICATE,
            0);

        cv::compare(masks[i], 255, masks[i], cv::CMP_GE);
      }
    }
  }

  if( dst.needed() ) {

    if( cn == 1 ) {

      cumulative_image =
          channels[0];
    }
    else {

      cv::merge(channels, cn,
          cumulative_image);
    }

    dst.move(cumulative_image);
  }

  if( dstmask.needed() ) {

    if( cn == 1 ) {

      cumulative_mask =
          masks[0];
    }
    else {

      cv::bitwise_and(masks[0], masks[1], cumulative_mask);
      for( int i = 2; i < cn; ++i ) {
        cv::bitwise_and(masks[i], cumulative_mask, cumulative_mask);
      }
    }

    dstmask.move(cumulative_mask);
  }

  return true;
}

bool c_align_color_channels::align(cv::InputArray reference_image,
    cv::InputArray src, cv::OutputArray dst,
    cv::InputArray reference_mask,
    cv::InputArray srcmask,
    cv::OutputArray dstmask)
{
  computed_eps_.clear();
  computed_rhos_.clear();
  computed_transforms_.clear();
  computed_iterations_.clear();

  if( reference_image.channels() != 1 ) {
    CF_ERROR("Invalid argument: reference_image must have only single channel");
    return false;
  }

  if( !reference_mask.empty() ) {

    if( reference_mask.channels() != 1 ) {
      CF_ERROR("Invalid argument: reference_mask must have only single channel");
      return false;
    }

    if( reference_mask.size() != reference_image.size() ) {
      CF_ERROR("Invalid argument: referencve image (%dx%d) and mask (%dx%d) sizes not match",
          reference_image.cols(), reference_image.rows(), reference_mask.cols(), reference_mask.rows());
      return false;
    }
  }

  if( !srcmask.empty() ) {

    if( srcmask.size() != src.size() ) {
      CF_ERROR("Invalid argument: input image (%dx%d) and mask (%dx%d) sizes not match",
          src.cols(), src.rows(), srcmask.cols(), srcmask.rows());
      return false;
    }

    if( srcmask.channels() > 1 && srcmask.channels() != src.channels() ) {
      CF_ERROR("Invalid argument: number of input image (%d) and mask (%d) channels not match",
          src.channels(), srcmask.channels());
      return false;
    }
  }

  const int cn = src.channels();

  cv::Mat channels[cn];
  cv::Mat masks[cn];
  cv::Mat cumulative_mask;
  cv::Mat cumulative_image;

  cv::split(src.getMat(), channels);

  if( !srcmask.empty() ) {
    if( srcmask.channels() > 1 ) {
      cv::split(srcmask.getMat(), masks);
    }
    else {
      for( int i = 0; i < cn; ++i ) {
        srcmask.copyTo(masks[i]);
      }
    }
  }

  computed_eps_.resize(cn);
  computed_rhos_.resize(cn);
  computed_transforms_.resize(cn);
  computed_iterations_.resize(cn);


  c_ecc_forward_additive ecc;

  ecc.set_interpolation(interpolation_);
  ecc.set_update_step_scale(update_step_scale_);
  ecc.set_reference_smooth_sigma(smooth_sigma_);
  ecc.set_input_smooth_sigma(smooth_sigma_);
  ecc.set_max_iterations(max_iterations_);
  ecc.set_max_eps(eps_);

  if( !ecc.set_reference_image(reference_image, reference_mask) ) {
    CF_ERROR("ecc.set_reference_image() fails");
    return false;
  }


  for( int i = 0; i < cn; ++i ) {

    c_ecc_motion_model::sptr model =
        create_ecc_motion_model(computed_transforms_[i] =
            create_image_transform(motion_type_));

    ecc.set_model(model.get());

    if( ! ecc.align_to_reference( channels[i], masks[i]) ) {
      CF_ERROR("ecc.align_to_reference() fails: failed=%d iterations=%d rho=%g eps=%g",
          i,
          ecc.failed(),
          ecc.num_iterations(),
          ecc.rho(),
          ecc.eps());
    }

    computed_eps_[i] = ecc.eps();
    computed_rhos_[i] = ecc.rho();
    computed_iterations_[i] = ecc.num_iterations();

    if( dst.needed() ) {

      cv::remap(channels[i], channels[i],
          ecc.current_remap(),
          cv::noArray(),
          interpolation_,
          border_mode_,
          border_value_[i]);

    }

    if( dstmask.needed() ) {

      cv::remap(masks[i].empty() ? cv::Mat1b(src.size(), 255) : masks[i],
          masks[i],
          ecc.current_remap(),
          cv::noArray(),
          cv::INTER_AREA,
          cv::BORDER_CONSTANT,
          0);

      cv::compare(masks[i], 255, masks[i],
          cv::CMP_EQ);
    }
  }

  if( dst.needed() ) {

    if( cn == 1 ) {

      cumulative_image =
          channels[0];
    }
    else {
      cv::merge(channels, cn,
          cumulative_image);
    }

    dst.move(cumulative_image);
  }

  if( dstmask.needed() ) {

    if( cn == 1 ) {

      cumulative_mask =
          masks[0];
    }
    else {

      cv::bitwise_and(masks[0], masks[1], cumulative_mask);
      for( int i = 2; i < cn; ++i ) {
        cv::bitwise_and(masks[i], cumulative_mask, cumulative_mask);
      }
    }

    dstmask.move(cumulative_mask);
  }

  return true;
}
