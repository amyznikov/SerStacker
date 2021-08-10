/*
 * align_channels.cc
 *
 *  Created on: Jul 11, 2021
 *      Author: amyznikov
 */

#include "align_channels.h"
#include <core/debug.h>

void c_align_color_channels::set_motion_type(ECC_MOTION_TYPE motion_type)
{
  ecc_.set_motion_type(motion_type);
}

ECC_MOTION_TYPE c_align_color_channels::motion_type() const
{
  return (ECC_MOTION_TYPE )ecc_.motion_type();
}

void c_align_color_channels::set_interpolation(cv::InterpolationFlags flags)
{
  ecc_.set_image_interpolation_flags(flags);
}

cv::InterpolationFlags c_align_color_channels::interpolation() const
{
  return (cv::InterpolationFlags )ecc_.image_interpolation_flags();
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
  ecc_.set_max_iterations(v);
}

int c_align_color_channels::max_iterations() const
{
  return ecc_.max_iterations();
}

void c_align_color_channels::set_smooth_sigma(double v)
{
  ecc_.set_reference_smooth_sigma(v);
  ecc_.set_input_smooth_sigma(v);
}

double c_align_color_channels::smooth_sigma() const
{
  return ecc_.reference_smooth_sigma();
}

void c_align_color_channels::set_update_step_scale(double v)
{
  ecc_.set_update_step_scale(v);
}

double c_align_color_channels::update_step_scale() const
{
  return ecc_.update_step_scale();
}

void c_align_color_channels::set_eps(double v)
{
  ecc_.set_eps(v);
}

double c_align_color_channels::eps() const
{
  return ecc_.eps();
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


const cv::Mat & c_align_color_channels::computed_transform(int channel_index) const
{
  return computed_transforms_[channel_index];
}

c_ecc_forward_additive & c_align_color_channels::ecc()
{
  return ecc_;
}

const c_ecc_forward_additive & c_align_color_channels::ecc() const
{
  return ecc_;
}

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
    computed_transforms_.emplace_back(createEyeTransform(motion_type()));

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
  ecc_.set_reference_image(channels[reference_channel],
      masks[reference_channel]);

  for ( int i = 0; i < cn; ++i ) {

    if ( i == reference_channel ) {
      computed_eps_[i] = 0;
      computed_rhos_[i] = 1;
      computed_iterations_[i] = 0;
      computed_transforms_[i] = createEyeTransform(motion_type());

      if ( dst.needed() || dstmask.needed() ) {
        if ( masks[i].empty() ) {
          masks[i] = cv::Mat1b(src.size(), 255);
        }
      }

    }
    else {

      bool fOk;

      fOk = ecc_.align_to_reference(
          channels[i],
          computed_transforms_[i],
          masks[i]);

      computed_eps_[i] = ecc_.current_eps();
      computed_rhos_[i] = ecc_.rho();
      computed_iterations_[i] = ecc_.num_iterations();

      if ( !fOk ) {
        CF_ERROR("ecc_.align_to_reference() %d-> %d fails: nbiters=%d rho=%g eps=%g",
            i,
            reference_channel,
            ecc_.num_iterations(),
            ecc_.rho(),
            ecc_.current_eps());
      }

      if ( dst.needed() ) {

        cv::remap(channels[i], channels[i], ecc_.current_remap(), cv::noArray(),
            interpolation(),
            cv::BORDER_REPLICATE,
            border_value_[i]);

      }

      if ( dst.needed() || dstmask.needed() ) {
        if ( masks[i].empty() ) {
          cv::remap(cv::Mat1b(src.size(), 255), masks[i], ecc_.current_remap(), cv::noArray(),
              cv::INTER_AREA,
              cv::BORDER_REPLICATE,
              0);
        }
        else {
          cv::remap(masks[i], masks[i], ecc_.current_remap(), cv::noArray(),
              cv::INTER_AREA,
              cv::BORDER_REPLICATE,
              0);
        }

        cv::compare(masks[i], 200, masks[i],
            cv::CMP_GE);

      }
    }
  }

  if ( dst.needed() || dstmask.needed() ) {

    if ( cn == 1 ) {
      cumulative_mask = masks[0];
    }
    else {
      cv::bitwise_and(masks[0], masks[1], cumulative_mask);
      for ( int i = 2; i < cn; ++i ) {
        cv::bitwise_and(masks[i], cumulative_mask, cumulative_mask);
      }
    }

    if ( dst.needed() ) {

      if ( cn == 1 ) {
        cumulative_image = channels[0];
      }
      else {
        cv::merge(channels, cn, cumulative_image);
      }

      //      cumulative_image.setTo(border_value_,
      //          ~cumulative_mask);

      dst.move(cumulative_image);
    }

    if ( dstmask.needed() ) {
      dstmask.move(cumulative_mask);
    }
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

  if ( reference_image.channels() != 1 ) {
    CF_ERROR("Invalid argument: reference_image must have only single channel");
    return false;
  }

  if ( !reference_mask.empty() ) {

    if ( reference_mask.channels() != 1 ) {
      CF_ERROR("Invalid argument: reference_mask must have only single channel");
      return false;
    }

    if ( reference_mask.size() != reference_image.size() ) {
      CF_ERROR("Invalid argument: referencve image (%dx%d) and mask (%dx%d) sizes not match",
          reference_image.cols(), reference_image.rows(), reference_mask.cols(), reference_mask.rows());
      return false;
    }
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


  const int cn = src.channels();

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
  ecc_.set_reference_image(reference_image,
      reference_mask);

  for ( int i = 0; i < cn; ++i ) {

    bool fOk = ecc_.align_to_reference(
        channels[i],
        computed_transforms_[i],
        masks[i]);

    computed_eps_[i] = ecc_.current_eps();
    computed_rhos_[i] = ecc_.rho();
    computed_iterations_[i] = ecc_.num_iterations();

    if ( !fOk ) {
      CF_ERROR("ecc_.align_to_reference() channel %d fails: nbiters=%d rho=%g eps=%g",
          i,
          ecc_.num_iterations(),
          ecc_.rho(),
          ecc_.current_eps());
    }

    if ( dst.needed() ) {

      cv::remap(channels[i], channels[i], ecc_.current_remap(), cv::noArray(),
          interpolation(),
          cv::BORDER_CONSTANT,
          border_value_[i]);

    }

    if ( dst.needed() || dstmask.needed() ) {
      if ( masks[i].empty() ) {
        cv::remap(cv::Mat1b(src.size(), 255), masks[i], ecc_.current_remap(), cv::noArray(),
            cv::INTER_AREA,
            cv::BORDER_CONSTANT,
            0);
      }
      else {
        cv::remap(masks[i], masks[i], ecc_.current_remap(), cv::noArray(),
            cv::INTER_AREA,
            cv::BORDER_CONSTANT,
            0);
      }

      cv::compare(masks[i], 255, masks[i],
          cv::CMP_EQ);

    }
  }

  if ( dst.needed() || dstmask.needed() ) {

    if ( cn == 1 ) {
      cumulative_mask = masks[0];
    }
    else {
      cv::bitwise_and(masks[0], masks[1], cumulative_mask);
      for ( int i = 2; i < cn; ++i ) {
        cv::bitwise_and(masks[i], cumulative_mask, cumulative_mask);
      }
    }

    if ( dst.needed() ) {

      if ( cn == 1 ) {
        cumulative_image = channels[0];
      }
      else {
        cv::merge(channels, cn, cumulative_image);
      }

      cumulative_image.setTo(border_value_,
          ~cumulative_mask);

      dst.move(cumulative_image);
    }

    if ( dstmask.needed() ) {
      dstmask.move(cumulative_mask);
    }
  }


  return true;

}
