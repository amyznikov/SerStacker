/*
 * c_align_color_channels.cc
 *
 *  Created on: Jul 11, 2021
 *      Author: amyznikov
 *
 *  Use of ECC for color channels alignment in multi-channel images
 */

#include "c_align_color_channels.h"
#include "ecc2.h"
#include <core/debug.h>

const c_image_transform::sptr & c_align_color_channels::computed_transform(int channel_index) const
{
  return _computed_transforms[channel_index];
}

bool c_align_color_channels::align(cv::InputArray _src, cv::OutputArray _dst,
    const c_align_color_channels_options & opts, int reference_channel_index,
    cv::InputArray _srcmask, cv::OutputArray _dstmask)
{
  const int cn = _src.channels();
  if( cn < 2 ) {
    _src.copyTo(_dst);
    return true;
  }

  if( reference_channel_index < 0 || reference_channel_index >= cn ) {
    reference_channel_index = cn / 2;
  }

  cv::Mat reference_channel, reference_mask;

  cv::extractChannel(_src, reference_channel, reference_channel_index);

  if ( _srcmask.channels() == 1 ) {
    reference_mask  = _srcmask.getMat();
  }
  else {
    cv::extractChannel(_srcmask, reference_mask, reference_channel_index);
  }

  return align(_src, _dst, reference_channel, reference_mask, opts, _srcmask, _dstmask);
}


bool c_align_color_channels::align(cv::InputArray _src, cv::OutputArray _dst,
    cv::InputArray reference_image, cv::InputArray reference_mask,
    const c_align_color_channels_options & opts,
    cv::InputArray _srcmask,
    cv::OutputArray _dstmask)
{
  _computed_transforms.clear();

  if( reference_image.channels() != 1 ) {
    CF_ERROR("Invalid arg: single channel reference image is expected");
    return false;
  }

  if( !reference_mask.empty() ) {

    if( reference_mask.channels() != 1 ) {
      CF_ERROR("Invalid argument: reference_mask must have only single channel");
      return false;
    }

    if( reference_mask.size() != reference_image.size() ) {
      CF_ERROR("Invalid argument: reference image (%dx%d) and mask (%dx%d) sizes not match",
          reference_image.cols(), reference_image.rows(), reference_mask.cols(), reference_mask.rows());
      return false;
    }
  }

  if( !_srcmask.empty() ) {

    if( _srcmask.size() != _src.size() ) {
      CF_ERROR("Invalid argument: input image (%dx%d) and mask (%dx%d) sizes not match",
          _src.cols(), _src.rows(), _srcmask.cols(), _srcmask.rows());
      return false;
    }

    if( _srcmask.channels() > 1 && _srcmask.channels() != _src.channels() ) {
      CF_ERROR("Invalid argument: number of input image (%d) and mask (%d) channels not match",
          _src.channels(), _srcmask.channels());
      return false;
    }
  }

  const int cn = _src.channels();

  std::vector<cv::Mat> channels;
  std::vector<cv::Mat> masks(cn);
  cv::Mat cumulative_mask;
  cv::Mat cumulative_image;
  cv::Mat2f current_remap;

  cv::split(_src, channels);

  if( !_srcmask.empty() ) {
    if( _srcmask.channels() > 1 ) {
      cv::split(_srcmask, masks);
    }
    else {
      for( int i = 0; i < cn; ++i ) {
        masks[i] = _srcmask.getMat();
      }
    }
  }

  _computed_transforms.resize(cn);

  c_ecch ecc(opts.method);
  ecc.set_maxlevel(opts.max_level);
  ecc.set_interpolation(opts.interpolation);
  ecc.set_update_step_scale(opts.update_step_scale);
  ecc.set_reference_smooth_sigma(opts.smooth_sigma);
  ecc.set_input_smooth_sigma(opts.smooth_sigma);
  ecc.set_max_iterations(opts.max_iterations);
  ecc.set_max_eps(opts.eps);

  if( !ecc.set_reference_image(reference_image, reference_mask) ) {
    CF_ERROR("ecc.set_reference_image() fails");
    return false;
  }

  for( int i = 0; i < cn; ++i ) {

    ecc.set_image_transform((_computed_transforms[i] =
        create_image_transform(opts.motion_type)).get());

    if( !ecc.align( channels[i], masks[i]) ) {
      CF_ERROR("ecc.align_to_reference(channel=%d) fails: iterations=%d eps=%g", i,
          ecc.num_iterations(),
          ecc.eps());
    }

    ecc.image_transform()->create_remap(ecc.reference_image().size(),
        current_remap);

    if( _dst.needed() ) {
      cv::remap(channels[i], channels[i], current_remap, cv::noArray(),
          opts.interpolation,
          opts.border_mode,
          opts.border_value[i]);
    }

    if( _dstmask.needed() ) {
      cv::remap(masks[i].empty() ? cv::Mat1b(_src.size(), 255) : masks[i], masks[i],
          current_remap, cv::noArray(),
          cv::INTER_AREA,
          cv::BORDER_CONSTANT,
          0);
     cv::compare(masks[i], 255, masks[i],
          cv::CMP_EQ);
    }
  }


  if( _dst.needed() ) {
    if( cn == 1 ) {
      cumulative_image = channels[0];
    }
    else {
      cv::merge(channels, cumulative_image);
    }

    _dst.move(cumulative_image);
  }

  if( _dstmask.needed() ) {
    if( cn == 1 ) {
      cumulative_mask = masks[0];
    }
    else {
      cv::bitwise_and(masks[0], masks[1], cumulative_mask);
      for( int i = 2; i < cn; ++i ) {
        cv::bitwise_and(masks[i], cumulative_mask, cumulative_mask);
      }
    }

    _dstmask.move(cumulative_mask);
  }

  return true;
}

bool serialize_align_color_channels_options(c_config_setting section, bool save, c_align_color_channels_options & opts)
{
  SERIALIZE_OPTION(section, save, opts, method);
  SERIALIZE_OPTION(section, save, opts, motion_type);
  SERIALIZE_OPTION(section, save, opts, interpolation);
  SERIALIZE_OPTION(section, save, opts, border_mode);
  SERIALIZE_OPTION(section, save, opts, border_value);
  SERIALIZE_OPTION(section, save, opts, smooth_sigma);
  SERIALIZE_OPTION(section, save, opts, eps);
  SERIALIZE_OPTION(section, save, opts, max_iterations);
  SERIALIZE_OPTION(section, save, opts, max_level);
  SERIALIZE_OPTION(section, save, opts, update_step_scale);
  SERIALIZE_OPTION(section, save, opts, normalization_level);
  return true;
}
