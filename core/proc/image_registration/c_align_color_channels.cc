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
  return _image_transforms[channel_index];
}

const std::vector<c_image_transform::sptr> & c_align_color_channels::computed_transforms() const
{
  return _image_transforms;
}

IMAGE_MOTION_TYPE c_align_color_channels::estimated_motion_type() const
{
  return _estimated_motion_type;
}

bool c_align_color_channels::setImageTransform(IMAGE_MOTION_TYPE motionType,
    const std::vector<float> parameters[4])
{
  _estimated_motion_type = motionType;

  _remaps.clear(), _remaps.resize(4);
  _fixed_remaps[0].clear(), _fixed_remaps[0].resize(4);
  _fixed_remaps[1].clear(), _fixed_remaps[1].resize(4);
  _image_transforms.clear(), _image_transforms.resize(4);
  for( int i = 0; i < 4; ++i ) {
    if( !parameters[i].empty() ) {
      _image_transforms[i] = create_image_transform(motionType);
      _image_transforms[i]->set_parameters(parameters[i]);
    }
  }

  return true;
}

void c_align_color_channels::remap(cv::InputArray src, cv::OutputArray dst,
    const c_align_color_channels_options & opts,
    int channel, int interpolation, int borderMode,
    const cv::Scalar & borderValue) const
{
  if( opts.use_fixed_remap ) {
    cv::remap(src, dst, _fixed_remaps[0][channel], _fixed_remaps[1][channel],
        interpolation,
        borderMode,
        borderValue);
  }
  else {
    cv::remap(src, dst, _remaps[channel], cv::noArray(),
        interpolation,
        borderMode,
        borderValue);
  }
}

bool c_align_color_channels::estimate(cv::InputArray _src, cv::InputArray _srcmask,
    int reference_channel_index,
    const c_align_color_channels_options & opts)
{
  const int cn = _src.channels();
  if( cn < 2 ) {
    CF_ERROR("Multi-channel image required for transfrom estimation");
    return false;
  }

  if( reference_channel_index < 0 || reference_channel_index >= cn ) {
    reference_channel_index = cn / 2;
  }

  cv::Mat reference_channel, reference_mask;

  cv::extractChannel(_src, reference_channel, reference_channel_index);

  if ( _srcmask.channels() == 1 ) {
    if (_srcmask.depth() == CV_8U) {
      reference_mask  = _srcmask.getMat();
    }
    else {
      cv::compare(_srcmask, 0, reference_mask, cv::CMP_GT);
    }
  }
  else {
    cv::extractChannel(_srcmask, reference_mask, reference_channel_index);
    if (reference_mask.depth() != CV_8U) {
      cv::compare(reference_mask, 0, reference_mask, cv::CMP_GT);
    }
  }

  std::vector<cv::Mat> channels;
  std::vector<cv::Mat> masks(cn);
  cv::split(_src, channels);

  if( !_srcmask.empty() ) {
    if( _srcmask.channels() < 2 ) {
      for( int i = 0; i < cn; ++i ) {
        masks[i] = reference_mask;
      }
    }
    else if (_srcmask.depth() == CV_8U) {
        cv::split(_srcmask, masks);
    }
    else {
      cv::Mat binary_srcmask;
      cv::compare(_srcmask, 0, binary_srcmask, cv::CMP_GT);
      cv::split(binary_srcmask, masks);
    }
  }

  _estimated_motion_type = opts.motion_type;
  _remaps.clear(), _fixed_remaps[0].clear(), _fixed_remaps[1].clear();
  _image_transforms.clear(), _image_transforms.resize(4);

  if ( !opts.use_fixed_remap ) {
    _remaps.resize(4);
  }
  else {
    _fixed_remaps[0].resize(4);
    _fixed_remaps[1].resize(4);
  }

  for( int i = 0; i < 4; ++i ) {
    if ( i == reference_channel_index ) {
      _image_transforms[i].reset();
    }
    else {
      _image_transforms[i] = create_image_transform(opts.motion_type);
      _image_transforms[i]->reset();
    }
  }

  c_ecch ecc(opts.method);
  ecc.set_maxlevel(opts.max_level);
  ecc.set_interpolation(opts.interpolation);
  ecc.set_update_step_scale(opts.update_step_scale);
  ecc.set_reference_smooth_sigma(opts.smooth_sigma);
  ecc.set_input_smooth_sigma(opts.smooth_sigma);
  ecc.set_max_iterations(opts.max_iterations);
  ecc.set_max_eps(opts.eps);

  if( !ecc.set_reference_image(reference_channel, reference_mask) ) {
    CF_ERROR("ecc.set_reference_image() fails");
    return false;
  }

  for( int i = 0; i < cn; ++i ) {
    if( _image_transforms[i] ) {
      ecc.set_image_transform(_image_transforms[i].get());
      if( !ecc.align(channels[i], masks[i]) ) {
        CF_ERROR("ecc.align_to_reference(channel=%d) fails: iterations=%d eps=%g", i,
            ecc.num_iterations(),
            ecc.eps());
      }
    }
  }

  return true;
}

bool c_align_color_channels::apply(cv::InputArray _src, cv::InputArray _srcmask,
    const c_align_color_channels_options & opts,
    cv::OutputArray _dst, cv::OutputArray _dstmask) const
{
  const int cn = _src.channels();
  if ( _image_transforms.size() < cn ) {
    CF_ERROR("Image transfrom was not estimated. Call c_align_color_channels::estimate() first");
    return false;
  }

  const cv::Size imageSize = _src.size();
  const int mask_depth = _srcmask.empty() ? CV_8U : _srcmask.depth();
  std::vector<cv::Mat> channels;

  if( opts.use_fixed_remap ) {
    if( _fixed_remaps[0].size() != 4 ) {
      _fixed_remaps[0].clear(), _fixed_remaps[0].resize(4);
      _fixed_remaps[1].clear(), _fixed_remaps[1].resize(4);
    }

    for( int i = 0; i < cn; ++i ) {
      if( _image_transforms[i] && _fixed_remaps[0][i].size() != imageSize ) {
        _image_transforms[i]->create_remap_fixed(imageSize, _fixed_remaps[0][i], _fixed_remaps[1][i]);
      }
    }
  }
  else {
    if( _remaps.size() != 4 ) {
      _remaps.clear(), _remaps.resize(4);
    }
    for( int i = 0; i < cn; ++i ) {
      if( _image_transforms[i] && _remaps[i].size() != imageSize ) {
        _image_transforms[i]->create_remap(imageSize, _remaps[i]);
      }
    }
  }


  cv::split(_src, channels);

  const bool is_realtime_mask = (_srcmask.empty() || mask_depth == CV_8U);

  if (is_realtime_mask) {
    // Case 1: fast path

    cv::Mat1b cumulative_mask;
    cv::Mat m2;

    cv::Mat srcm = _srcmask.empty() ? cv::Mat() : _srcmask.getMat();

    for( int i = 0; i < cn; ++i ) {

      if( _dst.needed() && _image_transforms[i] ) {
        remap(channels[i], channels[i], opts, i, opts.interpolation, opts.border_mode, opts.border_value[i]);
      }

      if( _dstmask.needed() && !srcm.empty() ) {
        cv::Mat current_m;
        if (srcm.channels() > 1) {
          cv::extractChannel(srcm, current_m, i);
        }
        else {
          current_m = srcm;
        }

        if (_image_transforms[i]) {
          remap(current_m, m2, opts, i, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
        }
        else {
          m2 = current_m;
        }

        if (cumulative_mask.empty() ) {
          cumulative_mask = std::move(m2);
        }
        else {
          cv::bitwise_and(cumulative_mask, m2, cumulative_mask);
        }
      }
    }

    if( _dstmask.needed() && srcm.empty() ) {
      if( _precomputed_frame_mask.size() != imageSize ) {
        cv::Mat1b total_edge(imageSize, (uchar) 255);
        cv::Mat1b tmp_remap;

        for( int k = 0; k < cn; ++k ) {
          if( _image_transforms[k] ) {
            remap(total_edge, tmp_remap, opts, k, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
            cv::bitwise_and(total_edge, tmp_remap, total_edge);
          }
        }
        _precomputed_frame_mask = std::move(total_edge);
      }
      cumulative_mask = _precomputed_frame_mask;
    }

    if( _dst.needed() ) {
      if( cn == 1 ) {
        _dst.move(channels[0]);
      }
      else {
        cv::merge(channels, _dst);
      }
    }

    if( _dstmask.needed() ) {
      if ( cumulative_mask.empty() ) {
        _srcmask.copyTo(_dstmask);
      }
      else {
        static const cv::Mat1b SE(3, 3, 255);
        cv::erode(cumulative_mask, _dstmask, SE, cv::Point(-1,-1), 1, cv::BORDER_REPLICATE);
      }
    }

  }
  else {
    // Case 2: slow path (map of CV_32F weights in srcmask)
    std::vector<cv::Mat> mask_channels;
    cv::Mat1b cumulative_edge_mask;

    cv::Mat srcm = _srcmask.getMat();
    const bool is_single_channel_mask = (srcm.channels() == 1);

    if( !is_single_channel_mask ) {
      cv::split(srcm, mask_channels);
    }

    if( _dynamic_dummy.size() != imageSize ) {
      _dynamic_dummy = cv::Mat1b(imageSize, uint8_t(255));
    }

    for( int i = 0; i < cn; ++i ) {
      if( _dst.needed() && _image_transforms[i] ) {
        remap(channels[i], channels[i], opts, i, opts.interpolation, opts.border_mode, opts.border_value[i]);
      }

      if( _dstmask.needed() ) {

        if( !is_single_channel_mask && _image_transforms[i] ) {
          const int interp = opts.interpolation;
          const int mask_interp = (interp == cv::INTER_NEAREST) ? cv::INTER_NEAREST : cv::INTER_LINEAR;
          remap(mask_channels[i], mask_channels[i], opts, i, mask_interp, cv::BORDER_CONSTANT, 0);
        }

        cv::Mat m2;
        if( !_image_transforms[i] ) {
          m2 = _dynamic_dummy;
        }
        else {
          remap(_dynamic_dummy, m2, opts, i, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
        }

        if( cumulative_edge_mask.empty() ) {
          cumulative_edge_mask = std::move(m2);
        }
        else {
          cv::bitwise_and(cumulative_edge_mask, m2, cumulative_edge_mask);
        }
      }
    }

    if( _dst.needed() ) {
      if( cn == 1 ) {
        _dst.move(channels[0]);
      }
      else {
        cv::merge(channels, _dst);
      }
    }

    if( _dstmask.needed() ) {
      cv::Mat b, w;

      if( is_single_channel_mask ) {
        w = srcm;
      }
      else {
        cv::merge(mask_channels, w);
      }

      static const cv::Mat1b SE(3, 3, 255);
      cv::erode(cumulative_edge_mask, b, SE, cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);

      _dstmask.assign(cv::Mat::zeros(imageSize, _srcmask.type()));
      w.copyTo(_dstmask, b);
    }
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
  SERIALIZE_OPTION(section, save, opts, use_fixed_remap);
  return true;
}
