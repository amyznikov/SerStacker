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

void c_align_color_channels::set_method(ECC_ALIGN_METHOD v)
{
  _opts.method = v;
}

ECC_ALIGN_METHOD c_align_color_channels::method() const
{
  return _opts.method;
}

void c_align_color_channels::set_motion_type(IMAGE_MOTION_TYPE motion_type)
{
  _opts.motion_type = motion_type;
}

IMAGE_MOTION_TYPE  c_align_color_channels::motion_type() const
{
  return _opts.motion_type;
}

void c_align_color_channels::set_interpolation(enum ECC_INTERPOLATION_METHOD flags)
{
  _opts.interpolation = flags;
}

enum ECC_INTERPOLATION_METHOD c_align_color_channels::interpolation() const
{
  return _opts.interpolation;
}

void c_align_color_channels::set_border_mode(enum ECC_BORDER_MODE border_mode)
{
  _opts.border_mode = border_mode;
}

enum ECC_BORDER_MODE c_align_color_channels::border_mode() const
{
  return _opts.border_mode;
}

void c_align_color_channels::set_border_value(const cv::Scalar & v)
{
  _opts.border_value = v;
}

const cv::Scalar & c_align_color_channels::border_value() const
{
  return _opts.border_value;
}


void c_align_color_channels::set_max_iterations(int v)
{
  _opts.max_iterations = v;
}

int c_align_color_channels::max_iterations() const
{
  return _opts.max_iterations;
}

void c_align_color_channels::set_smooth_sigma(double v)
{
  _opts.smooth_sigma = v;
}

double c_align_color_channels::smooth_sigma() const
{
  return _opts.smooth_sigma;
}

void c_align_color_channels::set_update_step_scale(double v)
{
  _opts.update_step_scale = v;
}

double c_align_color_channels::update_step_scale() const
{
  return _opts.update_step_scale;
}

void c_align_color_channels::set_eps(double v)
{
  _opts.eps = v;
}

double c_align_color_channels::eps() const
{
  return _opts.eps;
}

void c_align_color_channels::set_max_level(int v)
{
  _opts.max_level = v;
}

int c_align_color_channels::max_level() const
{
  return _opts.max_level;
}

void c_align_color_channels::set_normalization_level(int v)
{
  _opts.normalization_level = v;
}

int c_align_color_channels::normalization_level() const
{
  return _opts.normalization_level;
}

void c_align_color_channels::set_normalization_eps(double v)
{
  _opts.normalization_eps = v;
}

double c_align_color_channels::normalization_eps() const
{
  return _opts.normalization_eps;
}

//double c_align_color_channels::computed_rho(int channel_index) const
//{
//  return computed_rhos_[channel_index];
//}
//
//double c_align_color_channels::computed_eps(int channel_index) const
//{
//  return computed_eps_[channel_index];
//}
//
//int c_align_color_channels::computed_iterations(int channel_index) const
//{
//  return computed_iterations_[channel_index];
//}

const c_image_transform::sptr & c_align_color_channels::computed_transform(int channel_index) const
{
  return _computed_transforms[channel_index];
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

//  computed_eps_.clear();
//  computed_rhos_.clear();
  _computed_transforms.clear();
//  computed_iterations_.clear();

  const int cn =
      src.channels();

  if ( cn < 2 ) {

    if ( dst.needed() ) {
      src.copyTo(dst);
    }

    if ( dstmask.needed() ) {
      srcmask.copyTo(dstmask);
    }

//    computed_eps_.emplace_back(0);
//    computed_rhos_.emplace_back(1);
//    computed_iterations_.emplace_back(0);
    _computed_transforms.emplace_back(nullptr);

    return true;
  }

  if ( reference_channel < 0 || reference_channel >= cn ) {
    CF_ERROR("Invalid argument: reference_channel = %d is out if range 0..%d",
        reference_channel, cn - 1);
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


  cv::Mat channels[cn], normalized_channels[cn];
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

  if( _opts.normalization_level < 1 ) {
    for ( int i = 0; i < cn; ++i ) {
      normalized_channels[i] =
          channels[i];
    }
  }
  else {
    for ( int i = 0; i < cn; ++i ) {
      ecc_normalize_meanstdev(channels[i], masks[i], normalized_channels[i],
          _opts.normalization_level,
          _opts.normalization_eps);
    }
  }



//  computed_eps_.resize(cn);
//  computed_rhos_.resize(cn);
  _computed_transforms.resize(cn);
//  computed_iterations_.resize(cn);

  c_ecch ecc(_opts.method);
  ecc.set_maxlevel(_opts.max_level);
  ecc.set_interpolation(_opts.interpolation);
  ecc.set_update_step_scale(_opts.update_step_scale);
  ecc.set_reference_smooth_sigma(_opts.smooth_sigma);
  ecc.set_input_smooth_sigma(_opts.smooth_sigma);
  ecc.set_max_iterations(_opts.max_iterations);
  ecc.set_max_eps(_opts.eps);

  if( !ecc.set_reference_image(normalized_channels[reference_channel], masks[reference_channel]) ) {
    CF_ERROR("ecc.set_reference_image(reference_channel=%d) fails", reference_channel);
    return false;
  }

  cv::Mat2f current_remap;

  for ( int i = 0; i < cn; ++i ) {

    _computed_transforms[i] =
        create_image_transform(_opts.motion_type);

    if ( i == reference_channel ) {
//      computed_eps_[i] = 0;
//      computed_rhos_[i] = 1;
//      computed_iterations_[i] = 0;

      if ( dst.needed() || dstmask.needed() ) {
        if ( masks[i].empty() ) {
          masks[i] = cv::Mat1b(src.size(), 255);
        }
      }

    }
    else {

      ecc.set_image_transform(_computed_transforms[i].get());

      if( !ecc.align(normalized_channels[i], masks[i]) ) {
        CF_ERROR("ecc.align() %d-> %d fails: iterations=%d eps=%g",
            i,
            reference_channel,
            ecc.num_iterations(),
            ecc.eps());
      }

      _computed_transforms[i]->create_remap(ecc.reference_image().size(),
          current_remap);

//      computed_rhos_[i] =
//          compute_correlation(ecc.current_image(), ecc.current_mask(),
//              ecc.reference_image(), ecc.reference_mask(),
//              current_remap);

//      computed_iterations_[i] =
//          ecc.num_iterations();
//
//      computed_eps_[i] =
//          ecc.eps();

//      CF_DEBUG("align channel %d: iterations=%d rho=%g eps=%g", i, computed_iterations_[i], computed_rhos_[i], computed_eps_[i]);

      if ( dst.needed() ) {

        cv::remap(channels[i], channels[i],
            current_remap,
            cv::noArray(),
            _opts.interpolation,
            _opts.border_mode,
            _opts.border_value[i]);

      }

      if( dstmask.needed() ) {

        cv::remap(masks[i].empty() ? cv::Mat1b(src.size(), 255) : masks[i],
            masks[i],
            current_remap,
            cv::noArray(),
            cv::INTER_AREA,
            cv::BORDER_REPLICATE,
            0);

        cv::compare(masks[i], 255, masks[i],
            cv::CMP_GE);
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
//  computed_eps_.clear();
//  computed_rhos_.clear();
  _computed_transforms.clear();
//  computed_iterations_.clear();

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

//  computed_eps_.resize(cn);
//  computed_rhos_.resize(cn);
  _computed_transforms.resize(cn);
//  computed_iterations_.resize(cn);


  cv::Mat2f current_remap;

  c_ecch ecc(_opts.method);
  ecc.set_maxlevel(_opts.max_level);
  ecc.set_interpolation(_opts.interpolation);
  ecc.set_update_step_scale(_opts.update_step_scale);
  ecc.set_reference_smooth_sigma(_opts.smooth_sigma);
  ecc.set_input_smooth_sigma(_opts.smooth_sigma);
  ecc.set_max_iterations(_opts.max_iterations);
  ecc.set_max_eps(_opts.eps);

  if( !ecc.set_reference_image(reference_image, reference_mask) ) {
    CF_ERROR("ecc.set_reference_image() fails");
    return false;
  }


  for( int i = 0; i < cn; ++i ) {

    ecc.set_image_transform((_computed_transforms[i] =
        create_image_transform(_opts.motion_type)).get());

    if( !ecc.align( channels[i], masks[i]) ) {
      CF_ERROR("ecc.align_to_reference(channel=%d) fails: iterations=%d eps=%g",
          i,
          ecc.num_iterations(),
          ecc.eps());
    }

    ecc.image_transform()->create_remap(ecc.reference_image().size(),
        current_remap);

//    computed_rhos_[i] =
//        compute_correlation(ecc.current_image(), ecc.current_mask(),
//            ecc.reference_image(), ecc.reference_mask(),
//            current_remap);
//
//    computed_iterations_[i] =
//        ecc.num_iterations();
//
//    computed_eps_[i] =
//        ecc.eps();

    if( dst.needed() ) {

      cv::remap(channels[i], channels[i],
          current_remap,
          cv::noArray(),
          _opts.interpolation,
          _opts.border_mode,
          _opts.border_value[i]);

    }

    if( dstmask.needed() ) {

      cv::remap(masks[i].empty() ? cv::Mat1b(src.size(), 255) : masks[i],
          masks[i],
          current_remap,
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
