/*
 * c_gaussian_pyramid_routine.cc
 *
 *  Created on: Jul 11, 2022
 *      Author: amyznikov
 */

#include "c_gaussian_pyramid_routine.h"

template<>
const c_enum_member* members_of<c_gaussian_pyramid_routine::UnsharpMaskOrder>()
{
  static const c_enum_member members[] = {
      { c_gaussian_pyramid_routine::UnsharpMaskNone, "None", "" },
      { c_gaussian_pyramid_routine::UnsharpMaskBefore, "Before", "" },
      { c_gaussian_pyramid_routine::UnsharpMaskAfter, "After", "" },
      { c_gaussian_pyramid_routine::UnsharpMaskEachIteration, "EachIteration", "" },
      { c_gaussian_pyramid_routine::UnsharpMaskNone }
  };

  return members;
}

void c_gaussian_pyramid_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_SPINBOX_CTRL(ctls, count, -32, 32, 1, "count", "count of times for pyDown (negative value for pyrUp instead)");
  BIND_PCTRL(ctls, borderType, "enum cv::BorderTypes");
  BIND_PCTRL(ctls, unsharp_sigma, "Optional unsharp mask sigma before downscaling (set 0 to disable)");
  BIND_PCTRL(ctls, unsharp_alpha, "Optional unsharp mask alpha before downscaling (set 0 to disable)");
  BIND_PCTRL(ctls, unsharp_outmin, "");
  BIND_PCTRL(ctls, unsharp_outmax, "");
  BIND_PCTRL(ctls, unsharp_mask_order, "When to apply unsharp_mask");
}

bool c_gaussian_pyramid_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, count);
    SERIALIZE_PROPERTY(settings, save, *this, borderType);
    SERIALIZE_PROPERTY(settings, save, *this, unsharp_sigma);
    SERIALIZE_PROPERTY(settings, save, *this, unsharp_alpha);
    SERIALIZE_PROPERTY(settings, save, *this, unsharp_outmin);
    SERIALIZE_PROPERTY(settings, save, *this, unsharp_outmax);
    SERIALIZE_PROPERTY(settings, save, *this, unsharp_mask_order);
    return true;

  }
  return false;
}

bool c_gaussian_pyramid_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( count_ > 0 ) {

    const bool trivialMask =
        mask.empty() || cv::countNonZero(mask) == mask.size().area();

    if ( unsharp_mask_order_ == UnsharpMaskBefore && unsharp_sigma_ > 0 && unsharp_alpha_ > 0 ) {

      unsharp_mask(image.getMat(), mask.getMat(),
          image,
          unsharp_sigma_,
          unsharp_alpha_,
          unsharp_outmin_,
          unsharp_outmax_);
    }


    for( int i = 0; i < count_ && std::min(image.cols(), image.rows()) > 3; ++i ) {

      if( unsharp_mask_order_ == UnsharpMaskEachIteration && unsharp_sigma_ > 0 && unsharp_alpha_ > 0 ) {
        unsharp_mask(image.getMat(), mask.getMat(),
            image,
            unsharp_sigma_,
            unsharp_alpha_,
            unsharp_outmin_,
            unsharp_outmax_);
      }

      cv::pyrDown(image.getMat(), image, cv::Size(), borderType_);

      if( !trivialMask ) {
        cv::pyrDown(mask.getMat(), mask, cv::Size(), borderType_);
      }
    }

    if ( unsharp_mask_order_ == UnsharpMaskAfter && unsharp_sigma_ > 0 && unsharp_alpha_ > 0 ) {
      unsharp_mask(image.getMat(), mask.getMat(),
          image,
          unsharp_sigma_,
          unsharp_alpha_,
          unsharp_outmin_,
          unsharp_outmax_);
    }

    if( !mask.empty() ) {

      if( !trivialMask ) {
        cv::compare(mask, 255, mask, cv::CMP_GE);
      }
      else {
        cv::resize(mask, mask, image.size(), 0, 0, cv::INTER_NEAREST);
      }
    }

  }
  else if( count_ < 0 ) {

    const bool trivialMask =
        mask.empty() || cv::countNonZero(mask) == mask.size().area();

    if ( unsharp_mask_order_ == UnsharpMaskBefore && unsharp_sigma_ > 0 && unsharp_alpha_ > 0 ) {
      unsharp_mask(image.getMat(), mask.getMat(),
          image,
          unsharp_sigma_,
          unsharp_alpha_,
          unsharp_outmin_,
          unsharp_outmax_);
    }

    for( int i = 0; i < -count_ && std::max(image.cols(), image.rows()) < 16000; ++i ) {

      if( unsharp_mask_order_ == UnsharpMaskEachIteration && unsharp_sigma_ > 0 && unsharp_alpha_ > 0 ) {
        unsharp_mask(image.getMat(), mask.getMat(),
            image,
            unsharp_sigma_,
            unsharp_alpha_,
            unsharp_outmin_,
            unsharp_outmax_);
      }

      cv::pyrUp(image.getMat(), image, cv::Size(), borderType_);

      if( !trivialMask ) {
        cv::pyrUp(mask.getMat(), mask, cv::Size(), borderType_);
      }

    }

    if ( unsharp_mask_order_ == UnsharpMaskAfter && unsharp_sigma_ > 0 && unsharp_alpha_ > 0 ) {
      unsharp_mask(image.getMat(), mask.getMat(),
          image,
          unsharp_sigma_,
          unsharp_alpha_,
          unsharp_outmin_,
          unsharp_outmax_);
    }

    if( !mask.empty() ) {

      if( !trivialMask ) {
        cv::compare(mask.getMat(), 255, mask, cv::CMP_GE);
      }
      else {
        cv::resize(mask.getMat(), mask, image.size(), 0, 0, cv::INTER_NEAREST);
      }
    }
  }

  return true;
}
