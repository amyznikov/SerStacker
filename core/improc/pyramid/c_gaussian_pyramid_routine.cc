/*
 * c_gaussian_pyramid_routine.cc
 *
 *  Created on: Jul 11, 2022
 *      Author: amyznikov
 */

#include "c_gaussian_pyramid_routine.h"

template<>
const c_enum_member* members_of<c_gaussian_pyramid_routine::SharpenOrder>()
{
  static const c_enum_member members[] = {
      { c_gaussian_pyramid_routine::SharpenNone, "None", "" },
      { c_gaussian_pyramid_routine::SharpenBefore, "Before", "" },
      { c_gaussian_pyramid_routine::SharpenAfter, "After", "" },
      { c_gaussian_pyramid_routine::SharpenEachIteration, "EachIteration", "" },
      { c_gaussian_pyramid_routine::SharpenNone }
  };

  return members;
}


// https://jblindsay.github.io/ghrg/Whitebox/Help/FilterLaplacian.html
//static void compute_laplacian(cv::InputArray src, cv::OutputArray l/*, double alpha, double delta = 0*/)
//{
////  static float k[5 * 5] = {
////      0, 0, -1, 0, 0,
////      0, -1, -2, -1, 0,
////      -1, -2, 16, -2, -1,
////      0, -1, -2, -1, 0,
////      0, 0, -1, 0, 0,
////  };
////
////  const cv::Mat1f K =
////      cv::Mat1f(5, 5, k) * alpha / 16.;
//
////  static float k[3 * 3] = {
////      -1,  -4, -1,
////      -4,  20, -4,
////      -1,  -4, -1,
////  };
////
////  const cv::Mat1f K =
////      cv::Mat1f(3, 3, k) * (alpha / 20.);
////
////  cv::filter2D(src, l, -1, K, cv::Point(-1, -1), delta,
////      cv::BORDER_REPLICATE);
//
//}


static void sharpen_image(cv::InputArray src, cv::OutputArray dst, double alpha, double omin, double omax)
{
  cv::Mat m;
  cv::medianBlur(src, m, 3);
  cv::scaleAdd(m, -alpha, src, dst);

  if ( omax > omin ) {
    cv::max(dst.getMatRef(), cv::Scalar::all(omin), dst.getMatRef());
    cv::min(dst.getMatRef(), cv::Scalar::all(omax), dst.getMatRef());
  }
}

void c_gaussian_pyramid_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_SPINBOX_CTRL(ctls, count, -32, 32, 1, "count", "count of times for pyDown (negative value for pyrUp instead)");
  BIND_PCTRL(ctls, borderType, "enum cv::BorderTypes");
  BIND_PCTRL(ctls, sharpen_amount, "Optional sharpen factor (set 0 to disable)");
  BIND_PCTRL(ctls, sharpen_outmin, "");
  BIND_PCTRL(ctls, sharpen_outmax, "");
  BIND_PCTRL(ctls, sharpen_order, "When to apply unsharp_mask");
}

bool c_gaussian_pyramid_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, count);
    SERIALIZE_PROPERTY(settings, save, *this, borderType);
    SERIALIZE_PROPERTY(settings, save, *this, sharpen_amount);
    SERIALIZE_PROPERTY(settings, save, *this, sharpen_outmin);
    SERIALIZE_PROPERTY(settings, save, *this, sharpen_outmax);
    SERIALIZE_PROPERTY(settings, save, *this, sharpen_order);
    return true;

  }
  return false;
}

bool c_gaussian_pyramid_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( count_ > 0 ) {

    const bool trivialMask =
        mask.empty() || cv::countNonZero(mask) == mask.size().area();

    if ( sharpen_order_ == SharpenBefore && sharpen_amount_ != 0 ) {

      sharpen_image(image.getMat(), image, sharpen_amount_,
          sharpen_outmin_,
          sharpen_outmax_);
    }


    for( int i = 0; i < count_ && std::min(image.cols(), image.rows()) > 3; ++i ) {

      if( sharpen_order_ == SharpenEachIteration && sharpen_amount_ != 0 ) {

        sharpen_image(image.getMat(), image, sharpen_amount_,
            sharpen_outmin_,
            sharpen_outmax_);

      }

      cv::pyrDown(image.getMat(), image, cv::Size(), borderType_);

      if( !trivialMask ) {
        cv::pyrDown(mask.getMat(), mask, cv::Size(), borderType_);
      }
    }

    if ( sharpen_order_ == SharpenAfter && sharpen_amount_ != 0 ) {

      sharpen_image(image.getMat(), image, sharpen_amount_,
          sharpen_outmin_,
          sharpen_outmax_);
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

    if ( sharpen_order_ == SharpenBefore && sharpen_amount_ != 0 ) {

      sharpen_image(image.getMat(), image, sharpen_amount_,
          sharpen_outmin_,
          sharpen_outmax_);

    }

    for( int i = 0; i < -count_ && std::max(image.cols(), image.rows()) < 16000; ++i ) {

      if( sharpen_order_ == SharpenEachIteration && sharpen_amount_ != 0 ) {

        sharpen_image(image.getMat(), image, sharpen_amount_,
            sharpen_outmin_,
            sharpen_outmax_);

      }

      cv::pyrUp(image.getMat(), image, cv::Size(), borderType_);

      if( !trivialMask ) {
        cv::pyrUp(mask.getMat(), mask, cv::Size(), borderType_);
      }

    }

    if ( sharpen_order_ == SharpenAfter && sharpen_amount_ != 0 ) {

      sharpen_image(image.getMat(), image, sharpen_amount_,
          sharpen_outmin_,
          sharpen_outmax_);

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
