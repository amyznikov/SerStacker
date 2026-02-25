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

void c_gaussian_pyramid_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "count",  ctx(&this_class::_count), "");
   ctlbind(ctls, "borderType",  ctx(&this_class::_borderType), "");
   ctlbind(ctls, "sharpen_order",  ctx(&this_class::_sharpen_order), "");
   ctlbind(ctls, "sharpen_amount",  ctx(&this_class::_sharpen_amount), "");
   ctlbind(ctls, "sharpen_outmin",  ctx(&this_class::_sharpen_outmin), "");
   ctlbind(ctls, "sharpen_outmax",  ctx(&this_class::_sharpen_outmax), "");
}

bool c_gaussian_pyramid_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _count);
    SERIALIZE_OPTION(settings, save, *this, _borderType);
    SERIALIZE_OPTION(settings, save, *this, _sharpen_amount);
    SERIALIZE_OPTION(settings, save, *this, _sharpen_outmin);
    SERIALIZE_OPTION(settings, save, *this, _sharpen_outmax);
    SERIALIZE_OPTION(settings, save, *this, _sharpen_order);
    return true;
  }
  return false;
}

bool c_gaussian_pyramid_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( _count > 0 ) {

    const bool trivialMask =
        mask.empty() || cv::countNonZero(mask) == mask.size().area();

    if ( _sharpen_order == SharpenBefore && _sharpen_amount != 0 ) {

      sharpen_image(image.getMat(), image, _sharpen_amount,
          _sharpen_outmin,
          _sharpen_outmax);
    }


    for( int i = 0; i < _count && std::min(image.cols(), image.rows()) > 3; ++i ) {

      if( _sharpen_order == SharpenEachIteration && _sharpen_amount != 0 ) {

        sharpen_image(image.getMat(), image, _sharpen_amount,
            _sharpen_outmin,
            _sharpen_outmax);

      }

      cv::pyrDown(image.getMat(), image, cv::Size(), _borderType);

      if( !trivialMask ) {
        cv::pyrDown(mask.getMat(), mask, cv::Size(), _borderType);
      }
    }

    if ( _sharpen_order == SharpenAfter && _sharpen_amount != 0 ) {

      sharpen_image(image.getMat(), image, _sharpen_amount,
          _sharpen_outmin,
          _sharpen_outmax);
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
  else if( _count < 0 ) {

    const bool trivialMask =
        mask.empty() || cv::countNonZero(mask) == mask.size().area();

    if ( _sharpen_order == SharpenBefore && _sharpen_amount != 0 ) {

      sharpen_image(image.getMat(), image, _sharpen_amount,
          _sharpen_outmin,
          _sharpen_outmax);

    }

    for( int i = 0; i < -_count && std::max(image.cols(), image.rows()) < 16000; ++i ) {

      if( _sharpen_order == SharpenEachIteration && _sharpen_amount != 0 ) {

        sharpen_image(image.getMat(), image, _sharpen_amount,
            _sharpen_outmin,
            _sharpen_outmax);

      }

      cv::pyrUp(image.getMat(), image, cv::Size(), _borderType);

      if( !trivialMask ) {
        cv::pyrUp(mask.getMat(), mask, cv::Size(), _borderType);
      }

    }

    if ( _sharpen_order == SharpenAfter && _sharpen_amount != 0 ) {

      sharpen_image(image.getMat(), image, _sharpen_amount,
          _sharpen_outmin,
          _sharpen_outmax);

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
