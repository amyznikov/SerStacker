/*
 * c_normalize_mean_stdev_routine.cc
 *
 *  Created on: Jul 2, 2024
 *      Author: amyznikov
 */

#include "c_normalize_mean_stdev_routine.h"
#include <core/proc/image_registration/ecc2.h>

//static bool ecc_upscale(cv::Mat & image, cv::Size dstSize)
//{
//  const cv::Size inputSize =
//      image.size();
//
//  if ( inputSize != dstSize ) {
//
//    std::vector<cv::Size> spyramid;
//
//    spyramid.emplace_back(dstSize);
//
//    while ( 42 ) {
//
//      const cv::Size nextSize((spyramid.back().width + 1) / 2,
//          (spyramid.back().height + 1) / 2);
//
//      if ( nextSize == inputSize ) {
//        break;
//      }
//
//      if ( nextSize.width < inputSize.width || nextSize.height < inputSize.height ) {
//
//        CF_ERROR("FATAL: invalid next size : nextSize=%dx%d inputSize=%dx%d",
//            nextSize.width, nextSize.height,
//            inputSize.width, inputSize.height);
//
//        return false;
//      }
//
//      spyramid.emplace_back(nextSize);
//    }
//
//    for ( int i = spyramid.size() - 1; i >= 0; --i ) {
//      cv::pyrUp(image, image, spyramid[i]);
//    }
//  }
//
//  return true;
//}
//
//
//static void normalize_meanstdev(cv::InputArray _src, cv::InputArray _mask, cv::OutputArray dst, int lvl, double eps)
//{
//  const cv::Mat src =
//      _src.getMat();
//
//  const cv::Size src_size =
//      src.size();
//
//  cv::Size dst_size =
//      src_size;
//
//  for( int l = 0; l < lvl; ++l ) {
//
//    const cv::Size next_size((dst_size.width + 1) / 2, (dst_size.height + 1) / 2);
//    if( next_size.width < 4 || next_size.height < 4 ) {
//      break;
//    }
//
//    dst_size =
//        next_size;
//  }
//
//  cv::Mat m, s;
//
//  cv::resize(src, m, dst_size, 0, 0, cv::INTER_AREA);
//  cv::resize(src.mul(src), s, dst_size, 0, 0, cv::INTER_AREA);
//  cv::add(s, eps, s);
//
//  ecc_upscale(m, src_size);
//  ecc_upscale(s, src_size);
//
//  cv::subtract(src, m, dst, cv::noArray(), CV_32F);
//  cv::divide(dst, s, dst);
//
//  if ( !_mask.empty() ) {
//    dst.setTo(0, ~_mask.getMat());
//  }
//
//}
//
//

void c_normalize_mean_stdev_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "level", ctx(&this_class::_level), "normalization scalle");
   ctlbind(ctls, "eps", ctx(&this_class::_eps), "normalization eps");
}

bool c_normalize_mean_stdev_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _level);
    SERIALIZE_OPTION(settings, save, *this, _eps);
    return true;
  }
  return false;
}

bool c_normalize_mean_stdev_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  ecc_normalize_meanstdev(image.getMat(), mask, image, _level, _eps);
  return true;
}
