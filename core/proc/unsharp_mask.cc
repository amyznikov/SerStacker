/*
 * unsharp_mask.cc
 *
 *  Created on: Feb 20, 2021
 *      Author: amyznikov
 */

#include "unsharp_mask.h"

void unsharp_mask(cv::InputArray src, cv::OutputArray dst,
    double sigma, double alpha,
    double outmin, double outmax)
{

  if ( sigma <= 0 || alpha <= 0 ) {
    src.copyTo(dst);
  }
  else {

    if ( outmax <= outmin ) {

      const int ddepth = dst.fixedType() ? dst.depth() : src.depth();

      switch ( ddepth ) {
      case CV_8U :
        outmin = 0, outmax = UINT8_MAX;
        break;
      case CV_8S :
        outmin = -INT8_MIN, outmax = INT8_MAX;
        break;
      case CV_16U :
        outmin = 0, outmax = UINT16_MAX;
        break;
      case CV_16S :
        outmin = INT16_MIN, outmax = INT16_MAX;
        break;
      case CV_32S :
        outmin = INT32_MIN, outmax = INT32_MAX;
        break;
      default : /*cv::minMaxLoc(src, &outmin, &outmax); */
        break;
      }
    }

    cv::Mat lpass;
    cv::Mat1f G = cv::getGaussianKernel(2 * std::max(1, (int) (sigma * 5)) + 1, sigma, CV_32F);
    cv::sepFilter2D(src, lpass, -1, G, G, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
    cv::addWeighted(src, 1. / (1. - alpha), lpass, -alpha / (1. - alpha), 0, dst);
  }

  if ( outmax > outmin ) {
    cv::min(dst.getMatRef(), outmax, dst.getMatRef());
    cv::max(dst.getMatRef(), outmin, dst.getMatRef());
  }
}


