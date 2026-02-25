/*
 * c_blur_test_routine.cc
 *
 *  Created on: Jun 7, 2024
 *      Author: amyznikov
 */

#include "c_blur_test_routine.h"
#include <core/proc/gradient.h>

void c_blur_test_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "sigma", ctx(&this_class::_sigma), "");
  ctlbind(ctls, "w", ctx(&this_class::_w), "");
}

bool c_blur_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _sigma);
    SERIALIZE_OPTION(settings, save, *this, _w);
    return true;
  }
  return false;
}

bool c_blur_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{

  cv::Mat gx, gy, g;
  compute_sobel_gradients(image,
      gx,
      gy,
      CV_32F);
  // g = gx.mul(gx) + gy.mul(gy);
  cv::magnitude(gx, gy, g);
  if( g.channels() == 3 ) {
    cv::cvtColor(g, g, cv::COLOR_BGR2GRAY);
    cv::cvtColor(g, g, cv::COLOR_GRAY2BGR);
  }

  const int ksize =
      2 * (int) ((3 * _sigma + 1)) + 1;

  cv::Mat1f G =
      cv::getGaussianKernel(ksize, _sigma, CV_32F);

  cv::sepFilter2D(image, gx, CV_32F, G, G, cv::Point(-1, -1), 0,
      cv::BORDER_REPLICATE);

  image.getMat().convertTo(gy, CV_32F);

  const int ddepth =
      image.depth();

  const int rows =
      image.rows();

  const int cols =
      image.cols();

  const int cn =
      image.channels();

  for( int y = 0; y < rows; ++y ) {

    const float * gxp = gx.ptr<const float>(y);
    const float * gyp = gy.ptr<const float>(y);
    float * gp = g.ptr<float>(y);

    for( int x = 0; x < cols; ++x ) {
      for( int c = 0; c < cn; ++c ) {

        const float & v1 = gxp[x * cn + c];
        const float & v2 = gyp[x * cn + c];
        const float ww = gp[x * cn + c];
        float & v3 = gp[x * cn + c];

        v3 = (v1 + v2 * ww * _w) / (1 + ww * _w);
      }
    }
  }

  g.convertTo(image, ddepth);

  return true;
}
