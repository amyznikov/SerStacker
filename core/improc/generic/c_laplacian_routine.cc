/*
 * c_laplacian_routine.cc
 *
 *  Created on: Apr 5, 2023
 *      Author: amyznikov
 */

#include "c_laplacian_routine.h"


// https://jblindsay.github.io/ghrg/Whitebox/Help/FilterLaplacian.html
static void compute_laplacian(cv::InputArray src, cv::OutputArray l, double delta)
{
  static float k[5 * 5] = {
      0, 0, -1, 0, 0,
      0, -1, -2, -1, 0,
      -1, -2, 16, -2, -1,
      0, -1, -2, -1, 0,
      0, 0, -1, 0, 0,
  };

  static const thread_local cv::Mat1f K =
      cv::Mat1f(5, 5, k) / 16.;

  cv::filter2D(src, l, -1, K, cv::Point(-1, -1), delta,
      cv::BORDER_REPLICATE);
}


bool c_laplacian_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  compute_laplacian(image.getMat(), image, 0);
  return true;
}
