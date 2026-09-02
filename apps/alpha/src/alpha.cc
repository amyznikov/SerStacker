/*
 * alpha.cc
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */

#include <cmath>
#include <array>
#include <core/ssprintf.h>
#include <core/io/c_stdio_file.h>
#include <core/proc/levmar.h>
#include <core/proc/levmar3.h>
#include <core/ctrlbind/ctrlbind.h>
#include <core/proc/sharpness_measure/c_lpg_sharpness_measure.h>
#include <core/io/save_image.h>

#include <core/debug.h>

static void simulate_ecc_convert(cv::InputArray src, cv::Mat & dst)
{
  dst = src.getMat();
}

static void simulate_set_reference_image(cv::InputArray src)
{
  cv::Mat image;
  cv::Mat tmp;
  simulate_ecc_convert(src.getMat(), image);
  const cv::Mat G = cv::getGaussianKernel(31, 5, CV_32F);

  save_image(image, "debug/image_before_blur.tiff");
  cv::sepFilter2D(image, tmp, image.depth(), G, G, cv::Point(-1,-1), 0, cv::BORDER_DEFAULT);
  image = std::move(tmp);
  save_image(image, "debug/image_after_blur.tiff");

}



int main(int argc, char *argv[])
{
  const cv::Mat primary_image = cv::Mat1b::zeros(100, 100) ;

  cv::circle(primary_image, cv::Point(primary_image.cols / 2, primary_image.rows / 2),
      primary_image.cols / 4, cv::Scalar::all(255), -1);

  save_image(primary_image, "debug/primary_image_before_call.tiff");
  simulate_set_reference_image(primary_image);
  save_image(primary_image, "debug/primary_image_after_call.tiff");

  return 0;
}
