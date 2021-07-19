/*
 * alpha.cc
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */

#include <core/io/save_image.h>
#include <core/debug.h>

int main(int argc, char *argv[])
{

  cv::Mat2f some_image(256, 256, 0.f);


  some_image.setTo(cv::Scalar(0.5, 0.8));

  save_image(some_image, "some_image.tiff");




  return 0;
}
