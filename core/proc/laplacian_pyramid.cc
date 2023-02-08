/*
 * laplacian_pyramid.cc
 *
 *  Created on: Feb 7, 2023
 *      Author: amyznikov
 */

#include "laplacian_pyramid.h"


void build_laplacian_pyramid(cv::InputArray input_image, std::vector<cv::Mat> & pyramid, int minimum_image_size,
    cv::BorderTypes borderType)
{
  pyramid.emplace_back();
  input_image.getMat().convertTo(pyramid.back(), CV_32F);

  while (42) {

    const int currentMinSize =
        std::min(pyramid.back().cols,
            pyramid.back().rows);

    const int nextMinSize = currentMinSize / 2;
    if( nextMinSize <= minimum_image_size ) {
      break;
    }

    cv::Mat filtered_image, upscaled_image;
    cv::pyrDown(pyramid.back(), filtered_image, cv::Size(), borderType);
    cv::pyrUp(filtered_image, upscaled_image, pyramid.back().size(), borderType);
    cv::subtract(pyramid.back(), upscaled_image, pyramid.back());
    pyramid.emplace_back(filtered_image);
  }
}

void reconstruct_laplacian_pyramid(cv::OutputArray output_image,
    const std::vector<cv::Mat> & pyramid,
    cv::BorderTypes borderType)
{
  cv::Mat current_image;

  const int pyrsize =
      pyramid.size();

  if( pyrsize > 0 ) {

    pyramid.back().copyTo(current_image);

    for( int i = pyrsize - 2; i >= 0; --i ) {

      cv::pyrUp(current_image, current_image, pyramid[i].size(), borderType);
      cv::add(current_image, pyramid[i], current_image);
    }
  }

  if( !output_image.fixedType() || output_image.type() == current_image.type() ) {
    output_image.move(current_image);
  }
  else {
    current_image.convertTo(output_image,
        output_image.depth());
  }
}
