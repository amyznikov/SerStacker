/*
 * rgbamix.cc
 *
 *  Created on: Jul 14, 2021
 *      Author: amyznikov
 */


#include "rgbamix.h"

// Split BGRA to BGR and mask
bool splitbgra(const cv::Mat & input_bgra_image, cv::Mat & output_bgr_image, cv::Mat * output_alpha_mask)
{
  if ( input_bgra_image.channels() != 4 ) {
    return false;
  }

  cv::Mat dst[2];

  dst[0].create(input_bgra_image.size(),
      CV_MAKETYPE(input_bgra_image.depth(), input_bgra_image.channels() - 1));

  dst[1].create(input_bgra_image.size(),
      CV_MAKETYPE(input_bgra_image.depth(), 1));

  static const int from_to[] = {0, 0, 1, 1, 2, 2, 3, 3 };

  cv::mixChannels(&input_bgra_image, 1, dst, 2, from_to, 4 );

  output_bgr_image = std::move(dst[0]);

  if ( output_alpha_mask ) {
    cv::compare(dst[1], 0, *output_alpha_mask, cv::CMP_GT);
  }

  return true;
}

// Merge BGR and mask to to BGRA
bool mergebgra(const cv::Mat & input_bgr_image, const cv::Mat & input_alpha_mask,
    cv::Mat & output_bgra_image)
{
  if ( input_bgr_image.channels() != 3 || input_alpha_mask.empty() || input_alpha_mask.type() != CV_8UC1
      || input_alpha_mask.size() != input_bgr_image.size() ) {
    return false;
  }

  cv::Mat bgra;
  cv::Mat alpha;

  switch ( input_bgr_image.depth() ) {
  case CV_8U :
    alpha = input_alpha_mask;
    break;
  case CV_8S :
    alpha = input_alpha_mask;
    break;
  case CV_16U :
    input_alpha_mask.convertTo(alpha, input_bgr_image.depth(), UINT16_MAX / UINT8_MAX);
    break;
  case CV_16S :
    input_alpha_mask.convertTo(alpha, input_bgr_image.depth(), INT16_MAX / (double) UINT8_MAX);
    break;
  case CV_32S :
    input_alpha_mask.convertTo(alpha, input_bgr_image.depth(), INT32_MAX / (double) UINT8_MAX);
    break;
  case CV_32F :
    input_alpha_mask.convertTo(alpha, input_bgr_image.depth(), 1.0 / UINT8_MAX);
    break;
  case CV_64F :
    input_alpha_mask.convertTo(alpha, input_bgr_image.depth(), 1.0 / UINT8_MAX);
    break;
  }

  cv::Mat & dst = (output_bgra_image.data == input_bgr_image.data ||
      output_bgra_image.data == input_alpha_mask.data) ?
      bgra : output_bgra_image;

  cv::Mat src[2] = { input_bgr_image, alpha };

  static constexpr int from_to[] = { 0, 0, 1, 1, 2, 2, 3, 3 };

  dst.create(input_bgr_image.size(),
      CV_MAKETYPE(input_bgr_image.depth(), input_bgr_image.channels() + 1));

  cv::mixChannels(src, 2, &dst, 1, from_to, 4);

  if ( dst.data != output_bgra_image.data ) {
    output_bgra_image = std::move(dst);
  }

  return false;
}

