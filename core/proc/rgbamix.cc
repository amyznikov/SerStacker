/*
 * rgbamix.cc
 *
 *  Created on: Jul 14, 2021
 *      Author: amyznikov
 */


#include "rgbamix.h"

// Split BGRA to BGR and mask
bool splitbgra(const cv::Mat & input_image, cv::Mat & output_image, cv::Mat * output_alpha_mask)
{
  const int cn = input_image.channels();

  if ( cn == 2 ) {

    cv::Mat dst[2];

    cv::split(input_image, dst);

    output_image = std::move(dst[0]);

    if ( output_alpha_mask ) {
      cv::compare(dst[1], 0, *output_alpha_mask, cv::CMP_GT);
    }

    return true;
  }

  if ( cn == 4 ) {

    cv::Mat dst[2];

    dst[0].create(input_image.size(),
        CV_MAKETYPE(input_image.depth(), cn - 1));

    dst[1].create(input_image.size(),
        CV_MAKETYPE(input_image.depth(), 1));

    static const int from_to[] = { 0, 0, 1, 1, 2, 2, 3, 3 };

    cv::mixChannels(&input_image, 1, dst, 2, from_to, 4);

    output_image = std::move(dst[0]);

    if ( output_alpha_mask ) {
      cv::compare(dst[1], 0, *output_alpha_mask, cv::CMP_GT);
    }

    return true;
  }

  return false;
}

// Merge BGR and mask to to BGRA
bool mergebgra(const cv::Mat & input_image, const cv::Mat & input_alpha_mask, cv::Mat & output_image)
{
  const int cn = input_image.channels();
  if ( cn != 1 && cn != 3 ) {
    return false;
  }

  if ( input_alpha_mask.empty() || input_alpha_mask.type() != CV_8UC1 || input_alpha_mask.size() != input_image.size() ) {
    return false;
  }


  cv::Mat alpha;

  switch ( input_image.depth() ) {
  case CV_8U :
    alpha = input_alpha_mask;
    break;
  case CV_8S :
    alpha = input_alpha_mask;
    break;
  case CV_16U :
    input_alpha_mask.convertTo(alpha, input_image.depth(), UINT16_MAX / UINT8_MAX);
    break;
  case CV_16S :
    input_alpha_mask.convertTo(alpha, input_image.depth(), INT16_MAX / (double) UINT8_MAX);
    break;
  case CV_32S :
    input_alpha_mask.convertTo(alpha, input_image.depth(), INT32_MAX / (double) UINT8_MAX);
    break;
  case CV_32F :
    input_alpha_mask.convertTo(alpha, input_image.depth(), 1.0 / UINT8_MAX);
    break;
  case CV_64F :
    input_alpha_mask.convertTo(alpha, input_image.depth(), 1.0 / UINT8_MAX);
    break;
  }

  if ( cn == 1 ) {

    cv::Mat src[2] = { input_image, alpha };
    cv::merge(src, 2, output_image);
  }
  else { // if ( cn == 3 )

    cv::Mat bgra;

    cv::Mat & dst = (output_image.data == input_image.data ||
        output_image.data == input_alpha_mask.data) ?
        bgra : output_image;

    cv::Mat src[2] = { input_image, alpha };

    static constexpr int from_to[] = { 0, 0, 1, 1, 2, 2, 3, 3 };

    dst.create(input_image.size(),
        CV_MAKETYPE(input_image.depth(), input_image.channels() + 1));

    cv::mixChannels(src, 2, &dst, 1, from_to, 4);

    if ( dst.data != output_image.data ) {
      output_image = std::move(dst);
    }

  }


  return true;
}

