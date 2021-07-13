/*
 * create_image_histogram.cc
 *
 *  Created on: Dec 12, 2020
 *      Author: amyznikov
 */

#include "create_image_histogram.h"



bool create_image_histogram(cv::InputArray input_image,
    cv::Mat1f & output_histogram,
    int bins,
    int channel,
    double * input_output_range_min,
    double * input_output_range_max,
    bool uniform,
    bool accumulate)
{
  if ( input_image.empty() ) {
    return false;
  }

  const cv::Mat image =
      input_image.getMat();

  const int cn =
      image.channels();

  if ( channel >= cn ) {
    return false;
  }


  if ( bins < 1 ) {
    bins = 256;
  }
  else if ( bins > 65536 ) {
    bins = 65536;
  }


  double range_min = input_output_range_min ? * input_output_range_min : -1;
  double range_max = input_output_range_max ? * input_output_range_max : -1;

  if ( range_max <= range_min ) {

    switch ( image.depth() ) {
    case CV_8U :
      range_min = 0;
      range_max = UINT8_MAX + 1.;
      break;
    case CV_8S :
      range_min = INT8_MIN;
      range_max = INT8_MAX + 1.;
      break;
    case CV_16U :
      range_min = 0;
      range_max = UINT16_MAX + 1.;
      break;
    case CV_16S :
      range_min = INT16_MIN;
      range_max = INT16_MAX + 1.;
      break;
    case CV_32S :
      range_min = INT32_MIN;
      range_max = INT32_MAX + 1.;
      break;
    case CV_32F :
      case CV_64F : {

      double min, max;
      cv::minMaxLoc(image, &min, &max);

      if ( min >= 0 ) {
        min = 0;
      }
      if ( max <= 1 ) {
        max = 1. + FLT_EPSILON;
      }

      range_min = min;
      range_max = max;

      break;
    }
    default :
      return false;
    }

    if ( input_output_range_min ) {
      * input_output_range_min = range_min;
    }

    if ( input_output_range_max ) {
      *input_output_range_max = range_max;
    }
  }


  const float range[] = { (float)range_min, (float)range_max };
  const float * ranges[] = { range };
  const int sizes[] = { bins };

  if ( channel >= 0 ) {

    output_histogram.create(bins, 1);

    cv::calcHist(&image, 1,
        &channel,
        cv::noArray(),
        output_histogram,
        1,
        sizes,
        ranges,
        uniform,
        accumulate);

  }
  else {

    cv::Mat tmp;

    output_histogram.create(bins, cn);

    for ( int i = 0; i < cn; ++i ) {

      cv::calcHist(&image, 1,
          &i,
          cv::noArray(),
          tmp,
          1,
          sizes,
          ranges,
          uniform,
          accumulate);

      tmp.copyTo(output_histogram.colRange(i, i + 1));

    }
  }

  return true;
}
