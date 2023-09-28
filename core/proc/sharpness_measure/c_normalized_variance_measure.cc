/*
 * c_normalized_variance_measure.cc
 *
 *  Created on: Feb 9, 2023
 *      Author: amyznikov
 */

#include "c_normalized_variance_measure.h"
#include <core/proc/reduce_channels.h>
#include <core/debug.h>

void c_normalized_variance_measure::set_avgchannel(bool v)
{
  avgchannel_ = v;
}

bool c_normalized_variance_measure::avgchannel() const
{
  return avgchannel_;
}

cv::Scalar c_normalized_variance_measure::compute(cv::InputArray image, cv::InputArray mask) const
{
  cv::Scalar v;

  if ( !compute(image, mask, cv::noArray(), avgchannel_, &v) ) {
    CF_ERROR("c_normalized_variance_measure::compute() fails");
  }

  return v;
}

bool c_normalized_variance_measure::create_map(cv::InputArray image, cv::OutputArray output_map) const
{
  if ( !compute(image, cv::noArray(), output_map, avgchannel_, nullptr) ) {
    CF_ERROR("c_normalized_variance_measure::compute() fails");
    return false;
  }
  return true;
}

bool c_normalized_variance_measure::compute(cv::InputArray _image, cv::InputArray mask, cv::OutputArray output_map, bool avgchannel,
    cv::Scalar * output_sharpness_metric)
{
  cv::Mat image;

  if( !avgchannel || _image.channels() == 1 ) {
    image = _image.getMat();
  }
  else {
    reduce_color_channels(_image, image,
        cv::REDUCE_AVG,
        CV_32F);
  }


  if ( output_sharpness_metric ) {

    cv::Scalar m, s;
    cv::meanStdDev(image, m, s, mask);

    for ( int i = 0, cn = image.channels(); i < cn; ++i )  {
      if ( m[i] == 0 ) {
        s[i] = 0;
      }
      else {
        s[i] /= fabs(m[i]);
      }
    }
    * output_sharpness_metric = s;
  }

  return true;
}
