/*
 * reduce_channels.cc
 *
 *  Created on: Jun 22, 2021
 *      Author: amyznikov
 */

#include "reduce_channels.h"


void reduce_color_channels(cv::InputArray src, cv::OutputArray dst, enum cv::ReduceTypes rtype, int dtype)
{
  cv::Mat s, tmp;

  if ( src.isContinuous() ) {
    s = src.getMat();
  }
  else {
    src.copyTo(s);
  }

  const int src_rows = s.rows;
  cv::reduce(s.reshape(1, s.total()), tmp, 1, rtype, dtype);
  tmp.reshape(0, src_rows).copyTo(dst);
}


void reduce_color_channels(const cv::Mat & src, cv::Mat & dst, enum cv::ReduceTypes rtype, int dtype)
{
  cv::Mat s;

  if ( src.isContinuous() ) {
    s = src;
  }
  else {
    src.copyTo(s);
  }

  const int src_rows = src.rows;
  cv::reduce(s.reshape(1, s.total()), dst, 1, rtype, dtype);
  dst = dst.reshape(0, src_rows);
}

void reduce_color_channels(cv::Mat & image, enum cv::ReduceTypes rtype, int dtype)
{
  cv::Mat s;

  if ( image.isContinuous() ) {
    s = image;
  }
  else {
    image.copyTo(s);
  }

  const int src_rows = image.rows;
  cv::reduce(s.reshape(1, s.total()), s, 1, rtype, dtype);
  image = s.reshape(0, src_rows);
}
