/*
 * reduce_channels.cc
 *
 *  Created on: Jun 22, 2021
 *      Author: amyznikov
 */

#include "reduce_channels.h"


void reduce_color_channels(cv::InputArray src, cv::OutputArray dst, enum cv::ReduceTypes rtype, int dtype)
{
  cv::Mat tmp;
  const int src_rows = src.rows();
  cv::reduce(src.getMat().reshape(1, src.total()), tmp, 1, rtype, dtype);
  tmp.reshape(0, src_rows).copyTo(dst);
}


void reduce_color_channels(const cv::Mat & src, cv::Mat & dst, enum cv::ReduceTypes rtype, int dtype)
{
  const int src_rows = src.rows;
  cv::reduce(src.reshape(1, src.total()), dst, 1, rtype, dtype);
  dst = dst.reshape(0, src_rows);
}

void reduce_color_channels(cv::Mat & image, enum cv::ReduceTypes rtype, int dtype)
{
  const int src_rows = image.rows;
  cv::reduce(image.reshape(1, image.total()), image, 1, rtype, dtype);
  image = image.reshape(0, src_rows);
}
