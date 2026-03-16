/*
 * reduce_channels.cc
 *
 *  Created on: Jun 22, 2021
 *      Author: amyznikov
 */

#include "reduce_channels.h"


void reduce_color_channels(cv::InputArray _src, cv::OutputArray _dst, enum cv::ReduceTypes rtype, int dtype)
{
  // Reshape so that each pixel becomes a row, where the columns are its channels.
  // src.total() is the number of pixels (W*H)
  // src.channels() is the number of columns to reduce

  const cv::Mat src = _src.getMat();
  const cv::Mat flat = src.reshape(1, static_cast<int>(src.total()));

  cv::Mat reduced;
  cv::reduce(flat, reduced, 1, rtype, dtype);

  // Return the original shape (H x W)
  _dst.assign(reduced.reshape(1, src.rows));
}


//void reduce_color_channels(cv::InputArray src, cv::OutputArray dst, enum cv::ReduceTypes rtype, int dtype)
//{
//  cv::Mat s, tmp;
//
//  if ( src.isContinuous() ) {
//    s = src.getMat();
//  }
//  else {
//    src.copyTo(s);
//  }
//
//  const int src_rows = s.rows;
//  cv::reduce(s.reshape(1, s.total()), tmp, 1, rtype, dtype);
//  tmp.reshape(0, src_rows).copyTo(dst);
//}


void reduce_color_channels(const cv::Mat & src, cv::Mat & dst, enum cv::ReduceTypes rtype, int dtype)
{
//  cv::Mat s;
//
//  if ( src.isContinuous() ) {
//    s = src;
//  }
//  else {
//    src.copyTo(s);
//  }
//
//  const int src_rows = src.rows;
//  cv::reduce(s.reshape(1, s.total()), dst, 1, rtype, dtype);
//  dst = dst.reshape(0, src_rows);

  const cv::Mat flat = src.reshape(1, static_cast<int>(src.total()));

  cv::Mat reduced;
  cv::reduce(flat, reduced, 1, rtype, dtype);

  // Return the original shape (H x W)
  dst = reduced.reshape(1, src.rows);

}

void reduce_color_channels(cv::Mat & image, enum cv::ReduceTypes rtype, int dtype)
{
//  cv::Mat s;
//
//  if ( image.isContinuous() ) {
//    s = image;
//  }
//  else {
//    image.copyTo(s);
//  }
//
//  const int src_rows = image.rows;
//  cv::reduce(s.reshape(1, s.total()), s, 1, rtype, dtype);
//  image = s.reshape(0, src_rows);

  const cv::Mat flat = image.reshape(1, static_cast<int>(image.total()));

  cv::Mat reduced;
  cv::reduce(flat, reduced, 1, rtype, dtype);

  // Return the original shape (H x W)
  image = reduced.reshape(1, image.rows);

}

cv::Mat reduce_channels(cv::InputArray _src, enum cv::ReduceTypes rtype, int dtype)
{
//  cv::Mat s;
//
//  if ( src.isContinuous() ) {
//    s = src.getMat();
//  }
//  else {
//    src.copyTo(s);
//  }
//
//  const int src_rows = src.rows();
//  cv::reduce(s.reshape(1, s.total()), s, 1, rtype, dtype);
//  return s.reshape(0, src_rows);

  const cv::Mat src = _src.getMat();
  const cv::Mat flat = src.reshape(1, static_cast<int>(src.total()));

  cv::Mat reduced;
  cv::reduce(flat, reduced, 1, rtype, dtype);

  // Return the original shape (H x W)
  return reduced.reshape(1, src.rows);
}
