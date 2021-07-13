/*
 * c_average_with_mask.cc
 *
 *  Created on: Dec 29, 2019
 *      Author: amyznikov
 */

#include "c_average_with_mask.h"

void c_average_with_mask::release()
{
  accumulator_.release();
  counter_.release();
  nbframes_ = 0;
}

bool c_average_with_mask::add(cv::InputArray src, cv::InputArray mask)
{
  if ( accumulator_.empty() || counter_.empty() ) {
    accumulator_.create(src.size(), CV_MAKETYPE(acctype, src.channels()));
    counter_.create(src.size(), CV_MAKETYPE(acctype, src.channels()));
    accumulator_.setTo(0);
    counter_.setTo(0);
  }

  cv::add(accumulator_, src, accumulator_, mask, accumulator_.type());
  cv::add(counter_, 1, counter_, mask, counter_.type());

  ++nbframes_;

  return true;
}

const cv::Mat & c_average_with_mask::accumulator() const
{
  return accumulator_;
}

const cv::Mat & c_average_with_mask::counter() const
{
  return counter_;
}

int c_average_with_mask::nbframes() const
{
  return nbframes_;
}

bool c_average_with_mask::average(cv::OutputArray avg, cv::OutputArray mask, double scale, int dtype) const
{
  if ( mask.needed() ) {
    cv::compare(counter_, 1, mask, cv::CMP_GE);
  }
  if ( avg.needed() ) {
    cv::divide(accumulator_, counter_, avg, scale, dtype);
    avg.setTo(0, counter_ < 1);
  }
  return true;
}

