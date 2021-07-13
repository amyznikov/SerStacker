/*
 * c_average_with_weights.cc
 *
 *  Created on: Sep 20, 2019
 *      Author: amyznikov
 */

#include "c_average_with_weights.h"
#include <core/debug.h>

bool c_average_with_weights::add(cv::InputArray src, cv::InputArray weights)
{
  if ( accumulator_.empty() ) {
    accumulator_.create(src.size(), src.type());
    accumulator_.setTo(0);
  }
  if ( weights_.empty() ) {
    weights_.create(weights.size(), src.type());
    weights_.setTo(0);
  }

  if ( weights.channels() == src.channels() ) {
    cv::accumulateProduct(src, weights, accumulator_);
    cv::accumulate(weights, weights_);
  }
  else if ( weights.channels() == 1 ) {

    cv::Mat chnls[src.channels()];
    cv::Mat w  = weights.getMat();
    for ( int i = 0, n = src.channels(); i < n; ++i ) {
      chnls[i] = w;
    }
    cv::merge(chnls, src.channels(), tmp_);
    cv::accumulateProduct(src, tmp_, accumulator_);
    cv::accumulate(tmp_, weights_);
  }
//  else if ( src.channels() == 3 && weights.channels() == 1 ) {
//    cv::cvtColor(weights, tmp_, cv::COLOR_GRAY2BGR);
//    cv::accumulateProduct(src, tmp_, accumulator_);
//    cv::accumulate(tmp_, weights_);
//  }
  else {
    CF_ERROR("ERROR: unsupported combination of image channels encountered");
    return false;
  }

  ++nbframes_;
  return true;
}


bool c_average_with_weights::average(cv::OutputArray avg, cv::OutputArray mask, double scale, int ddepth) const
{
  if ( mask.needed() ) {
    cv::compare(weights_, 0, mask, cv::CMP_GT);
  }

  if ( avg.needed() ) {
    cv::divide(accumulator_, weights_, avg, scale, ddepth);
  }

  return true;
}


void c_average_with_weights::reset()
{
  accumulator_.release();
  weights_.release();
  nbframes_ = 0;
}

