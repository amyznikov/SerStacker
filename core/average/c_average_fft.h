/*
 * c_average_fft.h
 *
 *  Created on: Oct 21, 2020
 *      Author: amyznikov
 */

#ifndef __c_average_fft_h__
#define __c_average_fft_h__

#include <opencv2/opencv.hpp>


class c_average_fft
{
public:

  bool add(cv::InputArray src);

  bool average(cv::OutputArray avg,
      double scale = 1.0,
      int ddepth = -1) const;

  void reset();


protected:
  std::vector<cv::Mat> accumulators_;
  std::vector<cv::Mat> weights_;
  cv::Rect rc_;
  cv::Size fftSize_;
  int border_top_ = 0;
  int border_bottom_ = 0;
  int border_left_ = 0;
  int border_right_ = 0;
  int nbframes_ = 0;

};

#endif /* __c_average_fft_h__ */
