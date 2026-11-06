/*
 * morphse.h
 *
 *  Created on: Nov 6, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __morphse_h__
#define __morphse_h__

#include <opencv2/opencv.hpp>

inline const cv::Mat1b & morphse33()
{
  static const cv::Mat1b SE(3, 3, (uint8_t) 255);
  return SE;
}

inline const cv::Mat1b & morphse55()
{
  static const cv::Mat1b SE(5, 5, (uint8_t) 255);
  return SE;
}

#endif /* __morphse_h__ */
