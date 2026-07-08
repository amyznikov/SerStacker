/*
 * adaptive_gaussian_blur.h
 *
 *  Created on: Jul 8, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __adaptive_gaussian_blur_h__
#define __adaptive_gaussian_blur_h__

#include <opencv2/opencv.hpp>

enum ADAPTIVE_GAUSSIAN_BLUR_OUTPUT_TYPE {
  ADAPTIVE_GAUSSIAN_BLUR_OUTPUT_FILTERED,
  ADAPTIVE_GAUSSIAN_BLUR_OUTPUT_BLUR1,
  ADAPTIVE_GAUSSIAN_BLUR_OUTPUT_BLUR2,
  ADAPTIVE_GAUSSIAN_BLUR_OUTPUT_LPG,
};

void adaptive_gaussian_blur(cv::InputArray src, cv::OutputArray dst,
    double sigma_hpass, double sigma_lpass, double lpg_scale, double lpgk = 0.5,
    ADAPTIVE_GAUSSIAN_BLUR_OUTPUT_TYPE outputDisplay = ADAPTIVE_GAUSSIAN_BLUR_OUTPUT_FILTERED);


#endif /* __adaptive_gaussian_blur_h__ */
