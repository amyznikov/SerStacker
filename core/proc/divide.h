/*
 * divide.h
 *
 *  Created on: Sep 1, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __cv_divide_h__
#define __cv_divide_h__

#include <opencv2/opencv.hpp>
#include <limits>

bool divideImages(cv::InputArray src1, cv::InputArray src2, cv::OutputArray dst,
    double eps = std::numeric_limits<float>::min(), int ddepth = -1 );

#endif /* __cv_divide_h__ */
