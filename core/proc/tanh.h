/*
 * tanh.h
 *
 *  Created on: Nov 10, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __cv_tanh_h__
#define __cv_tanh_h__

#include <opencv2/opencv.hpp>

// compute dst = std::tanh(src * scale + offset)
bool tanh(cv::InputArray src, cv::OutputArray dst, double scale = 1, double offset = 0);

#endif /* __cv_tanh_h__ */
