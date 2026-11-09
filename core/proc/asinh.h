/*
 * asinh.h
 *
 *  Created on: Nov 9, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __cv_asinh_h__
#define __cv_asinh_h__

#include <opencv2/opencv.hpp>

// compute dst = std::asinh(src * scale + offset)
bool asinh(cv::InputArray src, cv::OutputArray dst, double scale = 1, double offset = 0);


#endif /* __cv_asinh_h__ */
