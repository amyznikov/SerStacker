/*
 * fast_gaussian_blur.h
 *
 *  Created on: May 16, 2025
 *      Author: amyznikov
 */

#pragma once
#ifndef __fast_gaussian_blur_h__
#define __fast_gaussian_blur_h__


#include <opencv2/opencv.hpp>

// When mask is not empty the src pixels MUST be also set to 0 where _mask is 0
void fast_gaussian_blur(cv::InputArray _src, cv::InputArray _mask, cv::OutputArray _dst,
    double sigma, int borderType = cv::BORDER_DEFAULT, int ddepth=-1);


#endif /* __fast_gaussian_blur_h__ */
