/*
 * pyrscale.h
 *
 *  Created on: Feb 13, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __pyrscale_h__
#define __pyrscale_h__

#include <opencv2/opencv.hpp>


bool pyramid_downscale(cv::InputArray src,
    cv::Mat & dst,
    int max_levels,
    int border_mode);

bool pyramid_upscale(cv::Mat & image,
    const cv::Size & dstSize);

#endif /* __pyrscale_h__ */
