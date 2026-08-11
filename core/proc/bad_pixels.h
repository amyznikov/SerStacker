/*
 * bad_pixels.h
 *
 *  Created on: May 13, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __bad_pixels_h__
#define __bad_pixels_h__

#include <opencv2/opencv.hpp>
#include <core/io/debayer.h>

bool median_filter_bad_pixels(cv::Mat & image, double variation_threshold,
    COLORID color_id);

#endif /* __bad_pixels_h__ */
