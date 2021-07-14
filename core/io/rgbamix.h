/*
 * split_rgba.h
 *
 *  Created on: Jul 14, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __rgbamix_h__
#define __rgbamix_h__


#include <opencv2/opencv.hpp>

// Split BGRA to BGR and mask
bool splitbgra(const cv::Mat & input_bgra_image,
    cv::Mat & output_bgr_image,
    cv::Mat * output_alpha_mask = nullptr);


// Merge BGR and mask to to BGRA
bool mergebgra(const cv::Mat & input_bgr_image, const cv::Mat & input_alpha_mask,
    cv::Mat & output_bgra_image);

#endif /* __rgbamix_h__ */
