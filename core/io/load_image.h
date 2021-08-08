/*
 * load_image.h
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef ___load_image_h___
#define _____load_image_h___

#include <opencv2/opencv.hpp>

bool load_image(cv::Mat & dst,
    const std::string & filename);

// Split BGRA to BGR and mask
bool splitbgra(const cv::Mat & input_bgra_image,
    cv::Mat & output_bgr_image,
    cv::Mat * output_alpha_mask = nullptr);

#endif /* _____load_image_h___ */
