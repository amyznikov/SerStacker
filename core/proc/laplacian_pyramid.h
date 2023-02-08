/*
 * laplacian_pyramid.h
 *
 *  Created on: Feb 7, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __laplacian_pyramid_h__
#define __laplacian_pyramid_h__

#include <opencv2/opencv.hpp>

void build_laplacian_pyramid(cv::InputArray input_image,
    std::vector<cv::Mat> & pyramid,
    int minimum_image_size = 4,
    cv::BorderTypes borderType = cv::BORDER_DEFAULT);

void reconstruct_laplacian_pyramid(cv::OutputArray output_image,
    const std::vector<cv::Mat> & pyramid,
    cv::BorderTypes borderType = cv::BORDER_DEFAULT);

#endif /* __laplacian_pyramid_h__ */
