/*
 * median_pyramid.h
 *
 *  Created on: Sep 10, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __median_pyramid_h__
#define __median_pyramid_h__

#include <opencv2/opencv.hpp>
#include <core/proc/downstrike.h>

bool build_median_pyramid(cv::InputArray src,
    int median_ksize,
    int median_iterations,
    std::vector<cv::Mat> & scaled_images,
    std::vector<cv::Mat> & median_blurs,
    std::vector<cv::Mat> & median_hats);


#endif /* __median_pyramid_h__ */
