/*
 * create_image_histogram.h
 *
 *  Created on: Dec 12, 2020
 *      Author: amyznikov
 */
#pragma once
#ifndef __create_image_histogram_h__
#define __create_image_histogram_h__

#include <opencv2/opencv.hpp>

bool create_image_histogram(cv::InputArray input_image,
    cv::Mat1f & output_histogram,
    int bins = 256,
    int channel = -1,
    double * input_output_range_min = nullptr,
    double * input_output_range_max = nullptr,
    bool uniform = true,
    bool accumulate = false);


#endif /* __create_image_histogram_h__ */
