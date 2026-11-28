/*
 * weighted_average.h
 *
 *  Created on: Nov 28, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __weighted_average_h__
#define __weighted_average_h__

#include <opencv2/opencv.hpp>

bool weighted_average_update(cv::InputArray _src_image, cv::InputArray _src_weights,
    cv::InputOutputArray _src_accumulator, cv::InputOutputArray _weights_accumulator);


#endif /* __weighted_average_h__ */
