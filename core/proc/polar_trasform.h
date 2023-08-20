/*
 * polar_trasform.h
 *
 *  Created on: Aug 20, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __polar_trasform_h__
#define __polar_trasform_h__

#include <opencv2/opencv.hpp>


void create_epipolar_remap(const cv::Size src_size, const cv::Point2f & center,
    cv::Mat2f & output_rmap);



#endif /* __polar_trasform_h__ */
