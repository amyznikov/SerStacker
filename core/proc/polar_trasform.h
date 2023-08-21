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

void create_epipolar_remaps(const cv::Size reference_image_size, const cv::Point2f & epipole,
    const cv::Matx33d & current_derotation_homography,
    cv::Mat2f & output_current_rmap,
    cv::Mat2f & output_reference_rmap);


#endif /* __polar_trasform_h__ */
