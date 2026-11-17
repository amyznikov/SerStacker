/*
 * image_correlation.h
 *
 *  Created on: Nov 14, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __image_correlation_h__
#define __image_correlation_h__


#include <opencv2/opencv.hpp>


double compute_image_correlation_sw(cv::InputArray src1, cv::InputArray src2,
    cv::InputArray mask);


double compute_image_correlation(cv::InputArray src1, cv::InputArray src2,
    cv::InputArray mask);

double compute_image_correlation(cv::InputArray current_image, cv::InputArray current_mask,
    cv::InputArray reference_image, cv::InputArray reference_mask,
    const cv::Mat2f & rmap);

#endif /* __image_correlation_h__ */
