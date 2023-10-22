/*
 * morph_gradient_flow.h
 *
 *  Created on: Oct 5, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __morph_gradient_flow_h__
#define __morph_gradient_flow_h__

#include <core/proc/morphology.h>


bool morph_gradient_flow(cv::InputArray current_image, cv::InputArray current_mask,
    cv::InputArray reference_image, cv::InputArray reference_mask,
    int max_pyramid_level,
    int block_radius,
    int search_radius,
    double alpha,
    double beta,
    double gradient_threshold,
    const cv::Point2f & epipole,
    cv::Mat1f & output_epipolar_disparity,
    cv::Mat1f & output_cost,
    const std::string & debug_path = "");


#endif /* __morph_gradient_flow_h__ */
