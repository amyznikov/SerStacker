/*
 * project_to_radius_vector.h
 *
 *  Created on: May 31, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __project_to_radius_vector_h__
#define __project_to_radius_vector_h__

#include <opencv2/opencv.hpp>

bool project_to_radius_vector(const cv::Point2f & rp,
    cv::InputArray gx, cv::InputArray gy,
    cv::OutputArray gr,
    cv::OutputArray gt = cv::noArray());



#endif /* __project_to_radius_vector_h__ */
