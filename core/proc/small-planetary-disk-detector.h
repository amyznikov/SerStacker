/*
 * small-planetary-disk-detector.h
 *
 *  Created on: Sep 13, 2019
 *      Author: amyznikov
 */

#ifndef __small_planetary_disk_detector_h__
#define __small_planetary_disk_detector_h__

#include <opencv2/opencv.hpp>


bool simple_small_planetary_disk_detector(
    cv::InputArray frame,
    cv::Point2f * centrold,
    double gbsigma = 0,
    cv::Rect * optional_output_component_rect = nullptr,
    cv::Mat * optional_output_cmponent_mask = nullptr);

#endif /* __small_planetary_disk_detector_h__ */
