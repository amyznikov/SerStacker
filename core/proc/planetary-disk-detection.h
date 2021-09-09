/*
 * planetary-disk-detection.h
 *
 *  Created on: Sep 13, 2019
 *      Author: amyznikov
 */

#ifndef __planetary_disk_detection_h__
#define __planetary_disk_detection_h__

#include <opencv2/opencv.hpp>


bool simple_planetary_disk_detector(
    cv::InputArray frame,
    cv::InputArray mask,
    cv::Point2f * centrold,
    double gbsigma = 0,
    cv::Rect * optional_output_component_rect = nullptr,
    cv::Mat * optional_output_cmponent_mask = nullptr);

#endif /* __planetary_disk_detection_h__ */
