/*
 * planetary-disk-detection.h
 *
 *  Created on: Sep 13, 2019
 *      Author: amyznikov
 */

#ifndef __planetary_disk_detection_h__
#define __planetary_disk_detection_h__

#include <opencv2/opencv.hpp>


/**
 * Threshold given grayscale input image and extract maximal area
 * connected component mask assuming it is planetary disk on a frame
 */
bool simple_planetary_disk_detector(cv::InputArray image, cv::InputArray mask,
    double gbsigma = 0,
    double stdev_factor = 0.5,
    int close_radius = 2,
    cv::Point2f * optional_output_centroid = nullptr,
    cv::Rect * optional_output_component_rect = nullptr,
    cv::Mat * optional_output_cmponent_mask = nullptr,
    cv::Point2f * optional_output_geometrical_center = nullptr,
    cv::Mat * optional_output_debug_image = nullptr);


bool detect_saturn(cv::InputArray _image, int se_close_radius, cv::RotatedRect & output_bbox,
    cv::OutputArray output_mask = cv::noArray(),
    double gbsigma = 1, double stdev_factor = 0.5);

#endif /* __planetary_disk_detection_h__ */
