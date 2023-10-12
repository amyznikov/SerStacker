/*
 * pyrflowlk2.h
 *
 *  Created on: Oct 3, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __pyrflowlk2_h__
#define __pyrflowlk2_h__

#include <opencv2/opencv.hpp>
#include <core/settings/opencv_settings.h>


struct c_pyrflowlk2_options
{
  int maxLevel = 3;
  cv::Size winSize = cv::Size(21, 21);
  int maxIterations = 30;
  int flags = 0;
  double eps = 0.01;
};

bool pyrflowlk2(cv::InputArray previous_image, cv::InputArray next_image,
    const std::vector<cv::KeyPoint> & previous_keypoints, std::vector<cv::KeyPoint> & predicted_keypoints,
    std::vector<uint8_t> & status,
    std::vector<float> & err,
    const c_pyrflowlk2_options & opts);


bool load_settings(c_config_setting settings,
    c_pyrflowlk2_options * args);
bool save_settings(c_config_setting settings,
    const c_pyrflowlk2_options & );

#endif /* __pyrflowlk2_h__ */
