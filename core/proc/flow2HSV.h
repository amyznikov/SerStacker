/*
 * flow2HSV.h
 *
 *  Created on: Jun 6, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __flow2HSV_h__
#define __flow2HSV_h__

#include <opencv2/opencv.hpp>


/*
 * Create flow BGR image using HSV color space
 *  flow: 2-channel input flow matrix
 *  dst : output BRG image
 *
 *  The code is extracted from OpenCV example dis_opticalflow.cpp
 * */
bool flow2HSV(cv::InputArray flow, cv::Mat & dst,
    double maxmotion = 0,
    bool invert_y = true);

#endif /* __flow2HSV_h__ */
