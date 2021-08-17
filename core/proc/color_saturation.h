/*
 * color_saturation.h
 *
 *  Created on: Aug 16, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __color_saturation_h__
#define __color_saturation_h__

#include <opencv2/opencv.hpp>


bool color_saturation_hls(cv::InputArray src,
    cv::OutputArray dst,
    double scale,
    cv::InputArray srcmask = cv::noArray());

bool color_saturation_hls(cv::Mat & image,
    double scale,
    cv::InputArray srcmask = cv::noArray());



#endif /* __color_saturation_h__ */
