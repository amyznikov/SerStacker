/*
 * reduce_channels.h
 *
 *  Created on: Jun 22, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __reduce_channels_h__
#define __reduce_channels_h__

#include <opencv2/opencv.hpp>

void reduce_color_channels(cv::InputArray src,
    cv::OutputArray dst,
    enum cv::ReduceTypes rtype,
    int dtype = -1);


void reduce_color_channels(const cv::Mat & src,
    cv::Mat & dst,
    enum cv::ReduceTypes rtype,
    int dtype = -1);

void reduce_color_channels(cv::Mat & image,
    enum cv::ReduceTypes rtype,
    int dtype = -1);


#endif /* __reduce_channels_h__ */
