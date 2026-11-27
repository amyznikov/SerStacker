/*
 * area_remap.h
 *
 *  Created on: Nov 26, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __area_remap_h__
#define __area_remap_h__

#include <opencv2/opencv.hpp>

bool areaRemap(cv::InputArray srcImage, const cv::Mat2f & rmap,
    cv::OutputArray dstImage);

// Combined overload (image + weight mask)
// Supported srcMask types: empty matrix (generates CV_8UC1), CV_8U, CV_32F (1–4 channels)
bool areaRemap(cv::InputArray srcImage, cv::InputArray srcMask, const cv::Mat2f & rmap,
    cv::OutputArray dstImage, cv::OutputArray dstMask,
    int mdepth = -1);

#endif /* __area_remap_h__ */
