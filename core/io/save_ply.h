/*
 * save_ply.h
 *
 *  Created on: May 28, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __save_ply_h__
#define __save_ply_h__

#include <opencv2/opencv.hpp>

bool save_ply(const std::vector<cv::Vec3d> & points, const std::vector<cv::Vec3b> & colors,
    const std::string & fname);

bool save_ply(const cv::Mat3f & points, const cv::Mat3b & colors, const cv::Mat1b & mask,
    const std::string & fname);

#endif /* __save_ply_h__ */
