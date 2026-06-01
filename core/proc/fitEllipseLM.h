/*
 * fitEllipseLM.h
 *
 *  Created on: Sep 17, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __fitEllipseLM_h__
#define __fitEllipseLM_h__


#include <opencv2/opencv.hpp>

// cv::Point3f::x => x coordinate
// cv::Point3f::y => y coordinate
// cv::Point3f::z => weight of the point
bool fitEllipseLMW(const std::vector<cv::Point3f> & edge_points,
    double fixed_axis_ratio, // b / a
    double fixed_orientation, // radians
    cv::RotatedRect * rc);




#endif /* __fitEllipseLM_h__ */
