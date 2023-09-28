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

bool fitEllipseLM1(const std::vector<cv::Point2f> & edge_points,
    double fixed_axis_ratio, // b / a
    double fixed_orientation, // radians
    cv::RotatedRect * rc);





#endif /* __fitEllipseLM_h__ */
