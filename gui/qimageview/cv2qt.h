/*
 * cv2qt.h
 *
 *  Created on: Mar 8, 2020
 *      Author: amyznikov
 */

#ifndef __cv2qt_h__
#define __cv2qt_h__

#include <opencv2/opencv.hpp>
#include <QtGui/QtGui>

/*
 * Convert cv::Mat to QImage
 */
bool cv2qt(cv::InputArray src,
    QImage * dst,
    bool rgbswap = true);

/*
 * Create flow BGR image using HSV color space
 *  flow: 2-channel input flow matrix
 *  dst : output BRG image
 *
 *  The code is extracted from OpenCV example dis_opticalflow.cpp
 * */
bool flow2HSV(cv::InputArray flow,
    cv::Mat & dst,
    double maxmotion,
    bool invert_y);

#endif /* __cv2qt_h__ */
