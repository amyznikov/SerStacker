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

bool cv2qt(cv::InputArray src,
    QImage * dst,
    bool rgbswap = true);

#endif /* __cv2qt_h__ */
