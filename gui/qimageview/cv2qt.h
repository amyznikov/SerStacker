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


// Qt::NoFormatConversion |
//                Qt::ThresholdDither |
//                Qt::ThresholdAlphaDither |
//                Qt::NoOpaqueDetection

QPixmap createPixmap(cv::InputArray src, bool rgbswap = true,
    Qt::ImageConversionFlags flags = Qt::AutoColor);

#endif /* __cv2qt_h__ */
