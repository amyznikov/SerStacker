/*
 * ImageDisplayFunction.h
 *
 *  Created on: Apr 22, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QImageDisplayFunction_h__
#define __QImageDisplayFunction_h__

#include <QtCore/QtCore>
#include <opencv2/opencv.hpp>

class ImageDisplayFunction
{
public:
  typedef ImageDisplayFunction ThisClass;

  virtual ~ImageDisplayFunction() = default;

  virtual void createDisplayImage(cv::InputArray currentImage, cv::InputArray currentMask,
      cv::Mat & mtfImage, cv::Mat & displayImage, int ddepth = CV_8U) = 0;
};

#endif /* __QImageDisplayFunction_h__ */
