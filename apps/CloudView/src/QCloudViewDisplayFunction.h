/*
 * QCloudViewDisplayFunction.h
 *
 *  Created on: Dec 12, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCloudViewDisplayFunction_h__
#define __QCloudViewDisplayFunction_h__

#include <QtCore/QtCore>
#include <opencv2/opencv.hpp>

namespace cloudview {

class QCloudViewDisplayFunction
{
public:
  typedef QCloudViewDisplayFunction ThisClass;

  virtual ~QCloudViewDisplayFunction() = default;

  virtual void createDisplayPoints(cv::InputArray currentPoints,
      cv::InputArray currentColors,
      cv::InputArray currentMask,
      cv::OutputArray displayPoints,
      cv::OutputArray mtfColors,
      cv::OutputArray displayColors) = 0;
};

} /* namespace cloudview */

#endif /* __QCloudViewDisplayFunction_h__ */
