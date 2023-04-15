/*
 * QMeasureMeanStdev.h
 *
 *  Created on: Apr 10, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMeasureMeanStdev_h__
#define __QMeasureMeanStdev_h__

#include "QMeasure.h"

class QMeasureMeanValue:
    public QMeasure
{
public:
  typedef QMeasureMeanValue ThisClass;
  typedef QMeasure Base;

  QMeasureMeanValue();
  int compute(cv::InputArray image, cv::InputArray mask, const cv::Rect& roi,
      cv::Scalar * value) const override;
};

class QMeasureStdevValue:
    public QMeasure
{
public:
  typedef QMeasureStdevValue ThisClass;
  typedef QMeasure Base;

  QMeasureStdevValue();
  int compute(cv::InputArray image, cv::InputArray mask, const cv::Rect& roi,
      cv::Scalar * value) const override;
};

#endif /* __QMeasureMeanStdev_h__ */
