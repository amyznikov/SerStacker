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

protected:
  int compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const override;
};

class QMeasureStdevValue:
    public QMeasure
{
public:
  typedef QMeasureStdevValue ThisClass;
  typedef QMeasure Base;

  QMeasureStdevValue();

protected:
  int compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const override;
};

#endif /* __QMeasureMeanStdev_h__ */
