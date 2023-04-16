/*
 * QMeasureMinMax.h
 *
 *  Created on: Apr 10, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMeasureMinMax_h__
#define __QMeasureMinMax_h__

#include "QMeasure.h"

class QMeasureMinValue :
    public QMeasure
{
public:
  typedef QMeasureMinValue ThisClass;
  typedef QMeasure Base;

  QMeasureMinValue();

protected:
  int compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const override;
};

class QMeasureMaxValue :
    public QMeasure
{
public:
  typedef QMeasureMaxValue ThisClass;
  typedef QMeasure Base;

  QMeasureMaxValue();

protected:
  int compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const override;
};



#endif /* __QMeasureMinMax_h__ */
