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
  int compute(cv::InputArray, cv::InputArray, const cv::Rect&,
      cv::Scalar * output_value) const override;
};

class QMeasureMaxValue :
    public QMeasure
{
public:
  typedef QMeasureMaxValue ThisClass;
  typedef QMeasure Base;

  QMeasureMaxValue();
  int compute(cv::InputArray, cv::InputArray, const cv::Rect&,
      cv::Scalar * output_value) const override;
};



#endif /* __QMeasureMinMax_h__ */
