/*
 * QMeasureNoise.h
 *
 *  Created on: Apr 16, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMeasureNoise_h__
#define __QMeasureNoise_h__

#include "QMeasure.h"

class QMeasureNoise:
    public QMeasure
{
public:
  typedef QMeasureNoise ThisClass;
  typedef QMeasure Base;

  QMeasureNoise();

protected:
  int compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const override;
};

#endif /* __QMeasureNoise_h__ */
