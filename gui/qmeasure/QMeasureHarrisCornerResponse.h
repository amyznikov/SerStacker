/*
 * QMeasureHarrisCornerResponse.h
 *
 *  Created on: Apr 11, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMeasureHarrisCornerResponse_h__
#define __QMeasureHarrisCornerResponse_h__

#include "QMeasure.h"
#include <core/proc/sharpness_measure/c_harris_sharpness_measure.h>

class QMeasureHarrisCornerResponse:
    public QMeasure,
    public c_harris_sharpness_measure
{
public :
  typedef QMeasureHarrisCornerResponse ThisClass;
  typedef QMeasure Base;

  QMeasureHarrisCornerResponse();

  QMeasureSettingsWidget * createSettingsWidget(QWidget * parent) const override;

//  void setAverageColorChannels(bool v) override;
//  bool averageColorChannels() const override;

  int compute(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const final;
};

class QHarrisCornerResponseSettingsWidget :
    public QMeasureSettingsWidgetTemplate<QMeasureHarrisCornerResponse>
{
public:
  typedef QHarrisCornerResponseSettingsWidget ThisClass;
  typedef QMeasureSettingsWidgetTemplate<QMeasureHarrisCornerResponse> Base;

  QHarrisCornerResponseSettingsWidget(QWidget * parent = nullptr);

protected:
  QNumericBox * k_ctl = nullptr;
  QNumericBox * dscale_ctl = nullptr;
  QNumericBox * uscale_ctl = nullptr;
};

#endif /* __QMeasureHarrisCornerResponse_h__ */
