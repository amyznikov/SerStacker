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
  int compute(cv::InputArray image, cv::InputArray mask, const cv::Rect& roi, cv::Scalar * value) const override;
  bool hasOptions() const override;
  QMeasureSettingsWidget * createSettingsWidget(QWidget * parent) const override;
};

class QHarrisCornerResponseSettingsWidget :
    public QMeasureSettingsWidgetImpl<QMeasureHarrisCornerResponse>
{
public:
  typedef QHarrisCornerResponseSettingsWidget ThisClass;
  typedef QMeasureSettingsWidgetImpl<QMeasureHarrisCornerResponse> Base;

  QHarrisCornerResponseSettingsWidget(QWidget * parent = nullptr);

protected:
  QCheckBox * avgc_ctl = nullptr;
  QNumericBox * k_ctl = nullptr;
  QNumericBox * dscale_ctl = nullptr;
  QNumericBox * uscale_ctl = nullptr;
};

#endif /* __QMeasureHarrisCornerResponse_h__ */
