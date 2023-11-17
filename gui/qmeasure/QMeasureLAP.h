/*
 * QMeasureLAP.h
 *
 *  Created on: Sep 19, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMeasureLAP_h__
#define __QMeasureLAP_h__

#include "QMeasure.h"

class QMeasureLAP :
    public QMeasure
{
public:
  typedef QMeasureLAP ThisClass;
  typedef QMeasure Base;

  QMeasureLAP();

  QMeasureSettingsWidget * createSettingsWidget(QWidget * parent) const override;

  void set_dscale(int v);
  int dscale() const;

  void set_se_size(const cv::Size & v);
  const cv::Size & se_size() const;

protected:
  int compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const override;

protected:
  int dscale_ = 3;
  cv::Size se_size_ = cv::Size(5,5);
};

class QMeasureLAPSettingsWidget :
    public QMeasureSettingsWidgetTemplate<QMeasureLAP>
{
public:
  typedef QMeasureLAPSettingsWidget ThisClass;
  typedef QMeasureSettingsWidgetTemplate<QMeasureLAP> Base;

  QMeasureLAPSettingsWidget(QWidget * parent = nullptr);

protected:
  QNumericBox * dscale_ctl = nullptr;
  QNumericBox * se_size_ctl = nullptr;
};

#endif /* __QMeasureLAP_h__ */
