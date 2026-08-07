/*
 * QMeasureLocalVariance.h
 *
 *  Created on: Jul 30, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMeasureLocalVariance_h__
#define __QMeasureLocalVariance_h__

#include "QMeasure.h"
#include <gui/widgets/QSettingsWidgetTemplate.h>
#include <core/proc/sharpness_measure/c_local_variance_sharpness_measure.h>

class QLocalVarianceMeasureSettingsWidget;

class QMeasureLocalVariance : public QMeasure,
    public c_local_variance_sharpness_measure
{
public :
  typedef QMeasureLocalVariance ThisClass;
  typedef QMeasure Base;

  QMeasureLocalVariance();
  int compute(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const final;
  QMeasureSettingsWidget* createSettingsWidget(QWidget * parent) const final;

  using c_control_list = c_ctlist<ThisClass> ;
  static void getcontrols(c_control_list & ctls)
  {
    ctlbind_local_variance_estimate_options(ctls, CTL_CONTEXT(
        c_ctlbind_context<ThisClass>(), opts));
  }
};

class QLocalVarianceMeasureSettingsWidget :
    public QMeasureSettingsWidgetTemplate<QMeasureLocalVariance>
{
public:
  typedef QLocalVarianceMeasureSettingsWidget ThisClass;
  typedef QMeasureSettingsWidgetTemplate<QMeasureLocalVariance> Base;
  QLocalVarianceMeasureSettingsWidget(QWidget * parent = nullptr);

//  QLocalVarianceMeasureSettingsWidget(QWidget * parent = nullptr) :
//    Base(parent)
//  {
//    c_ctlist<QMeasureLocalVariance> controls;
//    QMeasureLocalVariance::getcontrols(controls);
//    setupControls(this, controls);
//    updateControls();
//  }

protected:
  QEnumComboBox<color_channel_type> * color_channel_ctl = nullptr;
  QNumericBox * dscale_ctl = nullptr;
  QNumericBox * kradius_ctl = nullptr;
};

#endif /* __QMeasureLocalVariance_h__ */
