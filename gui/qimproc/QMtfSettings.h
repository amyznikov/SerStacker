/*
 * QMtfSettings.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMtfSettings_h__
#define __QMtfSettings_h__

#include <core/improc/c_mtf_routine.h>
#include <gui/qmtf/QMtfControl.h>
#include <gui/qimageview/QMtfImageDisplayFunction.h>
#include "QImageProcessorSelector.h"


class QMtfRoutineDisplaySettings
  : public QMtfDisplaySettingsBase
{
  Q_OBJECT;
public:
  typedef QMtfRoutineDisplaySettings ThisClass;
  typedef QMtfDisplaySettingsBase Base;

  QMtfRoutineDisplaySettings(const c_mtf_routine::ptr & processor,
      QObject * parent = Q_NULLPTR);

  const c_enum_member * displayTypes() const override;

  void setDisplayType(int v) override;
  virtual int displayType() const override;

  void setColormap(COLORMAP v) override;
  COLORMAP colormap() const override;

  c_pixinsight_mtf & mtf() override;
  const c_pixinsight_mtf & mtf() const override;

  void getInputDataRange(double * minval, double * maxval) const override;
  void getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax) override;
  void getOutputHistogramm(cv::OutputArray H, double * hmin, double * hmax) override;

  void loadParameters() override;
  void saveParameters() const override;

protected:
  virtual void loadParameters(const QString & prefix);
  virtual void saveParameters(const QString & prefix) const;

protected:
  c_mtf_routine::ptr processor_;
  double imin = 0, imax = 0;
  double omin = 0, omax = 255;
  cv::Mat iH, oH;
};


class QMtfSettings
  : public QImageProcessorRoutineSettings<c_mtf_routine>
{
  Q_OBJECT;
public:
  typedef QMtfSettings ThisClass;
  typedef QImageProcessorRoutineSettings Base;

  static const struct ClassFactory : public Base::ClassFactory {
    ClassFactory() :
        Base::ClassFactory(&RoutineType::class_factory,
            SettingsWidgetFactory([](const c_image_processor_routine::ptr & routine, QWidget * parent) {
              return new ThisClass(std::dynamic_pointer_cast<RoutineType>(routine), parent );
            }))
    {}
  } classFactory;

  QMtfSettings(const c_mtf_routine::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QMtfControl * mtf_ctl = Q_NULLPTR;
  QMtfRoutineDisplaySettings displaySettings_;
};

#endif /* __QMtfSettings_h__ */
