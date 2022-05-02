/*
 * QMtfImageDisplayFunction.h
 *
 *  Created on: Apr 22, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMtfImageDisplayFunction_h__
#define __QMtfImageDisplayFunction_h__

#include "QImageDisplayFunction.h"
#include <gui/qmtf/QMtfDisplaySettings.h>


class QMtfImageDisplaySettings :
    public QMtfDisplaySettings
{
  Q_OBJECT;
public:
  typedef QMtfImageDisplaySettings ThisClass;
  typedef QMtfDisplaySettings Base;

  QMtfImageDisplaySettings(QObject * parent = Q_NULLPTR);

  virtual void setCurrentImage(cv::InputArray image, cv::InputArray mask);
  const cv::Mat & currentImage() const;
  const cv::Mat & currentMask() const;

  void getInputDataRange(double * minval, double * maxval) const override;
  void getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax) override;
  void getOutputHistogramm(cv::OutputArray H, double * hmin, double * hmax) override;
  virtual void getDisplayImage(cv::OutputArray image, int ddepth = -1);

protected:
  cv::Mat currentImage_;
  cv::Mat currentMask_;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class QMtfImageDisplayFunction:
    public QImageDisplayFunction
{
  Q_OBJECT;
public:
  typedef QMtfImageDisplayFunction ThisClass;
  typedef QImageDisplayFunction Base;

  QMtfImageDisplayFunction(QObject * parent = Q_NULLPTR);
  QMtfImageDisplayFunction(QMtfImageDisplaySettings * settings, QObject * parent = Q_NULLPTR);

  void setDisplaySettings(QMtfImageDisplaySettings * settings);

  QMtfImageDisplaySettings * displaySettings();
  const QMtfImageDisplaySettings * displaySettings() const;

  void setCurrentImage(cv::InputArray image, cv::InputArray mask) override;
  void getDisplayImage(cv::OutputArray image, int ddepth = -1) override;

protected:
  QMtfImageDisplaySettings * displaySettings_ = Q_NULLPTR;
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class  QMtfImageDefaultDisplaySettings:
    public QMtfImageDisplaySettings
{
  Q_OBJECT;
public:
  typedef QMtfImageDefaultDisplaySettings ThisClass;
  typedef QMtfImageDisplaySettings Base;

  QMtfImageDefaultDisplaySettings(QObject * parent = Q_NULLPTR);
  const c_enum_member * displayTypes() const override;

  void loadParameters() override;
  void saveParameters() const override;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif /* __QMtfImageDisplayFunction_h__ */
