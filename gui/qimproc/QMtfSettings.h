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
#include <gui/qimageview/QImageViewMtfDisplayFunction.h>
#include "QImageProcessorChainEditor.h"
//#include "QImageProcessorSelector.h"


class QMtfRoutineDisplaySettings :
    public QMtfDisplay
{
  Q_OBJECT;
public:
  typedef QMtfRoutineDisplaySettings ThisClass;
  typedef QMtfDisplay Base;

  QMtfRoutineDisplaySettings(const c_mtf_routine::ptr & processor,
      QObject * parent = nullptr);

  const c_enum_member * displayChannels() const override;

  void setDisplayChannel(int v) override;
  virtual int displayChannel() const override;

  void setMtfInputRange(double min, double max) override;
  void getMtfInputRange(double * min, double * max) const override;

  void setShadows(double shadows) override;
  double shadows() const override;

  void setHighlights(double highlights) override;
  double highlights() const override;

  void setMidtones(double midtones) override;
  double midtones() const override;

  void setMtf(double shadows, double highlights, double midtones) override;
  void getMtf(double * shadows, double * highlights, double * midtones) const override;

  void setColormap(COLORMAP v) override;
  COLORMAP colormap() const override;

  void setInvertColormap(bool v) override;
  bool invertColormap() const override;

  void getInputDataRange(double * minval, double * maxval) const override;
  void getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax) override;
  void getOutputHistogramm(cv::OutputArray H, double * hmin, double * hmax) override;

//  void loadParameters() override;
//  void saveParameters() const override;

protected:
//  virtual void loadParameters(const QString & prefix);
//  virtual void saveParameters(const QString & prefix) const;

protected:
  c_mtf_routine::ptr processor_;
  double imin = 0, imax = 0;
  double omin = 0, omax = 255;
  cv::Mat iH, oH;
};


class QMtfSettings :
    public QImageProcessorSettingsControl
{
  Q_OBJECT;
public:
  typedef QMtfSettings ThisClass;
  typedef QImageProcessorSettingsControl Base;

  QMtfSettings(const c_mtf_routine::ptr & processor,
      QWidget * parent = nullptr);

protected:
  void setupControls() override;
  //void onupdatecontrols() override;

protected:
  QMtfControl * mtf_ctl = nullptr;
  //QMtfImageDisplayFunction displayFunction_;
  QMtfRoutineDisplaySettings displaySettings_;
};

#endif /* __QMtfSettings_h__ */
