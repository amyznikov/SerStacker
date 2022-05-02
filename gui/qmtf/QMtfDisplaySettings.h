/*
 * QMtfDisplaySettings.h
 *
 *  Created on: Apr 19, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMtfDisplaySettings_h__
#define __QMtfDisplaySettings_h__

#include <QtCore/QtCore>
#include <core/mtf/c_pixinsight_mtf.h>
#include <core/proc/colormap.h>
#include <core/ssprintf.h>

/**
 * Base interface for mtf display options
 */
class QMtfDisplaySettingsBase :
    public QObject
{
  Q_OBJECT;
public:
  typedef QMtfDisplaySettingsBase ThisClass;
  typedef QObject Base;

  QMtfDisplaySettingsBase(QObject * parent = Q_NULLPTR) :
      Base(parent)
  {
  }

  virtual const c_enum_member * displayTypes() const = 0;

  virtual void setDisplayType(int v) = 0;
  virtual int displayType() const = 0;

  virtual void setColormap(COLORMAP v) = 0;
  virtual COLORMAP colormap() const = 0;

  virtual c_pixinsight_mtf & mtf() = 0;
  virtual const c_pixinsight_mtf & mtf() const = 0;

  virtual void getInputDataRange(double * minval, double * maxval) const = 0;
  virtual void getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax) = 0;
  virtual void getOutputHistogramm(cv::OutputArray H, double * hmin, double * hmax) = 0;

  virtual void loadParameters()
  {
    loadParameters("QMtfDisplaySettingsBase");
  }

  virtual void saveParameters() const
  {
    saveParameters("QMtfDisplaySettingsBase");
  }

signals:
  void inputDataChanged();
  void updateDisplay();

protected:
  static void createLut(COLORMAP colormap, cv::Mat3b & lut);
  virtual void loadParameters(const QString & prefix) {}
  virtual void saveParameters(const QString & prefix) const {}
};


/**
 * Base interface for mtf display options
 */
class QMtfDisplaySettings :
    public QMtfDisplaySettingsBase
{
  Q_OBJECT;
public:
  typedef QMtfDisplaySettings ThisClass;
  typedef QMtfDisplaySettingsBase Base;

  struct DisplayParams {
    COLORMAP colormap = COLORMAP_NONE;
    cv::Mat3b lut;
    c_pixinsight_mtf mtf;
  };
  typedef std::map<int /* display_type*/, DisplayParams>
    DisplayMap;

  QMtfDisplaySettings(QObject * parent = Q_NULLPTR);

  virtual const c_enum_member * displayTypes() const = 0;

  virtual void setDisplayType(int v);
  virtual int displayType() const;

  virtual void setColormap(COLORMAP v);
  virtual COLORMAP colormap() const;

  virtual DisplayParams & displayParams();
  virtual const DisplayParams & displayParams() const;

  virtual c_pixinsight_mtf & mtf();
  virtual const c_pixinsight_mtf & mtf() const;

  virtual void getInputDataRange(double * minval, double * maxval) const = 0;
  virtual void getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax) = 0;
  virtual void getOutputHistogramm(cv::OutputArray H, double * hmin, double * hmax) = 0;

  virtual void loadParameters();
  virtual void saveParameters() const;

protected:
  virtual void loadParameters(const QString & prefix);
  virtual void saveParameters(const QString & prefix) const;

protected:
  static void addDisplay(DisplayMap & map, int type, double rmin, double rmax);

  int displayType_ = -1;
  DisplayMap displayParams_;
};

#endif /* __QMtfDisplaySettings_h__ */
