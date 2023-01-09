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
class QMtfDisplay :
    public QObject
{
  Q_OBJECT;
public:

  struct DisplayParams {
    COLORMAP colormap = COLORMAP_NONE;
    cv::Mat3b lut;
    c_pixinsight_mtf mtf;
    bool invert_colormap = false;
  };

  typedef QMtfDisplay ThisClass;
  typedef QObject Base;
  typedef std::map<int /*display_type*/, DisplayParams> DisplayMap;


  QMtfDisplay(const QString & prefix, QObject * parent = nullptr);

  virtual const c_enum_member * displayTypes() const = 0;

  virtual void setDisplayType(int v);
  virtual int displayType() const;

  virtual void setMtfInputRange(double min, double max);
  virtual void getMtfInputRange(double * min, double * max) const;

  virtual void setShadows(double shadows);
  virtual double shadows() const;

  virtual void setHighlights(double highlights);
  virtual double highlights() const;

  virtual void setMidtones(double midtones);
  virtual double midtones() const;

  virtual void setMtf(double shadows, double highlights, double midtones);
  virtual void getMtf(double * shadows, double * highlights, double * midtones) const;

  virtual void setColormap(COLORMAP v);
  virtual COLORMAP colormap() const;

  virtual void setInvertColormap(bool v);
  virtual bool invertColormap() const;

  virtual DisplayParams & displayParams();
  virtual const DisplayParams & displayParams() const;

  //  virtual c_pixinsight_mtf & mtf();
  //  virtual const c_pixinsight_mtf & mtf() const;

  void loadParameters();
  void saveParameters() const;

  virtual void getInputDataRange(double * minval, double * maxval) const = 0;
  virtual void getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax) = 0;
  virtual void getOutputHistogramm(cv::OutputArray H, double * hmin, double * hmax) = 0;


Q_SIGNALS:
  void parameterChanged();
  void displayImageChanged();

//  void inputDataChanged();
//  void updateDisplay();

protected:
  void addDisplay(int type, double rmin, double rmax);
  static void createLut(COLORMAP colormap, cv::Mat3b & lut, bool invert_colormap);

  virtual void adjustMtfInputRange(c_midtones_transfer_function *mtf, double * imin, double * imax) const;
  virtual void restoreMtfInputRange(c_midtones_transfer_function *mtf, double imin, double imax)  const;

  virtual void loadParameters(const QSettings & settings, const QString & prefix);
  virtual void saveParameters(QSettings & settings, const QString & prefix) const;



protected:
  int displayType_ = -1;
  DisplayMap displayParams_;
  QString prefix_;
};

#endif /* __QMtfDisplaySettings_h__ */
