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
class IMtfDisplay
{
public:
  struct DisplayParams {
    COLORMAP colormap = COLORMAP_NONE;
    cv::Mat3b lut;
    c_pixinsight_mtf mtf;
    bool invert_colormap = false;
  };

  typedef std::map<QString, DisplayParams> DisplayMap;

  IMtfDisplay(const QString & prefix = "");

  virtual ~IMtfDisplay() = default;


  virtual QStringList displayChannels() const;

  virtual void setDisplayChannel(const QString & v);
  virtual const QString & displayChannel() const;

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

  virtual void setAutoClip(bool v);
  virtual bool autoClip() const;

  virtual DisplayParams & displayParams();
  virtual const DisplayParams & displayParams() const;

  //  virtual c_pixinsight_mtf & mtf();
  //  virtual const c_pixinsight_mtf & mtf() const;

  void loadParameters();
  void saveParameters() const;

  virtual void getInputDataRange(double * output_minval, double * output_maxval) const = 0;
  virtual void getInputHistogramm(cv::OutputArray H, double * output_hmin, double * output_hmax) = 0;
  virtual void getOutputHistogramm(cv::OutputArray H, double * output_hmin, double * output_hmax) = 0;

  static DisplayParams & addDisplay(DisplayMap & map,
      const QString & displayChannelName, double input_min, double input_max);

public: // events
  virtual void displayTypeChanged() = 0;
  virtual void parameterChanged() = 0;
  virtual void displayImageChanged() = 0;

protected:
  void addDisplay(const QString & displayChannelName, double input_min, double input_max);
  static void createLut(COLORMAP colormap, cv::Mat3b & lut, bool invert_colormap);

  struct c_mtf_adjustment {
    double imin = -1, imax = -1;
    double omin = -1, omax = -1;
    bool adjusted_inputs = false;
    bool adjusted_outputs = false;
  };

  virtual void adjustMtfRange(c_midtones_transfer_function *mtf,
      cv::InputArray currentImage, cv::InputArray currentMask,
      c_mtf_adjustment * adjustment) const;

  virtual void restoreMtfRange(c_midtones_transfer_function *mtf,
      const c_mtf_adjustment & adjustment)  const;

  virtual void loadParameters(const QSettings & settings, const QString & prefix);
  virtual void saveParameters(QSettings & settings, const QString & prefix) const;



protected:
  QString displayChannel_;
  //int displayChannel_ = -1;
  DisplayMap displays_;
  bool autoClip_ = false;
  QString prefix_;
};

Q_DECLARE_INTERFACE(IMtfDisplay, "IMtfDisplay");


class QMtfDisplay :
    public QObject,
    public IMtfDisplay
{
  Q_OBJECT;
  Q_INTERFACES(IMtfDisplay)
public:
  typedef QMtfDisplay ThisClass;
  typedef QObject Base;

  QMtfDisplay(const QString & prefix, QObject * parent = nullptr);

Q_SIGNALS:
  void displayTypeChanged();
  void parameterChanged();
  void displayImageChanged();
};

#endif /* __QMtfDisplaySettings_h__ */
