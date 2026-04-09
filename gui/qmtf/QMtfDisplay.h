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
#include <core/mtf/mtf.h>
#include <core/proc/colormap.h>
#include <core/ssprintf.h>


class QMtfDisplayEvents :
    public QObject
{
  Q_OBJECT;
public:
  typedef QMtfDisplayEvents ThisClass;
  typedef QObject Base;

Q_SIGNALS:
  void displayChannelsChanged();
  void displayImageChanged();
  void parameterChanged();

protected:
  friend class IMtfDisplay;
  explicit QMtfDisplayEvents(QObject * parent) : Base(parent) {}
};


/**
 * Base interface for mtf display options
 */
class IMtfDisplay
{
public:
  struct DisplayParams {
    COLORMAP colormap = COLORMAP_NONE;
    cv::Mat3b lut;
    c_mtf mtf;
    bool invert_colormap = false;
  };

  typedef std::map<QString, DisplayParams, std::less<QString>> DisplayMap;

  IMtfDisplay(QObject * parent, const QString & prefix = "");
  virtual ~IMtfDisplay() = default;

  QMtfDisplayEvents * mtfDisplayEvents() const
  {
    return _events;
  }

  virtual QStringList displayChannels() const;

  virtual void setDisplayChannel(const QString & v);
  virtual const QString & displayChannel() const;

  virtual void setlclip(double v);
  virtual double lclip() const;

  virtual void sethclip(double v);
  virtual double hclip() const;

  virtual void setShadows(double v);
  virtual double shadows() const;

  virtual void setHighlights(double v);
  virtual double highlights() const;

  virtual void setMidtones(double v);
  virtual double midtones() const;

  virtual void setMtf(double imin, double imax, const c_mtf_options * opts = nullptr);
  virtual void getMtf(c_mtf_options * opts) const;
  virtual void getMtfInputRange(double * min, double * max) const;

  virtual void setColormap(COLORMAP v);
  virtual COLORMAP colormap() const;

  virtual void setInvertColormap(bool v);
  virtual bool invertColormap() const;

  virtual void setAutoClip(bool v);
  virtual bool autoClip() const;

  virtual DisplayParams & displayParams();
  virtual const DisplayParams & displayParams() const;

  void loadParameters();
  void saveParameters() const;

  virtual void getInputDataRange(double * output_minval, double * output_maxval) const = 0;
  virtual void getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax, bool cumulative = false, bool normalized = false) = 0;
  virtual void getOutputHistogramm(cv::OutputArray H, double * output_hmin, double * output_hmax) = 0;
  virtual void getMtfCurve(std::vector<float> & cy, size_t n)  = 0;

  static DisplayParams & addDisplay(DisplayMap & map,
      const QString & displayChannelName, double input_min, double input_max);

protected:
  void addDisplay(const QString & displayChannelName, double input_min, double input_max);
  static void createLut(COLORMAP colormap, cv::Mat3b & lut, bool invert_colormap);

  struct c_mtf_adjustment {
    double imin = -1, imax = -1;
    double omin = -1, omax = -1;
    bool adjusted_inputs = false;
    bool adjusted_outputs = false;
  };

  virtual void adjustMtfRange(c_mtf *mtf,
      cv::InputArray currentImage, cv::InputArray currentMask,
      c_mtf_adjustment * adjustment) const;

  virtual void restoreMtfRange(c_mtf *mtf,
      const c_mtf_adjustment & adjustment)  const;

  virtual void loadParameters(const QSettings & settings, const QString & prefix);
  virtual void saveParameters(QSettings & settings, const QString & prefix) const;



protected:
  QMtfDisplayEvents * _events = nullptr;
  QString _displayChannel;
  DisplayMap _displays;
  bool _autoClip = false;
  QString _prefix;
};

#endif /* __QMtfDisplaySettings_h__ */
