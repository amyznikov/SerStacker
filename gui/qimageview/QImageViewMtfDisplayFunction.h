/*
 * QImageViewMtfDisplayFunction.h
 *
 *  Created on: Jan 6, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QImageViewMtfDisplayFunction_h__
#define __QImageViewMtfDisplayFunction_h__

#include <gui/qmtf/QMtfDisplay.h>
#include "QImageViewer.h"

class QImageViewMtfDisplayFunction :
    public QMtfDisplay,
    public QImageDisplayFunction
{
  Q_OBJECT;
public:
  typedef QImageViewMtfDisplayFunction ThisClass;

  QImageViewMtfDisplayFunction(QImageViewer * imageViewer,
      const QString & prefix = "");

  QImageViewer * imageViewer() const;

  const c_enum_member * displayTypes() const override;
  void getInputDataRange(double * minval, double * maxval) const override;
  void getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax) override;
  void getOutputHistogramm(cv::OutputArray H, double * hmin, double * hmax) override;

  void createDisplayImage(cv::InputArray currentImage, cv::InputArray currentMask,
      cv::OutputArray displayImage, int ddepth = CV_8U) override;

  bool applyMtf(cv::InputArray currentImage, cv::InputArray currentMask,
      cv::OutputArray displayImage, int ddepth = CV_8U);

  bool applyColorMap(cv::InputArray displayImage, cv::InputArray displayMask,
      cv::OutputArray colormapImage);

protected:
  void adjustMtfInputRange(c_midtones_transfer_function *mtf, double * imin, double * imax) const override;
  void restoreMtfInputRange(c_midtones_transfer_function *mtf, double imin, double imax) const override;

protected:
  QImageViewer * imageViewer_ = nullptr;
};

#endif /* __QImageViewMtfDisplayFunction_h__ */
