/*
 * QVideoFrameDisplay.h
 *
 *  Created on: Mar 19, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QVideoFrameDisplay_h__
#define __QVideoFrameDisplay_h__

#include <gui/qimageview/QImageEditor.h>
#include <gui/qimageview/QImageViewMtfDisplayFunction.h>
#include <gui/qgraphicsshape/QGraphicsRectShape.h>
#include <gui/qgraphicsshape/QGraphicsLineShape.h>
#include <gui/qgraphicsshape/QGraphicsTargetShape.h>
#include <gui/widgets/QSettingsWidget.h>
#include <core/io/debayer.h>

namespace serimager {

class QVideoFrameMtfDisplayFunction :
    public QImageViewMtfDisplayFunction
{
  Q_OBJECT;
public:
  typedef QVideoFrameMtfDisplayFunction ThisClass;
  typedef QImageViewMtfDisplayFunction Base;

  QVideoFrameMtfDisplayFunction(QImageViewer * imageViewer);

  std::mutex & mutex()
  {
    return mutex_;
  }

  bool isBusy() const
  {
    return isBusy_;
  }

  void getInputDataRange(double * minval, double * maxval) const override;
  void getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax) override;
  void getOutputHistogramm(cv::OutputArray H, double * hmin, double * hmax) override;

  void createDisplayImage(cv::InputArray currentImage, cv::InputArray currentMask,
      cv::OutputArray displayImage, int ddepth = CV_8U) override;

protected:
  mutable std::mutex mutex_;
  bool isBusy_ = false;
};


class QVideoFrameDisplay :
    public QImageEditor
{
  Q_OBJECT;
public:
  typedef QVideoFrameDisplay ThisClass;
  typedef QImageEditor Base;

  QVideoFrameDisplay(QWidget * parent = nullptr);
  ~QVideoFrameDisplay();

  void showVideoFrame(const cv::Mat & image, COLORID colorid, int bpp);


  const QVideoFrameMtfDisplayFunction * mtfDisplayFunction() const;
  QVideoFrameMtfDisplayFunction * mtfDisplayFunction();

  void setFrameProcessor(const c_image_processor::sptr & processor);

  QGraphicsRectShape * rectShape() const;
  QGraphicsLineShape * lineShape() const;
  QGraphicsTargetShape * targetShape() const;

Q_SIGNALS:
  void pixmapChanged();

protected Q_SLOTS:
  void onPixmapChanged();

protected:
  void createShapes();

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

protected:
  QVideoFrameMtfDisplayFunction mtfDisplayFunction_;

  QPixmap pixmap_;

  QGraphicsRectShape * rectShape_ = nullptr;
  QGraphicsLineShape * lineShape_ = nullptr;
  QGraphicsTargetShape * targetShape_ = nullptr;
};


} /* namespace serimager */

#endif /* __QVideoFrameDisplay_h__ */
