/*
 * QCameraFrameDisplay.h
 *
 *  Created on: Dec 21, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCameraFrameDisplay_h__
#define __QCameraFrameDisplay_h__

#include <gui/qimageview/QImageFileEditor.h>
#include <gui/qimageview/QImageViewMtfDisplayFunction.h>
#include <gui/qgraphicsshape/QGraphicsRectShape.h>
#include "QImagingCamera.h"

namespace serimager {

class QCameraFrameMtfDisplayFunction :
    public QImageViewMtfDisplayFunction
{
  Q_OBJECT;
public:
  typedef QCameraFrameMtfDisplayFunction ThisClass;
  typedef QImageViewMtfDisplayFunction Base;

  QCameraFrameMtfDisplayFunction(QImageViewer * imageViewer);

  std::mutex & mutex();

  bool isBusy() const
  {
    return isBusy_;
  }

//  void setCurrentImage(cv::InputArray image, cv::InputArray mask) override;
  void getInputDataRange(double * minval, double * maxval) const override;
  void getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax) override;
  void getOutputHistogramm(cv::OutputArray H, double * hmin, double * hmax) override;

/*  void getDisplayImage(cv::OutputArray image, int ddepth = -1) override;*/
  void createDisplayImage(cv::InputArray currentImage, cv::InputArray currentMask,
      cv::OutputArray displayImage, int ddepth = CV_8U) override;

protected:
  mutable std::mutex mutex_;
  bool isBusy_ = false;
};


class QCameraFrameDisplay:
    public QImageEditor
{
  Q_OBJECT;
public:
  typedef QCameraFrameDisplay ThisClass;
  typedef QImageEditor Base;

  QCameraFrameDisplay(QWidget * parent = nullptr);
  ~QCameraFrameDisplay();

  void setCamera(const QImagingCamera::sptr & camera);
  const QImagingCamera::sptr & camera() const;

  const QCameraFrameMtfDisplayFunction * mtfDisplayFunction() const;
  QCameraFrameMtfDisplayFunction * mtfDisplayFunction();

  void setFrameProcessor(const c_image_processor::ptr & processor);

  void setShowROI(bool show);
  bool showROI() const;
  QRect roi() const;


Q_SIGNALS:
  void pixmapChanged();
  void roiChanged(const QRect & rc);

protected Q_SLOTS:
  void onPixmapChanged();
  void onCameraStateChanged(QImagingCamera::State oldSate,
      QImagingCamera::State newState);

protected:
  void startWorkerThread();
  void stopWorkerThread();
  void workerThread();
  void createFocusRoiRectItem();
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

protected:
  QImagingCamera::sptr camera_;
  QCameraFrameMtfDisplayFunction mtfDisplayFunction_;

  //std::mutex mutex_;
  enum WorkerState {
    Worker_Idle,
    Worker_Starting,
    Worker_Running,
    Worker_Stopping,
  } workerState_ = Worker_Idle;

  enum COLORID colorid_ = COLORID_UNKNOWN;
  int bpp_ = 0;
  QPixmap pixmap_;

  mutable QGraphicsRectShape * roiItem_ = nullptr;

};

} /* namespace qserimager */

#endif /* __QCameraFrameDisplay_h__ */
