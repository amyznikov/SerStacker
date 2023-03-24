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
#include <gui/qgraphicsshape/QGraphicsLineShape.h>
#include <gui/qgraphicsshape/QGraphicsTargetShape.h>
#include <gui/widgets/QSettingsWidget.h>
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

  void set_debayer_algorithm(DEBAYER_ALGORITHM algo);
  DEBAYER_ALGORITHM debayer_algorithm();

  const QCameraFrameMtfDisplayFunction * mtfDisplayFunction() const;
  QCameraFrameMtfDisplayFunction * mtfDisplayFunction();

  void setFrameProcessor(const c_image_processor::sptr & processor);

  QGraphicsRectShape * rectShape() const;
  QGraphicsLineShape * lineShape() const;
  QGraphicsTargetShape * targetShape() const;


Q_SIGNALS:
  void pixmapChanged();

protected Q_SLOTS:
  void onPixmapChanged();
  void onCameraStateChanged(QImagingCamera::State oldSate,
      QImagingCamera::State newState);

protected:
  void startWorkerThread();
  void stopWorkerThread();
  void workerThread();
  void createShapes();

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

protected:
  QImagingCamera::sptr camera_;
  QCameraFrameMtfDisplayFunction mtfDisplayFunction_;

  enum WorkerState {
    Worker_Idle,
    Worker_Starting,
    Worker_Running,
    Worker_Stopping,
  } workerState_ = Worker_Idle;

  enum COLORID colorid_ = COLORID_UNKNOWN;
  enum DEBAYER_ALGORITHM debayer_algorithm_ = DEBAYER_NN;
  int bpp_ = 0;
  QPixmap pixmap_;

  QGraphicsRectShape * rectShape_ = nullptr;
  QGraphicsLineShape * lineShape_ = nullptr;
  QGraphicsTargetShape * targetShape_ = nullptr;

};


class QDisplayFrameProcessorSettingsWidget :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QDisplayFrameProcessorSettingsWidget ThisClass;
  typedef QSettingsWidget Base;

  QDisplayFrameProcessorSettingsWidget(QWidget * parent = nullptr);

  void setDisplay(QCameraFrameDisplay * display);
  QCameraFrameDisplay* display() const;

protected:
  void onupdatecontrols() override;

protected:
  QCameraFrameDisplay * display_ = nullptr;
  QEnumComboBox<DEBAYER_ALGORITHM> * debayer_ctl = nullptr;
};

class QDisplayFrameProcessorSettingsDialogBox :
    public QDialog
{
  Q_OBJECT;
public:
  typedef QDisplayFrameProcessorSettingsDialogBox ThisClass;
  typedef QDialog Base;

  QDisplayFrameProcessorSettingsDialogBox(QWidget * parent = nullptr);

  void setDisplay(QCameraFrameDisplay * display);
  QCameraFrameDisplay* display() const;

Q_SIGNALS:
  void visibilityChanged(bool visible);

protected:
  void closeEvent(QCloseEvent *) override;
  void showEvent(QShowEvent *e) override;
  void hideEvent(QHideEvent *e) override;

protected:
  QVBoxLayout * lv_ = nullptr;
  QDisplayFrameProcessorSettingsWidget * setiingsWidget_ = nullptr;
};


} /* namespace serimager */

#endif /* __QCameraFrameDisplay_h__ */
