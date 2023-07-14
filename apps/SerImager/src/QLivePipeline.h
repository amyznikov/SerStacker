/*
 * QLivePipeline.h
 *
 *  Created on: Mar 20, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QLivePipeline_h__
#define __QLivePipeline_h__

#include <QtCore/QtCore>
#include <gui/widgets/UpdateControls.h>
#include <gui/qimageview/QImageEditor.h>
#include <gui/qimageview/QImageViewMtfDisplayFunction.h>
#include <gui/qgraphicsshape/QGraphicsRectShape.h>
#include <gui/qgraphicsshape/QGraphicsLineShape.h>
#include <gui/qgraphicsshape/QGraphicsTargetShape.h>
#include <gui/widgets/QSettingsWidget.h>
#include <gui/qpipeline/QImageProcessingPipeline.h>
#include <core/io/debayer.h>
#include <core/settings/opencv_settings.h>
#include "camera/QImagingCamera.h"
#include <thread>
#include <condition_variable>

namespace serimager {

///////////////////////////////////////////////////////////////////////////////

class QLiveDisplayMtfFunction :
    public QImageViewMtfDisplayFunction
{
  Q_OBJECT;
public:
  typedef QLiveDisplayMtfFunction ThisClass;
  typedef QImageViewMtfDisplayFunction Base;

  QLiveDisplayMtfFunction(QImageViewer * imageViewer);

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

protected:
  mutable std::mutex mutex_;
  bool isBusy_ = false;
};

///////////////////////////////////////////////////////////////////////////////

class QLiveDisplay :
    public QImageEditor
{
  Q_OBJECT;
public:
  typedef QLiveDisplay ThisClass;
  typedef QImageEditor Base;

  QLiveDisplay(QWidget * parent = nullptr);
  ~QLiveDisplay();

  void showVideoFrame(const cv::Mat & image, COLORID colorid, int bpp);


  const QLiveDisplayMtfFunction * mtfDisplayFunction() const;
  QLiveDisplayMtfFunction * mtfDisplayFunction();

  void setFrameProcessor(const c_image_processor::sptr & processor);

  QGraphicsRectShape * rectShape() const;
  QGraphicsLineShape * lineShape() const;
  QGraphicsTargetShape * targetShape() const;

Q_SIGNALS:
  void pixmapChanged(QPrivateSignal * p = nullptr);

protected Q_SLOTS:
  void onPixmapChanged();

protected:
  void createShapes();

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

protected:
  QLiveDisplayMtfFunction mtfDisplayFunction_;

  QPixmap pixmap_;

  QGraphicsRectShape * rectShape_ = nullptr;
  QGraphicsLineShape * lineShape_ = nullptr;
  QGraphicsTargetShape * targetShape_ = nullptr;
};

///////////////////////////////////////////////////////////////////////////////

class QLivePipelineThread :
    public QThread
{
  Q_OBJECT;
public:
  typedef QLivePipelineThread ThisClass;
  typedef QThread Base;

  QLivePipelineThread(QObject * parent = nullptr);
  ~QLivePipelineThread();

//  void finish(bool wait = true);

  void setDisplay(QLiveDisplay * display);
  QLiveDisplay* display() const;

  void setCamera(const QImagingCamera::sptr & camera);
  const QImagingCamera::sptr& camera() const;

  bool setPipeline(const c_image_processing_pipeline::sptr & pipeline);
  const c_image_processing_pipeline::sptr & pipeline() const;

  void setDebayer(DEBAYER_ALGORITHM algo);
  DEBAYER_ALGORITHM debayer() const;

  void setDarkFramePath(const QString & pathfilename);
  const QString & darkFramePath() const;

  void setDarkFrameScale(double v);
  double darkFrameScale() const;

protected Q_SLOTS:
  // void onRestartAfterException();
  void onCameraStateChanged(QImagingCamera::State oldState,
      QImagingCamera::State newState);

Q_SIGNALS:
  void pipelineChanged();

protected:
  void setDarkFrame(const QString & pathfilename);
  void load_settings();
  void save_settings();
  void run() override;


protected:
  using unique_lock = std::unique_lock<std::mutex>;
  using lock_guard = std::lock_guard<std::mutex>;
  std::mutex mutex_;
  std::condition_variable condvar_;

  QImagingCamera::sptr camera_;
  QLiveDisplay *display_ = nullptr;
  c_image_processing_pipeline::sptr pipeline_;

  std::atomic<DEBAYER_ALGORITHM> debayer_ = DEBAYER_NN;
  QString darkFramePath_;
  cv::Mat darkFrame_;
  double darkFrameScale_ = 1; // auto
  std::mutex darkFrameLock_;

  std::atomic_bool finish_ = false;
};

///////////////////////////////////////////////////////////////////////////////

class QLivePipelineSelectionWidget :
    public QFrame,
    public HasUpdateControls
{
  Q_OBJECT;
public:
  typedef QLivePipelineSelectionWidget ThisClass;
  typedef QFrame Base;

  QLivePipelineSelectionWidget(QWidget * parent = nullptr);

  void setLiveThread(QLivePipelineThread * liveThread);
  QLivePipelineThread * liveThread() const;

  c_image_processing_pipeline::sptr selectedPipeline() const;

  void loadPipelines(const std::string & cfgfilename = "");
  void savePipelines(const std::string & cfgfilename = "");

Q_SIGNALS:
  void parameterChanged();

protected Q_SLOTS:
  void onPipelinesComboboxCurrentIndexChanged(int);
  void onStartStopCtlClicked();
  void onMenuCtlClicked();
  void onAddLivePipelineClicked();
  void onRemoveLivePipelineClicked();
  void onRenameLivePipelineClicked();
  void onPipelineChanged();
  // void onLiveThreadStateChanged();

protected:
  void onupdatecontrols() override;

protected:
  QLivePipelineThread * liveThread_ = nullptr;
  std::string config_filename_;

  QVBoxLayout * layout_ = nullptr;
  QToolBar * toolbar_ctl = nullptr;
  QComboBox * combobox_ctl = nullptr;
  QToolButton * startStop_ctl = nullptr;
  QToolButton * menuButton_ctl = nullptr;

  QScrollArea * scrollArea_ctl = nullptr;
  QList<QPipelineSettingsWidget *> settingsWidgets_;
};

///////////////////////////////////////////////////////////////////////////////


class QLiveThreadSettingsWidget :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QLiveThreadSettingsWidget ThisClass;
  typedef QSettingsWidget Base;

  QLiveThreadSettingsWidget(QWidget * parent = nullptr);
  QLiveThreadSettingsWidget(QLivePipelineThread * liveThread, QWidget * parent = nullptr);

  void setLiveThread(QLivePipelineThread * liveThread);
  QLivePipelineThread * liveThread() const;

protected:
  void onupdatecontrols() override;

protected:
  QLivePipelineThread * liveThread_ = nullptr;
  QEnumComboBox<DEBAYER_ALGORITHM> * debayer_ctl = nullptr;
  QBrowsePathCombo * darkframe_ctl = nullptr;
  QNumericBox * darkFrameScale_ctl  = nullptr;
};

class QLiveThreadSettingsDialogBox :
    public QDialog
{
  Q_OBJECT;
public:
  typedef QLiveThreadSettingsDialogBox ThisClass;
  typedef QDialog Base;

  QLiveThreadSettingsDialogBox(QWidget * parent = nullptr);

  void setLiveThread(QLivePipelineThread * liveThread);
  QLivePipelineThread * liveThread() const;

Q_SIGNALS:
  void visibilityChanged(bool visible);

protected:
  void closeEvent(QCloseEvent *) override;
  void showEvent(QShowEvent *e) override;
  void hideEvent(QHideEvent *e) override;

protected:
  QVBoxLayout * layout_ = nullptr;
  QLiveThreadSettingsWidget * setiingsWidget_ = nullptr;

};

///////////////////////////////////////////////////////////////////////////////

} /* namespace serimager */

#endif /* __QLivePipeline_h__ */
