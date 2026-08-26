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
#include <gui/qimageview/ImageViewMtfDisplayFunction.h>
#include <gui/qgraphicsshape/QGraphicsRectShape.h>
#include <gui/qgraphicsshape/QGraphicsLineShape.h>
#include <gui/qgraphicsshape/QGraphicsTargetShape.h>
#include <gui/widgets/QSettingsWidget.h>
#include <gui/qpipeline/QImageProcessingPipeline.h>
#include <core/pipeline/c_generic_image_processor_pipeline/c_generic_image_processor_pipeline.h>
#include <core/io/debayer.h>
#include <core/settings/opencv_settings.h>
#include "camera/QImagingCamera.h"
#include "camera/QFrameQualityEstimation.h"
#include <thread>
#include <condition_variable>

namespace serimager {

///////////////////////////////////////////////////////////////////////////////
class QLivePipelineThread;
//
//class QLivePipeline :
//  public QImageProcessingPipelineTemplate<c_generic_image_processor_pipeline>
//{
//public:
//  typedef QLivePipeline ThisClass;
//  typedef QImageProcessingPipelineTemplate<c_generic_image_processor_pipeline> Base;
//  typedef Base::PipelineClass PipelineClass;
//
//  QLivePipeline(const QString & name, QLivePipelineThread * parent);
//
//protected:
//  bool get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask) final;
//
//protected:
//  QLivePipelineThread * _liveThread = nullptr;
//};

///////////////////////////////////////////////////////////////////////////////

class QLiveDisplay :
  public QImageEditor,
  public ImageViewMtfDisplayFunction
{
  Q_OBJECT;
public:
  typedef QLiveDisplay ThisClass;
  typedef QImageEditor Base;
  typedef ImageViewMtfDisplayFunction MtfDisplayFunction;

  enum INPUT_SOURCE {
    INPUT_SOURCE_CAMERA,
    INPUT_SOURCE_PIPELINE,
  };


  QLiveDisplay(QWidget * parent = nullptr);
  ~QLiveDisplay();

  QGraphicsRectShape * roiShape() const;
  QGraphicsLineShape * lineShape() const;
  QGraphicsTargetShape * targetShape() const;

  void setInputSource(INPUT_SOURCE v);
  INPUT_SOURCE inputSource() const;

  void setCamera(const QImagingCamera::sptr & camera);
  const QImagingCamera::sptr& camera() const;

  void setCurrentPipeline(const c_image_processing_pipeline::sptr & pipeline);
  const c_image_processing_pipeline::sptr & currentPipeline() const;

  void setPaused(bool paused);
  bool paused() const;

  void setDebayer(DEBAYER_ALGORITHM algo);
  DEBAYER_ALGORITHM debayer() const;

protected Q_SLOTS:
  void onCameraStateChanged(QImagingCamera::State oldState,
      QImagingCamera::State newState);

protected:
  void createShapes();
  void toggleUpdateTimer();
  void timerEvent(QTimerEvent *event) override;

protected:
  friend class QLivePipelineThread;
  QGraphicsRectShape * _roiShape = nullptr;
  QGraphicsLineShape * _lineShape = nullptr;
  QGraphicsTargetShape * _targetShape = nullptr;

  QImagingCamera::sptr _camera;
  c_image_processing_pipeline::sptr _currentPipeline;
  INPUT_SOURCE _inputSource = INPUT_SOURCE_PIPELINE;
  DEBAYER_ALGORITHM _debayer_algo = DEBAYER_NN2;

  int _updateTimerId = -1;
  int _currentUpdateInterval = 0;
  int _lastCameraFrameIndex = -1;
  bool _paused = false;
  bool _pipelineFrameReady = false;
};


class QLiveDisplaySettingsWidget :
    public QSettingsWidgetTemplate<QLiveDisplay>
{
  Q_OBJECT;
public:
  typedef QLiveDisplaySettingsWidget ThisClass;
  typedef QSettingsWidgetTemplate<QLiveDisplay> Base;

  QLiveDisplaySettingsWidget(QWidget * parent = nullptr);
  QLiveDisplaySettingsWidget(QLiveDisplay * liveDisplay, QWidget * parent = nullptr);

  void setLiveDisplay(QLiveDisplay * liveDisplay);
  QLiveDisplay * liveDisplay() const;

protected:
  QEnumComboBox<DEBAYER_ALGORITHM> * debayer_ctl = nullptr;
};

class QLiveDisplaySettingsDialogBox :
    public QSettingsDialogBoxTemplate<QLiveDisplaySettingsWidget>
{
  Q_OBJECT;
public:
  typedef QLiveDisplaySettingsDialogBox ThisClass;
  typedef QSettingsDialogBoxTemplate<QLiveDisplaySettingsWidget> Base;

  QLiveDisplaySettingsDialogBox(QWidget * parent = nullptr);

  void setLiveDisplay(QLiveDisplay * liveDisplay);
  QLiveDisplay * liveDisplay() const;

protected:
  void closeEvent(QCloseEvent *) override;
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

  void setCamera(const QImagingCamera::sptr & camera);
  const QImagingCamera::sptr& camera() const;

  void setPipeline(const c_image_processing_pipeline::sptr & pipeline);
  const c_image_processing_pipeline::sptr & pipeline() const;

  void setFrameQualityEstimator(QFrameQualityEstimation * estimator);
  QFrameQualityEstimation* frameQualityEstimator() const;

protected Q_SLOTS:
  void onCameraStateChanged(QImagingCamera::State oldState,
      QImagingCamera::State newState);

Q_SIGNALS:
  void pipelineChanged();
  //void frameReady();

protected:
//  void setCurrentPipeline(const c_image_processing_pipeline::sptr & pipeline);
//  void setDarkFrame(const QString & pathfilename);
  void loadSettings();
  void saveSettings();
  void run() final;

protected:
  QMutex _lock;
  QWaitCondition _condvar;
  std::atomic_bool _stopRequested{false};
  QImagingCamera::sptr _camera;
  c_image_processing_pipeline::sptr _currentPipeline;
  QFrameQualityEstimation * _qualityEstimator = nullptr;
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
  QLivePipelineThread * _liveThread = nullptr;
  std::string _configFilename;

  QVBoxLayout * _layout = nullptr;
  QToolBar * toolbar_ctl = nullptr;
  QComboBox * combobox_ctl = nullptr;
  QToolButton * startStop_ctl = nullptr;
  QToolButton * menuButton_ctl = nullptr;

  QScrollArea * scrollArea_ctl = nullptr;
  QList<QPipelineSettingsWidget *> _settingsWidgets;
};

///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////

} /* namespace serimager */

#endif /* __QLivePipeline_h__ */
