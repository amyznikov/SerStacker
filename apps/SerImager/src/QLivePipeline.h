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
#include <gui/qpipeline/QGenericImageProcessingPipeline/QGenericImageProcessingPipeline.h>
#include <core/io/debayer.h>
#include <core/settings/opencv_settings.h>
#include "camera/QImagingCamera.h"
#include <thread>
#include <condition_variable>

namespace serimager {

///////////////////////////////////////////////////////////////////////////////

class QLivePipeline :
  public QGenericImageProcessingPipeline
{
public:
  typedef QLivePipeline ThisClass;
  typedef QGenericImageProcessingPipeline Base;
  typedef Base::PipelineClass PipelineClass;

  QLivePipeline(const QString & name, QObject * parent) :
      Base(name, nullptr, parent)
  {
  }
};

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

  QLiveDisplay(QWidget * parent = nullptr);
  ~QLiveDisplay();

  QGraphicsRectShape * rectShape() const;
  QGraphicsLineShape * lineShape() const;
  QGraphicsTargetShape * targetShape() const;

protected:
  void createShapes();

Q_SIGNALS:
  void inputImageReady(QPrivateSignal*p = nullptr);

protected:
  friend class QLivePipelineThread;
  std::atomic_bool _canAcceptFrame {true};
  QGraphicsRectShape * _rectShape = nullptr;
  QGraphicsLineShape * _lineShape = nullptr;
  QGraphicsTargetShape * _targetShape = nullptr;
};

///////////////////////////////////////////////////////////////////////////////

class QLivePipelineThread : public QThread
{
  Q_OBJECT;
public:
  typedef QLivePipelineThread ThisClass;
  typedef QThread Base;

  QLivePipelineThread(QObject * parent = nullptr);
  ~QLivePipelineThread();

  void setDisplay(QLiveDisplay * display);
  QLiveDisplay* display() const;

  void setCamera(const QImagingCamera::sptr & camera);
  const QImagingCamera::sptr& camera() const;

  void setPipeline(const c_image_processing_pipeline::sptr & pipeline);
  const c_image_processing_pipeline::sptr & pipeline() const;

  void setDebayer(DEBAYER_ALGORITHM algo);
  DEBAYER_ALGORITHM debayer() const;

  void setDarkFramePath(const QString & pathfilename);
  const QString & darkFramePath() const;

  void setDarkFrameScale(double v);
  double darkFrameScale() const;

protected Q_SLOTS:
  void onCameraStateChanged(QImagingCamera::State oldState,
      QImagingCamera::State newState);

Q_SIGNALS:
  void pipelineChanged();
  void frameReady();

protected:
  void setCurrentPipeline(const c_image_processing_pipeline::sptr & pipeline);
  void setDarkFrame(const QString & pathfilename);
  void loadSettings();
  void saveSettings();
  void run() final;


protected:
  QMutex _lock;
  QWaitCondition _condvar;

  QImagingCamera::sptr _camera;
  c_image_processing_pipeline::sptr _userPipeline;
  c_image_processing_pipeline::sptr _currentPipeline;
  QLiveDisplay * _display = nullptr;

  std::atomic<DEBAYER_ALGORITHM> _debayer = DEBAYER_NN;

  double _darkFrameScale = 1;
  QString _darkFramePath;
  cv::Mat _darkFrame;
  QMutex _darkFrameLock;
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


class QLiveThreadSettingsWidget :
    public QSettingsWidgetTemplate<QLivePipelineThread>
{
  Q_OBJECT;
public:
  typedef QLiveThreadSettingsWidget ThisClass;
  typedef QSettingsWidgetTemplate<QLivePipelineThread> Base;

  QLiveThreadSettingsWidget(QWidget * parent = nullptr);
  QLiveThreadSettingsWidget(QLivePipelineThread * liveThread, QWidget * parent = nullptr);

  void setLiveThread(QLivePipelineThread * liveThread);
  QLivePipelineThread * liveThread() const;

protected:
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
  QVBoxLayout * _layout = nullptr;
  QLiveThreadSettingsWidget * _setiingsWidget = nullptr;

};

///////////////////////////////////////////////////////////////////////////////

} /* namespace serimager */

#endif /* __QLivePipeline_h__ */
