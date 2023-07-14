/*
 * MainWindow.h
 *
 *  Created on: Dec 21, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __qserimager_MainWindow_h__
#define __qserimager_MainWindow_h__

#include <gui/mainwindow/QMainAppWindow.h>

#include <gui/qgraphicsshape/QGraphicsRectShapeSettings.h>
#include <gui/qgraphicsshape/QGraphicsTargetShapeSettings.h>
#include <gui/qgraphicsshape/QGraphicsLineShapeSettings.h>
#include <gui/qdisplayvideowriter/QDisplayVideoWriterOptions.h>
#include <gui/widgets/QScaleSelectionButton.h>
#include "camera/QImagingCameraControlsWidget.h"
#include "QLivePipeline.h"

#if HAVE_INDIGO
# include "focus/indigo/QIndigoFocuserWidget.h"
#endif

namespace serimager {

class MainWindow:
    public QMainAppWindow
{
  Q_OBJECT;

public:
  typedef MainWindow ThisClass;
  typedef QMainAppWindow Base;

  MainWindow(QWidget * parent = nullptr);
  ~MainWindow();

protected:
  void setupPipelines();
  void setupMainMenu();
  void setupMainToolbar();
  void setupStatusbar();
  void setupCameraControls();
  void setupShapeOptions();
  void setupIndigoFocuser();
  void setupDisplayImageVideoWriter();

protected Q_SLOTS:
  void onCurrentImageChanged();
  void onCurrentDisplayImageChanged();
  void onCameraWriterStatusUpdate();
  void onShowLiveThreadSettingsActionTriggered(bool checked);
  void onExposureStatusUpdate(QImagingCamera::ExposureStatus status, double exposure, double elapsed);
  void onCentralDisplayROIShapeChanged();
  void onCentralDisplayLineShapeChanged();
  void onCentralDisplayTargetShapeChanged();
  void updateMeasurements();


protected:
  void onSaveState(QSettings & settings) override;
  void onRestoreState(QSettings & settings) override;
  void onMtfControlVisibilityChanged(bool visible) override;
  void onPlotProfileDialogBoxVisibilityChanged(bool visible) override;
  void onImageProcessorParameterChanged() override;
  void onMeasureRightNowRequested() override;

protected:
  QLivePipelineThread * liveView_ = nullptr;
  QLiveDisplay * centralDisplay_ = nullptr;
  QCameraWriter cameraWriter_;


  QImagingCameraControlsWidget * cameraControls_ctl = nullptr;
  QImagingCameraControlsDock * cameraControlsDock_ = nullptr;
  QAction * showCameraControlsAction_ = nullptr;


  QLivePipelineSelectionWidget * pipelineSelector_ctl = nullptr;
  QCustomDockWidget * pipelineSelectorDock_ = nullptr;
  QAction * showPipelineSelectorAction_ = nullptr;

  QToolButton * measureActionsToolButton_ = nullptr;


  QToolButton * show_log_ctl = nullptr;
  QLabel * shape_status_ctl = nullptr;
  QLabel * mouse_status_ctl = nullptr;
  QLabel * capture_status_ctl = nullptr;
  QLabel * exposure_status_ctl = nullptr;
  QMenu displayOptionsMenu_;


  QToolBar * manToolbar_ = nullptr;
  QMenu * menuViewShapes_ = nullptr;

  QAction * showRectShapeAction_ = nullptr;
  QToolButton * rectShapeActionsButton_ = nullptr;
  QGraphicsRectShapeSettingsDialogBox * rectShapeOptionsDialogBox_ = nullptr;
  QMenu rectShapeActionsMenu_;


  QAction * showTargetShapeAction_ = nullptr;
  QToolButton * targetShapeActionsButton_ = nullptr;
  QGraphicsTargetShapeSettingsDialogBox * targetShapeOptionsDialogBox_ = nullptr;
  QMenu targetShapeActionsMenu_;


  QAction * showLineShapeAction_ = nullptr;
  QToolButton * lineShapeActionsButton_ = nullptr;
  QGraphicsLineShapeSettingsDialogBox * lineShapeOptionsDialogBox_ = nullptr;
  QMenu lineShapeActionsMenu_;


  QAction * showLiveThreadSettingsAction_ = nullptr;
  QLiveThreadSettingsDialogBox * liveThreadSettingsDialogBox_ = nullptr;

  QScaleSelectionButton * displayScaleControl_ = nullptr;

  QAction * copyDisplayImageAction = nullptr;
  QAction * copyDisplayViewportAction = nullptr;


#if HAVE_INDIGO
  QIndigoClient * indigoClient_ = nullptr;
  QIndigoFocuserWidget * indigoFocuser_ = nullptr;
  QCustomDockWidget * indigoFocuserDock_ = nullptr;
#endif // HAVE_INDIGO


  QDisplayVideoWriter diplayImageWriter_;
  QToolButton* displayImageVideoWriterToolButton_ = nullptr;

};

} /* namespace serimager */

#endif /* __qserimager_MainWindow_h__ */
