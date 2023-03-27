/*
 * MainWindow.h
 *
 *  Created on: Dec 21, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __qserimager_MainWindow_h__
#define __qserimager_MainWindow_h__

#include <QtWidgets/QtWidgets>
#include <gui/qmtf/QMtfControl.h>
#include <gui/qgraphicsshape/QGraphicsRectShapeSettings.h>
#include <gui/qgraphicsshape/QGraphicsTargetShapeSettings.h>
#include <gui/qgraphicsshape/QGraphicsLineShapeSettings.h>
#include <gui/qlogwidget/QLogWidget.h>
#include <gui/qfocus/QFocusGraph.h>
#include "camera/QImagingCameraControlsWidget.h"
#include "camera/QCameraFrameProcessorSelector.h"
#include "focus/QCameraFocusMeasure.h"
#include "QLivePipeline.h"


#if HAVE_INDIGO
# include "focus/indigo/QIndigoFocuserWidget.h"
#endif

namespace serimager {

class MainWindow:
    public QMainWindow
{
  Q_OBJECT;

public:
  typedef MainWindow ThisClass;
  typedef QMainWindow Base;

  MainWindow(QWidget * parent = nullptr);
  ~MainWindow();

protected:
  void saveState();
  void restoreState();
  void setupMainMenu();
  void setupMainToolbar();
  void setupStatusbar();
  void setupLogWidget();
  void setupCameraControls();
  void setupShapeOptions();
  void setupLivePipelineControls();
  void setupDisplayProcessingControls();
  void setupFocusGraph();
  void setupIndigoFocuser();

protected Q_SLOTS:
  void onCameraWriterStatusUpdate();
  void onShowMtfControlActionTriggered(bool checked);
  void onShowDisplayFrameProcessorSettingsActionTriggered(bool checked);
  void onExposureStatusUpdate(QImagingCamera::ExposureStatus status, double exposure, double elapsed);

protected:
  bool eventFilter(QObject *watched, QEvent *event) override;

protected:
  QLivePipelineThread * liveView_ = nullptr;
  QVideoFrameDisplay * centralDisplay_ = nullptr;
  QCameraWriter cameraWriter_;
  QLivePipelineCollection pipelineCollection_;


  QLogWidget * logwidget_ctl = nullptr;
  QCustomDockWidget * logwidgetDock_ = nullptr;



  QImagingCameraControlsWidget * cameraControls_ctl = nullptr;
  QImagingCameraControlsDock * cameraControlsDock_ = nullptr;
  QAction * showCameraControlsAction_ = nullptr;



  QLivePipelineSelectionWidget * pipelineSelector_ctl = nullptr;
  QCustomDockWidget * pipelineSelectorDock_ = nullptr;
  QAction * showPipelineSelectorAction_ = nullptr;



  QCameraFrameProcessorSelector * frameProcessor_ctl = nullptr;
  QCustomDockWidget * frameProcessorDock_ = nullptr;
  QAction * showFrameProcessorAction_ = nullptr;



  QCameraFocusMeasure * focusMeasure_ = nullptr;
  QFocusGraph * focusGraph_ = nullptr;
  QFocusGraphDock * focusGraphDock_ = nullptr;



  QMtfControlDialogBox * mtfControl_ = nullptr;
  QAction * showMtfControlAction_ = nullptr;
  QToolButton * showMtfControlButton_ = nullptr;
  QLabel * capture_status_ctl = nullptr;
  QLabel * mousepos_ctl = nullptr;
  QLabel * exposure_status_ctl = nullptr;
  QMenu displayOptionsMenu_;



  QToolBar * manToolbar_ = nullptr;
  QMenu * menuFile_ = nullptr;
  QMenu * menuView_ = nullptr;
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


  QAction * showLiveThreadSettingsDialogBoxAction_ = nullptr;
  QLiveThreadSettingsDialogBox * liveThreadSettingsDialogBox_ = nullptr;


#if HAVE_INDIGO
  QIndigoClient * indigoClient_ = nullptr;
  QIndigoFocuserWidget * indigoFocuser_ = nullptr;
  QCustomDockWidget * indigoFocuserDock_ = nullptr;
#endif // HAVE_INDIGO
};

} /* namespace serimager */

#endif /* __qserimager_MainWindow_h__ */
