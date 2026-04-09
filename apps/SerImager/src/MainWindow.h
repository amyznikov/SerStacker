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
  //void updateMeasureChannels();
  void updateMeasurements();


protected:
  void closeEvent(QCloseEvent *event) override;
  void onSaveState(QSettings & settings) override;
  void onRestoreState(QSettings & settings) override;
  void onMtfControlVisibilityChanged(bool visible) override;
  void onPlotProfileDialogBoxVisibilityChanged(bool visible) override;
  void onImageProcessorParameterChanged() override;
  void onMeasureRightNowRequested() override;
  IMtfDisplay * getCurrentMtfDisplay() override;

protected:
  QLivePipelineThread * _liveThread = nullptr;
  QLiveDisplay * _liveDisplay = nullptr;
  QCameraWriter _cameraWriter;

  QImagingCameraControlsWidget * cameraControls_ctl = nullptr;
  QImagingCameraControlsDock * _cameraControlsDock = nullptr;
  QAction * _showCameraControlsAction = nullptr;

  QLivePipelineSelectionWidget * pipelineSelector_ctl = nullptr;
  QCustomDockWidget * _pipelineSelectorDock = nullptr;
  QAction * _showPipelineSelectorAction = nullptr;

  QToolButton * _measureActionsToolButton = nullptr;

  QToolButton * show_log_ctl = nullptr;
  QLabel * shape_status_ctl = nullptr;
  QLabel * mouse_status_ctl = nullptr;
  QLabel * capture_status_ctl = nullptr;
  QLabel * exposure_status_ctl = nullptr;
  QMenu _displayOptionsMenu;

  QToolBar * _mainToolbar = nullptr;
  QMenu * _menuViewShapes = nullptr;

  QAction * _showRectShapeAction = nullptr;
  QToolButton * _rectShapeActionsButton = nullptr;
  QGraphicsRectShapeSettingsDialogBox * _rectShapeOptionsDialogBox = nullptr;
  QMenu _rectShapeActionsMenu;


  QAction * _showTargetShapeAction = nullptr;
  QToolButton * _targetShapeActionsButton = nullptr;
  QGraphicsTargetShapeSettingsDialogBox * _targetShapeOptionsDialogBox = nullptr;
  QMenu _targetShapeActionsMenu;


  QAction * _showLineShapeAction = nullptr;
  QToolButton * _lineShapeActionsButton = nullptr;
  QGraphicsLineShapeSettingsDialogBox * _lineShapeOptionsDialogBox = nullptr;
  QMenu _lineShapeActionsMenu;


  QAction * _showLiveThreadSettingsAction = nullptr;
  QLiveThreadSettingsDialogBox * _liveThreadSettingsDialogBox = nullptr;

  QScaleSelectionButton * _displayScaleControl = nullptr;

  QAction * copyDisplayImageAction = nullptr;
  QAction * copyDisplayViewportAction = nullptr;


#if HAVE_INDIGO
  QIndigoClient * _indigoClient = nullptr;
  QIndigoFocuserWidget * _indigoFocuser = nullptr;
  QCustomDockWidget * _indigoFocuserDock = nullptr;
#endif // HAVE_INDIGO


  QDisplayVideoWriter _diplayImageWriter;
  QToolButton* _displayImageVideoWriterToolButton = nullptr;
  bool _lockDiplayImageWriter = false;

};

} /* namespace serimager */

#endif /* __qserimager_MainWindow_h__ */
