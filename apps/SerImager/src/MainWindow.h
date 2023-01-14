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
#include "QCameraFrameProcessorSelector.h"
#include "QCameraFrameDisplay.h"
#include "QImagerSettingsWidget.h"
#include "QFocusGraph.h"

#define HAVE_INDIGO 1
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
  void setupImagerSettings();
  void setupFrameProcessorControls();
  void setupFocusGraph();
  void setupIndigoFocuser();

protected Q_SLOTS:
  void onCameraWriterStatisticsUpdate();
  void onShowMtfControlActionTriggered(bool checked);
  void onShowDisplayFrameProcessorSettingsActionTriggered(bool checked);

protected:
  bool eventFilter(QObject *watched, QEvent *event) override;

protected:
  QCameraFrameDisplay * centralDisplay_ = nullptr;
  QCameraWriter cameraWriter_;

  QImagerSettingsWidget * imagerSettings_ctl = nullptr;
  QCameraControlDock * imagerSettingsDock_ = nullptr;
  QAction * showCameraControlsAction_ = nullptr;

  QCameraFrameProcessorSelector * frameProcessor_ctl = nullptr;
  QCustomDockWidget * frameProcessorDock_ = nullptr;
  QAction * showFrameProcessorAction_ = nullptr;

  QFocusGraph * focusGraph_ = nullptr;
  QFocusGraphDock * focusGraphDock_ = nullptr;
  QCameraFocusMeasureThread * focusMeasureThread_ = nullptr;

  QMtfControlDialogBox * mtfControl_ = nullptr;
  QAction * showMtfControlAction_ = nullptr;
  QToolButton * showMtfControlButton_ = nullptr;
  QLabel * statistics_ctl = nullptr;
  QLabel * mousepos_ctl = nullptr;
  QLabel * exposure_status_ctl = nullptr;
  QMenu displayOptionsMenu_;

  QToolBar * manToolbar_ = nullptr;
  QMenu * menuFile_ = nullptr;
  QMenu * menuView_ = nullptr;
  QAction * showRoiAction_ = nullptr;

  QAction * showdisplayFrameProcessorSettingsAction_ = nullptr;
  QDisplayFrameProcessorSettingsDialogBox * displayFrameProcessorSettingsDialogBox_ = nullptr;

#if HAVE_INDIGO
  QIndigoClient * indigoClient_ = nullptr;
  QIndigoFocuserWidget * indigoFocuser_ = nullptr;
  QCustomDockWidget * indigoFocuserDock_ = nullptr;
#endif // HAVE_INDIGO
};

} /* namespace qserimager */

#endif /* __qserimager_MainWindow_h__ */
