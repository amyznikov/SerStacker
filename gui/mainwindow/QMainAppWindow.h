/*
 * QMainAppWindow.h
 *
 *  Created on: Apr 14, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMainAppWindow_h__
#define __QMainAppWindow_h__

#include <QtWidgets/QtWidgets>
#include <gui/qlogwidget/QLogWidget.h>
#include <gui/qcustomdock/QCustomDock.h>
#include <gui/qmtf/QMtfControl.h>
#include <gui/qdataproc/QDataFrameProcessorEditor.h>
#include <gui/qimproc/QImageProcessorSelector.h>
#include <gui/qmeasure/QMeasureGraph.h>
#include <gui/qmeasure/QMeasureDisplay.h>
#include <gui/qmeasure/QMeasureSelection.h>
#include <gui/qmeasure/QProfileGraph.h>
#include <gui/widgets/createAction.h>
#include <gui/widgets/style.h>

class QMainAppWindow:
    public QMainWindow
{
  Q_OBJECT;
public:
  typedef QMainAppWindow ThisClass;
  typedef QMainWindow Base;

  QMainAppWindow(QWidget * parent = nullptr);

protected:
  void saveState();
  void restoreState();

protected:
  void setupMainMenu();
  void setupMainToolbar();
  void setupStatusbar();
  void setupLogWidget();
  void setupImageProcessingControls();
  void setupDataProcessingControls();
  void setupMeasures();
  void setupMtfControls();
  void setupProfileGraph();

protected:
  bool eventFilter(QObject *watched, QEvent *event) override;

  virtual void onSaveState(QSettings & settings);
  virtual void onRestoreState(QSettings & settings);

  virtual void onShowDataframeProcessorControlActionTriggered(bool checked);
  virtual void onDataframeProcessorControlVisibilityChanged(bool visible);
  virtual void onDataframeProcessorParameterChanged();

  virtual void onShowImageProcessorControlActionTriggered(bool checked);
  virtual void onImageProcessorControlVisibilityChanged(bool visible);
  virtual void onImageProcessorParameterChanged();

  virtual void onLogWidgetVisibilityChanged(bool visible);

  virtual void onShowMtfControlActionTriggered(bool checked);
  virtual void onShowMtfControlActionContextMenuRequested(QToolButton * tb, const QPoint & pos);
  virtual void onMtfControlVisibilityChanged(bool visible);

  virtual void onShowMeasureSettingsActionTriggered(bool checked);
  virtual void onMeasureSettingsVisibilityChanged(bool visible);

  virtual void onShowMeasureDisplayActionTriggered(bool checked);
  virtual void onMeasuresDisplayVisibilityChanged(bool visible);
  virtual void onMeasuresGraphVisibilityChanged(bool visible);
  virtual void onMeasureRightNowRequested();

  virtual void onShowProfileGraphActionTriggered(bool checked);
  virtual void onPlotProfileDialogBoxVisibilityChanged(bool visible);
  virtual void updateProfileGraph(QGraphicsItem * lineItem = nullptr);

  virtual IMtfDisplay * getCurrentMtfDisplay();

protected:
  QToolButton * createMtfControlButton();
  static QToolButton* createToolButtonWithPopupMenu(QAction * defaultAction, QMenu * menu);
  static QToolButton* createToolButtonWithMenu(const QIcon & icon, const QString & text, const QString & tooltip, QMenu * menu);
  static QWidget* addStretch(QToolBar * toolbar);
  static QWidget* createStretch();

  static inline bool is_visible(QWidget * w)
  {
    return w && w->isVisible();
  }

protected:
  /// Main menu
  QMenu * menuFile = nullptr;
  QMenu * menuEdit = nullptr;
  QMenu * menuView = nullptr;

  /// Log Widget
  QAction * showLogWidgetAction = nullptr;
  QLogWidget * logWidget_ctl = nullptr;
  QLogWidgetDock * logWidgetDock = nullptr;

  /// Display Options /  MTF
  QMtfControlDialogBox * mtfControl = nullptr;
  QAction * showMtfControlAction = nullptr;

  /// Current dataframe processor
  QDataFrameProcessorSelector * dataframeProcessor_ctl = nullptr;
  QCustomDockWidget * dataframeProcessorDock = nullptr;
  QAction * showDataframeProcessorAction = nullptr;

  /// Current image processor
  QImageProcessorSelector * imageProcessor_ctl = nullptr;
  QCustomDockWidget * imageProcessorDock = nullptr;
  QAction * showImageProcessorAction = nullptr;


  /// Measures
  QMeasureSettingsDialogBox * measureSettingsDisplay = nullptr;
  QMeasureDisplayDialogBox * measureDisplay = nullptr;
  QMeasureGraph * measuresGraph = nullptr;
  QMeasureGraphDock * measuresGraphDock = nullptr;
  QAction * showMeasuresSettingsAction = nullptr;
  QAction * showMeasuresDisplayAction = nullptr;
  QAction * showMeasuresGraphAction = nullptr;
  QAction * measuresMenuAction = nullptr;
  QMenu measuresMenu;

  QProfileGraph * profileGraph_ctl_ = nullptr;
  QProfileGraphDialogBox * plotProfileDialogBox_ = nullptr;
  QAction * showProfileGraphAction_ = nullptr;

};

#endif /* __QMainAppWindow_h__ */
