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
  void setupMeasures();
  void setupMtfControls();
  void setupProfileGraph();

protected:
  bool eventFilter(QObject *watched, QEvent *event) override;

  virtual void onSaveState(QSettings & settings);
  virtual void onRestoreState(QSettings & settings);

  virtual void onShowImageProcessorControlActionTriggered(bool checked);
  virtual void onImageProcessorControlVisibilityChanged(bool visible);
  virtual void onImageProcessorParameterChanged();

  virtual void onLogWidgetVisibilityChanged(bool visible);

  virtual void onShowMtfControlActionTriggered(bool checked);
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

protected:
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
  QMenu * menuFile_ = nullptr;
  QMenu * menuEdit_ = nullptr;
  QMenu * menuView_ = nullptr;

  /// Log Widget
  QAction * showLogWidgetAction_ = nullptr;
  QLogWidget * logWidget_ctl = nullptr;
  QLogWidgetDock * logWidgetDock_ = nullptr;

  /// Display Options /  MTF
  QMtfControlDialogBox * mtfControl_ = nullptr;
  QAction * showMtfControlAction_ = nullptr;

  /// Current image processor
  QImageProcessorSelector * imageProcessor_ctl = nullptr;
  QCustomDockWidget * imageProcessorDock_ = nullptr;
  QAction * showImageProcessorAction_ = nullptr;


  /// Measures
  QMeasureSettingsDialogBox * measureSettingsDisplay_ = nullptr;
  QMeasureDisplayDialogBox * measureDisplay_ = nullptr;
  QMeasureGraph * measuresGraph_ = nullptr;
  QMeasureGraphDock * measuresGraphDock_ = nullptr;
  QAction * showMeasuresSettingsAction_ = nullptr;
  QAction * showMeasuresDisplayAction_ = nullptr;
  QAction * showMeasuresGraphAction_ = nullptr;
  QAction * measuresMenuAction_ = nullptr;
  QMenu measuresMenu_;
  // QToolButton * measureActionsToolButton_ = nullptr;


  QProfileGraph * profileGraph_ctl_ = nullptr;
  QProfileGraphDialogBox * plotProfileDialogBox_ = nullptr;
  QAction * showProfileGraphAction_ = nullptr;

};

#endif /* __QMainAppWindow_h__ */
