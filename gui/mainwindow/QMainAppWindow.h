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

protected:
  static QToolButton* createToolButtonWithPopupMenu(QAction * defaultAction, QMenu * menu);
  static QToolButton* createToolButtonWithMenu(const QIcon & icon, const QString & text, const QString & tooltip, QMenu * menu);
  static QWidget* addStretch(QToolBar * toolbar);
  static QWidget* createStretch();


  template<class Obj, typename Fn>
  static QAction* createAction(const QIcon & icon, const QString & text, const QString & tooltip,
      Obj * receiver, Fn fn,
      QShortcut * shortcut = nullptr)
  {
    QAction *action = new QAction(icon, text);
    action->setToolTip(tooltip);

    QObject::connect(action, &QAction::triggered, receiver, fn);

    if( shortcut ) {
      QObject::connect(shortcut, &QShortcut::activated,
          action, &QAction::trigger);
    }

    return action;
  }

  template<typename Slot>
  static QAction* createAction(const QIcon & icon, const QString & text, const QString & tooltip, Slot && slot, QShortcut * shortcut = nullptr)
  {
    QAction *action = new QAction(icon, text);
    action->setToolTip(tooltip);

    QObject::connect(action, &QAction::triggered, slot);

    if( shortcut ) {
      QObject::connect(shortcut, &QShortcut::activated,
          action, &QAction::trigger);
    }

    return action;
  }

  template<class Obj, typename Fn>
  static QAction* createCheckableAction(const QIcon & icon, const QString & text, const QString & tooltip,
      Obj * receiver, Fn fn,
      QShortcut * shortcut = nullptr)
  {
    QAction *action = new QAction(icon, text);
    action->setToolTip(tooltip);
    action->setCheckable(true);

    QObject::connect(action, &QAction::triggered, receiver, fn);

    if( shortcut ) {
      QObject::connect(shortcut, &QShortcut::activated,
          action, &QAction::trigger);
    }

    return action;
  }

  template<typename Slot>
  static QAction* createCheckableAction(const QIcon & icon, const QString & text, const QString & tooltip, Slot && slot,
      QShortcut * shortcut = nullptr)
  {
    QAction *action = new QAction(icon, text);
    action->setToolTip(tooltip);
    action->setCheckable(true);

    QObject::connect(action, &QAction::triggered, slot);

    if( shortcut ) {
      QObject::connect(shortcut, &QShortcut::activated,
          action, &QAction::trigger);
    }

    return action;
  }

protected:
  /// Main menu
  QMenu * menuFile_ = nullptr;
  QMenu * menuEdit_ = nullptr;
  QMenu * menuView_ = nullptr;

  /// Log Widget
  QAction * showLogWidgetAction_ = nullptr;
  QLogWidget * logWidget_ctl = nullptr;
  QCustomDockWidget * logWidgetDock_ = nullptr;

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
  QMenu measuresMenu_;
  // QToolButton * measureActionsToolButton_ = nullptr;


};

#endif /* __QMainAppWindow_h__ */