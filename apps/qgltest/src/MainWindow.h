/*
 * MainWindow.h
 *
 *  Created on: Dec 21, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __qgltest_MainWindow_h__
#define __qgltest_MainWindow_h__

#include <QtWidgets/QtWidgets>
#include <gui/qcustomdock/QCustomDock.h>
#include "QTestGLView.h"
#include "QTestGLViewSettings.h"

namespace qgltest {

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
  void setupGLViewSettingsDock();

protected Q_SLOTS:

protected:
  bool eventFilter(QObject *watched, QEvent *event) override;

protected:
  QTestGLView * glView_ = nullptr;
  QTestGLViewSettings * glViewSettings_ = nullptr;
  QCustomDockWidget * glViewSettingsDock_ = nullptr;
  QMenu * menuFile_ = nullptr;
  QMenu * menuView_ = nullptr;
};

} /* namespace qgltest */

#endif /* __qgltest_MainWindow_h__ */
