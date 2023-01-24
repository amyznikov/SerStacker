/*
 * QStackSequencesTreeDock.h
 *
 *  Created on: Jan 12, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStackSequencesTreeDock_h__
#define __QStackSequencesTreeDock_h__

#include <gui/qcustomdock/QCustomDock.h>
#include "QStackTree.h"

class QStackTreeViewDock :
    public QCustomDockWidget
{
  Q_OBJECT;
public:
  typedef QStackTreeViewDock ThisClass;
  typedef QCustomDockWidget Base;

  QStackTreeViewDock(const QString &title,
      QWidget * parent = nullptr);


  QStackTree * treeView() const;

protected:
  QStackTree * treeView_ = nullptr;
};


QStackTreeViewDock * addSequencesTreeDock(QMainWindow * parent,
    Qt::DockWidgetArea area,
    const QString & dockName,
    const QString & title,
    QMenu * viewMenu);

#endif /* __QStackSequencesTreeDock_h__ */
