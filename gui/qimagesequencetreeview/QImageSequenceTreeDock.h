/*
 * QImageSequenceTreeDock.h
 *
 *  Created on: Jan 12, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStackSequencesTreeDock_h__
#define __QStackSequencesTreeDock_h__

#include <gui/qcustomdock/QCustomDock.h>
#include "QImageSequenceTreeView.h"

class QImageSequenceTreeDock :
    public QCustomDockWidget
{
  Q_OBJECT;
public:
  typedef QImageSequenceTreeDock ThisClass;
  typedef QCustomDockWidget Base;

  QImageSequenceTreeDock(const QString &title, QWidget * parent = nullptr);

  QImageSequenceTree * treeView() const;

protected:
  QImageSequenceTree * treeView_ = nullptr;
};


QImageSequenceTreeDock * addImageSequenceTreeDock(QMainWindow * parent,
    Qt::DockWidgetArea area,
    const QString & dockName,
    const QString & title,
    QMenu * viewMenu);

#endif /* __QStackSequencesTreeDock_h__ */
