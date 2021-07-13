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
#include "QStackSequencesTree.h"

class QStackSequencesTreeDock
    : public QCustomDockWidget
{
  Q_OBJECT;
public:
  typedef QStackSequencesTreeDock ThisClass;
  typedef QCustomDockWidget Base;

  QStackSequencesTreeDock(const QString &title,
      QWidget * parent = Q_NULLPTR);


  QStackSequencesTree * sequencesView() const;

protected:
  QStackSequencesTree * sequencesView_ = Q_NULLPTR;
};


QStackSequencesTreeDock * addSequencesTreeDock(QMainWindow * parent,
    Qt::DockWidgetArea area,
    const QString & dockName,
    const QString & title,
    QMenu * viewMenu);

#endif /* __QStackSequencesTreeDock_h__ */
