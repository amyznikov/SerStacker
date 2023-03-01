/*
 * QImageSequenceTreeDock.cc
 *
 *  Created on: Jan 12, 2021
 *      Author: amyznikov
 */

#include "QImageSequenceTreeDock.h"

QImageSequenceTreeDock::QImageSequenceTreeDock(const QString &title, QWidget * parent) :
  Base(title, parent)
{
  Base::setWidget(treeView_ = new QImageSequenceTree(this));

  const QList<QAction *> actions = treeView_->toolbarActions();
  for ( int i = 0, n = actions.size(); i < n; ++i ) {
    titleBar()->addButton(actions[n - i - 1]);
  }

}


QImageSequenceTree * QImageSequenceTreeDock::treeView() const
{
  return treeView_;
}

QImageSequenceTreeDock * addImageSequenceTreeDock(QMainWindow * parent,
    Qt::DockWidgetArea area,
    const QString & dockName,
    const QString & title,
    QMenu * viewMenu)
{
  QImageSequenceTreeDock * dock = new QImageSequenceTreeDock(title, parent);
  dock->setObjectName(dockName);
  parent->addDockWidget(area, dock);

  if ( viewMenu ) {
    viewMenu->addAction(dock->toggleViewAction());
  }

    return dock;
}

