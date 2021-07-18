/*
 * QStackSequencesTreeDock.cc
 *
 *  Created on: Jan 12, 2021
 *      Author: amyznikov
 */

#include "QStackTreeViewDock.h"

QStackTreeViewDock::QStackTreeViewDock(const QString &title, QWidget * parent)
    : Base(title, parent)
{
  Base::setWidget(sequencesView_ = new QStackTree(this));


  const QList<QAction *> actions = sequencesView_->toolbarActions();
  for ( int i = 0, n = actions.size(); i < n; ++i ) {
    titleBar()->addButton(actions[n - i - 1]);
  }

}


QStackTree * QStackTreeViewDock::sequencesView() const
{
  return sequencesView_;
}




QStackTreeViewDock * addSequencesTreeDock(QMainWindow * parent,
    Qt::DockWidgetArea area,
    const QString & dockName,
    const QString & title,
    QMenu * viewMenu)
{
  QStackTreeViewDock * dock = new QStackTreeViewDock(title, parent);
  dock->setObjectName(dockName);
  parent->addDockWidget(area, dock);

  if ( viewMenu ) {
    viewMenu->addAction(dock->toggleViewAction());
  }

    return dock;
}

