/*
 * QStackSequencesTreeDock.cc
 *
 *  Created on: Jan 12, 2021
 *      Author: amyznikov
 */

#include "QStackSequencesTreeDock.h"

QStackSequencesTreeDock::QStackSequencesTreeDock(const QString &title, QWidget * parent)
    : Base(title, parent)
{
  Base::setWidget(sequencesView_ = new QStackSequencesTree(this));


  const QList<QAction *> actions = sequencesView_->toolbarActions();
  for ( int i = 0, n = actions.size(); i < n; ++i ) {
    titleBar()->addButton(actions[n - i - 1]);
  }

}


QStackSequencesTree * QStackSequencesTreeDock::sequencesView() const
{
  return sequencesView_;
}




QStackSequencesTreeDock * addSequencesTreeDock(QMainWindow * parent,
    Qt::DockWidgetArea area,
    const QString & dockName,
    const QString & title,
    QMenu * viewMenu)
{
  QStackSequencesTreeDock * dock = new QStackSequencesTreeDock(title, parent);
  dock->setObjectName(dockName);
  parent->addDockWidget(area, dock);

  if ( viewMenu ) {
    viewMenu->addAction(dock->toggleViewAction());
  }

    return dock;
}

