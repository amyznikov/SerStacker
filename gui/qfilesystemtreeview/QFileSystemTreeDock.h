/*
 * QFileSystemTreeDock.h
 *
 *  Created on: Sep 29, 2019
 *      Author: amyznikov
 */
#pragma once
#ifndef __QFileSystemTreeDock_h__
#define __QFileSystemTreeDock_h__

#include <gui/qcustomdock/QCustomDock.h>
#include "QFileSystemTreeView.h"

class QFileSystemTreeDock :
    public QCustomDockWidget
{
  Q_OBJECT;
public:
  typedef QFileSystemTreeDock ThisClass;
  typedef QCustomDockWidget Base;

  QFileSystemTreeDock(const QString &title,
      QWidget * parent = Q_NULLPTR);

  QFileSystemTreeView * treeView() const;

  void setFilter(QDir::Filters filters);
  QDir::Filters filter() const;

  QModelIndex setRootPath(const QString &path);
  QString rootPath() const;
  QDir rootDirectory() const;

  QString absoluteFilePath(const QModelIndex &index);
  QString currentAbsoluteFilePath(void);

  bool displayPath(const QString & path, bool showErrMsgIfNotExists = false);

  void fillContextMenu(QMenu & menu, const QFileInfoList & flist);

Q_SIGNALS:
  void currentDirectoryChanged(const QString & absPath);
  void directoryItemPressed(const QString & absPath);
  void customContextMenuRequested(const QPoint & pos, const QFileInfoList & );
  void showPathInTreeRequested();
  void filterChanged();

public Q_SLOTS:
  void refresh();


protected:
  void updateTitlebarIcons();
  void updateHistory(const QString & abspath);
  void onShowHistoryClicked();

protected:
  QFileSystemTreeView * fileSystemTreeView_ = Q_NULLPTR;
  QToolButton * refreshButton_ = Q_NULLPTR;
  QToolButton * historyButton_ = Q_NULLPTR;
  QToolButton * fileFilterButton_ = Q_NULLPTR;
  QToolButton * jumpButton_ = Q_NULLPTR;
};

QFileSystemTreeDock * addFileSystemTreeDock(QMainWindow * parent,
    Qt::DockWidgetArea area,
    const QString & dockName,
    const QString & title,
    QMenu * viewMenu = Q_NULLPTR);

#endif /* __QFileSystemTreeViewDock_h__ */
