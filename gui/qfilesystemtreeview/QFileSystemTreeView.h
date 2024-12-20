/*
 * QFileSystemTreeView.h
 *
 *  Created on: Dec 17, 2016
 *      Author: amyznikov
 *
 * Based on the tutorial from
 *  http://www.bogotobogo.com/Qt/Qt5_QTreeView_QFileSystemModel_ModelView_MVC.php
 */
#pragma once
#ifndef ___QFileSystemTreeView__h___
#define ___QFileSystemTreeView__h___

#include <QtWidgets/QtWidgets>
#if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))
# include <QtGui/QFileSystemModel>
#else
# include <QtWidgets/QFileSystemModel>
#endif

class QFileSystemTreeView;
class QFileSystemCustomTreeView;

class QFileSystemTreeView
  : public QWidget
{
  Q_OBJECT;
public:
  typedef QFileSystemTreeView ThisClass;
  typedef QWidget Base;
  QFileSystemTreeView(QWidget * parent = 0);

  void hideColumn(int column);
  void showColumn(int column);

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
  void customContextMenuRequested(const QPoint & pos, const QFileInfoList & );
  void directoryItemPressed(const QString & absPath);
  void showPathInTreeRequested();
  void filterChanged();

public Q_SLOTS:
  void refresh();

private Q_SLOTS:
  void onCurrentDirectoryChanged(const QModelIndex & current,
      const QModelIndex & previous);
  void onCustomContextMenu(const QPoint & pos);
  void onItemPressed(const QModelIndex &index);

private:
  QFileSystemModel * model = nullptr;
  QFileSystemCustomTreeView * treeView = nullptr;
};


class QFileSystemCustomTreeView
    : public QTreeView
{
public:
  typedef QTreeView Base;
  QFileSystemCustomTreeView(QWidget * parent = nullptr);

protected:
  void dropEvent(QDropEvent *event) override;
};

#endif /* ___QFileSystemTreeView__h___ */
