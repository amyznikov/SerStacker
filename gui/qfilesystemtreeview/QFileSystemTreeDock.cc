/*
 * QFileSystemTreeViewDock.cc
 *
 *  Created on: Sep 29, 2019
 *      Author: amyznikov
 */

#include "QFileSystemTreeDock.h"

#define ICON_reload         "reload"
#define ICON_edit           "edit"
#define ICON_folder_closed  "closed"
#define ICON_folder_open    "open"


static QIcon getIcon(const QString & name)
{
  return QIcon(QString(":/qfilesystemtreeview/icons/%1").arg(name));
}

///////////////////////////////////////////////////////////////////////////////


QFileSystemTreeDock * addFileSystemTreeDock(QMainWindow * parent,
    Qt::DockWidgetArea area,
    const QString & dockName,
    const QString & title,
    QMenu * viewMenu)
{
  QFileSystemTreeDock * dock = new QFileSystemTreeDock(title, parent);
  dock->setObjectName(dockName);
  parent->addDockWidget(area, dock);

  if ( viewMenu ) {
    viewMenu->addAction(dock->toggleViewAction());
  }

  return dock;
}


QFileSystemTreeDock::QFileSystemTreeDock(const QString & title, QWidget * parent)
  : Base(title, parent)
{

  Q_INIT_RESOURCE(qfilesystemtreeview_resources);

  Base::setWidget(fileSystemTreeView_ = new QFileSystemTreeView());

  QCustomDockTitleBar * titleBar_ = Base::titleBar();

  refreshButton_ = titleBar_->addButton(getIcon(ICON_reload), "Refresh",
      fileSystemTreeView_,  &QFileSystemTreeView::refresh);

  fileFilterButton_ = titleBar_->addButton(getIcon(ICON_folder_closed), "Show Files",
      [this] () {
        fileSystemTreeView_->setFilter(fileSystemTreeView_->filter() ^ QDir::Files);
        updateTitlebarIcons();
      });

  jumpButton_ = titleBar_->addButton(getIcon(ICON_edit), "Jump to...",
      [this] () {
        bool fok = false;
        QString path = QInputDialog::getText(this, "Expand path",
            "Enter path to expand", QLineEdit::EchoMode::Normal,
            fileSystemTreeView_->currentAbsoluteFilePath(),
            &fok);
        if ( fok && !path.isEmpty() ) {
          fileSystemTreeView_->displayPath(path, true);
        }
      });


  connect(fileSystemTreeView_, &QFileSystemTreeView::filterChanged,
      this, &ThisClass::updateTitlebarIcons);

  connect(fileSystemTreeView_, &QFileSystemTreeView::currentDirectoryChanged,
      this, &ThisClass::currentDirectoryChanged);

  connect(fileSystemTreeView_, &QFileSystemTreeView::directoryItemPressed,
      this, &ThisClass::directoryItemPressed);

  connect(fileSystemTreeView_, &QFileSystemTreeView::customContextMenuRequested,
      this, &ThisClass::customContextMenuRequested);

  connect(fileSystemTreeView_, &QFileSystemTreeView::showPathInTreeRequested,
      this, &ThisClass::showPathInTreeRequested);

  connect(fileSystemTreeView_, &QFileSystemTreeView::filterChanged,
      this, &ThisClass::filterChanged);

}


QFileSystemTreeView * QFileSystemTreeDock::treeView() const
{
  return fileSystemTreeView_;
}

void QFileSystemTreeDock::updateTitlebarIcons()
{
  const QDir::Filters filter = fileSystemTreeView_->filter();
  fileFilterButton_->setIcon((filter & QDir::Files) ? getIcon(ICON_folder_open) : getIcon(ICON_folder_closed));
}

void QFileSystemTreeDock::setFilter(QDir::Filters filters)
{
  return fileSystemTreeView_->setFilter(filters);
}

QDir::Filters QFileSystemTreeDock::filter() const
{
  return fileSystemTreeView_->filter();
}

QModelIndex QFileSystemTreeDock::setRootPath(const QString &path)
{
  return fileSystemTreeView_->setRootPath(path);
}

QString QFileSystemTreeDock::rootPath() const
{
  return fileSystemTreeView_->rootPath();
}

QDir QFileSystemTreeDock::rootDirectory() const
{
  return fileSystemTreeView_->rootDirectory();
}

QString QFileSystemTreeDock::absoluteFilePath(const QModelIndex &index)
{
  return fileSystemTreeView_->absoluteFilePath(index);
}

QString QFileSystemTreeDock::currentAbsoluteFilePath(void)
{
  return fileSystemTreeView_->currentAbsoluteFilePath();
}

bool QFileSystemTreeDock::displayPath(const QString & path, bool showErrMsgIfNotExists)
{
  return fileSystemTreeView_->displayPath(path, showErrMsgIfNotExists);
}

void QFileSystemTreeDock::fillContextMenu(QMenu & menu, const QFileInfoList & flist)
{
  return fileSystemTreeView_->fillContextMenu(menu, flist);
}

void QFileSystemTreeDock::refresh()
{
  return fileSystemTreeView_->refresh();
}
