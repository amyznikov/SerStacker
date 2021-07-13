/*
 * QFileSystemTreeView.cc
 *
 *  Created on: Dec 17, 2016
 *      Author: amyznikov
 */

#include "QFileSystemTreeView.h"
#include <gui/widgets/QWaitCursor.h>

#define ICON_copy   "copy"
#define ICON_delete "delete"

static QIcon getIcon(const QString & name)
{
  return QIcon(QString(":/qfilesystemtreeview/icons/%1").arg(name));
}

///////////////////////////////////////////////////////////////////////////////

QFileSystemTreeView::QFileSystemTreeView(QWidget * parent /*= 0*/)
    : Base(parent)
{

  Q_INIT_RESOURCE(qfilesystemtreeview_resources);

  QVBoxLayout * layout = new QVBoxLayout(this);

  layout->addWidget(treeView = new QFileSystemCustomTreeView(this), 100);
  treeView->setSelectionMode(QAbstractItemView::ExtendedSelection);
  treeView->setContextMenuPolicy(Qt::CustomContextMenu);

  refresh();

  connect(treeView, &QFileSystemCustomTreeView::customContextMenuRequested,
      this, &QFileSystemTreeView::onCustomContextMenu);

  connect(treeView, &QFileSystemCustomTreeView::pressed,
      this, &QFileSystemTreeView::onItemPressed);

}

void QFileSystemTreeView::refresh()
{
  QString origPath;
  QModelIndex index;
  QDir::Filters origFilter = QDir::Drives | QDir::Dirs | QDir::NoDotAndDotDot;

  if ( model ) {
    origPath = model->fileInfo(treeView->currentIndex()).absoluteFilePath();
    origFilter = model->filter();
    treeView->setModel(NULL);
    delete model;
  }

  treeView->setModel(model = new QFileSystemModel(this));
  model->setReadOnly(false);
  model->setRootPath("/");
  model->setFilter(origFilter);

  treeView->viewport()->setAcceptDrops(true);
  treeView->setDragEnabled(true);
  treeView->setDragDropMode(QAbstractItemView::DragDrop);
  treeView->setDefaultDropAction(Qt::CopyAction);
  treeView->setAcceptDrops(true);
  treeView->setDropIndicatorShown(true);

  treeView->setHeaderHidden(true);
  for ( int i = 1; i < model->columnCount(); ++i ) {
    treeView->setColumnHidden(i, true);
  }

  if ( (index = model->index(origPath)).isValid() ) {
    treeView->setCurrentIndex(index);
    treeView->expand(index);
    treeView->scrollTo(index, QAbstractItemView::EnsureVisible);
  }

  connect(treeView->selectionModel(), &QItemSelectionModel::currentChanged,  // (const QModelIndex &, const QModelIndex &)
      this, &QFileSystemTreeView::onCurrentDirectoryChanged,  // (const QModelIndex &, const QModelIndex &))
      Qt::DirectConnection);
}

void QFileSystemTreeView::hideColumn(int column)
{
  return treeView->hideColumn(column);
}

void QFileSystemTreeView::showColumn(int column)
{
  return treeView->showColumn(column);
}

void QFileSystemTreeView::setFilter(QDir::Filters filters)
{
  model->setFilter(filters);
  emit filterChanged();
  //updateFilterIcon();
  refresh();
}

QDir::Filters QFileSystemTreeView::filter() const
{
  return model->filter();
}

QModelIndex QFileSystemTreeView::setRootPath(const QString &path)
{
  return model->setRootPath(path);
}

QString QFileSystemTreeView::rootPath() const
{
  return model->rootPath();
}

QDir QFileSystemTreeView::rootDirectory() const
{
  return model->rootDirectory();
}

QString QFileSystemTreeView::absoluteFilePath(const QModelIndex &index)
{
  return model->fileInfo(index).absoluteFilePath();
}

QString QFileSystemTreeView::currentAbsoluteFilePath(void)
{
  return model->fileInfo(treeView->currentIndex()).absoluteFilePath();
}

bool QFileSystemTreeView::displayPath(const QString & path, bool showErrMsgIfNotExists)
{
  QModelIndex index = model->index(path);
  if ( !index.isValid() ) {
    if ( showErrMsgIfNotExists ) {
      QMessageBox::critical(this, "ERROR", QString("Requested path not exists.\n'%1'").arg(path));
    }
    return false;
  }

  treeView->scrollTo(index, QAbstractItemView::PositionAtCenter);
  treeView->setCurrentIndex(index);
  treeView->expand(index);

  return true;
}

void QFileSystemTreeView::onCurrentDirectoryChanged(const QModelIndex & current, const QModelIndex & /*previous*/)
{
  if ( model->fileInfo(current).isDir() ) {
    emit currentDirectoryChanged(model->fileInfo(current).absoluteFilePath());
  }
}

void QFileSystemTreeView::onItemPressed(const QModelIndex &index)
{
  QFileInfo fileInfo = model->fileInfo(index);
  if ( fileInfo.isDir() ) {
    emit directoryItemPressed(fileInfo.absoluteFilePath());
  }
}

void QFileSystemTreeView::onCustomContextMenu(const QPoint & pos)
{
  QFileInfoList fileList;

  do {  // force put QModelIndexList into separate scope

    QModelIndexList selectedIndexes = treeView->selectionModel()->selectedRows();

    if ( !selectedIndexes.empty() ) {
      for ( int i = 0, n = selectedIndexes.size(); i < n; ++i ) {
        if ( selectedIndexes[i].isValid() ) {
          fileList.append(model->fileInfo(selectedIndexes[i]));
        }
      }
    }
    else {
      QModelIndex index = treeView->indexAt(pos);
      if ( index.isValid() ) {
        fileList.append(model->fileInfo(index));
      }
    }
  } while ( 0 );

  emit customContextMenuRequested(treeView->viewport()->mapToGlobal(pos), fileList);
}

void QFileSystemTreeView::fillContextMenu(QMenu & menu, const QFileInfoList & flist)
{
  QAction * act;

  if ( flist.size() > 0 && QApplication::clipboard() ) {
    menu.addAction(act = new QAction(getIcon(ICON_copy),
        tr("Copy full path"), this));

    connect(act, &QAction::triggered, [flist] {
      QString text = flist[0].absoluteFilePath();
      for ( int i = 1, n = flist.size(); i < n; ++i ) {
        text.append("\n");
        text.append(flist[i].absoluteFilePath());
      }
      QApplication::clipboard()->setText(text);
    });
  }

  if ( flist.size() > 0 ) {
    menu.addAction(act = new QAction(tr("Move to...."), this));
    connect(act, &QAction::triggered,
        [this, flist] {
          QString newName = QFileDialog::getExistingDirectory(this, "MOVE TO FOLDER",
              flist[0].absoluteFilePath(), QFileDialog::DontUseNativeDialog | QFileDialog::ShowDirsOnly);

          if ( !newName.isEmpty() ) {
            for ( int i = 0, n = flist.size(); i < n; ++i ) {

              const QString & oldName = flist[i].fileName();
              const QString & oldPathName = flist[i].absoluteFilePath();
              const QString newPathName = QString("%1/%2").arg(newName).arg(oldName);

              QWaitCursor wait(this);
              if ( rename(oldPathName.toStdString().c_str(), newPathName.toStdString().c_str()) != 0 ) {
                int resp = QMessageBox::critical(this, "ERROR",
                    QString("rename() fails for '%1': %2").arg(oldPathName).arg(strerror(errno)),
                    QMessageBox::Ok, QMessageBox::Cancel);
                if ( resp == QMessageBox::Cancel ) {
                  break;
                }
              }
            }
          }
        });
  }

  if ( flist.size() == 1 ) {

    QString abspath = flist[0].absoluteFilePath();

    menu.addAction(act = new QAction(style()->standardIcon(QStyle::SP_FileDialogNewFolder),
        tr("Create directory..."), this));

    connect(act, &QAction::triggered,
        [this, abspath] {
          bool ok = false;
          QString dirName = QInputDialog::getText(this, tr("Specify Folder name"),
              tr("Folder name:"), QLineEdit::Normal, QString(),&ok);
          if ( ok && !dirName.isEmpty() ) {
            dirName = QString("%1/%2").arg(abspath).arg(dirName);

            QWaitCursor wait(this);

            if ( !QDir().mkpath(dirName) ) {
              //  if ( !create_path(dirName.toStdString()) != 0 )  {
        QMessageBox::critical(this, "ERROR", QString("create_path() fails: %1").arg(strerror(errno)));
      }
    }});
  }

  if ( flist.size() > 0 ) {
    menu.addAction(act = new QAction(getIcon(ICON_delete),
        tr("Delete selected items ..."), this));

    connect(act, &QAction::triggered,
        [this, flist] {

          int resp;

          if ( flist.size() < 2 ) {
            resp = QMessageBox::warning(this, "CONFIRMATION IS MANDATORY",
                QString("ARE YOU SURE TO COMPLETELY RECURSIVELY DELETE THIS FOLDER ???"),
                QMessageBox::Yes | QMessageBox::No);
          }
          else {
            resp = QMessageBox::warning(this, "CONFIRMATION IS MANDATORY",
                QString("ARE YOU SURE TO COMPLETELY RECURSIVELY DELETE %1 SELECTED FOLDERS ???").arg(flist.size()),
                QMessageBox::Yes | QMessageBox::No);
          }

          if ( resp == QMessageBox::Yes ) {

            QWaitCursor wait(this);

            QStringList pathList;
            for ( int i = 0, n = flist.size(); i < n; ++i ) {
              pathList.append(flist[i].absoluteFilePath());
            }

            for ( int i = 0, n = pathList.size(); i < n; ++i ) {
              const QString & absPath = pathList[i];
              if ( absPath != "/" ) {
                if ( !QDir(absPath).removeRecursively() ) {
                  resp = QMessageBox::critical(this, "ERROR",
                      QString("recursive_rmdir() fails: %1\n%2").arg(strerror(errno)).arg(absPath),
                      QMessageBox::Ok, QMessageBox::Cancel);
                  if ( resp == QMessageBox::Cancel ) {
                    break;
                  }
                }
              }
            }
          }

        });
  }

}

QFileSystemCustomTreeView::QFileSystemCustomTreeView(QWidget * parent)
    : Base(parent)
{
}

void QFileSystemCustomTreeView::dropEvent(QDropEvent * e)
{
  QFileSystemModel * m;
  QModelIndex index;
  Qt::DropActions possibleActsions;

  if ( !(m = dynamic_cast<QFileSystemModel *>(model())) ) {
    return Base::dropEvent(e);
  }

  if ( !((possibleActsions = e->possibleActions()) & Qt::ActionMask) ) {
    return Base::dropEvent(e);
  }

  if ( !(index = indexAt(e->pos())).isValid() ) {
    return Base::dropEvent(e);
  }

  if ( e->source() == this ) {
    // QTreeView can handle only internal moves
    e->setDropAction(Qt::MoveAction);
  }
  else {

    QMenu menu(this);

    QAction * copyAction = Q_NULLPTR;
    QAction * moveAction = Q_NULLPTR;
    QAction * linkAction = Q_NULLPTR;
    QAction * act = Q_NULLPTR;

    if ( possibleActsions & Qt::CopyAction ) {
      menu.addAction(copyAction = new QAction("Copy here", this));
    }
    if ( possibleActsions & Qt::MoveAction ) {
      menu.addAction(moveAction = new QAction("Move here", this));
    }
    if ( possibleActsions & Qt::LinkAction ) {
      menu.addAction(linkAction = new QAction("Link here", this));
    }

    if ( !(act = menu.exec(viewport()->mapToGlobal(e->pos()))) ) {
      e->setDropAction(Qt::IgnoreAction);
    }
    else if ( act == copyAction ) {
      e->setDropAction(Qt::CopyAction);
    }
    else if ( act == moveAction ) {
      e->setDropAction(Qt::MoveAction);
    }
    else if ( act == linkAction ) {
      e->setDropAction(Qt::LinkAction);
    }
    else {
      e->setDropAction(Qt::IgnoreAction);
    }
  }

  if ( e->dropAction() != Qt::IgnoreAction ) {
    if ( m->dropMimeData(e->mimeData(), e->dropAction(), -1, -1, index) ) {
      e->accept();
    }
    else {
      QMessageBox::critical(this, "ERROR",
          "dropMimeData() fails\n\n"
              "Check the permissions and make sure destination is not already exists");
    }
  }
}
