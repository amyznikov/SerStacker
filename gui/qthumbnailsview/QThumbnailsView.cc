/*
 * QThumbnailsView.cc
 *
 *  Created on: Sep 29, 2019
 *      Author: amyznikov
 */

#include "QThumbnailsView.h"
#include "QThumbnails.h"
#include <gui/widgets/QWaitCursor.h>
#include <core/debug.h>

#define MAX_ICON_LOAD_SIZE 384
#define INITIAL_ZOOM  8
#define MAX_ZOOM  24
#define MIN_ZOOM  1

#define ICON_file_reload   "reload"
#define ICON_file_delete   "delete"
#define ICON_dirtree      "dirtree"
#define ICON_filter        "filter"
#define ICON_filter_clear  "filter-clear"

#define ICON_hourglass    "hourglass2"
#define ICON_badimage     "badimage"


static QIcon hourglass_icon;
static QIcon badimage_icon;


static QIcon getIcon(const QString & name)
{
  return QIcon(QString(":/qthumbnailsview/icons/%1").arg(name));
}

static QWidget * addSpacer(QToolBar * toolBar)
{
  QWidget * spacer = new QWidget();
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  toolBar->addWidget(spacer);
  return spacer;
}


///////////////////////////////////////////////////////////////////////////////
QThumbnailsListWidget::QThumbnailsListWidget(QWidget * parent)
  : Base(parent)
{
  QAction * action;

  Q_INIT_RESOURCE(qthumbnailsview_resources);
  if ( hourglass_icon.isNull() ) {
    hourglass_icon = getIcon(ICON_hourglass);
  }
  if ( badimage_icon.isNull() ) {
    badimage_icon = getIcon(ICON_badimage);
  }


  setViewMode(QListWidget::IconMode);
  setResizeMode(QListWidget::Adjust);
  setTextElideMode (Qt::TextElideMode::ElideLeft);
  setWordWrap(true);
  setSelectionMode(QAbstractItemView::ExtendedSelection);
  setSortingEnabled(true);

  // set default icon sizes
  setZoom(INITIAL_ZOOM);




//  addAction(action = new QAction("Delete Files", this));
//  action->setShortcut(QKeySequence::StandardKey::Delete);
//  connect(action, &QAction::triggered, [this] () {
//    QList<QListWidgetItem*> selectedItems = this->selectedItems();
//    if ( !selectedItems.empty() ) {
//      deleteFiles(selectedItems, true);
//    }
//  });

}

int QThumbnailsListWidget::zoom(void) const
{
  return currentZoom_;
}

void QThumbnailsListWidget::setZoom(int z)
{
  int newzoom = std::max(std::min(z, MAX_ZOOM), MIN_ZOOM);
  if ( currentZoom_ != newzoom ) {
    currentZoom_ = newzoom;
    setIconSize(QSize(currentZoom_ * 16, currentZoom_ * 16));
    onZoomChanged();
  }
}

const QString & QThumbnailsListWidget::quickFilter() const
{
  return quickFilter_;
}

Qt::MatchFlags QThumbnailsListWidget::quickFilterMatchingFlags() const
{
  return quickFilterMatchingFlags_;
}

void QThumbnailsListWidget::setQuickFilter(const QString & v, Qt::MatchFlags flags)
{
  quickFilter_ = v;
  quickFilterMatchingFlags_ = flags;
  quickFilterUpdateItemsVisibility();
}

void QThumbnailsListWidget::clearQuickFilter()
{
  quickFilter_.clear();
  quickFilterUpdateItemsVisibility();
}

bool QThumbnailsListWidget::matchQuickFilter(const QString & text)
{
  return ::matchQuickFilter(text, quickFilter_, quickFilterMatchingFlags_);
}

void QThumbnailsListWidget::quickFilterUpdateItemsVisibility()
{
  QWaitCursor wait(this);

  if ( quickFilter_.isEmpty() ) {
    for ( int  i = 0, n = this->count(); i < n; ++i ) {
      this->item(i)->setHidden(false);
    }
  }
  else {
    for ( int  i = 0, n = this->count(); i < n; ++i ) {
      QListWidgetItem * item = this->item(i);
      item->setHidden(!matchQuickFilter(item->text()));
    }
  }
}

void QThumbnailsListWidget::clear()
{
  Base::clear();
}

void QThumbnailsListWidget::addIcon(const QIcon & icon, const QString & fullPathName, const QVariant & data)
{
  QFileInfo fi(fullPathName);
  QListWidgetItem * item = new QListWidgetItem(icon, fi.fileName());
  item->setWhatsThis(fullPathName);
  item->setData(Qt::UserRole, data);

  addItem(item);
  sortItems(Qt::AscendingOrder);

  if ( !matchQuickFilter(fi.fileName()) ) {
    item->setHidden(true);
  }
}

void QThumbnailsListWidget::updateIcon(const QIcon & icon, const QString & fullPathName, const QVariant & data)
{
  QListWidgetItem * foundItem = Q_NULLPTR;
  for ( int i = 0, n = count(); i < n; ++i ) {
    QListWidgetItem * currentItem = this->item(i);
    if ( currentItem->whatsThis() == fullPathName ) {
      foundItem = currentItem;
      break;
    }
  }

  if ( foundItem ) {
    foundItem->setData(Qt::UserRole, data);
    foundItem->setIcon(icon.isNull() ? badimage_icon : icon);
    repaint();
  }
}


QString QThumbnailsListWidget::currentItemPath()
{
  QListWidgetItem * current = Base::currentItem();
  return current ? current->whatsThis() : QString();
}

void QThumbnailsListWidget::selectNextIcon()
{
  const int cn = count() ;
  if ( cn > 1 ) {
    const int start_row = currentRow();
    int next_row = start_row + 1 >= cn ? 0 : start_row + 1;
    while ( item(next_row)->isHidden() && next_row != start_row ) {
      if ( ++next_row >= cn ) {
        next_row = 0;
      }
    }
    setCurrentRow(next_row);
  }
}

void QThumbnailsListWidget::selectPrevIcon()
{
  const int cn = count() ;
  if ( cn > 1 ) {
    const int start_row = currentRow();
    int next_row = start_row - 1 < 0 ? cn - 1 : start_row - 1;
    while ( item(next_row)->isHidden() && next_row != start_row ) {
      if ( --next_row  < 0 ) {
        next_row = cn - 1;
      }
    }
    setCurrentRow(next_row);
  }
}


void QThumbnailsListWidget::onZoomChanged(void)
{
  emit zoomChanged(zoom());
}

void QThumbnailsListWidget::mouseMoveEvent(QMouseEvent *e)
{
  if ( !(e->buttons() & Qt::LeftButton) ) {
    return Base::mouseMoveEvent(e);
  }

  QList<QListWidgetItem*> selection = selectedItems();
  if ( selection.count() < 1 ) {
    return Base::mouseMoveEvent(e);
  }


  if ( selection.count() > 1 ) {

    std::sort(selection.begin(), selection.end(),
        [](const QListWidgetItem *prev, const QListWidgetItem *next) {
          return prev->text() < next->text();
        });
  }

  QList<QUrl> list;
  for ( int i = 0, n = selection.count(); i < n; ++i ) {
    list.append(QUrl(QString("file:///%1").arg(selection[i]->whatsThis())));
  }

  // mime stuff && start drag
  QMimeData *mimeData = new QMimeData;
  mimeData->setUrls(list);

  QDrag * drag = new QDrag(this);
  drag->setMimeData(mimeData);

  Qt::DropAction act = drag->exec(Qt::CopyAction | Qt::MoveAction); // , Qt::MoveAction

  switch ( act ) {
  case Qt::MoveAction :
    for ( int i = 0, n = selection.size(); i < n; ++i ) {
      QListWidgetItem * item = selection[i];
      if ( !QFile::exists(item->whatsThis()) ) {
        delete takeItem(row(item));
      }
    }
    break;

  case Qt::CopyAction :
    break;
  case Qt::LinkAction:
    break;
  case Qt::ActionMask:
    break;
  case Qt::TargetMoveAction:
    break;
  case Qt::IgnoreAction:
    break;

  default :
    CF_DEBUG("QIconListWidget: unknown action");
    break;
  }
}

void QThumbnailsListWidget::wheelEvent(QWheelEvent *e)
{
  if ( e->modifiers() & Qt::ControlModifier ) {
    const int amount = e->angleDelta().y();
    if ( amount != 0 ) {
      setZoom(currentZoom_ + (amount > 0 ? 1 : -1));
      return;
    }
  }

  Base::wheelEvent(e);
}

void QThumbnailsListWidget::keyPressEvent(QKeyEvent *e)
{
  switch ( e->key() ) {

  case Qt::Key_Plus :
    if ( e->modifiers() & Qt::ControlModifier ) {
      setZoom(currentZoom_ + 1);
      return;
    }
    break;

  case Qt::Key_Minus :
    if ( e->modifiers() & Qt::ControlModifier ) {
      setZoom(currentZoom_ - 1);
      return;
    }
    break;

  case Qt::Key_Return :
    emit enterPressed(Base::currentItem());
    break;

  case Qt::Key_Delete : {
    QList<QListWidgetItem*> selectedItems = this->selectedItems();
    if ( !selectedItems.empty() ) {
      deleteFiles(selectedItems, true);
      return;
    }
    break;
  }

  }

  Base::keyPressEvent(e);
  //emit keyPressed(e);
}


bool QThumbnailsListWidget::deleteFiles(const QList<QListWidgetItem*> & selectedItems, bool askConfirmation)
{
  if ( askConfirmation ) {
    int status = QMessageBox::question(this, "Confirmation required",
        QString("Delete %1 files ?").arg(selectedItems.size()));
    if ( status != QMessageBox::Yes ) {
      return false;
    }
  }

  for ( QListWidgetItem * item : selectedItems ) {
    if ( !QFile::remove(item->whatsThis()) ) {
      CF_DEBUG("QFile::remove('%s') fails", item->whatsThis().toStdString().c_str());
    }
    else if ( !QFile::exists(item->whatsThis()) ) {
      removeItemWidget(item);
      delete item;
    }
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////
QThumbnailsView::QThumbnailsView(QWidget * parent)
  : Base(parent)
{
  Q_INIT_RESOURCE(qthumbnailsview_resources);
  if ( hourglass_icon.isNull() ) {
    hourglass_icon = getIcon(ICON_hourglass);
  }
  if ( badimage_icon.isNull() ) {
    badimage_icon = getIcon(ICON_badimage);
  }


  layout_ = new QVBoxLayout(this);
  layout_->setContentsMargins(0,0,0,0);

  layout_->addWidget(toolbar_ = new QToolBar(this));
  layout_->addWidget(stack_ = new QStackedWidget());


  stack_->addWidget(whiteSheet_ = new QLabel(this));
  whiteSheet_->setFrameShape(QFrame::Box);
  whiteSheet_->setTextFormat(Qt::RichText);
  whiteSheet_->setAlignment(Qt::AlignCenter | Qt::AlignHCenter);
  whiteSheet_->setWordWrap(true);
  whiteSheet_->setText("<H2>No items to display</H2>");


  stack_->addWidget(listWidget_ = new QThumbnailsListWidget(this));
  listWidget_->setContextMenuPolicy(Qt::CustomContextMenu);

  connect(listWidget_, &QListWidget::currentItemChanged,
      this, &ThisClass::onCurrentItemChanged);
  connect(listWidget_, &QListWidget::itemDoubleClicked,
      this, &ThisClass::onItemDoubleClicked);
  connect(listWidget_, &QThumbnailsListWidget::enterPressed,
    this, &ThisClass::onItemEnterPressed);
  connect(listWidget_, &QThumbnailsListWidget::customContextMenuRequested,
    this, &ThisClass::customContextMenuRequested);

  //onViewItemCustomContextMenuRequested

//  connect(new QShortcut(QKeySequence(Qt::Key_Return), listWidget_),&QShortcut::activated,
//      this, SLOT(deleteItem()));







  stack_->setCurrentWidget(whiteSheet_);


  // Setup toolbar
  toolbar_->setIconSize(QSize(16,16));

  refreshAction_ = toolbar_->addAction(getIcon(ICON_file_reload),
      "Refresh",
      [this] () {reload();
      });

  showInDirTreeAction_ = toolbar_->addAction(getIcon(ICON_dirtree),
      "Show current path in directory tree browser",
      [this] () {
        emit showInDirTreeRequested(currentPath());
      });

  toolbar_->addSeparator();

  toolbar_->addWidget(currentPathLabel_ = new QLabel());
  currentPathLabel_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  currentPathLabel_->setAlignment(Qt::AlignRight|Qt::AlignVCenter);


  addSpacer(toolbar_);
  toolbar_->addSeparator();

  filerAction_= toolbar_->addAction(getIcon(ICON_filter),
      "Quick filter",
      this, &ThisClass::onShowQuickFilter);

  clearFilterAction_= toolbar_->addAction(getIcon(ICON_filter_clear),
      "Clear filter",
      this, &ThisClass::clearQuickFilter);


  thumbnailExtractor_.setThumbnailSize(MAX_ICON_LOAD_SIZE);


  //////////////////////////////////////////////////////////////////////


  connect(stack_, &QStackedWidget::currentChanged,
      this, &ThisClass::onCurrentWidgetChanged);



  connect(&searchImageFiles_, &QSearchImageFiles::started,
      this, &ThisClass::onSearchImageFilesStarted,
      Qt::QueuedConnection);
  connect(&searchImageFiles_, &QSearchImageFiles::finished,
      this, &ThisClass::onSearchImageFilesFinished,
      Qt::QueuedConnection);
  connect(&searchImageFiles_, &QSearchImageFiles::imageFound,
      this, &ThisClass::onImageFileFound,
      Qt::QueuedConnection);



  connect(&thumbnailExtractor_, &QThumbnailExtractor::started,
      this, &ThisClass::onThumbnailExtractorStarted,
      Qt::QueuedConnection);
  connect(&thumbnailExtractor_, &QThumbnailExtractor::finished,
      this, &ThisClass::onThumbnailExtractorFinished,
      Qt::QueuedConnection);
  connect(&thumbnailExtractor_, &QThumbnailExtractor::extracted,
      this, &ThisClass::onThumbnailExtrated,
      Qt::QueuedConnection);

}

void QThumbnailsView::updateProgressIndicator()
{
  if ( thumbnailExtractor_.isRunning() || searchImageFiles_.isRunning() ) {
    listWidget_->setCursor(Qt::WaitCursor);
  }
  else {
    listWidget_->setCursor(Qt::ArrowCursor);
  }
}

void QThumbnailsView::updateCurrentStackWidget()
{
  QWidget * w = Q_NULLPTR;

  if ( listWidget_->count() > 0 ) {
    w = listWidget_;
  }
  else {
    w = whiteSheet_;
  }

  if ( stack_->currentWidget() != w ) {
    stack_->setCurrentWidget(w);
  }

  updateProgressIndicator();
}


void QThumbnailsView::onSearchImageFilesStarted()
{
  updateCurrentStackWidget();
}

void QThumbnailsView::onImageFileFound(int rid, const QString fullPathName)
{
  if ( rid == lastSearchImageFilesRID_ ) {
    listWidget_->addIcon(hourglass_icon, fullPathName, QVariant(false));
  }

}

void QThumbnailsView::onSearchImageFilesFinished()
{
  extractMissingThumbiails();
  updateCurrentStackWidget();
}

void QThumbnailsView::extractMissingThumbiails()
{
  for ( int i = 0, n = listWidget_->count(); i < n; ++i ) {
    QListWidgetItem * item = listWidget_->item(i);
    if ( !item->data(Qt::UserRole).toBool() ) {
      thumbnailExtractor_.start(item->whatsThis());
      break;
    }
  }
}

void QThumbnailsView::onThumbnailExtractorStarted()
{
  updateProgressIndicator();
}

void QThumbnailsView::onThumbnailExtrated(int rid, const QIcon & icon, const QString & fullPathName)
{
  listWidget_->updateIcon(icon, fullPathName, QVariant(true));
}

void QThumbnailsView::onThumbnailExtractorFinished()
{
  extractMissingThumbiails();
  updateCurrentStackWidget();
}

void QThumbnailsView::displayPath(const QString & path)
{
  setCurrentPath(path, true);
}

bool QThumbnailsView::setCurrentPath(const QString & path, bool refreshNow)
{
  if ( !path.isEmpty() ) {

    if ( path != currentPath_ ) {
      refreshNow = true;
    }

    cancelPendingUpdates();
    currentPath_ = path;
    currentPathLabel_->setText(currentPath_);
    if ( refreshNow ) {
      reload();
    }

    return true;
  }

  return false;
}

const QString & QThumbnailsView::currentPath() const
{
  return currentPath_;
}

void QThumbnailsView::reload()
{
  cancelPendingUpdates();
  startUpdate();
}

void QThumbnailsView::cancelPendingUpdates()
{
  searchImageFiles_.cancel();
  thumbnailExtractor_.cancel();
  updateProgressIndicator();
}

void QThumbnailsView::startUpdate()
{
  listWidget_->clear();
  if ( !currentPath_.isEmpty()  ) {
    lastSearchImageFilesRID_ = searchImageFiles_.start(currentPath_);
  }
}



void QThumbnailsView::onCurrentWidgetChanged(int)
{
  if (  stack_->currentWidget() == whiteSheet_ ) {
    refreshWhiteListTextMessage();
  }
}

void QThumbnailsView::refreshWhiteListTextMessage()
{
  if ( QFileInfo::exists(currentPath_) ) {
    whiteSheet_->setText("<H2>No images to display</H2>");
  }
  else {
    QString text = QString("<H2>This folder does not exists.</H2>"
        "<p>%1</p>"
        "<p>The requested directory does not exists yet.</p>"
        "<p>Most likely it will be created automatically later</p>").
            arg(currentPath_);

    whiteSheet_->setText(text);
  }
}

void QThumbnailsView::onCurrentItemChanged(QListWidgetItem * current, QListWidgetItem */*previous*/)
{
  if ( current ) {
    emit currentIconChanged(current->whatsThis());
  }
}

void QThumbnailsView::onItemDoubleClicked(QListWidgetItem *item)
{
  if ( item ) {
    emit iconDoubleClicked(item->whatsThis());
  }
}

void QThumbnailsView::onItemEnterPressed(QListWidgetItem *item)
{
  if ( item ) {
    emit iconEnterPressed(item->whatsThis());
  }
}

void QThumbnailsView::selectNextIcon()
{
  listWidget_->selectNextIcon();
}

void QThumbnailsView::selectPrevIcon()
{
  listWidget_->selectPrevIcon();
}

void QThumbnailsView::populateContextMenu(QMenu * menu, const QPoint &pos)
{
  QListWidgetItem * currentItem = nullptr;
  QList<QListWidgetItem*> selectedItems;
  QClipboard * clipboard = nullptr;

  selectedItems = listWidget_->selectedItems();
  currentItem = listWidget_->itemAt(pos);
  clipboard = QApplication::clipboard();

  if ( currentItem ) {

    if ( clipboard ) {

      menu->addAction("Copy file name", [clipboard, currentItem]() {
        clipboard->setText(currentItem->text());
      });

      menu->addAction("Copy full path name", [clipboard, currentItem]() {
        clipboard->setText(currentItem->whatsThis());
      });

    }
  }

  if ( !selectedItems.empty() ) {
    if ( clipboard ) {
      menu->addSeparator();

      menu->addAction("Copy file names", [clipboard, selectedItems]() {
        QString itemNames;
        for ( const QListWidgetItem * item : selectedItems ) {
          itemNames.append(item->text());
          itemNames.append('\n');
        }

        clipboard->setText(itemNames);
      });

      menu->addAction("Copy full path names", [clipboard, selectedItems]() {
        QString itemNames;
        for ( const QListWidgetItem * item : selectedItems ) {
          itemNames.append(item->whatsThis());
          itemNames.append('\n');
        }
        clipboard->setText(itemNames);
      });
    }
  }


  if ( clipboard ) {
    menu->addSeparator();
    menu->addAction("Copy current path", [clipboard, this]() {
      clipboard->setText(this->currentPath());
    });
  }

  if ( !selectedItems.empty() ) {
    menu->addSeparator();

    menu->addAction(getIcon(ICON_file_delete),
        QString("Delete %1 File(s) ...").arg(selectedItems.size()),
        [this, selectedItems]() {
          listWidget_->deleteFiles(selectedItems, true);
        });
  }

  if ( currentItem ) {

    menu->addSeparator();

    menu->addAction(getIcon(ICON_filter),
        QString("Quick filter ..."),
        [this, currentItem ]() {
          showQuickFilter(currentItem->text());
        });

  }

}


QPoint QThumbnailsView::contextMenuPosToGlobal(const QPoint & pos) const
{
  return listWidget_->mapToGlobal(pos);
}

bool QThumbnailsView::moveToBads(const QString & pathfilename)
{
  if ( !pathfilename.isEmpty() ) {

    QFileInfo file(pathfilename);
    if ( !file.exists() ) {
      CF_ERROR("File '%s' not exists", pathfilename.toStdString().c_str());
    }
    else if ( file.path() != currentPath_ ) {
      CF_ERROR("File '%s' not under '%s'",  pathfilename.toStdString().c_str(), currentPath_.toStdString().c_str());
    }
    else {

      QDir curDir(currentPath_);
      QString newpathname = QString("%1/BAD/%2").arg(currentPath_).arg(file.fileName());
      if ( !curDir.mkpath("BAD") ) {
        CF_ERROR("Can not create sub-directory under %s", currentPath_.toStdString().c_str());
      }
      else if ( !QFile::rename(pathfilename, newpathname) ) {
        CF_ERROR("QFile::rename() '%s' -> '%s' fails", pathfilename.toStdString().c_str(), newpathname.toStdString().c_str());
      }
      else {

        for ( int i = 0, n = listWidget_->count(); i < n; ++i ) {
          QListWidgetItem * item = listWidget_->item(i);
          if ( item->whatsThis() == pathfilename ) {

            listWidget_->removeItemWidget(item);
            delete item;

            QListWidgetItem * nextItem = listWidget_->item(i < n -1 ? i: 0);
            if ( nextItem ) {
              listWidget_->setCurrentItem(nextItem);
            }

            break;
          }
        }


        return true;
      }
    }
  }

  return false;
}

void QThumbnailsView::onShowQuickFilter()
{
  showQuickFilter();
}

void QThumbnailsView::showQuickFilter(const QString & wildcard)
{
  if ( !quickfilterDialogBox_ ) {

    quickfilterDialogBox_ = new QThumbnailsQuickFilterDialogBox(this);

    connect(quickfilterDialogBox_, &QThumbnailsQuickFilterDialogBox::parameterChanged,
        [this] () {
          listWidget_->setQuickFilter(quickfilterDialogBox_->searchText(), quickfilterDialogBox_->matchingFlags());
        });
  }

  if ( !wildcard.isEmpty() ) {
    quickfilterDialogBox_->setSearchText(wildcard);
  }
  else if ( quickfilterDialogBox_->searchText().isEmpty() ) {
    QListWidgetItem * currentItem = listWidget_->currentItem();
    if ( currentItem ) {
      quickfilterDialogBox_->setSearchText(currentItem->text());
    }
  }

  quickfilterDialogBox_->show();
  listWidget_->setQuickFilter(quickfilterDialogBox_->searchText(), quickfilterDialogBox_->matchingFlags());
}

void QThumbnailsView::clearQuickFilter()
{
  listWidget_->clearQuickFilter();
}
