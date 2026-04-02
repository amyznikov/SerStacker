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

#define ICON_file_reload   ":/qthumbnailsview/icons/reload"
#define ICON_file_delete   ":/qthumbnailsview/icons/delete"
#define ICON_dirtree       ":/qthumbnailsview/icons/dirtree"
#define ICON_filter        ":/qthumbnailsview/icons/filter"
#define ICON_filter_clear  ":/qthumbnailsview/icons/filter-clear"

#define ICON_hourglass     ":/qthumbnailsview/icons/hourglass2"
#define ICON_badimage      ":/qthumbnailsview/icons/badimage"
#define ICON_textfile      ":/qthumbnailsview/icons/textfile"
#define ICON_plyfile       ":/qthumbnailsview/icons/plyfile"


static QIcon hourglass_icon;
static QIcon badimage_icon;
static QIcon textfile_icon;
static QIcon plyfile_icon;

static QWidget * addSpacer(QToolBar * toolBar)
{
  QWidget * spacer = new QWidget();
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  toolBar->addWidget(spacer);
  return spacer;
}

static void init_thumbnailsview_resources()
{
  // MAX_ICON_LOAD_SIZE
  Q_INIT_RESOURCE(qthumbnailsview_resources);

  if ( hourglass_icon.isNull() ) {
    hourglass_icon = getIcon(ICON_hourglass);
  }
  if ( badimage_icon.isNull() ) {
    badimage_icon = getIcon(ICON_badimage);
  }
  if ( textfile_icon.isNull() ) {
    textfile_icon = getIcon(ICON_textfile);
  }
  if ( plyfile_icon.isNull() ) {
    plyfile_icon = getIcon(ICON_plyfile);
  }

}


///////////////////////////////////////////////////////////////////////////////
QThumbnailsListWidget::QThumbnailsListWidget(QWidget * parent)
  : Base(parent)
{
  QAction * action;

  init_thumbnailsview_resources();


  setViewMode(QListWidget::IconMode);
  setResizeMode(QListWidget::Adjust);
  setTextElideMode (Qt::TextElideMode::ElideLeft);
  setWordWrap(true);
  setSelectionMode(QAbstractItemView::ExtendedSelection);
  setSortingEnabled(true);

  setZoom(INITIAL_ZOOM);
}

int QThumbnailsListWidget::zoom(void) const
{
  return _currentZoom;
}

void QThumbnailsListWidget::setZoom(int z)
{
  int newzoom = std::max(std::min(z, MAX_ZOOM), MIN_ZOOM);
  if ( _currentZoom != newzoom ) {
    _currentZoom = newzoom;
    setIconSize(QSize(_currentZoom * 16, _currentZoom * 16));
    onZoomChanged();
  }
}

const QString & QThumbnailsListWidget::quickFilter() const
{
  return _quickFilter;
}

Qt::MatchFlags QThumbnailsListWidget::quickFilterMatchingFlags() const
{
  return _quickFilterMatchingFlags;
}

void QThumbnailsListWidget::setQuickFilter(const QString & v, Qt::MatchFlags flags, bool invertMatch)
{
  _quickFilter = v;
  _quickFilterMatchingFlags = flags;
  _quickFilterInvertMatch = invertMatch;
  quickFilterUpdateItemsVisibility();
}

void QThumbnailsListWidget::clearQuickFilter()
{
  _quickFilter.clear();
  quickFilterUpdateItemsVisibility();
}

bool QThumbnailsListWidget::matchQuickFilter(const QString & text)
{
  return ::matchQuickFilter(text, _quickFilter, _quickFilterMatchingFlags);
}

void QThumbnailsListWidget::quickFilterUpdateItemsVisibility()
{
  QWaitCursor wait(this);

  if ( _quickFilter.isEmpty() ) {
    for ( int  i = 0, n = this->count(); i < n; ++i ) {
      this->item(i)->setHidden(false);
    }
  }
  else {
    for ( int  i = 0, n = this->count(); i < n; ++i ) {

      QListWidgetItem * item = this->item(i);

      if ( !_quickFilterInvertMatch ) {
        item->setHidden(!matchQuickFilter(item->text()));
      }
      else {
        item->setHidden(matchQuickFilter(item->text()));
      }
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
  QListWidgetItem * foundItem = nullptr;
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
    if ( !foundItem->isHidden() ) {
      repaint();
    }
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

void QThumbnailsListWidget::scheduleUpdateItemsLayout()
{
  QTimer::singleShot(250, this, &ThisClass::scheduleDelayedItemsLayout);
}

void QThumbnailsListWidget::updateItemsLayout()
{
  Base::scheduleDelayedItemsLayout();
  Base::executeDelayedItemsLayout();
}


void QThumbnailsListWidget::onZoomChanged(void)
{
  Q_EMIT zoomChanged(zoom());
}

void QThumbnailsListWidget::mouseMoveEvent(QMouseEvent *e)
{
  if ( !(e->buttons() & Qt::LeftButton) ) {
    return Base::mouseMoveEvent(e);
  }

  //  QList<QListWidgetItem*> selection = selectedItems();

  QList<QListWidgetItem*> selection;
  for ( int i = 0, n = this->count(); i < n; ++i ) {
    QListWidgetItem * item = this->item(i);
    if ( item->isSelected() && !item->isHidden() ) {
      selection.append(item);
    }
  }

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

  Qt::DropAction act =
      drag->exec(Qt::CopyAction | Qt::MoveAction);

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
    CF_ERROR("QIconListWidget: unknown action %d", act);
    break;
  }
}

void QThumbnailsListWidget::wheelEvent(QWheelEvent *e)
{
  if ( e->modifiers() & Qt::ControlModifier ) {
    const int amount = e->angleDelta().y();
    if ( amount != 0 ) {
      setZoom(_currentZoom + (amount > 0 ? 1 : -1));
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
      setZoom(_currentZoom + 1);
      return;
    }
    break;

  case Qt::Key_Minus :
    if ( e->modifiers() & Qt::ControlModifier ) {
      setZoom(_currentZoom - 1);
      return;
    }
    break;

  case Qt::Key_Return :
    Q_EMIT enterPressed(Base::currentItem());
    break;

  case Qt::Key_Delete : {
    QList<QListWidgetItem*> selectedItems = this->selectedItems();
    if ( !selectedItems.empty() ) {
      deleteFiles(selectedItems, true);
      return;
    }
    break;
  }

  case Qt::Key_A :
    if ( e->modifiers() & Qt::ControlModifier ) {
      selectAll();
      return;
    }
    break;

  }

  Base::keyPressEvent(e);
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

void QThumbnailsListWidget::selectAll(bool includeHiddenItems)
{
  if ( includeHiddenItems ) {
    Base::selectAll();
  }
  else {
    clearSelection();

    for ( int i = 0, n = this->count(); i < n; ++i ) {
      QListWidgetItem * item = this->item(i);
      if ( !item->isHidden() ) {
        item->setSelected(true);
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
QThumbnailsView::QThumbnailsView(QWidget * parent)
  : Base(parent)
{
  init_thumbnailsview_resources();

  _layout = new QVBoxLayout(this);
  _layout->setContentsMargins(0,0,0,0);

  _layout->addWidget(_toolbar = new QToolBar(this));
  _layout->addWidget(_stack = new QStackedWidget());


  _stack->addWidget(_whiteSheet = new QLabel(this));
  _whiteSheet->setFrameShape(QFrame::Box);
  _whiteSheet->setTextFormat(Qt::RichText);
  _whiteSheet->setAlignment(Qt::AlignCenter | Qt::AlignHCenter);
  _whiteSheet->setWordWrap(true);
  _whiteSheet->setText("<H2>No items to display</H2>");


  _stack->addWidget(_listWidget = new QThumbnailsListWidget(this));
  _listWidget->setContextMenuPolicy(Qt::CustomContextMenu);

  connect(_listWidget, &QListWidget::currentItemChanged,
      this, &ThisClass::onCurrentItemChanged);
  connect(_listWidget, &QListWidget::itemDoubleClicked,
      this, &ThisClass::onItemDoubleClicked);
  connect(_listWidget, &QThumbnailsListWidget::enterPressed,
    this, &ThisClass::onItemEnterPressed);
  connect(_listWidget, &QThumbnailsListWidget::customContextMenuRequested,
    this, &ThisClass::customContextMenuRequested);


  _stack->setCurrentWidget(_whiteSheet);


  // Setup toolbar
  _toolbar->setIconSize(QSize(16,16));

  _refreshAction = _toolbar->addAction(getIcon(ICON_file_reload),
      "Refresh",
      [this] () {
        reload();
      });

  _showInDirTreeAction = _toolbar->addAction(getIcon(ICON_dirtree),
      "Show current path in directory tree browser",
      [this] () {
        Q_EMIT showInDirTreeRequested(currentPath());
      });

  _toolbar->addSeparator();

  _toolbar->addWidget(_currentPathLabel = new QLabel());
  _currentPathLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
  _currentPathLabel->setAlignment(Qt::AlignRight|Qt::AlignVCenter);


  addSpacer(_toolbar);
  _toolbar->addSeparator();

  _toolbar->addAction(_quickFilterAction =
      new QAction(getIcon(ICON_filter),
          "Quick filter"));
  _quickFilterAction->setCheckable(true);
  connect(_quickFilterAction, &QAction::triggered,
      this, &ThisClass::onShowQuickFilter);

  _thumbnailExtractor.setThumbnailSize(MAX_ICON_LOAD_SIZE);

  //////////////////////////////////////////////////////////////////////


  connect(_stack, &QStackedWidget::currentChanged,
      this, &ThisClass::onCurrentWidgetChanged);



  connect(&_searchImageFiles, &QSearchImageFiles::started,
      this, &ThisClass::onSearchImageFilesStarted,
      Qt::QueuedConnection);
  connect(&_searchImageFiles, &QSearchImageFiles::finished,
      this, &ThisClass::onSearchImageFilesFinished,
      Qt::QueuedConnection);
  connect(&_searchImageFiles, &QSearchImageFiles::imageFound,
      this, &ThisClass::onImageFileFound,
      Qt::QueuedConnection);


  connect(&_thumbnailExtractor, &QThumbnailExtractor::started,
      this, &ThisClass::onThumbnailExtractorStarted,
      Qt::QueuedConnection);
  connect(&_thumbnailExtractor, &QThumbnailExtractor::finished,
      this, &ThisClass::onThumbnailExtractorFinished,
      Qt::QueuedConnection);
  connect(&_thumbnailExtractor, &QThumbnailExtractor::extracted,
      this, &ThisClass::onThumbnailExtrated,
      Qt::QueuedConnection);

}

void QThumbnailsView::updateProgressIndicator()
{
  if ( _thumbnailExtractor.isRunning() || _searchImageFiles.isRunning() ) {
    _listWidget->setCursor(Qt::WaitCursor);
  }
  else {
    _listWidget->setCursor(Qt::ArrowCursor);
  }
}

void QThumbnailsView::updateCurrentStackWidget()
{
  QWidget * w = nullptr;

  if ( _listWidget->count() > 0 ) {
    w = _listWidget;
  }
  else {
    w = _whiteSheet;
  }

  if ( _stack->currentWidget() != w ) {
    _stack->setCurrentWidget(w);
  }

  updateProgressIndicator();
}


void QThumbnailsView::onSearchImageFilesStarted()
{
  QApplication::setOverrideCursor(Qt::WaitCursor);
  updateCurrentStackWidget();
}

void QThumbnailsView::onImageFileFound(int rid, const QString fullPathName)
{
  if ( rid == _lastSearchImageFilesRID ) {

    _listWidget->addIcon(hourglass_icon, fullPathName, QVariant(false));

    if ( _stack->currentWidget() != _listWidget ) {
      _stack->setCurrentWidget(_listWidget);
    }

    QCoreApplication::processEvents(); // QEventLoop::ExcludeUserInputEvents
  }
}

void QThumbnailsView::onSearchImageFilesFinished()
{
  extractMissingThumbiails();
  updateCurrentStackWidget();
  QApplication::restoreOverrideCursor();
}

void QThumbnailsView::extractMissingThumbiails()
{
  bool hasupdates = false;
  bool finished = true;

  for ( int i = 0, n = _listWidget->count(); i < n; ++i ) {
    QListWidgetItem * item = _listWidget->item(i);
    if ( !item->data(Qt::UserRole).toBool() ) {

      const QString filename =
          item->whatsThis();

      const QString suffix =
          QString(".%1").arg(QFileInfo(filename).suffix());

      bool is_textfle = false;
      bool is_plyfile = false;

      if ( isTextFileSuffix(suffix) ) {
        is_textfle = true;
      }
      else if ( isPlyFileSuffix(suffix) ) {
        is_plyfile = true;
      }

      // CF_DEBUG("is_plyfile=%d", is_plyfile);


      if ( is_textfle ) {
        item->setIcon(textfile_icon);
        item->setData(Qt::UserRole, QVariant(true));

        if ( !item->isHidden() ) {
          hasupdates = true;
        }
      }
      else if ( is_plyfile ){
        item->setIcon(plyfile_icon);
        item->setData(Qt::UserRole, QVariant(true));

        if ( !item->isHidden() ) {
          hasupdates = true;
        }
      }
      else {
        finished = false;
        _thumbnailExtractor.start(filename);
        break;
      }
    }
  }

  if ( finished ) {
    _listWidget->updateItemsLayout();
  }
  else if ( hasupdates ) {
    _listWidget->repaint();
  }
}

void QThumbnailsView::onThumbnailExtractorStarted()
{
  updateProgressIndicator();
}

void QThumbnailsView::onThumbnailExtrated(int rid, const QIcon & icon, const QString & fullPathName)
{
  _listWidget->updateIcon(icon, fullPathName, QVariant(true));
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

    if ( path != _currentPath ) {
      refreshNow = true;
    }

    cancelPendingUpdates();
    _currentPath = path;
    _currentPathLabel->setText(_currentPath);
    if ( refreshNow ) {
      reload();
    }

    return true;
  }

  return false;
}

const QString & QThumbnailsView::currentPath() const
{
  return _currentPath;
}

void QThumbnailsView::reload()
{
  cancelPendingUpdates();
  startUpdate();
}

void QThumbnailsView::cancelPendingUpdates()
{
//  QWaitCursor wait(this);

  _searchImageFiles.cancel();
  _thumbnailExtractor.cancel();
  updateProgressIndicator();
}

void QThumbnailsView::startUpdate()
{
  _listWidget->clear();
  if ( !_currentPath.isEmpty()  ) {
    _lastSearchImageFilesRID = _searchImageFiles.start(_currentPath);
  }
}



void QThumbnailsView::onCurrentWidgetChanged(int)
{
  if (  _stack->currentWidget() == _whiteSheet ) {
    refreshWhiteListTextMessage();
  }
}

void QThumbnailsView::refreshWhiteListTextMessage()
{
  if ( QFileInfo::exists(_currentPath) ) {
    _whiteSheet->setText("<H2>No images to display</H2>");
  }
  else {
    QString text = QString("<H2>This folder does not exists.</H2>"
        "<p>%1</p>"
        "<p>The requested directory does not exists yet.</p>"
        "<p>Most likely it will be created automatically later</p>").
            arg(_currentPath);

    _whiteSheet->setText(text);
  }
}

void QThumbnailsView::onCurrentItemChanged(QListWidgetItem * current, QListWidgetItem */*previous*/)
{
  if ( current ) {
    Q_EMIT currentIconChanged(current->whatsThis());
  }
}

void QThumbnailsView::onItemDoubleClicked(QListWidgetItem *item)
{
  if ( item ) {
    Q_EMIT iconDoubleClicked(item->whatsThis());
  }
}

void QThumbnailsView::onItemEnterPressed(QListWidgetItem *item)
{
  if ( item ) {
    Q_EMIT iconEnterPressed(item->whatsThis());
  }
}

void QThumbnailsView::selectNextIcon()
{
  _listWidget->selectNextIcon();
}

void QThumbnailsView::selectPrevIcon()
{
  _listWidget->selectPrevIcon();
}

void QThumbnailsView::populateContextMenu(QMenu * menu, const QPoint &pos)
{
  QListWidgetItem * currentItem = nullptr;
  QList<QListWidgetItem*> selectedItems;
  QClipboard * clipboard = nullptr;

  selectedItems = _listWidget->selectedItems();
  currentItem = _listWidget->itemAt(pos);
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

  if ( selectedItems.size() > 1 ) {
    if ( clipboard ) {
      menu->addSeparator();

      menu->addAction("Copy file names", [clipboard, selectedItems]() {
        QString itemNames;
        for ( const QListWidgetItem * item : selectedItems ) {
          itemNames.append(QString("'%1' ").arg(item->text()));
        }

        clipboard->setText(itemNames);
      });

      menu->addAction("Copy full path names", [clipboard, selectedItems]() {
        QString itemNames;
        for ( const QListWidgetItem * item : selectedItems ) {
          itemNames.append(QString("'%1' ").arg(item->whatsThis()));
          //  itemNames.append(item->whatsThis());
          //  itemNames.append('\n');
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
          _listWidget->deleteFiles(selectedItems, true);
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
  return _listWidget->mapToGlobal(pos);
}

bool QThumbnailsView::moveToBads(const QString & pathfilename)
{
  if ( !pathfilename.isEmpty() ) {

    QFileInfo file(pathfilename);
    if ( !file.exists() ) {
      CF_ERROR("File '%s' not exists", pathfilename.toStdString().c_str());
    }
    else if ( file.path() != _currentPath ) {
      CF_ERROR("File '%s' not under '%s'",  pathfilename.toStdString().c_str(), _currentPath.toStdString().c_str());
    }
    else {

      QDir curDir(_currentPath);
      QString newpathname = QString("%1/BAD/%2").arg(_currentPath).arg(file.fileName());
      if ( !curDir.mkpath("BAD") ) {
        CF_ERROR("Can not create sub-directory under %s", _currentPath.toStdString().c_str());
      }
      else if ( !QFile::rename(pathfilename, newpathname) ) {
        CF_ERROR("QFile::rename() '%s' -> '%s' fails", pathfilename.toStdString().c_str(), newpathname.toStdString().c_str());
      }
      else {

        for ( int i = 0, n = _listWidget->count(); i < n; ++i ) {
          QListWidgetItem * item = _listWidget->item(i);
          if ( item->whatsThis() == pathfilename ) {

            _listWidget->removeItemWidget(item);
            delete item;

            QListWidgetItem * nextItem = _listWidget->item(i < n -1 ? i: 0);
            if ( nextItem ) {
              _listWidget->setCurrentItem(nextItem);
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
  if ( !_ignoreQuickFilterAction ) {
    if ( _quickFilterAction->isChecked() ) {
      showQuickFilter();
    }
    else {
      clearQuickFilter();
    }
  }
}

void QThumbnailsView::showQuickFilter(const QString & wildcard)
{

  if ( !_quickfilterDialogBox ) {

    _quickfilterDialogBox = new QThumbnailsQuickFilterDialogBox(this);

    connect(_quickfilterDialogBox, &QThumbnailsQuickFilterDialogBox::parameterChanged,
        [this] () {
          if ( _quickFilterAction->isChecked() ) {
            _listWidget->setQuickFilter(_quickfilterDialogBox->searchText(),
                _quickfilterDialogBox->matchingFlags(),
                _quickfilterDialogBox->invertMatch());
          }
        });
  }

  if ( !wildcard.isEmpty() ) {
    _quickfilterDialogBox->setSearchText(wildcard);
  }
  else if ( _quickfilterDialogBox->searchText().isEmpty() ) {
    QListWidgetItem * currentItem = _listWidget->currentItem();
    if ( currentItem ) {
      _quickfilterDialogBox->setSearchText(currentItem->text());
    }
  }

  _ignoreQuickFilterAction = true;
  _quickFilterAction->setChecked(true);
  _ignoreQuickFilterAction = false;

  _quickfilterDialogBox->show();
  _listWidget->setQuickFilter(_quickfilterDialogBox->searchText(),
      _quickfilterDialogBox->matchingFlags(),
      _quickfilterDialogBox->invertMatch());
}

void QThumbnailsView::clearQuickFilter()
{
  _listWidget->clearQuickFilter();
}
