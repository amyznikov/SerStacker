/*
 * QStackingSequencesTree.cc
 *
 *  Created on: Jan 12, 2021
 *      Author: amyznikov
 */

#include "QImageSequenceTreeView.h"
#include <gui/qpipelinethread/QImageProcessingPipeline.h>
#include <gui/widgets/style.h>
#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define ICON_pipeline       ":/qstacktree/icons/stack"
#define ICON_add_pipeline   ":/qstacktree/icons/add-stack"
#define ICON_add_frames     ":/qstacktree/icons/add-frames"
#define ICON_add_items      ":/qstacktree/icons/add-items"
#define ICON_delete_item    ":/qstacktree/icons/delete-item"
#define ICON_options        ":/qstacktree/icons/options"
#define ICON_start          ":/qstacktree/icons/start"
#define ICON_start_all      ":/qstacktree/icons/start-all"
#define ICON_stop           ":/qstacktree/icons/stop"
#define ICON_pause          ":/qstacktree/icons/pause"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



QImageSequenceTreeItem::QImageSequenceTreeItem(QTreeWidget * treeview, const c_image_sequence::sptr & image_sequence) :
    Base(treeview, (int)QImageSequenceTreeItemType)
{
  set_image_sequence(image_sequence);
}

void QImageSequenceTreeItem::refreshInputSources()
{
  QTreeWidgetItem * childItem;


  while ( (childItem = takeChild(0)) ) {
    delete childItem;
  }

  if ( image_sequence_ && image_sequence_->input_sequence() ) {
    for ( const c_input_source::sptr & input_source : image_sequence_->input_sequence()->sources() ) {
      new QInputSourceTreeItem(input_source, this);
    }
  }

}

const c_image_sequence::sptr & QImageSequenceTreeItem::image_sequence() const
{
  return image_sequence_;
}

void QImageSequenceTreeItem::set_image_sequence(const c_image_sequence::sptr & image_sequence)
{
  if ( (image_sequence_ = image_sequence) ) {
    setFlags(flags() | Qt::ItemIsEditable | Qt::ItemIsUserCheckable|Qt::ItemIsSelectable);
    setText(0, image_sequence_->name().c_str());
    setCheckState(0, Qt::Checked);
    refreshInputSources();
  }
}


QInputSourceTreeItem::QInputSourceTreeItem(const c_input_source::sptr & input_source, QTreeWidgetItem * parent) :
    Base(parent, (int) QInputSourceTreeItemType)
{
  set_input_source(input_source);
}

const c_input_source::sptr & QInputSourceTreeItem::input_source() const
{
  return input_source_;
}

void QInputSourceTreeItem::set_input_source(const c_input_source::sptr & input_source)
{
  if ( (input_source_ = input_source) ) {

    setText(0, QFileInfo(input_source->filename().c_str()).fileName());
    setWhatsThis(0, input_source->filename().c_str());
    setToolTip(0, input_source->filename().c_str());
    setFlags((flags() | Qt::ItemIsUserCheckable) & ~Qt::ItemIsEditable);

    updateCheckState();
  }
}

void QInputSourceTreeItem::setCkecked(bool v)
{
  setCheckState(0, v ? Qt::Checked : Qt::Unchecked);
}

void QInputSourceTreeItem::updateCheckState()
{
  setCkecked(input_source_ && input_source_->enabled());
}




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QImageSequenceTreeView::QImageSequenceTreeView(QWidget * parent) :
    Base(parent)
{

  setHeaderHidden(true);
  //setRootIsDecorated(false);
  setSelectionMode(QAbstractItemView::ExtendedSelection);
  setSortingEnabled(false);
  setContextMenuPolicy(Qt::CustomContextMenu);

  viewport()->setAcceptDrops(true);
  setDefaultDropAction(Qt::CopyAction);
  setDragDropMode(QAbstractItemView::DropOnly);
  setDropIndicatorShown(true);

  setEditTriggers(QAbstractItemView::EditKeyPressed /*| QAbstractItemView::SelectedClicked*/);

  setExpandsOnDoubleClick(false);


//  connect(this, &QTreeWidget::customContextMenuRequested,
//      this, &ThisClass::onCustomContextMenuRequested);
//
  connect(this, &QTreeWidget::itemChanged,
      this, &ThisClass::onItemChanged);
//
//  connect(this, &QTreeWidget::currentItemChanged,
//      this, &ThisClass::onCurrentItemChanged);
//
//  connect(this, &QTreeWidget::itemDoubleClicked,
//      this, &ThisClass::onItemDoubleClicked);

  setEnabled(false);
}

bool QImageSequenceTreeView::setUpdatingControls(bool v)
{
  const bool oldValue = updatingControls_;
  updatingControls_ = v;
  return oldValue;
}

bool QImageSequenceTreeView::updatingControls() const
{
  return updatingControls_;
}

void QImageSequenceTreeView::set_image_sequence_collection(const c_image_sequence_collection::sptr & collection)
{
  setEnabled((this->image_sequence_collection_ = collection) != nullptr);
  populateTreeView();
}

const c_image_sequence_collection::sptr & QImageSequenceTreeView::image_sequence_collection() const
{
  return this->image_sequence_collection_;
}

void QImageSequenceTreeView::refresh()
{
  setEnabled(this->image_sequence_collection_ != nullptr);
  populateTreeView();
}

void QImageSequenceTreeView::populateTreeView()
{
  QTreeWidgetItem *item;

  const bool oldValue =
      setUpdatingControls(true);

  while ((item = takeTopLevelItem(0))) {
    delete item;
  }

  if( image_sequence_collection_ ) {
    for( uint i = 0, n = image_sequence_collection_->size(); i < n; ++i ) {
      addImageSequenceItem(image_sequence_collection_->item(i));
    }
  }

  setUpdatingControls(oldValue);
}

QTreeWidgetItem * QImageSequenceTreeView::addImageSequenceItem(const c_image_sequence::sptr & image_sequence)
{
  return new QImageSequenceTreeItem(this, image_sequence);
}


QTreeWidgetItem* QImageSequenceTreeView::addNewImageSequence(const QString & name)
{
  QTreeWidgetItem *item = nullptr;

  if( image_sequence_collection_ ) {

    std::string cname =
        name.toStdString();

    /* generate new name for new stacking pipeline */
    if( cname.empty() ) {

      char tmp[256];
      for( int i = 0; i < 1000; ++i ) {
        sprintf(tmp, "stack%03d", i);
        if( image_sequence_collection_->indexof(tmp) < 0 ) {
          cname = tmp;
          break;
        }
      }
    }

    /* add new stack to collection */
    const c_image_sequence::sptr image_sequence(new c_image_sequence(cname));
    image_sequence_collection_->add(image_sequence);

    /* add image sequence item to tree widget */
    item = addImageSequenceItem(image_sequence);
  }

  return item;
}


void QImageSequenceTreeView::updateImageSequenceName(const c_image_sequence::sptr & image_sequence)
{
  for ( int i = 0, n = this->topLevelItemCount(); i < n; ++i ) {

    QImageSequenceTreeItem * item =
        dynamic_cast<QImageSequenceTreeItem*>(this->topLevelItem(i));

    if ( item && item->image_sequence() == image_sequence ) {
      item->setText(0, image_sequence ->name().c_str());
      return;
    }
  }
}


void QImageSequenceTreeView::onAddNewImageSequence()
{
  QTreeWidgetItem * item = addNewImageSequence();
  if ( item ) {
    setCurrentItem(item);
    Q_EMIT imageSequenceCollectionChanged();
  }
}

void QImageSequenceTreeView::onAddSourcesToCurrentImageSequence()
{
  QImageSequenceTreeItem * item =
      dynamic_cast<QImageSequenceTreeItem*>(currentItem());

  if ( item ) {

    static QString filter;

    if ( filter.isEmpty() ) {

      filter.append("SER files (");
      for ( const std::string & s : c_ser_input_source::suffixes() ) {
        filter.append(QString("*%1 ").arg(s.c_str()));
      }
      filter.append(");;");

      filter.append("Regular images (");
      for ( const std::string & s : c_regular_image_input_source::suffixes() ) {
        filter.append(QString("*%1 ").arg(s.c_str()));
      }
      filter.append(");;");

#if HAVE_CFITSIO
      filter.append("FITS files (");
      for ( const std::string & s : c_fits_input_source::suffixes() ) {
        filter.append(QString("*%1 ").arg(s.c_str()));
      }
      filter.append(");;");
#endif // HAVE_CFITSIO

#if HAVE_LIBRAW
      filter.append("RAW/DSLR images (");
      for ( const std::string & s : c_raw_image_input_source::suffixes() ) {
        filter.append(QString("*%1 ").arg(s.c_str()));
      }
      filter.append(");;");
#endif // HAVE_LIBRAW

      filter.append("Movies (");
      for ( const std::string & s : c_movie_input_source::suffixes() ) {
        filter.append(QString("*%1 ").arg(s.c_str()));
      }
      filter.append(");;");

      filter.append("All Files (*.*);;");
    }

    static const QString lastSourcesDirectoryKeyName =
        "lastSourcesDirectory";

    static const QString lastSelectedFilterKeyName =
        "lastSelectedFilter";

    QSettings settings;


    QString selectedFilter =
        settings.value(lastSelectedFilterKeyName).toString();

    QStringList selectedFiles = QFileDialog::getOpenFileNames(this,
        "Select input sources",
        settings.value(lastSourcesDirectoryKeyName).toString(),
        filter,
        &selectedFilter);

    if ( !selectedFiles.empty() ) {

      settings.setValue(lastSourcesDirectoryKeyName,
          QFileInfo(selectedFiles.back()).absolutePath());

      settings.setValue(lastSelectedFilterKeyName,
          selectedFilter);

      std::vector<std::string> cfilenames;
      cfilenames.reserve(selectedFiles.size());
      for ( const QString & s: selectedFiles ) {
        cfilenames.emplace_back(s.toStdString());
      }


      if ( item->image_sequence()->input_sequence()->add_sources(cfilenames) ) {

        const bool oldValue =
            setUpdatingControls(true);

        item->refreshInputSources();
        expandItem(item);

        setUpdatingControls(oldValue);
      }
    }
  }
}

void QImageSequenceTreeView::onDeleteSelectedItems()
{
  if ( QImageProcessingPipeline::isRunning() ) {
    //running_stack = QStackingThread::currentStack();
    return;
  }

  QList<QTreeWidgetItem*> cursel = selectedItems();
  if ( !cursel.empty() ) {

    int responce = QMessageBox::question(this, "Confirmation required",
        QString("Are you sure to remove %1 items from here ?").arg(cursel.count()),
        QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel);

    if ( responce == QMessageBox::Ok ) {
      deleteItems(cursel);
    }
  }
}

void QImageSequenceTreeView::deleteItems(QList<QTreeWidgetItem*> & items)
{
  QImageSequenceTreeItem * imageSequeneItem;
  QInputSourceTreeItem * inputSourceItem;

  //c_image_stacking_options::ptr & running_stack;
  //QList<QTreeWidgetItem*> running_items;

  if ( QImageProcessingPipeline::isRunning() ) {
    //running_stack = QStackingThread::currentStack();
    return;
  }

  const bool oldValue =
      setUpdatingControls(true);

  for ( QTreeWidgetItem * item : items ) {

    switch ( item->type() ) {

    case QImageSequenceTreeItemType :
      if ( (imageSequeneItem = dynamic_cast<QImageSequenceTreeItem *>(item)) ) {

        c_image_sequence::sptr image_sequence =
            imageSequeneItem->image_sequence();

        for ( int i = 0, n = item->childCount(); i < n; ++i ) {
          if ( (inputSourceItem = dynamic_cast<QInputSourceTreeItem*>(item->child(i))) ) {

            if ( image_sequence ) {
              image_sequence->input_sequence()->remove_source(inputSourceItem->input_source());
            }

            inputSourceItem->set_input_source(nullptr);
          }
        }

        if ( image_sequence ) {
          imageSequeneItem->set_image_sequence(nullptr);
          image_sequence_collection_->remove(image_sequence);
        }

      }
      break;

    case QInputSourceTreeItemType :
      if ( (inputSourceItem = dynamic_cast<QInputSourceTreeItem *>(item)) ) {

        if ( (imageSequeneItem = dynamic_cast<QImageSequenceTreeItem *>(item->parent())) ) {
//          if ( sequenceItem->image_sequence() == running_stack ) {
//            running_items.append(sequenceItem);
//            continue;
//          }
        }

        c_input_source::sptr input_source =
            inputSourceItem->input_source();

        if ( input_source ) {
          inputSourceItem->set_input_source(nullptr);

          if ( imageSequeneItem && imageSequeneItem->image_sequence() ) {
            imageSequeneItem->image_sequence()->input_sequence()->remove_source(input_source);
          }

        }
      }
      break;
    }
  }

  for ( int i = 0; i < items.size(); ++i ) {
    QTreeWidgetItem * item = items[i];
    if ( item->type() == QInputSourceTreeItemType ) {
      items.removeAt(i--);
      delete item;
    }
  }

  qDeleteAll(items);

  setUpdatingControls(oldValue);

  Q_EMIT imageSequenceCollectionChanged();
}

void QImageSequenceTreeView::onItemChanged(QTreeWidgetItem *item, int column)
{
  if ( updatingControls() ) {
    return;
  }

  if ( item ) {
    switch ( item->type() ) {

    case QImageSequenceTreeItemType : {

      QImageSequenceTreeItem * ppItem =
          dynamic_cast<QImageSequenceTreeItem *>(item);

      if ( ppItem->text(0).isEmpty() ) {

        const bool oldValue = setUpdatingControls(true);
        ppItem->setText(0, ppItem->image_sequence()->name().c_str());
        setUpdatingControls(oldValue);

      }
      else if ( ppItem->text(0).toStdString() != ppItem->image_sequence()->name() ) {
        ppItem->image_sequence()->set_name(ppItem->text(0).toStdString());
        Q_EMIT imageSequenceNameChanged(ppItem->image_sequence());
      }

      break;
    }

    case QInputSourceTreeItemType : {

      QInputSourceTreeItem * sourceItem =
          dynamic_cast<QInputSourceTreeItem *>(item);

      if ( sourceItem ) {

        QImageSequenceTreeItem * ppItem =
            dynamic_cast<QImageSequenceTreeItem *>(item->parent());

        if ( ppItem && ppItem->image_sequence() ) {

          if ( QImageProcessingPipeline::isRunning() ) {
            if ( QImageProcessingPipeline::current_sequence() == ppItem->image_sequence() ) {
              break;
            }
          }

          sourceItem->input_source()->set_enabled( sourceItem->checkState(0) == Qt::Checked);
          Q_EMIT imageSequenceSourcesChanged(ppItem->image_sequence());
        }
      }

      break;
    }

    default :
      break;
    }
  }

}

void QImageSequenceTreeView::keyPressEvent(QKeyEvent *e)
{
  switch ( e->key() ) {
  case Qt::Key_Delete :
    onDeleteSelectedItems();
    return;
  }

  Base::keyPressEvent(e);
}

void QImageSequenceTreeView::mouseMoveEvent(QMouseEvent * e)
{
  if( !(e->buttons() & Qt::LeftButton) ) {
    return Base::mouseMoveEvent(e);
  }

  if( QImageProcessingPipeline::isRunning() ) {
    return Base::mouseMoveEvent(e);
  }

  QList<QTreeWidgetItem*> selection = selectedItems();
  if( selection.count() < 1 ) {
    return Base::mouseMoveEvent(e);
  }

  bool all_selected_items_are_frame_sources = true;
  for( const QTreeWidgetItem *item : selection ) {
    if( item->type() != QInputSourceTreeItemType ) {
      all_selected_items_are_frame_sources = false;
      break;
    }
  }

  if( !all_selected_items_are_frame_sources ) {
    return Base::mouseMoveEvent(e);
  }

  QList<QUrl> list;
  for( const QTreeWidgetItem *item : selection ) {

    const QInputSourceTreeItem *inputSourceItem =
        dynamic_cast<const QInputSourceTreeItem*>(item);

    const QImageSequenceTreeItem *parentItem =
        dynamic_cast<const QImageSequenceTreeItem*>(inputSourceItem->parent());

    const c_input_source::sptr &input_source =
        inputSourceItem->input_source();

    const c_image_sequence::sptr &image_sequence =
        parentItem->image_sequence();

    list.append(QUrl(QString("cinputsource://%1@%2").
        arg(image_sequence->name().c_str()).
        arg(input_source->filename().c_str())));
  }

  // mime stuff && start drag
  QMimeData *mimeData = new QMimeData;
  mimeData->setUrls(list);

  QDrag *drag = new QDrag(this);
  drag->setMimeData(mimeData);

  drag->exec(Qt::MoveAction);
}



Qt::DropActions QImageSequenceTreeView::supportedDropActions() const
{
  return Qt::CopyAction | Qt::MoveAction;
}

void QImageSequenceTreeView::dragEnterEvent(QDragEnterEvent *event)
{
  if ( event->mimeData()->hasUrls() ) {
    event->acceptProposedAction();
  }
  else {
    Base::dragEnterEvent(event);
  }
}

void QImageSequenceTreeView::dragMoveEvent(QDragMoveEvent *event)
{
  if ( event->mimeData()->hasUrls() ) {
    event->setDropAction(Qt::DropAction::CopyAction);
    event->accept();
    return;
  }
  event->setDropAction(Qt::DropAction::IgnoreAction);
}

void QImageSequenceTreeView::dropEvent(QDropEvent * e)
{
  QTreeWidgetItem *selectedItem = nullptr;
  QImageSequenceTreeItem *imageSequenceItem = nullptr;
  bool newStackItemCreated = false;
  Qt::DropAction action = Qt::IgnoreAction;

  if( !e->mimeData()->hasUrls() ) {
    return;
  }

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
  const QPoint epos = e->position().toPoint();
#else
  const QPoint epos = e->pos();
#endif

  if( (selectedItem = Base::itemAt(epos)) ) {
    switch (selectedItem->type()) {
      case QImageSequenceTreeItemType:
        imageSequenceItem = dynamic_cast<QImageSequenceTreeItem*>(selectedItem);
        break;
      case QInputSourceTreeItemType:
        imageSequenceItem = dynamic_cast<QImageSequenceTreeItem*>(selectedItem->parent());
        break;
      default:
        CF_DEBUG("selectedItem->type()=%d", selectedItem->type());
        break;
    }
  }

  const bool oldValue =
      setUpdatingControls(true);

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
  const Qt::KeyboardModifiers keyboardModifiers = e->modifiers();
#else
  const Qt::KeyboardModifiers keyboardModifiers = e->keyboardModifiers();
#endif

  if( imageSequenceItem || (keyboardModifiers & Qt::ControlModifier) ) {
    //
    // Add dropped items to single stack
    //

    if( !imageSequenceItem ) {
      if( (imageSequenceItem = dynamic_cast<QImageSequenceTreeItem*>(addNewImageSequence())) ) {
        newStackItemCreated = true;
      }
    }

    if( imageSequenceItem ) {

      if( !dropSources(e, imageSequenceItem, selectedItem) ) {
        if( newStackItemCreated ) {
          delete imageSequenceItem;
        }
      }
      else {

        action = Qt::CopyAction;

        if( newStackItemCreated ) {

          const QFileInfo fileInfo(
              imageSequenceItem->image_sequence()->input_sequence()->source(0)->filename().c_str());
          const std::string name = fileInfo.completeBaseName().toStdString();

          if( image_sequence_collection_->indexof(name) < 0 ) {
            imageSequenceItem->image_sequence()->set_name(name);
            imageSequenceItem->setText(0, name.c_str());
          }

          setCurrentItem(imageSequenceItem);
        }
      }
    }

  }
  else {
    //
    // Add dropped items each to separate stacks
    //

    const QList<QUrl> urls = e->mimeData()->urls();
    bool drop_confirmed = false;

    for( const QUrl &url : urls ) {

      if( !drop_confirmed && url.scheme() == "cinputsource" ) {

        int reply = QMessageBox::question(this, "Confirmation required",
            "Accept this Drag&Drop ?\n\n"
                "This confirmation is requested to prevent unintentional random mouse drags.",
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No);

        if( reply != QMessageBox::Yes ) {
          action = Qt::IgnoreAction;
          break;
        }

        drop_confirmed = true;
      }

      if( (imageSequenceItem = dynamic_cast<QImageSequenceTreeItem*>(addNewImageSequence())) ) {
        if( !dropSource(e, url, imageSequenceItem, selectedItem) ) {
          delete imageSequenceItem;
        }
        else {

          action = Qt::CopyAction;

          const QFileInfo fileInfo(
              imageSequenceItem->image_sequence()->input_sequence()->source(0)->filename().c_str());
          const std::string name = fileInfo.completeBaseName().toStdString();

          if( image_sequence_collection_->indexof(name) < 0 ) {
            imageSequenceItem->image_sequence()->set_name(name);
            imageSequenceItem->setText(0, name.c_str());
          }

          setCurrentItem(imageSequenceItem);
        }
      }
    }
  }

  e->setDropAction(action);
  e->accept();

  setFocus();

  setUpdatingControls(oldValue);

  if( action != Qt::IgnoreAction ) {
    Q_EMIT imageSequenceCollectionChanged();
  }
}


bool QImageSequenceTreeView::dropSource(QDropEvent *e, const QUrl & url, QImageSequenceTreeItem * targetSequenceItem, QTreeWidgetItem * selectedItem)
{

  bool dropped = false;

  const c_input_sequence::sptr & target_sequence =
      targetSequenceItem->image_sequence()->input_sequence();

  QInputSourceTreeItem * targetSourceItem = nullptr;

  if ( selectedItem && selectedItem->type() == QInputSourceTreeItemType ) {
    targetSourceItem = dynamic_cast<QInputSourceTreeItem * >(selectedItem);
  }

  if ( url.scheme() != "cinputsource" ) {


    QFileInfo fileInfo(url.toLocalFile());

    if ( !fileInfo.isDir() ) {

      const std::string pathfilename =
          fileInfo.absoluteFilePath().toStdString();

      if ( target_sequence->indexof(pathfilename) < 0 ) {

        const int targetIndex = (!targetSourceItem) ? -1 :
            target_sequence->indexof(targetSourceItem->input_source());

        c_input_source::sptr input_source = target_sequence->add_source(pathfilename, targetIndex);
        if (  input_source ) {

          targetSequenceItem->insertChild( target_sequence->indexof(input_source),
              new QInputSourceTreeItem(input_source));

          dropped = true;
        }
      }
    }
  }

  else  {

    QImageSequenceTreeItem * sourceStackItem;
    QInputSourceTreeItem * inputSourceItem;

    if ( (sourceStackItem = findImageSequenceItem(url.userName())) ) {
      if ( (inputSourceItem = findInputSourceItem(sourceStackItem, url.path())) ) {

        const std::string source_file_name = url.path().toStdString();
        const c_input_sequence::sptr & source_sequence = sourceStackItem->image_sequence()->input_sequence();
        const int sourceIndex = source_sequence->indexof(source_file_name);
        const int targetIndex = (!targetSourceItem) ? -1 : target_sequence->indexof(targetSourceItem->input_source());

        if ( sourceIndex >= 0 ) {

          if ( sourceStackItem == targetSequenceItem ) {

            if ( targetIndex != sourceIndex ) {

              sourceStackItem->removeChild(inputSourceItem);
              source_sequence->remove_source(sourceIndex);
              delete inputSourceItem;

              c_input_source::sptr input_source = target_sequence->add_source(source_file_name, targetIndex);
              if (  input_source ) {

                targetSequenceItem->insertChild(target_sequence->indexof(input_source),
                    new QInputSourceTreeItem(input_source));

                dropped = true;
              }
            }
          }
          else if ( target_sequence->indexof(source_file_name) < 0 ) {

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
            const Qt::KeyboardModifiers keyboardModifiers = e->modifiers();
#else
            const Qt::KeyboardModifiers keyboardModifiers = e->keyboardModifiers();
#endif

            if( !(keyboardModifiers & Qt::ControlModifier) ) {
              sourceStackItem->removeChild(inputSourceItem);
              source_sequence->remove_source(sourceIndex);
              delete inputSourceItem;
            }

            c_input_source::sptr input_source = target_sequence->add_source(source_file_name, targetIndex);
            if (  input_source ) {

              targetSequenceItem->insertChild(target_sequence->indexof(input_source),
                  new QInputSourceTreeItem(input_source));

              dropped = true;
            }

          }

        }

      }
    }
  }

  return dropped;

}

int QImageSequenceTreeView::dropSources(QDropEvent *e, QImageSequenceTreeItem * targetSequenceItem, QTreeWidgetItem * targetItem)
{

  int num_sourcess_added = 0;

  const c_input_sequence::sptr & target_sequence =
      targetSequenceItem->image_sequence()->input_sequence();

  QInputSourceTreeItem * targetSourceItem = nullptr;

  if ( targetItem && targetItem->type() == QInputSourceTreeItemType ) {
    targetSourceItem = dynamic_cast<QInputSourceTreeItem * >(targetItem);
  }

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
  const Qt::KeyboardModifiers keyboardModifiers = e->modifiers();
#else
  const Qt::KeyboardModifiers keyboardModifiers = e->keyboardModifiers();
#endif

  const QList<QUrl> urls = e->mimeData()->urls();



  bool drop_confirmed = false;

  static const auto confirmThisDragDrop =
      [](QImageSequenceTreeView * _this) -> bool {

        const int reply = QMessageBox::question(_this,
            "Confirmation required",
            "Accept this Drag&Drop ?\n\n"
            "This confirmation is requested to prevent unintentional random mouse drags.",
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No);

        return reply == QMessageBox::Yes;
      };


  for ( const QUrl & url : urls ) {

    if ( url.scheme() != "cinputsource" ) {


      QFileInfo fileInfo(url.toLocalFile());

      if ( !fileInfo.isDir() ) {

        const std::string pathfilename =
            fileInfo.absoluteFilePath().toStdString();

        if ( target_sequence->indexof(pathfilename) < 0 ) {

          const int targetIndex = (!targetSourceItem) ? -1 :
              target_sequence->indexof(targetSourceItem->input_source());

          c_input_source::sptr input_source = target_sequence->add_source(pathfilename, targetIndex);
          if (  input_source ) {

            targetSequenceItem->insertChild(target_sequence->indexof(input_source),
                new QInputSourceTreeItem(input_source));

            ++ num_sourcess_added;
          }
        }
      }

    }

    else  {

//      if ( !drop_confirmed ) {
//
//        int reply = QMessageBox::question(this, "Confirmation required",
//            "Accept this Drag&Drop ?\n\n"
//            "This confirmation is requested to prevent unintentional random mouse drags.",
//            QMessageBox::Yes | QMessageBox::No,
//            QMessageBox::No);
//
//        if ( reply != QMessageBox::Yes ) {
//          return false;
//        }
//
//        drop_confirmed = true;
//      }



      QImageSequenceTreeItem * sourceSequenceItem;
      QInputSourceTreeItem * inputSourceItem;

      if ( (sourceSequenceItem = findImageSequenceItem(url.userName())) ) {
        if ( (inputSourceItem = findInputSourceItem(sourceSequenceItem, url.path())) ) {

          const std::string source_file_name = url.path().toStdString();
          const c_input_sequence::sptr & source_sequence = sourceSequenceItem->image_sequence()->input_sequence();
          const int sourceIndex = source_sequence->indexof(source_file_name);
          const int targetIndex = (!targetSourceItem) ? -1 : target_sequence->indexof(targetSourceItem->input_source());

          if ( sourceIndex >= 0 ) {

            if ( sourceSequenceItem == targetSequenceItem ) {

              if ( targetIndex != sourceIndex ) {

                if ( !drop_confirmed && !(drop_confirmed = confirmThisDragDrop(this)) ) {
                  return false;
                }

                sourceSequenceItem->removeChild(inputSourceItem);
                source_sequence->remove_source(sourceIndex);
                delete inputSourceItem;

                c_input_source::sptr input_source = target_sequence->add_source(source_file_name, targetIndex);
                if (  input_source ) {

                  targetSequenceItem->insertChild(target_sequence->indexof(input_source),
                      new QInputSourceTreeItem(input_source));

                  ++ num_sourcess_added;
                }
              }
            }
            else if ( target_sequence->indexof(source_file_name) < 0 ) {

              if ( !drop_confirmed && !(drop_confirmed = confirmThisDragDrop(this)) ) {
                return false;
              }

              if ( !(keyboardModifiers & Qt::ControlModifier) ) {
                sourceSequenceItem->removeChild(inputSourceItem);
                source_sequence->remove_source(sourceIndex);
                delete inputSourceItem;
              }

              c_input_source::sptr input_source = target_sequence->add_source(source_file_name, targetIndex);
              if (  input_source ) {

                targetSequenceItem->insertChild(target_sequence->indexof(input_source),
                    new QInputSourceTreeItem(input_source));

                ++ num_sourcess_added;
              }

            }

          }

        }
      }
    }
  }

  return num_sourcess_added;
}

QImageSequenceTreeItem * QImageSequenceTreeView::findImageSequenceItem(const QString & name) const
{
  if ( !name.isEmpty() ) {

    const std::string cname = name.toStdString();

    for ( int i = 0, n = topLevelItemCount(); i < n; ++i ) {
      QImageSequenceTreeItem * item = dynamic_cast<QImageSequenceTreeItem * >(topLevelItem(i));
      if ( item && item->image_sequence()->name() == cname ) {
        return item;
      }
    }
  }

  return nullptr;
}

QImageSequenceTreeItem * QImageSequenceTreeView::findImageSequenceItem(const c_image_sequence::sptr & image_sequence) const
{
  if ( image_sequence ) {
    for ( int i = 0, n = topLevelItemCount(); i < n; ++i ) {
      QImageSequenceTreeItem * item = dynamic_cast<QImageSequenceTreeItem * >(topLevelItem(i));
      if ( item && image_sequence == item->image_sequence() ) {
        return item;
      }
    }
  }
  return nullptr;
}


QInputSourceTreeItem * QImageSequenceTreeView::findInputSourceItem(QImageSequenceTreeItem * sequenceItem, const QString & filename) const
{
  if( sequenceItem && !filename.isEmpty() ) {

    const std::string cfilename = filename.toStdString();

    for ( int i = 0, n = sequenceItem->childCount(); i < n; ++i ) {
      QInputSourceTreeItem * item = dynamic_cast<QInputSourceTreeItem * >(sequenceItem->child(i));
      if ( item && item->input_source()->filename() == cfilename ) {
        return item;
      }
    }
  }

  return nullptr;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QImageSequenceTree::QImageSequenceTree(QWidget * parent) : Base(parent)
{
  Q_INIT_RESOURCE(qstacktree_resources);

  //
  toolbarActions_.append(addImageSequenceAction =
      new QAction(getIcon(ICON_add_pipeline),
          "Add stack"));
  addImageSequenceAction->setToolTip(
      "Add new image stack...");


  //
  toolbarActions_.append(addSourcesAction =
      new QAction(getIcon(ICON_add_frames),
          "Add sources"));
  addSourcesAction->setToolTip(
      "Add sources to selected stack...");
  addSourcesAction->setEnabled(false);



  //
  toolbarActions_.append(deleteItemAction =
      new QAction(getIcon(ICON_delete_item),
          "Delete selected items"));
  deleteItemAction->setToolTip(
      "Delete selecteted items...");
  deleteItemAction->setEnabled(false);


  //
  toolbarActions_.append(showStackOptionsAction =
      new QAction(getIcon(ICON_options),
          "Stack options"));
  showStackOptionsAction->setToolTip(
      "Show stacking options");
  showStackOptionsAction->setEnabled(false);


  //
  toolbarActions_.append(startStackingMenuAction = new QAction("Start"));
  startStackingMenuAction->setMenu(startStacking = new QMenu());
  startStacking->addAction(startAction = new QAction(getIcon(ICON_start), "Start"));
  startStacking->addAction(startAllAction = new QAction(getIcon(ICON_start_all), "Start all"));
  startStacking->setDefaultAction(startAction);
  startStacking->setIcon(getIcon(ICON_start));
  //startStackingMenu->setEnabled(false);
  startStackingMenuAction->setEnabled(false);


  //
  toolbarActions_.append(stopStacking =
      new QAction(getIcon(ICON_stop),
          "Stop"));
  stopStacking->setToolTip(
      "Cancel current stacking");
  stopStacking->setEnabled(false);

//  toolbarActions_.append(startStopStackingAction =
//      new QAction(getIcon(ICON_start),
//          "Start / Stop stacking"));
//  startStopStackingAction->setToolTip(
//      "Start / Stop staking this sequence");
//  startStopStackingAction->setEnabled(false);





  /* Setup Treeview */
  treeView_ = new QImageSequenceTreeView(this);
  treeView_->installEventFilter(this);
  connect(treeView_, &QImageSequenceTreeView::imageSequenceCollectionChanged,
      this, &ThisClass::imageSequenceCollectionChanged);


  /* Setup layout */
  vbox_ = new QVBoxLayout(this);
//  vbox_->addWidget(toolbar_);
  vbox_->addWidget(treeView_);


  /* Setup event handlers */



  connect(addImageSequenceAction, &QAction::triggered,
      treeView_, &QImageSequenceTreeView::onAddNewImageSequence);

  connect(addSourcesAction, &QAction::triggered,
      treeView_, &QImageSequenceTreeView::onAddSourcesToCurrentImageSequence);

  connect(deleteItemAction, &QAction::triggered,
      treeView_, &QImageSequenceTreeView::onDeleteSelectedItems);



//  connect(startStopStackingAction, &QAction::triggered,
//      treeView_, &QStackListTreeView::onStartStopStackingActionClicked);

  connect(stopStacking, &QAction::triggered,
      this, &ThisClass::onStopStackingClicked);

  connect(startAction, &QAction::triggered,
      this, &ThisClass::onStartStackingClicked);

  connect(startAllAction, &QAction::triggered,
      this, &ThisClass::onStartAllStackingClicked);

  connect(showStackOptionsAction, &QAction::triggered,
      this, &ThisClass::onShowStackOptionsClicked);

  connect(treeView_, &QTreeWidget::customContextMenuRequested,
      this, &ThisClass::onCustomContextMenuRequested);

//  connect(treeView_, &QTreeWidget::itemChanged,
//      this, &ThisClass::onItemChanged);

  connect(treeView_, &QImageSequenceTreeView::imageSequenceNameChanged,
      this, &ThisClass::imageSequenceNameChanged);

  connect(treeView_, &QImageSequenceTreeView::imageSequenceSourcesChanged,
      this, &ThisClass::imageSequenceSourcesChanged);

  connect(treeView_, &QTreeWidget::currentItemChanged,
      this, &ThisClass::onCurrentItemChanged);

  connect(treeView_, &QTreeWidget::itemDoubleClicked,
      this, &ThisClass::onItemDoubleClicked);

//  connect(treeView_, &QTreeWidget::itemPressed,
//      this, &ThisClass::onItemDoubleClicked);

  connect(QImageProcessingPipeline::singleton(), &QThread::started,
      this, &ThisClass::onPipelineThreadStarted);

  connect(QImageProcessingPipeline::singleton(), &QImageProcessingPipeline::finishing,
      this, &ThisClass::onPipelineThreadFinishing);

  connect(QImageProcessingPipeline::singleton(), &QThread::finished,
      this, &ThisClass::onPipelineThreadFinished);

}

void QImageSequenceTree::set_image_sequence_collection(const c_image_sequence_collection::sptr & image_sequence_collection)
{
  treeView_->set_image_sequence_collection(image_sequence_collection);
}

const c_image_sequence_collection::sptr & QImageSequenceTree::image_sequence_collection() const
{
  return treeView_->image_sequence_collection();
}

const QList<QAction *> & QImageSequenceTree::toolbarActions() const
{
  return toolbarActions_;
}

c_input_source::sptr QImageSequenceTree::getCurrentInputSource(c_image_sequence::sptr * parent_sequence) const
{
  QTreeWidgetItem * currentItem =
      treeView_->currentItem();

  if ( currentItem ) {

    switch (currentItem->type()) {

    case QInputSourceTreeItemType: {

      c_input_source::sptr inputSource;

      QInputSourceTreeItem * inputSourceItem =
          dynamic_cast<QInputSourceTreeItem *>(currentItem);

      if ( inputSourceItem ) {

        inputSource = inputSourceItem->input_source();

        if( parent_sequence ) {

          QImageSequenceTreeItem *sequenceItem =
              dynamic_cast<QImageSequenceTreeItem*>(inputSourceItem->parent());

          if( !sequenceItem ) {
            parent_sequence->reset();
          }
          else {
            *parent_sequence = sequenceItem->image_sequence();
          }
        }
      }

      return inputSource;
    }

    case QImageSequenceTreeItemType:
      if( parent_sequence ) {

        QImageSequenceTreeItem *sequenceItem =
            dynamic_cast<QImageSequenceTreeItem*>(currentItem);

        if( !sequenceItem ) {
          parent_sequence->reset();
        }
        else {
          *parent_sequence = sequenceItem->image_sequence();
        }
      }

      return nullptr;
    }
  }

  if ( parent_sequence ) {
    parent_sequence->reset();
  }

  return nullptr;
}

void QImageSequenceTree::refresh()
{
  treeView_->refresh();
}


void QImageSequenceTree::updateImageSequenceName(const c_image_sequence::sptr & image_sequece)
{
  treeView_->updateImageSequenceName(image_sequece);
}

void QImageSequenceTree::addNewImageSequence()
{
  treeView_->onAddNewImageSequence();
}

void QImageSequenceTree::addSourcesToCurrentImageSequence()
{
  treeView_->onAddSourcesToCurrentImageSequence();
}

void QImageSequenceTree::deleteSelectedItems()
{
  treeView_->onDeleteSelectedItems();
}

bool QImageSequenceTree::eventFilter(QObject *watched, QEvent *event)
{
  if ( watched == treeView_ && event->type() == QEvent::KeyPress ) {

    const QKeyEvent * e = (const QKeyEvent*) event;
    if ( e->key() == Qt::Key_Return && treeView_ ->state() != QAbstractItemView::EditingState ) {

      QTreeWidgetItem * item = treeView_->currentItem();
      if ( item ) {

        QImageSequenceTreeItem * sequenceItem = nullptr;
        QInputSourceTreeItem * inputSourceItem = nullptr;

        switch ( item->type() ) {

        case QImageSequenceTreeItemType :
          sequenceItem = dynamic_cast<QImageSequenceTreeItem*>(item);
          break;

        case QInputSourceTreeItemType :
          inputSourceItem = dynamic_cast<QInputSourceTreeItem *>(item);
          sequenceItem = dynamic_cast<QImageSequenceTreeItem*>(item->parent());
          break;
        }

        Q_EMIT itemDoubleClicked(sequenceItem ? sequenceItem->image_sequence() : nullptr,
            inputSourceItem ? inputSourceItem->input_source() :nullptr);

        return true;
      }

    }
  }

  return false;
}


void QImageSequenceTree::onCurrentItemChanged(QTreeWidgetItem * current, QTreeWidgetItem * previous)
{

  if ( !current ) {
    deleteItemAction->setEnabled(false);
    startStackingMenuAction->setEnabled(false);
    stopStacking->setEnabled(false);
    showStackOptionsAction->setEnabled(false);
  }
  else {

    QImageSequenceTreeItem * sequenceItem = nullptr;
    QInputSourceTreeItem * inputSourceItem = nullptr;

    deleteItemAction->setEnabled(true);
    showStackOptionsAction->setEnabled(true);

    switch ( current->type() ) {

    case QImageSequenceTreeItemType :
      sequenceItem = dynamic_cast<QImageSequenceTreeItem*>(current);
      addSourcesAction->setEnabled(true);
      startStackingMenuAction->setEnabled(!QImageProcessingPipeline::isRunning());
      break;

    case QInputSourceTreeItemType :
      inputSourceItem = dynamic_cast<QInputSourceTreeItem *>(current);
      sequenceItem = dynamic_cast<QImageSequenceTreeItem*>(current->parent());
      startStackingMenuAction->setEnabled(false);
      break;

    default :
      addSourcesAction->setEnabled(false);
      startStackingMenuAction->setEnabled(!QImageProcessingPipeline::isRunning());
      break;
    }

    Q_EMIT currentItemChanged(sequenceItem ? sequenceItem->image_sequence() : nullptr,
        inputSourceItem ? inputSourceItem->input_source() :nullptr);
  }
}

void QImageSequenceTree::onItemDoubleClicked(QTreeWidgetItem * item, int /*column*/)
{
  QImageSequenceTreeItem * sequenceItem = nullptr;
  QInputSourceTreeItem * inputSourceItem = nullptr;

  switch ( item->type() ) {

  case QImageSequenceTreeItemType :
    sequenceItem = dynamic_cast<QImageSequenceTreeItem*>(item);
    break;

  case QInputSourceTreeItemType :
    inputSourceItem = dynamic_cast<QInputSourceTreeItem *>(item);
    sequenceItem = dynamic_cast<QImageSequenceTreeItem*>(item->parent());
    break;
  }

  Q_EMIT itemDoubleClicked(sequenceItem ? sequenceItem->image_sequence() : nullptr,
      inputSourceItem ? inputSourceItem->input_source() :nullptr);
}

void QImageSequenceTree::onCustomContextMenuRequested(const QPoint &pos)
{
  QMenu menu;
  QList<QTreeWidgetItem*> contextItems;
  QAction * action;

  if ( true ) {

    QModelIndexList selectedIndexes =
        treeView_->selectionModel()->selectedRows();

    if ( !selectedIndexes.empty() ) {
      for ( int i = 0, n = selectedIndexes.size(); i < n; ++i ) {
        if ( selectedIndexes[i].isValid() ) {
          contextItems.append(treeView_->itemFromIndex(selectedIndexes[i]));
        }
      }
    }
    else {
      QModelIndex index = treeView_->indexAt(pos);
      if ( index.isValid() ) {
        contextItems.append(treeView_->itemFromIndex(index));
      }
    }
  }

  if ( contextItems.size() == 1 ) {

    QTreeWidgetItem * item =
        contextItems[0];

    switch ( item->type() ) {
    case QImageSequenceTreeItemType : {

      menu.addAction("Copy Name",
          [this, item]() {

            QClipboard * clipboard =
                QApplication::clipboard();

            if ( clipboard ) {
              clipboard->setText(item->text(0));
            }
          });

      menu.addAction("Rename...",
          [this, item]() {
            treeView_->editItem(item);
          });
      break;
    }

    case QInputSourceTreeItemType : {

      menu.addAction("Copy Name",
          [this, item]() {

            QClipboard * clipboard =
                QApplication::clipboard();

            if ( clipboard ) {
              clipboard->setText(item->text(0));
            }
          });


      menu.addAction("Copy full path name",
          [this, item]() {

            QClipboard * clipboard =
                QApplication::clipboard();

            if ( clipboard ) {

              clipboard->setText( ((QInputSourceTreeItem*)item)->
                  input_source()->cfilename());
            }
          });

      break;
    }
    }

  }

  if ( contextItems.size() > 0 ) {
    menu.addAction(deleteItemAction);
  }

  if ( !menu.isEmpty() ) {
    menu.exec(treeView_->mapToGlobal(pos));
  }

}

void QImageSequenceTree::onShowStackOptionsClicked()
{
  QTreeWidgetItem * currentItem =
      treeView_->currentItem();

  if ( currentItem ) {

    if ( currentItem->type() == QInputSourceTreeItemType ) {
      currentItem = currentItem->parent();
    }

    QImageSequenceTreeItem * sequenceItem =
        dynamic_cast<QImageSequenceTreeItem *>(currentItem);

    if ( sequenceItem ) {
      Q_EMIT showImageSequenceOptionsClicked(sequenceItem->image_sequence());
    }
  }
}

void QImageSequenceTree::onStartStackingClicked()
{
  if ( !QImageProcessingPipeline::isRunning() ) {

    QImageSequenceTreeItem * item =
        dynamic_cast<QImageSequenceTreeItem *>(
            treeView_-> currentItem());

    if ( item ) {
      currentProcessingMode_ = ProcessSingleStack;
      QImageProcessingPipeline::start(item->image_sequence());
    }
  }
}

void QImageSequenceTree::onStartAllStackingClicked()
{
  if ( !QImageProcessingPipeline::isRunning() ) {
    currentProcessingMode_ = ProcessBatch;
    if ( !startNextStacking() ) {
      currentProcessingMode_ = ProcessIdle;
    }
  }
}

void QImageSequenceTree::onStopStackingClicked()
{
  if ( QImageProcessingPipeline::isRunning() ) {
    currentProcessingMode_ = ProcessIdle;
    QImageProcessingPipeline::cancel();
  }

}



bool QImageSequenceTree::startNextStacking()
{
  if ( !QImageProcessingPipeline::isRunning() && currentProcessingMode_ == ProcessBatch ) {

    for ( int i = 0, n = treeView_->topLevelItemCount(); i < n; ++i ) {

      QImageSequenceTreeItem * item =
          dynamic_cast<QImageSequenceTreeItem *>(
              treeView_->topLevelItem(i));

      if ( item && item->checkState(0) == Qt::Checked ) {

        QImageProcessingPipeline::start(item->image_sequence());

        return true;
      }
    }
  }

  return false;
}


void QImageSequenceTree::onPipelineThreadStarted()
{
  startStacking->setEnabled(false);
  stopStacking->setEnabled(true);

  QImageSequenceTreeItem *sequenceItem =
      treeView_->findImageSequenceItem(QImageProcessingPipeline::
          current_sequence());

  if ( sequenceItem ) {

    const bool oldValue =
        treeView_->setUpdatingControls(true);

    for ( int j = 0, m = sequenceItem->childCount(); j < m; ++j ) {

      QInputSourceTreeItem * sourceItem =
          dynamic_cast<QInputSourceTreeItem *>(sequenceItem->child(j));

      if ( sourceItem ) {
        sourceItem->setFlags(sourceItem->flags() & ~Qt::ItemIsEnabled);
      }
    }

    treeView_->setUpdatingControls(oldValue);

  }
}

void QImageSequenceTree::onPipelineThreadFinishing()
{
  stopStacking->setEnabled(false);
}

void QImageSequenceTree::onPipelineThreadFinished()
{
  QImageSequenceTreeItem * sequenceItem =
      treeView_->findImageSequenceItem(QImageProcessingPipeline::
          current_sequence());

  if ( sequenceItem ) {

    const bool oldValue =
        treeView_->setUpdatingControls(true);

    for ( int j = 0, m = sequenceItem->childCount(); j < m; ++j ) {

      QInputSourceTreeItem * sourceItem =
          dynamic_cast<QInputSourceTreeItem *>(sequenceItem->child(j));

      if ( sourceItem ) {
        sourceItem->setFlags(sourceItem->flags() | Qt::ItemIsEnabled);
      }
    }

    if ( currentProcessingMode_ == ProcessBatch ) {
      sequenceItem->setCheckState(0, Qt::Unchecked);
    }

    treeView_->setUpdatingControls(oldValue);
  }

  if ( currentProcessingMode_ != ProcessBatch || !startNextStacking() ) {
    currentProcessingMode_ = ProcessIdle;
    startStacking->setEnabled(true);
    stopStacking->setEnabled(true);
  }

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
