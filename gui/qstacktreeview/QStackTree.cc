/*
 * QStackingSequencesTree.cc
 *
 *  Created on: Jan 12, 2021
 *      Author: amyznikov
 */

#include "QStackTree.h"

#include <gui/qstackingthread/QStackingThread.h>

#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define ICON_stack          "stack"
#define ICON_add_stack      "add-stack"
#define ICON_add_frames     "add-frames"
#define ICON_add_items      "add-items"
#define ICON_delete_item    "delete-item"
#define ICON_options        "options"
#define ICON_start          "start"
#define ICON_start_all      "start-all"
#define ICON_stop           "stop"
#define ICON_pause          "pause"


static QIcon getIcon(const QString & name)
{
  return QIcon(QString(":/qstacktree/icons/%1").arg(name));
}

//static QString getResource(const QString & name)
//{
//  return QString(":/qstacktree/%1").arg(name);
//}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace {

  enum TreeItemType {
    ItemType_Stack = QTreeWidgetItem::UserType + 1,
    ItemType_InputSource = QTreeWidgetItem::UserType + 2,
  };

}




class QStackTreeView::QStackItem
    : public QTreeWidgetItem
{
public:
  typedef QStackItem ThisClass;
  typedef QTreeWidgetItem Base;

  QStackItem(QTreeWidget * treeview, const c_image_stacking_options::ptr & ppline);
  void refreshInputSources();

  const c_image_stacking_options::ptr & stack() const;
  void setStack(const c_image_stacking_options::ptr & options);

protected:
  c_image_stacking_options::ptr stack_;
};

class QStackTreeView::QInputSourceItem
    : public QTreeWidgetItem
{
public:
  typedef QStackTreeView::QInputSourceItem ThisClass;
  typedef QTreeWidgetItem Base;

  QInputSourceItem(const c_input_source::ptr & input_source, QTreeWidgetItem * parent = Q_NULLPTR );
  const c_input_source::ptr & inputSource() const;
  void setInputSource(const c_input_source::ptr & input_source ) ;
  void setCkecked(bool v);
  void updateCheckState();

protected:
  c_input_source::ptr input_source_;
};



QStackTreeView::QStackItem::QStackItem(QTreeWidget * treeview, const c_image_stacking_options::ptr & ppline)
  : Base(treeview, (int)ItemType_Stack)
{
  setStack(ppline);
}

void QStackTreeView::QStackItem::refreshInputSources()
{
  QTreeWidgetItem * childItem;


  while ( (childItem = takeChild(0)) ) {
    delete childItem;
  }

  if ( stack_ && stack_->input_sequence() ) {
    for ( const c_input_source::ptr & input_source : stack_->input_sources() ) {
      new QStackTreeView::QInputSourceItem(input_source, this);
    }
  }

}

const c_image_stacking_options::ptr & QStackTreeView::QStackItem::stack() const
{
  return stack_;
}

void QStackTreeView::QStackItem::setStack(const c_image_stacking_options::ptr & options)
{
  if ( (stack_ = options) ) {
    setFlags(flags() | Qt::ItemIsEditable | Qt::ItemIsUserCheckable|Qt::ItemIsSelectable);
    setText(0, stack_->name().c_str());
    setCheckState(0, Qt::Checked);
    refreshInputSources();
  }
}


QStackTreeView::QInputSourceItem::QInputSourceItem(const c_input_source::ptr & input_source, QTreeWidgetItem * parent)
  : Base(parent, (int) ItemType_InputSource)
{
  setInputSource(input_source);
}

const c_input_source::ptr & QStackTreeView::QStackTreeView::QInputSourceItem::inputSource() const
{
  return input_source_;
}

void QStackTreeView::QStackTreeView::QInputSourceItem::setInputSource(const c_input_source::ptr & input_source)
{
  if ( (input_source_ = input_source) ) {

    setText(0, QFileInfo(input_source->filename().c_str()).fileName());
    setWhatsThis(0, input_source->filename().c_str());
    setToolTip(0, input_source->filename().c_str());
    setFlags((flags() | Qt::ItemIsUserCheckable) & ~Qt::ItemIsEditable);

    updateCheckState();
  }
}

void QStackTreeView::QStackTreeView::QInputSourceItem::setCkecked(bool v)
{
  setCheckState(0, v ? Qt::Checked : Qt::Unchecked);
}

void QStackTreeView::QStackTreeView::QInputSourceItem::updateCheckState()
{
  setCkecked(input_source_ && input_source_->enabled());
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QStackTreeView::QStackTreeView(QWidget * parent)
  : Base(parent)
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

bool QStackTreeView::setUpdatingControls(bool v)
{
  const bool oldValue = updatingControls_;
  updatingControls_ = v;
  return oldValue;
}

bool QStackTreeView::updatingControls() const
{
  return updatingControls_;
}

void QStackTreeView::set_stacklist(const c_image_stacks_collection::ptr & stacklist)
{
  setEnabled((this->stacklist_ = stacklist) != nullptr);
  populateTreeView();
}

const c_image_stacks_collection::ptr & QStackTreeView::stacklist() const
{
  return this->stacklist_;
}

void QStackTreeView::refresh()
{
  setEnabled(this->stacklist_ != nullptr);
  populateTreeView();
}

void QStackTreeView::populateTreeView()
{
  QTreeWidgetItem * item;

  const bool oldValue =
      setUpdatingControls(true);

  while ( (item = takeTopLevelItem(0)) ) {
    delete item;
  }

  if ( stacklist_ ) {
    for ( uint i = 0, n = stacklist_->size(); i < n; ++i ) {
      addStackItem(stacklist_->item(i));
    }
  }

  setUpdatingControls(oldValue);
}

QTreeWidgetItem * QStackTreeView::addStackItem(const c_image_stacking_options::ptr & options)
{
  return new QStackTreeView::QStackItem(this, options);
}

QTreeWidgetItem * QStackTreeView::addNewStack(const QString & name)
{
  QTreeWidgetItem * item = Q_NULLPTR;

  if ( stacklist_ ) {

    std::string ppname = name.toStdString();

    /* generate new name for new stacking pipeline */
    if ( ppname.empty() ) {

      char tmp[256];
      for ( int i = 0; i < 1000; ++i ) {
        sprintf(tmp, "stack%03d", i);
        if ( stacklist_->indexof(tmp) < 0 ) {
          ppname = tmp;
          break;
        }
      }
    }

    /* add new stack to colection */
    const c_image_stacking_options::ptr options =
        c_image_stacking_options::create(ppname);

    if ( options ) {

      options->set_input_sequence(c_input_sequence::create());
      stacklist_->add(options);

      /* add stack item to tree widget */
      item = addStackItem(options);
    }
  }

  return item;
}


void QStackTreeView::updateStackName(const c_image_stacking_options::ptr & stack)
{
  for ( int i = 0, n = this->topLevelItemCount(); i < n; ++i ) {

    QStackItem * item =
        dynamic_cast<QStackItem *>(this->
            topLevelItem(i));

    if ( item && item->stack() == stack ) {
      item->setText(0, stack->name().c_str());
      return;
    }
  }
}


void QStackTreeView::onAddNewStack()
{
  QTreeWidgetItem * item = addNewStack();
  if ( item ) {
    setCurrentItem(item);
    emit stackCollectionChanged();
  }
}

void QStackTreeView::onAddSourcesToCurrentStackingOptions()
{
  QStackItem * ppitem = dynamic_cast<QStackItem *>(currentItem());
  if ( ppitem ) {

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


      if ( ppitem->stack()->input_sequence()->add_sources(cfilenames) ) {

        const bool oldValue =
            setUpdatingControls(true);

        ppitem->refreshInputSources();
        expandItem(ppitem);

        setUpdatingControls(oldValue);
      }


    }

  }
}

void QStackTreeView::onDeleteSelectedItems()
{
  if ( QStackingThread::isRunning() ) {
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

void QStackTreeView::deleteItems(QList<QTreeWidgetItem*> & items)
{
  QStackTreeView::QInputSourceItem * inputSourceItem;
  QStackItem * stackItem;

  //c_image_stacking_options::ptr & running_stack;
  //QList<QTreeWidgetItem*> running_items;

  if ( QStackingThread::isRunning() ) {
    //running_stack = QStackingThread::currentStack();
    return;
  }

  const bool oldValue =
      setUpdatingControls(true);

  for ( QTreeWidgetItem * item : items ) {

    switch ( item->type() ) {

    case ItemType_Stack :
      if ( (stackItem = dynamic_cast<QStackItem *>(item)) ) {

        c_image_stacking_options::ptr stack =
            stackItem->stack();

//        if ( stack == running_stack ) {
//          running_items.append(stackItem);
//          continue;
//        }

        for ( int i = 0, n = item->childCount(); i < n; ++i ) {
          if ( (inputSourceItem = dynamic_cast<QStackTreeView::QInputSourceItem *>(item->child(i))) ) {

            if ( stack ) {
              stack->remove_input_source(inputSourceItem->inputSource());
            }

            inputSourceItem->setInputSource(nullptr);
          }
        }

        if ( stack ) {
          stackItem->setStack(nullptr);
          stacklist_->remove(stack);
        }

      }
      break;

    case ItemType_InputSource :
      if ( (inputSourceItem = dynamic_cast<QStackTreeView::QInputSourceItem *>(item)) ) {

        if ( (stackItem = dynamic_cast<QStackItem *>(item->parent())) ) {
//          if ( stackItem->stack() == running_stack ) {
//            running_items.append(stackItem);
//            continue;
//          }
        }

        c_input_source::ptr input_source =
            inputSourceItem->inputSource();

        if ( input_source ) {
          inputSourceItem->setInputSource(nullptr);

          if ( stackItem && stackItem->stack() ) {
            stackItem->stack()->remove_input_source(input_source);
          }

        }
      }
      break;
    }
  }

  for ( int i = 0; i < items.size(); ++i ) {
    QTreeWidgetItem * item = items[i];
    if ( item->type() == ItemType_InputSource ) {
      items.removeAt(i--);
      delete item;
    }
  }

  qDeleteAll(items);

  setUpdatingControls(oldValue);

  emit stackCollectionChanged();
}

void QStackTreeView::onItemChanged(QTreeWidgetItem *item, int column)
{
  if ( updatingControls() ) {
    return;
  }

  if ( item ) {
    switch ( item->type() ) {

    case ItemType_Stack : {

      QStackTreeView::QStackItem * ppItem =
          dynamic_cast<QStackTreeView::QStackItem *>(item);

      if ( ppItem->text(0).isEmpty() ) {

        const bool oldValue = setUpdatingControls(true);
        ppItem->setText(0, ppItem->stack()->name().c_str());
        setUpdatingControls(oldValue);

      }
      else if ( ppItem->text(0).toStdString() != ppItem->stack()->name() ) {
        ppItem->stack()->set_name(ppItem->text(0).toStdString());
        emit stackNameChanged(ppItem->stack());
      }

      break;
    }

    case ItemType_InputSource : {

      QStackTreeView::QInputSourceItem * sourceItem =
          dynamic_cast<QStackTreeView::QInputSourceItem *>(item);

      if ( sourceItem ) {

        QStackTreeView::QStackItem * stackItem =
            dynamic_cast<QStackTreeView::QStackItem *>(item->parent());

        if ( stackItem && stackItem->stack() ) {

          if ( QStackingThread::isRunning() ) {
            if ( QStackingThread::currentStack()->input_sequence() == stackItem->stack()->input_sequence() ) {
              break;
            }
          }

          sourceItem->inputSource()->set_enabled( sourceItem->checkState(0) == Qt::Checked);
          emit stackSourcesChanged(stackItem->stack());
        }
      }

      break;
    }

    default :
      break;
    }
  }

}

void QStackTreeView::keyPressEvent(QKeyEvent *e)
{
  switch ( e->key() ) {
  case Qt::Key_Delete :
    onDeleteSelectedItems();
    return;
  }

  Base::keyPressEvent(e);
}


void QStackTreeView::mouseMoveEvent(QMouseEvent *e)
{
  if ( !(e->buttons() & Qt::LeftButton) ) {
    return Base::mouseMoveEvent(e);
  }

  if ( QStackingThread::isRunning() ) {
    return Base::mouseMoveEvent(e);
  }


  QList<QTreeWidgetItem*> selection = selectedItems();
  if ( selection.count() < 1 ) {
    return Base::mouseMoveEvent(e);
  }

  bool all_selected_items_are_frame_sources = true;
  for ( const QTreeWidgetItem * item : selection ) {
    if ( item->type() != ItemType_InputSource ) {
      all_selected_items_are_frame_sources = false;
      break;
    }
  }

  if ( !all_selected_items_are_frame_sources ) {
    return Base::mouseMoveEvent(e);
  }

  QList<QUrl> list;
  for ( const QTreeWidgetItem * item : selection ) {

    const QStackTreeView::QInputSourceItem * inputSourceItem =
        dynamic_cast<const QStackTreeView::QInputSourceItem*>(item);

    const QStackItem * parentItem =
        dynamic_cast<const QStackItem*>(inputSourceItem->parent());

    const c_input_source::ptr & input_source =
        inputSourceItem->inputSource();

    const c_image_stacking_options::ptr & stacking_options =
        parentItem->stack();


    list.append(QUrl(QString("cinputsource://%1@%2").
        arg(stacking_options->name().c_str()).
        arg(input_source->filename().c_str())));
  }


  // mime stuff && start drag
  QMimeData *mimeData = new QMimeData;
  mimeData->setUrls(list);

  QDrag * drag = new QDrag(this);
  drag->setMimeData(mimeData);

  drag->exec(Qt::MoveAction);
}



Qt::DropActions QStackTreeView::supportedDropActions() const
{
  return Qt::CopyAction | Qt::MoveAction;
}

void QStackTreeView::dragEnterEvent(QDragEnterEvent *event)
{
  if ( event->mimeData()->hasUrls() ) {
    event->acceptProposedAction();
  }
  else {
    Base::dragEnterEvent(event);
  }
}

void QStackTreeView::dragMoveEvent(QDragMoveEvent *event)
{
  if ( event->mimeData()->hasUrls() ) {
    event->setDropAction(Qt::DropAction::CopyAction);
    event->accept();
    return;
  }
  event->setDropAction(Qt::DropAction::IgnoreAction);
}


void QStackTreeView::dropEvent(QDropEvent *e)
{
  QTreeWidgetItem * selectedItem = Q_NULLPTR;
  QStackItem * stackItem = Q_NULLPTR;
  bool newStackItemCreated = false;
  Qt::DropAction action = Qt::IgnoreAction;


  if ( !e->mimeData()->hasUrls() ) {
    return;
  }

  if ( (selectedItem = Base::itemAt(e->pos())) ) {
    switch ( selectedItem->type() ) {
    case ItemType_Stack :
      stackItem = dynamic_cast<QStackItem *>(selectedItem);
      break;
    case ItemType_InputSource :
      stackItem = dynamic_cast<QStackItem *>(selectedItem->parent());
      break;
    default :
      CF_DEBUG("selectedItem->type()=%d", selectedItem->type());
      break;
    }
  }

  const bool oldValue =
      setUpdatingControls(true);


  if ( stackItem || (e->keyboardModifiers() & Qt::ControlModifier) ) {
    //
    // Add dropped items to single stack
    //

    if ( !stackItem ) {
      if ( (stackItem = dynamic_cast<QStackItem *>(addNewStack())) ) {
        newStackItemCreated = true;
      }
    }

    if ( stackItem ) {

      if ( !dropSources(e, stackItem, selectedItem) ) {
        if ( newStackItemCreated ) {
          delete stackItem;
        }
      }
      else {

        action = Qt::CopyAction;

        if ( newStackItemCreated ) {

          const QFileInfo fileInfo(stackItem->stack()->input_sequence()->source(0)->filename().c_str());
          const std::string name = fileInfo.completeBaseName().toStdString();

          if ( stacklist_->indexof(name) < 0 ) {
            stackItem->stack()->set_name(name);
            stackItem->setText(0, name.c_str());
          }

          setCurrentItem(stackItem);
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

    for ( const QUrl & url : urls ) {

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


      if ( (stackItem = dynamic_cast<QStackItem *>(addNewStack())) ) {
        if ( !dropSource(e, url, stackItem, selectedItem) ) {
          delete stackItem;
        }
        else {

          action = Qt::CopyAction;

          const QFileInfo fileInfo(stackItem->stack()->input_sequence()->source(0)->filename().c_str());
          const std::string name = fileInfo.completeBaseName().toStdString();

          if ( stacklist_->indexof(name) < 0 ) {
            stackItem->stack()->set_name(name);
            stackItem->setText(0, name.c_str());
          }

          setCurrentItem(stackItem);
        }
      }
    }
  }


  e->setDropAction(action);
  e->accept();

  setFocus();

  setUpdatingControls(oldValue);

  if ( action != Qt::IgnoreAction ) {
    emit stackCollectionChanged();
  }
}


bool QStackTreeView::dropSource(QDropEvent *e, const QUrl & url, QStackItem * targetStackItem, QTreeWidgetItem * selectedItem)
{

  bool dropped = false;

  const c_input_sequence::ptr & target_sequence =
      targetStackItem->stack()->input_sequence();

  QStackTreeView::QInputSourceItem * targetSourceItem = Q_NULLPTR;

  if ( selectedItem && selectedItem->type() == ItemType_InputSource ) {
    targetSourceItem = dynamic_cast<QStackTreeView::QInputSourceItem * >(selectedItem);
  }

  if ( url.scheme() != "cinputsource" ) {


    QFileInfo fileInfo(url.toLocalFile());

    if ( !fileInfo.isDir() ) {

      const std::string pathfilename =
          fileInfo.absoluteFilePath().toStdString();

      if ( target_sequence->indexof(pathfilename) < 0 ) {

        const int targetIndex = (!targetSourceItem) ? -1 :
            target_sequence->indexof(targetSourceItem->inputSource());

        c_input_source::ptr input_source = target_sequence->add_source(pathfilename, targetIndex);
        if (  input_source ) {

          targetStackItem->insertChild( target_sequence->indexof(input_source),
              new QStackTreeView::QInputSourceItem(input_source));

          dropped = true;
        }
      }
    }
  }

  else  {
    QStackItem * sourceStackItem;
    QStackTreeView::QInputSourceItem * inputSourceItem;

    if ( (sourceStackItem = findStackItem(url.userName())) ) {
      if ( (inputSourceItem = findInputSourceItem(sourceStackItem, url.path())) ) {

        const std::string source_file_name = url.path().toStdString();
        const c_input_sequence::ptr & source_sequence = sourceStackItem->stack()->input_sequence();
        const int sourceIndex = source_sequence->indexof(source_file_name);
        const int targetIndex = (!targetSourceItem) ? -1 : target_sequence->indexof(targetSourceItem->inputSource());

        if ( sourceIndex >= 0 ) {

          if ( sourceStackItem == targetStackItem ) {

            if ( targetIndex != sourceIndex ) {

              sourceStackItem->removeChild(inputSourceItem);
              source_sequence->remove_source(sourceIndex);
              delete inputSourceItem;

              c_input_source::ptr input_source = target_sequence->add_source(source_file_name, targetIndex);
              if (  input_source ) {

                targetStackItem->insertChild(target_sequence->indexof(input_source),
                    new QStackTreeView::QInputSourceItem(input_source));

                dropped = true;
              }
            }
          }
          else if ( target_sequence->indexof(source_file_name) < 0 ) {

            if ( !(e->keyboardModifiers() & Qt::ControlModifier) ) {
              sourceStackItem->removeChild(inputSourceItem);
              source_sequence->remove_source(sourceIndex);
              delete inputSourceItem;
            }

            c_input_source::ptr input_source = target_sequence->add_source(source_file_name, targetIndex);
            if (  input_source ) {

              targetStackItem->insertChild(target_sequence->indexof(input_source),
                  new QStackTreeView::QInputSourceItem(input_source));

              dropped = true;
            }

          }

        }

      }
    }
  }

  return dropped;

}

int QStackTreeView::dropSources(QDropEvent *e, QStackItem * targetStackItem, QTreeWidgetItem * targetItem)
{

  int num_sourcess_added = 0;

  const c_input_sequence::ptr & target_sequence =
      targetStackItem->stack()->input_sequence();

  QStackTreeView::QInputSourceItem * targetSourceItem = Q_NULLPTR;

  if ( targetItem && targetItem->type() == ItemType_InputSource ) {
    targetSourceItem = dynamic_cast<QStackTreeView::QInputSourceItem * >(targetItem);
  }


  const QList<QUrl> urls = e->mimeData()->urls();
  const Qt::KeyboardModifiers keyboardModifiers = e->keyboardModifiers();

  bool drop_confirmed = false;

  static const auto confirmThisDragDrop =
      [](QStackTreeView * _this) -> bool {

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
              target_sequence->indexof(targetSourceItem->inputSource());

          c_input_source::ptr input_source = target_sequence->add_source(pathfilename, targetIndex);
          if (  input_source ) {

            targetStackItem->insertChild(target_sequence->indexof(input_source),
                new QStackTreeView::QInputSourceItem(input_source));

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



      QStackItem * sourceStackItem;
      QStackTreeView::QInputSourceItem * inputSourceItem;

      if ( (sourceStackItem = findStackItem(url.userName())) ) {
        if ( (inputSourceItem = findInputSourceItem(sourceStackItem, url.path())) ) {

          const std::string source_file_name = url.path().toStdString();
          const c_input_sequence::ptr & source_sequence = sourceStackItem->stack()->input_sequence();
          const int sourceIndex = source_sequence->indexof(source_file_name);
          const int targetIndex = (!targetSourceItem) ? -1 : target_sequence->indexof(targetSourceItem->inputSource());

          if ( sourceIndex >= 0 ) {

            if ( sourceStackItem == targetStackItem ) {

              if ( targetIndex != sourceIndex ) {

                if ( !drop_confirmed && !(drop_confirmed = confirmThisDragDrop(this)) ) {
                  return false;
                }

                sourceStackItem->removeChild(inputSourceItem);
                source_sequence->remove_source(sourceIndex);
                delete inputSourceItem;

                c_input_source::ptr input_source = target_sequence->add_source(source_file_name, targetIndex);
                if (  input_source ) {

                  targetStackItem->insertChild(target_sequence->indexof(input_source),
                      new QStackTreeView::QInputSourceItem(input_source));

                  ++ num_sourcess_added;
                }
              }
            }
            else if ( target_sequence->indexof(source_file_name) < 0 ) {

              if ( !drop_confirmed && !(drop_confirmed = confirmThisDragDrop(this)) ) {
                return false;
              }

              if ( !(keyboardModifiers & Qt::ControlModifier) ) {
                sourceStackItem->removeChild(inputSourceItem);
                source_sequence->remove_source(sourceIndex);
                delete inputSourceItem;
              }

              c_input_source::ptr input_source = target_sequence->add_source(source_file_name, targetIndex);
              if (  input_source ) {

                targetStackItem->insertChild(target_sequence->indexof(input_source),
                    new QStackTreeView::QInputSourceItem(input_source));

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

QStackTreeView::QStackItem * QStackTreeView::findStackItem(const QString & name) const
{
  if ( !name.isEmpty() ) {

    const std::string cname = name.toStdString();

    for ( int i = 0, n = topLevelItemCount(); i < n; ++i ) {
      QStackItem * item = dynamic_cast<QStackItem * >(topLevelItem(i));
      if ( item && item->stack()->name() == cname ) {
        return item;
      }
    }
  }

  return Q_NULLPTR;
}

QStackTreeView::QStackItem * QStackTreeView::findStackItem(const c_image_stacking_options::ptr & stack) const
{
  if ( stack ) {
    for ( int i = 0, n = topLevelItemCount(); i < n; ++i ) {
      QStackItem * item = dynamic_cast<QStackItem * >(topLevelItem(i));
      if ( item && stack == item->stack() ) {
        return item;
      }
    }
  }
  return Q_NULLPTR;
}


QStackTreeView::QInputSourceItem * QStackTreeView::findInputSourceItem(QStackItem * stackItem, const QString & filename) const
{
  if( stackItem && !filename.isEmpty() ) {

    const std::string cfilename = filename.toStdString();

    for ( int i = 0, n = stackItem->childCount(); i < n; ++i ) {
      QStackTreeView::QInputSourceItem * item = dynamic_cast<QStackTreeView::QInputSourceItem * >(stackItem->child(i));
      if ( item && item->inputSource()->filename() == cfilename ) {
        return item;
      }
    }
  }

  return Q_NULLPTR;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QStackTree::QStackTree(QWidget * parent)
  : Base(parent)
{
  Q_INIT_RESOURCE(qstacktree_resources);

  //
  toolbarActions_.append(addStackAction =
      new QAction(getIcon(ICON_add_stack),
          "Add stack"));
  addStackAction->setToolTip(
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
          "Delete selected item"));
  deleteItemAction->setToolTip(
      "Delete selecteted item...");
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
  treeView_ = new QStackTreeView(this);
  treeView_->installEventFilter(this);
  connect(treeView_, &QStackTreeView::stackCollectionChanged,
      this, &ThisClass::stackCollectionChanged);


  /* Setup layout */
  vbox_ = new QVBoxLayout(this);
//  vbox_->addWidget(toolbar_);
  vbox_->addWidget(treeView_);


  /* Setup event handlers */



  connect(addStackAction, &QAction::triggered,
      treeView_, &QStackTreeView::onAddNewStack);

  connect(addSourcesAction, &QAction::triggered,
      treeView_, &QStackTreeView::onAddSourcesToCurrentStackingOptions);

  connect(deleteItemAction, &QAction::triggered,
      treeView_, &QStackTreeView::onDeleteSelectedItems);



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

  connect(treeView_, &QStackTreeView::stackNameChanged,
      this, &ThisClass::stackNameChanged);

  connect(treeView_, &QStackTreeView::stackSourcesChanged,
      this, &ThisClass::stackSourcesChanged);

  connect(treeView_, &QTreeWidget::currentItemChanged,
      this, &ThisClass::onCurrentItemChanged);

  connect(treeView_, &QTreeWidget::itemDoubleClicked,
      this, &ThisClass::onItemDoubleClicked);

//  connect(treeView_, &QTreeWidget::itemPressed,
//      this, &ThisClass::onItemDoubleClicked);

  connect(QStackingThread::singleton(), &QThread::started,
      this, &ThisClass::onStackingThreadStarted);

  connect(QStackingThread::singleton(), &QStackingThread::finishing,
      this, &ThisClass::onStackingThreadFinishing);

  connect(QStackingThread::singleton(), &QThread::finished,
      this, &ThisClass::onStackingThreadFinished);

}

void QStackTree::set_stacklist(const c_image_stacks_collection::ptr & pipelines)
{
  treeView_->set_stacklist(pipelines);
}

const c_image_stacks_collection::ptr & QStackTree::stacklist() const
{
  return treeView_->stacklist();
}

const QList<QAction *> & QStackTree::toolbarActions() const
{
  return toolbarActions_;
}

c_input_source::ptr QStackTree::getCurrentInputSource(c_image_stacking_options::ptr * parentStack) const
{
  QTreeWidgetItem * currentItem =
      treeView_->currentItem();

  if ( currentItem ) {

    switch (currentItem->type()) {

    case ItemType_InputSource: {

      c_input_source::ptr inputSource;

      QStackTreeView::QInputSourceItem * inputSourceItem =
          dynamic_cast<QStackTreeView::QInputSourceItem *>(currentItem);

      if ( inputSourceItem ) {

        inputSource = inputSourceItem->inputSource();

        if( parentStack ) {

          QStackTreeView::QStackItem *stackItem =
              dynamic_cast<QStackTreeView::QStackItem*>(inputSourceItem->
                  parent());

          if( !stackItem ) {
            parentStack->reset();
          }
          else {
            *parentStack = stackItem->stack();
          }
        }
      }

      return inputSource;
    }

    case ItemType_Stack:
      if( parentStack ) {

        QStackTreeView::QStackItem *stackItem =
            dynamic_cast<QStackTreeView::QStackItem*>(currentItem);

        if( !stackItem ) {
          parentStack->reset();
        }
        else {
          *parentStack = stackItem->stack();
        }

      }
      return nullptr;
    }
  }

  if ( parentStack ) {
    parentStack->reset();
  }

  return nullptr;
}


void QStackTree::applyInputOptionsToAll(const c_input_options & options)
{
  bool hasUpdates = false;

  const int n = treeView_->topLevelItemCount();
  for ( int i = 0; i < n; ++i ) {

    QStackTreeView::QStackItem * stackItem =
        dynamic_cast<QStackTreeView::QStackItem*>(treeView_->topLevelItem(i));

    if ( !stackItem || !stackItem->isSelected() ) {
      continue;
    }

    const c_image_stacking_options::ptr & stack = stackItem->stack();
    if ( !stack || &stack->input_options() == &options ) {
      continue;
    }

    if ( QStackingThread::isRunning() && QStackingThread::currentStack() == stack ) {
      continue;
    }

    // copy here
    hasUpdates = true;

    const std::vector<int> backup_bad_frames =
        stack->input_options().bad_frames;

    stack->input_options() = options;
    stack->input_options().bad_frames = backup_bad_frames;
  }

  if ( hasUpdates ) {
    emit stackCollectionChanged();
  }

}

//void QStackTree::applyMasterFrameOptionsToAll(const c_master_frame_options & options)
//{
//  bool hasUpdates = false;
//
//  const int n = treeView_->topLevelItemCount();
//
//  for ( int i = 0; i < n; ++i ) {
//
//    QStackTreeView::QStackItem * stackItem =
//        dynamic_cast<QStackTreeView::QStackItem*>(treeView_->topLevelItem(i));
//
//    if ( !stackItem || !stackItem->isSelected() ) {
//      continue;
//    }
//
//    const c_image_stacking_options::ptr & stack = stackItem->stack();
//    if ( !stack || &stack->master_frame_options() == &options ) {
//      continue;
//    }
//
//    if ( QStackingThread::isRunning() && QStackingThread::currentStack() == stack ) {
//      continue;
//    }
//
//    // copy here
//    hasUpdates = true;
//    const std::string backup_master_source_path = stack->master_frame_options().master_source_path;
//    const int backup_master_frame_index = stack->master_frame_options().master_frame_index;
//    stack->master_frame_options() = options;
//    stack->master_frame_options().master_source_path = backup_master_source_path;
//    stack->master_frame_options().master_frame_index = backup_master_frame_index;
// }
//
//  if ( hasUpdates ) {
//    emit stackCollectionChanged();
//  }
//
//}

void QStackTree::applyROISelectionOptionsToAll(const c_roi_selection_options & options)
{
  bool hasUpdates = false;

  const int n = treeView_->topLevelItemCount();
  for ( int i = 0; i < n; ++i ) {

    QStackTreeView::QStackItem * stackItem =
        dynamic_cast<QStackTreeView::QStackItem*>(treeView_->topLevelItem(i));

    if ( !stackItem || !stackItem->isSelected() ) {
      continue;
    }

    const c_image_stacking_options::ptr & stack = stackItem->stack();
    if ( !stack || &stack->roi_selection_options() == &options ) {
      continue;
    }

    if ( QStackingThread::isRunning() && QStackingThread::currentStack() == stack ) {
      continue;
    }

    // copy here
    hasUpdates = true;
    stack->roi_selection_options() = options;
  }

  if ( hasUpdates ) {
    emit stackCollectionChanged();
  }

}

void QStackTree::applyFrameUpscaleOptionsToAll(const c_frame_upscale_options & options)
{
  bool hasUpdates = false;

  const int n = treeView_->topLevelItemCount();
  for ( int i = 0; i < n; ++i ) {

    QStackTreeView::QStackItem * stackItem =
        dynamic_cast<QStackTreeView::QStackItem*>(treeView_->topLevelItem(i));

    if ( !stackItem || !stackItem->isSelected() ) {
      continue;
    }

    const c_image_stacking_options::ptr & stack = stackItem->stack();
    if ( !stack || &stack->upscale_options() == &options ) {
      continue;
    }

    if ( QStackingThread::isRunning() && QStackingThread::currentStack() == stack ) {
      continue;
    }

    // copy here
    hasUpdates = true;
    stack->upscale_options() = options;
  }

  if ( hasUpdates ) {
    emit stackCollectionChanged();
  }

}


void QStackTree::applyFrameAccumulationOptionsToAll(const c_frame_accumulation_options & options)
{
  bool hasUpdates = false;

  const int n = treeView_->topLevelItemCount();
  for ( int i = 0; i < n; ++i ) {

    QStackTreeView::QStackItem * stackItem =
        dynamic_cast<QStackTreeView::QStackItem*>(treeView_->topLevelItem(i));

    if ( !stackItem || !stackItem->isSelected() ) {
      continue;
    }

    const c_image_stacking_options::ptr & stack = stackItem->stack();
    if ( !stack || &stack->accumulation_options() == &options ) {
      continue;
    }

    if ( QStackingThread::isRunning() && QStackingThread::currentStack() == stack ) {
      continue;
    }

    // copy here
    hasUpdates = true;
    stack->accumulation_options() = options;
  }

  if ( hasUpdates ) {
    emit stackCollectionChanged();
  }

}

void QStackTree::applyFrameRegistrationOptionsToAll(const c_image_stacking_options::ptr & stack)
{
  bool hasUpdates = false;

  const int n = treeView_->topLevelItemCount();
  for ( int i = 0; i < n; ++i ) {

    QStackTreeView::QStackItem * stackItem =
        dynamic_cast<QStackTreeView::QStackItem*>(treeView_->topLevelItem(i));

    if ( !stackItem || !stackItem->isSelected() ) {
      continue;
    }

    const c_image_stacking_options::ptr & current_stack = stackItem->stack();
    if ( !current_stack || current_stack == stack ) {
      continue;
    }

    if ( QStackingThread::isRunning() && QStackingThread::currentStack() == stack ) {
      continue;
    }

    // copy here
    hasUpdates = true;

    current_stack->frame_registration_options() = stack->frame_registration_options();

    const std::string backup_master_source_path = current_stack->master_frame_options().master_source_path;
    const int backup_master_frame_index = current_stack->master_frame_options().master_frame_index;
    current_stack->master_frame_options() = stack->master_frame_options();
    current_stack->master_frame_options().master_source_path = backup_master_source_path;
    current_stack->master_frame_options().master_frame_index = backup_master_frame_index;

  }

  if ( hasUpdates ) {
    emit stackCollectionChanged();
  }

}

void QStackTree::applyOutputOptionsToAll(const c_image_stacking_output_options & options)
{
  bool hasUpdates = false;

  const int n = treeView_->topLevelItemCount();

  for ( int i = 0; i < n; ++i ) {

    QStackTreeView::QStackItem * stackItem =
        dynamic_cast<QStackTreeView::QStackItem*>(treeView_->topLevelItem(i));

    if ( !stackItem || !stackItem->isSelected() ) {
      continue;
    }

    const c_image_stacking_options::ptr & stack = stackItem->stack();
    if ( !stack || &stack->output_options() == &options ) {
      continue;
    }

    if ( QStackingThread::isRunning() && QStackingThread::currentStack() == stack ) {
      continue;
    }

    // copy here
    hasUpdates = true;
    stack->output_options() = options;
  }

  if ( hasUpdates ) {
    emit stackCollectionChanged();
  }

}

void QStackTree::applyAllStackOptionsToAll(const c_image_stacking_options::ptr & fromStack)
{
  bool hasUpdates = false;

  if ( fromStack ) {
    const int n = treeView_->topLevelItemCount();

    for ( int i = 0; i < n; ++i ) {

      QStackTreeView::QStackItem * stackItem =
          dynamic_cast<QStackTreeView::QStackItem*>(treeView_->topLevelItem(i));

      if ( !stackItem || !stackItem->isSelected() ) {
        continue;
      }

      const c_image_stacking_options::ptr & stack = stackItem->stack();
      if ( !stack || stack == fromStack ) {
        continue;
      }

      if ( QStackingThread::isRunning() && QStackingThread::currentStack() == stack ) {
        continue;
      }

      // copy here
      hasUpdates = true;
      const std::vector<int> backup_bad_frames = stack->input_options().bad_frames;
      stack->input_options() = fromStack->input_options();
      stack->input_options().bad_frames = backup_bad_frames;


      stack->roi_selection_options() = fromStack->roi_selection_options();
      stack->upscale_options() = fromStack->upscale_options();
      stack->frame_registration_options() = fromStack->frame_registration_options();
      stack->accumulation_options() = fromStack->accumulation_options();
      stack->output_options() = fromStack->output_options();

      const std::string backup_master_source_path = stack->master_frame_options().master_source_path;
      const int backup_master_frame_index = stack->master_frame_options().master_frame_index;
      stack->master_frame_options() = fromStack->master_frame_options();
      stack->master_frame_options().master_source_path = backup_master_source_path;
      stack->master_frame_options().master_frame_index = backup_master_frame_index;

    }

  }

  if ( hasUpdates ) {
    emit stackCollectionChanged();
  }

}


void QStackTree::refresh()
{
  treeView_->refresh();
}


void QStackTree::updateStackName(const c_image_stacking_options::ptr & ppline)
{
  treeView_->updateStackName(ppline);
}

void QStackTree::addNewStack()
{
  treeView_->onAddNewStack();
}

void QStackTree::addSourcesToCurrentStack()
{
  treeView_->onAddSourcesToCurrentStackingOptions();
}

void QStackTree::deleteSelectedItems()
{
  treeView_->onDeleteSelectedItems();
}

bool QStackTree::eventFilter(QObject *watched, QEvent *event)
{
  if ( watched == treeView_ && event->type() == QEvent::KeyPress ) {

    const QKeyEvent * e = (const QKeyEvent*) event;
    if ( e->key() == Qt::Key_Return && treeView_ ->state() != QAbstractItemView::EditingState ) {

      QTreeWidgetItem * item = treeView_->currentItem();
      if ( item ) {

        QStackTreeView::QStackItem * stackItem = Q_NULLPTR;
        QStackTreeView::QInputSourceItem * inputSourceItem = Q_NULLPTR;

        switch ( item->type() ) {

        case ItemType_Stack :
          stackItem = dynamic_cast<QStackTreeView::QStackItem*>(item);
          break;

        case ItemType_InputSource :
          inputSourceItem = dynamic_cast<QStackTreeView::QInputSourceItem *>(item);
          stackItem = dynamic_cast<QStackTreeView::QStackItem*>(item->parent());
          break;
        }

        emit itemDoubleClicked(stackItem ? stackItem->stack() : nullptr,
            inputSourceItem ? inputSourceItem->inputSource() :nullptr);

        return true;
      }

    }
  }

  return false;
}


void QStackTree::onCurrentItemChanged(QTreeWidgetItem * current, QTreeWidgetItem * previous)
{

  if ( !current ) {
    deleteItemAction->setEnabled(false);
    startStackingMenuAction->setEnabled(false);
    stopStacking->setEnabled(false);
    showStackOptionsAction->setEnabled(false);
  }
  else {

    QStackTreeView::QStackItem * stackItem = Q_NULLPTR;
    QStackTreeView::QInputSourceItem * inputSourceItem = Q_NULLPTR;

    deleteItemAction->setEnabled(true);
    showStackOptionsAction->setEnabled(true);

    switch ( current->type() ) {

    case ItemType_Stack :
      stackItem = dynamic_cast<QStackTreeView::QStackItem*>(current);
      addSourcesAction->setEnabled(true);
      startStackingMenuAction->setEnabled(!QStackingThread::isRunning());
      break;

    case ItemType_InputSource :
      inputSourceItem = dynamic_cast<QStackTreeView::QInputSourceItem *>(current);
      stackItem = dynamic_cast<QStackTreeView::QStackItem*>(current->parent());
      startStackingMenuAction->setEnabled(false);
      break;

    default :
      addSourcesAction->setEnabled(false);
      startStackingMenuAction->setEnabled(!QStackingThread::isRunning());
      break;
    }

    emit currentItemChanged(stackItem ? stackItem->stack() : nullptr,
        inputSourceItem ? inputSourceItem->inputSource() :nullptr);
  }
}

void QStackTree::onItemDoubleClicked(QTreeWidgetItem * item, int /*column*/)
{
  QStackTreeView::QStackItem * stackItem = Q_NULLPTR;
  QStackTreeView::QInputSourceItem * inputSourceItem = Q_NULLPTR;

  switch ( item->type() ) {

  case ItemType_Stack :
    stackItem = dynamic_cast<QStackTreeView::QStackItem*>(item);
    break;

  case ItemType_InputSource :
    inputSourceItem = dynamic_cast<QStackTreeView::QInputSourceItem *>(item);
    stackItem = dynamic_cast<QStackTreeView::QStackItem*>(item->parent());
    break;
  }

  emit itemDoubleClicked(stackItem ? stackItem->stack() : nullptr,
      inputSourceItem ? inputSourceItem->inputSource() :nullptr);
}

void QStackTree::onCustomContextMenuRequested(const QPoint &pos)
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

    QTreeWidgetItem * item = contextItems[0];

    switch ( item->type() ) {
    case ItemType_Stack : {

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

    case ItemType_InputSource : {

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

              clipboard->setText( ((QStackTreeView::QInputSourceItem*)item)->
                  inputSource()->cfilename());
            }
          });

      break;
    }
    }

  }

  if ( !menu.isEmpty() ) {
    menu.exec(treeView_->mapToGlobal(pos));
  }

}

void QStackTree::onShowStackOptionsClicked()
{
  QTreeWidgetItem * currentItem =
      treeView_->currentItem();

  if ( currentItem ) {

    if ( currentItem->type() == ItemType_InputSource ) {
      currentItem = currentItem->parent();
    }

    QStackTreeView::QStackItem * stackItem =
        dynamic_cast<QStackTreeView::QStackItem *>(currentItem);

    if ( stackItem ) {
      emit showStackOptionsClicked(stackItem->stack());
    }
  }
}

void QStackTree::onStartStackingClicked()
{
  if ( !QStackingThread::isRunning() ) {

    QStackTreeView::QStackItem * item =
        dynamic_cast<QStackTreeView::QStackItem *>(
            treeView_-> currentItem());

    if ( item ) {
      currentProcessingMode_ = ProcessSingleStack;
      QStackingThread::start(item->stack());
    }
  }
}

void QStackTree::onStartAllStackingClicked()
{
  if ( !QStackingThread::isRunning() ) {
    currentProcessingMode_ = ProcessBatch;
    if ( !startNextStacking() ) {
      currentProcessingMode_ = ProcessIdle;
    }
  }
}

void QStackTree::onStopStackingClicked()
{
  if ( QStackingThread::isRunning() ) {
    currentProcessingMode_ = ProcessIdle;
    QStackingThread::cancel();
  }

}



bool QStackTree::startNextStacking()
{
  if ( !QStackingThread::isRunning() && currentProcessingMode_ == ProcessBatch ) {

    for ( int i = 0, n = treeView_->topLevelItemCount(); i < n; ++i ) {

      QStackTreeView::QStackItem * item =
          dynamic_cast<QStackTreeView::QStackItem *>(
              treeView_->topLevelItem(i));

      if ( item && item->checkState(0) == Qt::Checked ) {

        QStackingThread::start(item->stack());

        return true;
      }
    }
  }

  return false;
}


void QStackTree::onStackingThreadStarted()
{
  startStacking->setEnabled(false);
  stopStacking->setEnabled(true);

  QStackTreeView::QStackItem * stackItem =
      treeView_->findStackItem(QStackingThread::currentStack());

  if ( stackItem ) {

    const bool oldValue =
        treeView_->setUpdatingControls(true);

    for ( int j = 0, m = stackItem->childCount(); j < m; ++j ) {

      QStackTreeView::QInputSourceItem * sourceItem =
          dynamic_cast<QStackTreeView::QInputSourceItem *>(stackItem->child(j));

      if ( sourceItem ) {
        sourceItem->setFlags(sourceItem->flags() & ~Qt::ItemIsEnabled);
      }
    }

    treeView_->setUpdatingControls(oldValue);

  }
}

void QStackTree::onStackingThreadFinishing()
{
  stopStacking->setEnabled(false);
}

void QStackTree::onStackingThreadFinished()
{
  QStackTreeView::QStackItem * stackItem =
      treeView_->findStackItem(QStackingThread::currentStack());

  if ( stackItem ) {

    const bool oldValue =
        treeView_->setUpdatingControls(true);

    for ( int j = 0, m = stackItem->childCount(); j < m; ++j ) {

      QStackTreeView::QInputSourceItem * sourceItem =
          dynamic_cast<QStackTreeView::QInputSourceItem *>(stackItem->child(j));

      if ( sourceItem ) {
        sourceItem->setFlags(sourceItem->flags() | Qt::ItemIsEnabled);
      }
    }

    if ( currentProcessingMode_ == ProcessBatch ) {
      stackItem->setCheckState(0, Qt::Unchecked);
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
