/*
 * QStackingSequencesTree.cc
 *
 *  Created on: Jan 12, 2021
 *      Author: amyznikov
 */

#include "QImageSequencesTreeView.h"
#include <gui/qpipeline/QPipelineThread.h>
#include <gui/widgets/style.h>
#include <core/io/video/c_video_input_source.h>
#include <core/io/vlo/c_vlo_input_source.h>
#include <core/readdir.h>
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
    Base(treeview, (int) QImageSequenceTreeItemType)
{
  set_input_sequence(image_sequence);
}

bool QImageSequenceTreeItem::isRunnng() const
{
  return input_sequence_ && input_sequence_->current_pipeline() && input_sequence_->current_pipeline()->is_running();
}

void QImageSequenceTreeItem::refreshInputSources()
{
  QTreeWidgetItem *childItem;

  while ((childItem = takeChild(0))) {
    delete childItem;
  }

  if( input_sequence_ ) {
    for( const c_input_source::sptr &input_source : input_sequence_->sources() ) {
      new QInputSourceTreeItem(input_source, this);
    }
  }

}

const c_image_sequence::sptr& QImageSequenceTreeItem::input_sequence() const
{
  return input_sequence_;
}

void QImageSequenceTreeItem::set_input_sequence(const c_image_sequence::sptr & image_sequence)
{
  if( (input_sequence_ = image_sequence) ) {
    setFlags(flags() | Qt::ItemIsEditable | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable);
    setText(0, input_sequence_->name().c_str());

    setCheckState(0, Qt::Checked);
    refreshInputSources();
  }
}

QInputSourceTreeItem::QInputSourceTreeItem(const c_input_source::sptr & input_source, QTreeWidgetItem * parent) :
    Base(parent, (int) QInputSourceTreeItemType)
{
  set_input_source(input_source);
}

const c_input_source::sptr& QInputSourceTreeItem::input_source() const
{
  return input_source_;
}

void QInputSourceTreeItem::set_input_source(const c_input_source::sptr & input_source)
{
  if( (input_source_ = input_source) ) {

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

QImageSequencesTreeView::QImageSequencesTreeView(QWidget * parent) :
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

  //setEnabled(false);
}

std::string QImageSequencesTreeView::default_config_filename_ =
    "~/.config/SerStacker/corrent_work.cfg";

void QImageSequencesTreeView::loadSequences(const std::string & cfgfilename)
{
  c_update_controls_lock lock(this);

  std::string filename;

  if( !cfgfilename.empty() ) {
    filename = cfgfilename;
  }
  else if( !config_filename_.empty() ) {
    filename = config_filename_;
  }
  else {
    filename = default_config_filename_;
  }

  if( (filename = expand_path(filename)).empty() ) {
    CF_ERROR("No output config file name specified QImageSequencesTreeView::loadSequences()");
    return;
  }

  CF_DEBUG("Loading '%s' ...", filename.c_str());

  c_config cfg(filename);

  if( !cfg.read() ) {
    CF_FATAL("cfg.read('%s') fails", filename.c_str());
    return;
  }

  std::string object_class;
  if( !::load_settings(cfg.root(), "object_class", &object_class) ) {
    CF_FATAL("[%s] load_settings(object_class) fails", filename.c_str());
    return;
  }

  if( object_class != "c_image_processing_pipeline_collection" ) {
    CF_FATAL("Incorrect object_class='%s' from file '%s'",
        object_class.c_str(), filename.c_str());
    return;
  }

  c_config_setting section =
      cfg.root().get("items");

  if( !section || !section.isList() ) {
    CF_FATAL("section 'items' is not found in file '%s''", filename.c_str());
    return;
  }


  Base::clear();


  const int N =
      section.length();

  for( int i = 0; i < N; ++i ) {

    c_config_setting item =
        section.get_element(i);

    if( !item || !item.isGroup() ) {
      CF_DEBUG("items[%d] is not a group", i);
    }
    else {

      c_image_sequence::sptr sequence(new c_image_sequence());
      std::string class_name, object_name;
      c_config_setting subsection;

      if( (subsection = item["input_sequence"]) && subsection.isGroup() ) {
        if( !sequence->serialize(subsection, false) ) {
          CF_ERROR("sequence->serialize() fails for item index %d", i);
          continue;
        }
      }

      if( (subsection = item["pipelines"]) && subsection.isList() ) {

        const int n =
            subsection.length();

        for( int i = 0; i < n; ++i ) {

          c_config_setting ppitem =
              subsection[i];

          if( !ppitem.isGroup() ) {
            CF_ERROR("pipeline item %d is not a libconfig group", i);
            continue;
          }

          if( !load_settings(ppitem, "class_name", &class_name) || class_name.empty() ) {
            CF_ERROR("can not extract pipeline class name for libconfig item %d", i);
            continue;
          }

          if( !load_settings(ppitem, "name", &object_name) || object_name.empty() ) {
            CF_ERROR("can not extract pipeline object name for libconfig item %d of class '%s'", i,
                class_name.c_str());
            continue;
          }

          c_image_processing_pipeline::sptr pipeline =
              c_image_processing_pipeline::create_instance(class_name,
                  object_name,
                  sequence);

          if( !pipeline ) {
            CF_ERROR("c_image_processing_pipeline::create_instance(class_name='%s' object_name=%s) fails",
                class_name.c_str(), object_name.c_str());
            continue;
          }

          if( !pipeline->serialize(ppitem, false) ) {
            CF_ERROR("pipeline->serialize(class='%s', name='%s', save=false) fails for item %d",
                class_name.c_str(), object_name.c_str(), i);
            continue;
          }

          sequence->add_pipeline(pipeline);
        }
      }

      if( load_settings(item, "current_pipeline", &object_name) && !object_name.empty() ) {
        sequence->set_current_pipeline(object_name);
      }

      addImageSequenceItem(sequence);
    }
  }

  config_filename_ = filename;
}

void QImageSequencesTreeView::saveSequences(const std::string & cfgfilename)
{
  std::string filename;

  if( !cfgfilename.empty() ) {
    filename = cfgfilename;
  }
  else if( !config_filename_.empty() ) {
    filename = config_filename_;
  }
  else {
    filename = default_config_filename_;
  }

  if( (filename = expand_path(filename)).empty() ) {
    CF_ERROR("No output config file name specified for QImageSequencesTreeView::saveSequences()");
    return;
  }

  CF_DEBUG("Saving '%s' ...",
      filename.c_str());

  c_config cfg(filename);

  time_t t = time(0);

  if( !save_settings(cfg.root(), "object_class", std::string("c_image_processing_pipeline_collection")) ) {
    CF_FATAL("save_settings(object_class) fails");
    return;
  }

  if( !save_settings(cfg.root(), "created", asctime(localtime(&t))) ) {
    CF_FATAL("save_settings(created) fails");
    return;
  }

  c_config_setting list =
      cfg.root().add_list("items");

  const int N =
      Base::topLevelItemCount();

  for( int i = 0; i < N; ++i ) {

    const QImageSequenceTreeItem *item =
        dynamic_cast<QImageSequenceTreeItem*>(Base::topLevelItem(i));

    if( item ) {

      const c_image_sequence::sptr &sequence =
          item->input_sequence();

      if( sequence ) {

        c_config_setting sequence_item =
            list.add_group();

        sequence->serialize(sequence_item.add_group("input_sequence"), true);

        if( !sequence->pipelines().empty() ) {

          if( sequence->current_pipeline() ) {
            save_settings(sequence_item, "current_pipeline",
                sequence->current_pipeline()->name());
          }

          c_config_setting pplist =
              sequence_item.add_list("pipelines");

          for( const c_image_processing_pipeline::sptr &pipeline : sequence->pipelines() ) {
            if( pipeline ) {
              pipeline->serialize(pplist.add_group(), true);
            }
          }
        }
      }
    }
  }

  if( !cfg.write() ) {
    CF_FATAL("cfg.write('%s') fails", cfg.filename().c_str());
    return;
  }

  config_filename_ = filename;
}

//void QImageSequencesTreeView::set_image_sequence_collection(const c_image_sequence_collection::sptr & collection)
//{
//  setEnabled((this->image_sequence_collection_ = collection) != nullptr);
//  populateTreeView();
//}
//
//const c_image_sequence_collection::sptr& QImageSequencesTreeView::image_sequence_collection() const
//{
//  return this->image_sequence_collection_;
//}

//void QImageSequencesTreeView::refresh()
//{
//  setEnabled(this->image_sequence_collection_ != nullptr);
//  populateTreeView();
//}

//void QImageSequencesTreeView::populateTreeView()
//{
//  QTreeWidgetItem *item;
//
//  const bool oldValue =
//      setUpdatingControls(true);
//
//  while ((item = takeTopLevelItem(0))) {
//    delete item;
//  }
//
//  if( image_sequence_collection_ ) {
//    for( uint i = 0, n = image_sequence_collection_->size(); i < n; ++i ) {
//      addImageSequenceItem(image_sequence_collection_->item(i));
//    }
//  }
//
//  setUpdatingControls(oldValue);
//}

QImageSequenceTreeItem * QImageSequencesTreeView::addImageSequenceItem(const c_image_sequence::sptr & image_sequence)
{
  return new QImageSequenceTreeItem(this, image_sequence);
}

/* add new sequence to collection */
QImageSequenceTreeItem * QImageSequencesTreeView::addNewImageSequence(const QString & name)
{
  c_update_controls_lock lock(this);

  QTreeWidgetItem *item = nullptr;

  std::string cname =
      name.toStdString();

  /* generate new name for new stacking pipeline */
  if( cname.empty() ) {

    for( int i = 0; i < 1000; ++i ) {

      const QString tmp =
          qsprintf("stack%03d", i);

      if ( !findImageSequenceItem(tmp) ) {
        cname = tmp.toStdString();
        break;
      }
    }
  }

  return addImageSequenceItem(c_image_sequence::sptr(new c_image_sequence(cname)));
}

void QImageSequencesTreeView::updateImageSequenceName(const c_image_sequence::sptr & image_sequence)
{
  c_update_controls_lock lock(this);

  for( int i = 0, n = this->topLevelItemCount(); i < n; ++i ) {

    QImageSequenceTreeItem *item =
        dynamic_cast<QImageSequenceTreeItem*>(this->topLevelItem(i));

    if( item && item->input_sequence() == image_sequence ) {
      item->setText(0, image_sequence->name().c_str());
      return;
    }
  }
}

void QImageSequencesTreeView::onAddNewImageSequence()
{
  QTreeWidgetItem *item = addNewImageSequence();
  if( item ) {
    setCurrentItem(item);
    Q_EMIT imageSequenceCollectionChanged();
  }
}

void QImageSequencesTreeView::onAddSourcesToCurrentImageSequence()
{
  c_update_controls_lock lock(this);

  QImageSequenceTreeItem *item =
      dynamic_cast<QImageSequenceTreeItem*>(currentItem());

  if( item ) {

    static QString filter;

    if( filter.isEmpty() ) {

      filter.append("SER files (");
      for( const std::string &s : c_ser_input_source::suffixes() ) {
        filter.append(QString("*%1 ").arg(s.c_str()));
      }
      filter.append(");;");

      filter.append("Regular images (");
      for( const std::string &s : c_regular_image_input_source::suffixes() ) {
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
      for( const std::string &s : c_movie_input_source::suffixes() ) {
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

    if( !selectedFiles.empty() ) {

      settings.setValue(lastSourcesDirectoryKeyName,
          QFileInfo(selectedFiles.back()).absolutePath());

      settings.setValue(lastSelectedFilterKeyName,
          selectedFilter);

      std::vector<std::string> cfilenames;
      cfilenames.reserve(selectedFiles.size());
      for( const QString &s : selectedFiles ) {
        cfilenames.emplace_back(s.toStdString());
      }

      if( item->input_sequence()->add_sources(cfilenames) ) {
        c_update_controls_lock lock(this);
        item->refreshInputSources();
        expandItem(item);
      }
    }
  }
}

void QImageSequencesTreeView::onDeleteSelectedItems()
{
//  if ( QImageProcessingPipeline::isRunning() ) {
//    //running_stack = QStackingThread::currentStack();
//    return;
//  }

  QList<QTreeWidgetItem*> cursel = selectedItems();
  if( !cursel.empty() ) {

    if( cursel.size() == 1 ) {

      const QImageSequenceTreeItem *sequenceItem =
          getImageSequenceItem(cursel[0]);

      if( !sequenceItem || sequenceItem->isRunnng() ) {
        return;
      }
    }

    int responce = QMessageBox::question(this, "Confirmation required",
        QString("Are you sure to remove %1 items from here ?").arg(cursel.count()),
        QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel);

    if( responce == QMessageBox::Ok ) {
      deleteItems(cursel);
    }
  }
}

void QImageSequencesTreeView::deleteItems(QList<QTreeWidgetItem*> & items)
{
  c_update_controls_lock lock(this);

  QImageSequenceTreeItem *imageSequenceItem;
  QInputSourceTreeItem *inputSourceItem;

  for( QTreeWidgetItem *item : items ) {

    switch (item->type()) {

      case QImageSequenceTreeItemType:
        if( (imageSequenceItem = dynamic_cast<QImageSequenceTreeItem*>(item)) ) {

          c_image_sequence::sptr image_sequence =
              imageSequenceItem->input_sequence();

          if( image_sequence->current_pipeline() && image_sequence->current_pipeline()->is_running() ) {
            continue;
          }

//          for( int i = 0, n = item->childCount(); i < n; ++i ) {
//            if( (inputSourceItem = dynamic_cast<QInputSourceTreeItem*>(item->child(i))) ) {
//
//              if( image_sequence ) {
//                image_sequence->remove_source(inputSourceItem->input_source());
//              }
//
//              inputSourceItem->set_input_source(nullptr);
//            }
//          }
//
//          if( image_sequence ) {
//            imageSequenceItem->set_input_sequence(nullptr);
//            image_sequence_collection_->remove(image_sequence);
//          }

        }
        break;

      case QInputSourceTreeItemType:
        if( (inputSourceItem = dynamic_cast<QInputSourceTreeItem*>(item)) ) {

          if( (imageSequenceItem = dynamic_cast<QImageSequenceTreeItem*>(item->parent())) ) {
            if( imageSequenceItem->isRunnng() ) {
              continue;
            }
          }

          c_input_source::sptr input_source =
              inputSourceItem->input_source();

          if( input_source ) {
            inputSourceItem->set_input_source(nullptr);

            if( imageSequenceItem && imageSequenceItem->input_sequence() ) {
              imageSequenceItem->input_sequence()->remove_source(input_source);
            }

          }
        }
        break;
    }
  }

  for( int i = 0; i < items.size(); ++i ) {
    QTreeWidgetItem *item = items[i];
    if( item->type() == QInputSourceTreeItemType ) {
      items.removeAt(i--);
      delete item;
    }
  }

  qDeleteAll(items);

  Q_EMIT imageSequenceCollectionChanged();
}

void QImageSequencesTreeView::onItemChanged(QTreeWidgetItem * item, int column)
{
  if( updatingControls() ) {
    return;
  }

  if( item ) {

    switch (item->type()) {

      case QImageSequenceTreeItemType: {

        QImageSequenceTreeItem *ppItem =
            dynamic_cast<QImageSequenceTreeItem*>(item);

        if( ppItem->text(0).isEmpty() ) {
          c_update_controls_lock lock(this);
          ppItem->setText(0, ppItem->input_sequence()->name().c_str());
        }
        else if( ppItem->text(0).toStdString() != ppItem->input_sequence()->name() ) {
          ppItem->input_sequence()->set_name(ppItem->text(0).toStdString());
          Q_EMIT imageSequenceNameChanged(ppItem->input_sequence());
        }

        break;
      }

      case QInputSourceTreeItemType: {

        QInputSourceTreeItem *sourceItem =
            dynamic_cast<QInputSourceTreeItem*>(item);

        if( sourceItem ) {

          QImageSequenceTreeItem *ppItem =
              dynamic_cast<QImageSequenceTreeItem*>(item->parent());

          if( ppItem && ppItem->input_sequence() && !ppItem->isRunnng() ) {
            sourceItem->input_source()->set_enabled(sourceItem->checkState(0) == Qt::Checked);
            Q_EMIT imageSequenceSourcesChanged(ppItem->input_sequence());
          }
        }

        break;
      }

      default:
        break;
    }
  }

}

void QImageSequencesTreeView::keyPressEvent(QKeyEvent * e)
{
  switch (e->key()) {
    case Qt::Key_Delete:
      onDeleteSelectedItems();
      return;
  }

  Base::keyPressEvent(e);
}

void QImageSequencesTreeView::mouseMoveEvent(QMouseEvent * e)
{
  if( !(e->buttons() & Qt::LeftButton) ) {
    return Base::mouseMoveEvent(e);
  }

//  if( QImageProcessingPipeline::isRunning() ) {
//    return Base::mouseMoveEvent(e);
//  }

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

    const QImageSequenceTreeItem *sequenceItem =
        dynamic_cast<const QImageSequenceTreeItem*>(inputSourceItem->parent());

    const c_input_source::sptr &input_source =
        inputSourceItem->input_source();

    const c_image_sequence::sptr &image_sequence =
        sequenceItem->input_sequence();

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

Qt::DropActions QImageSequencesTreeView::supportedDropActions() const
{
  return Qt::CopyAction | Qt::MoveAction;
}

void QImageSequencesTreeView::dragEnterEvent(QDragEnterEvent * event)
{
  if( event->mimeData()->hasUrls() ) {
    event->acceptProposedAction();
  }
  else {
    Base::dragEnterEvent(event);
  }
}

void QImageSequencesTreeView::dragMoveEvent(QDragMoveEvent * event)
{
  if( event->mimeData()->hasUrls() ) {
    event->setDropAction(Qt::DropAction::CopyAction);
    event->accept();
    return;
  }
  event->setDropAction(Qt::DropAction::IgnoreAction);
}

void QImageSequencesTreeView::dropEvent(QDropEvent * e)
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

  c_update_controls_lock lock(this);

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
      if( (imageSequenceItem = addNewImageSequence()) ) {
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
              imageSequenceItem->input_sequence()->source(0)->filename().c_str());

          const QString name =
              fileInfo.completeBaseName();

          if( !findImageSequenceItem(name) ) {
            imageSequenceItem->input_sequence()->set_name(name.toStdString());
            imageSequenceItem->setText(0, name);
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

      if( (imageSequenceItem = addNewImageSequence()) ) {
        if( !dropSource(e, url, imageSequenceItem, selectedItem) ) {
          delete imageSequenceItem;
        }
        else {

          action = Qt::CopyAction;

          const QFileInfo fileInfo(
              imageSequenceItem->input_sequence()->source(0)->filename().c_str());

          const QString name =
              fileInfo.completeBaseName();

          if( !findImageSequenceItem(name) ) {
            imageSequenceItem->input_sequence()->set_name(name.toStdString());
            imageSequenceItem->setText(0, name);
          }

          setCurrentItem(imageSequenceItem);
        }
      }
    }
  }

  e->setDropAction(action);
  e->accept();

  setFocus();

  if( action != Qt::IgnoreAction ) {
    Q_EMIT imageSequenceCollectionChanged();
  }
}

bool QImageSequencesTreeView::dropSource(QDropEvent * e, const QUrl & url, QImageSequenceTreeItem * targetSequenceItem,
    QTreeWidgetItem * selectedItem)
{

  bool dropped = false;

  const c_input_sequence::sptr &target_sequence =
      targetSequenceItem->input_sequence();

  QInputSourceTreeItem *targetSourceItem = nullptr;

  if( selectedItem && selectedItem->type() == QInputSourceTreeItemType ) {
    targetSourceItem = dynamic_cast<QInputSourceTreeItem*>(selectedItem);
  }

  if( url.scheme() != "cinputsource" ) {

    QFileInfo fileInfo(url.toLocalFile());

    if( !fileInfo.isDir() ) {

      const std::string pathfilename =
          fileInfo.absoluteFilePath().toStdString();

      if( target_sequence->indexof(pathfilename) < 0 ) {

        const int targetIndex = (!targetSourceItem) ? -1 :
            target_sequence->indexof(targetSourceItem->input_source());

        c_input_source::sptr input_source = target_sequence->add_source(pathfilename, targetIndex);
        if( input_source ) {

          targetSequenceItem->insertChild(target_sequence->indexof(input_source),
              new QInputSourceTreeItem(input_source));

          dropped = true;
        }
      }
    }
  }

  else {

    QImageSequenceTreeItem *sourceStackItem;
    QInputSourceTreeItem *inputSourceItem;

    if( (sourceStackItem = findImageSequenceItem(url.userName())) ) {
      if( (inputSourceItem = findInputSourceItem(sourceStackItem, url.path())) ) {

        const std::string source_file_name = url.path().toStdString();
        const c_input_sequence::sptr &source_sequence = sourceStackItem->input_sequence();
        const int sourceIndex = source_sequence->indexof(source_file_name);
        const int targetIndex = (!targetSourceItem) ? -1 : target_sequence->indexof(targetSourceItem->input_source());

        if( sourceIndex >= 0 ) {

          if( sourceStackItem == targetSequenceItem ) {

            if( targetIndex != sourceIndex ) {

              sourceStackItem->removeChild(inputSourceItem);
              source_sequence->remove_source(sourceIndex);
              delete inputSourceItem;

              c_input_source::sptr input_source = target_sequence->add_source(source_file_name, targetIndex);
              if( input_source ) {

                targetSequenceItem->insertChild(target_sequence->indexof(input_source),
                    new QInputSourceTreeItem(input_source));

                dropped = true;
              }
            }
          }
          else if( target_sequence->indexof(source_file_name) < 0 ) {

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
            if( input_source ) {

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

int QImageSequencesTreeView::dropSources(QDropEvent * e, QImageSequenceTreeItem * targetSequenceItem,
    QTreeWidgetItem * targetItem)
{

  int num_sourcess_added = 0;

  const c_input_sequence::sptr &target_sequence =
      targetSequenceItem->input_sequence();

  QInputSourceTreeItem *targetSourceItem = nullptr;

  if( targetItem && targetItem->type() == QInputSourceTreeItemType ) {
    targetSourceItem = dynamic_cast<QInputSourceTreeItem*>(targetItem);
  }

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
  const Qt::KeyboardModifiers keyboardModifiers = e->modifiers();
#else
  const Qt::KeyboardModifiers keyboardModifiers = e->keyboardModifiers();
#endif

  const QList<QUrl> urls = e->mimeData()->urls();

  bool drop_confirmed = false;

  static const auto confirmThisDragDrop =
      [](QImageSequencesTreeView * _this) -> bool {

        const int reply = QMessageBox::question(_this,
            "Confirmation required",
            "Accept this Drag&Drop ?\n\n"
            "This confirmation is requested to prevent unintentional random mouse drags.",
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No);

        return reply == QMessageBox::Yes;
      };

  for( const QUrl &url : urls ) {

    if( url.scheme() != "cinputsource" ) {

      QFileInfo fileInfo(url.toLocalFile());

      if( !fileInfo.isDir() ) {

        const std::string pathfilename =
            fileInfo.absoluteFilePath().toStdString();

        if( target_sequence->indexof(pathfilename) < 0 ) {

          const int targetIndex = (!targetSourceItem) ? -1 :
              target_sequence->indexof(targetSourceItem->input_source());

          c_input_source::sptr input_source = target_sequence->add_source(pathfilename, targetIndex);
          if( input_source ) {

            targetSequenceItem->insertChild(target_sequence->indexof(input_source),
                new QInputSourceTreeItem(input_source));

            ++num_sourcess_added;
          }
        }
      }

    }

    else {

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

      QImageSequenceTreeItem *sourceSequenceItem;
      QInputSourceTreeItem *inputSourceItem;

      if( (sourceSequenceItem = findImageSequenceItem(url.userName())) ) {
        if( (inputSourceItem = findInputSourceItem(sourceSequenceItem, url.path())) ) {

          const std::string source_file_name = url.path().toStdString();
          const c_input_sequence::sptr &source_sequence = sourceSequenceItem->input_sequence();
          const int sourceIndex = source_sequence->indexof(source_file_name);
          const int targetIndex = (!targetSourceItem) ? -1 : target_sequence->indexof(targetSourceItem->input_source());

          if( sourceIndex >= 0 ) {

            if( sourceSequenceItem == targetSequenceItem ) {

              if( targetIndex != sourceIndex ) {

                if( !drop_confirmed && !(drop_confirmed = confirmThisDragDrop(this)) ) {
                  return false;
                }

                sourceSequenceItem->removeChild(inputSourceItem);
                source_sequence->remove_source(sourceIndex);
                delete inputSourceItem;

                c_input_source::sptr input_source = target_sequence->add_source(source_file_name, targetIndex);
                if( input_source ) {

                  targetSequenceItem->insertChild(target_sequence->indexof(input_source),
                      new QInputSourceTreeItem(input_source));

                  ++num_sourcess_added;
                }
              }
            }
            else if( target_sequence->indexof(source_file_name) < 0 ) {

              if( !drop_confirmed && !(drop_confirmed = confirmThisDragDrop(this)) ) {
                return false;
              }

              if( !(keyboardModifiers & Qt::ControlModifier) ) {
                sourceSequenceItem->removeChild(inputSourceItem);
                source_sequence->remove_source(sourceIndex);
                delete inputSourceItem;
              }

              c_input_source::sptr input_source = target_sequence->add_source(source_file_name, targetIndex);
              if( input_source ) {

                targetSequenceItem->insertChild(target_sequence->indexof(input_source),
                    new QInputSourceTreeItem(input_source));

                ++num_sourcess_added;
              }

            }

          }

        }
      }
    }
  }

  return num_sourcess_added;
}

QImageSequenceTreeItem* QImageSequencesTreeView::getImageSequenceItem(QTreeWidgetItem * item) const
{
  if( item ) {
    switch (item->type()) {
      case QImageSequenceTreeItemType:
        return dynamic_cast<QImageSequenceTreeItem*>(item);
      case QInputSourceTreeItemType:
        return dynamic_cast<QImageSequenceTreeItem*>(item->parent());
    }
  }
  return nullptr;
}

QImageSequenceTreeItem* QImageSequencesTreeView::findImageSequenceItem(const QString & name) const
{
  if( !name.isEmpty() ) {

    const std::string cname = name.toStdString();

    for( int i = 0, n = topLevelItemCount(); i < n; ++i ) {
      QImageSequenceTreeItem *item = dynamic_cast<QImageSequenceTreeItem*>(topLevelItem(i));
      if( item && item->input_sequence()->name() == cname ) {
        return item;
      }
    }
  }

  return nullptr;
}

QImageSequenceTreeItem* QImageSequencesTreeView::findImageSequenceItem(
    const c_input_sequence::sptr & image_sequence) const
{
  if( image_sequence ) {
    for( int i = 0, n = topLevelItemCount(); i < n; ++i ) {
      QImageSequenceTreeItem *item = dynamic_cast<QImageSequenceTreeItem*>(topLevelItem(i));
      if( item && image_sequence == item->input_sequence() ) {
        return item;
      }
    }
  }
  return nullptr;
}

QInputSourceTreeItem* QImageSequencesTreeView::findInputSourceItem(QImageSequenceTreeItem * sequenceItem,
    const QString & filename) const
{
  if( sequenceItem && !filename.isEmpty() ) {

    const std::string cfilename = filename.toStdString();

    for( int i = 0, n = sequenceItem->childCount(); i < n; ++i ) {
      QInputSourceTreeItem *item = dynamic_cast<QInputSourceTreeItem*>(sequenceItem->child(i));
      if( item && item->input_source()->filename() == cfilename ) {
        return item;
      }
    }
  }

  return nullptr;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QImageSequencesTree::QImageSequencesTree(QWidget * parent) :
    Base(parent)
{
  Q_INIT_RESOURCE(qstacktree_resources);

  //
  toolbarActions_.append(addImageSequenceAction =
      new QAction(getIcon(ICON_add_pipeline),
          "Add stack"));
  addImageSequenceAction->setToolTip("Add new image stack...");

  //
  toolbarActions_.append(addSourcesAction =
      new QAction(getIcon(ICON_add_frames),
          "Add sources"));
  addSourcesAction->setToolTip("Add sources to selected stack...");
  addSourcesAction->setEnabled(false);

  //
  toolbarActions_.append(deleteItemAction =
      new QAction(getIcon(ICON_delete_item),
          "Delete selected items"));
  deleteItemAction->setToolTip("Delete selected items...");
  deleteItemAction->setEnabled(false);

  //
  toolbarActions_.append(showStackOptionsAction =
      new QAction(getIcon(ICON_options),
          "Stack options"));
  showStackOptionsAction->setToolTip("Show pipeline options");
  showStackOptionsAction->setEnabled(false);

  //
  toolbarActions_.append(startMenuAction = new QAction("Start"));
  startMenuAction->setMenu(startMenu = new QMenu());
  startMenu->addAction(startAction = new QAction(getIcon(ICON_start), "Start"));
  startMenu->addAction(startAllAction = new QAction(getIcon(ICON_start_all), "Start all"));
  startMenu->setDefaultAction(startAction);
  startMenu->setIcon(getIcon(ICON_start));
  //startStackingMenu->setEnabled(false);
  startMenuAction->setEnabled(false);

  //
  toolbarActions_.append(stopAction =
      new QAction(getIcon(ICON_stop),
          "Stop"));
  stopAction->setToolTip("Cancel current pipeline");
  stopAction->setEnabled(false);

//  toolbarActions_.append(startStopStackingAction =
//      new QAction(getIcon(ICON_start),
//          "Start / Stop stacking"));
//  startStopStackingAction->setToolTip(
//      "Start / Stop staking this sequence");
//  startStopStackingAction->setEnabled(false);

  /* Setup Treeview */
  treeView_ = new QImageSequencesTreeView(this);
  treeView_->installEventFilter(this);
  connect(treeView_, &QImageSequencesTreeView::imageSequenceCollectionChanged,
      this, &ThisClass::imageSequenceCollectionChanged);

  /* Setup layout */
  vbox_ = new QVBoxLayout(this);
//  vbox_->addWidget(toolbar_);
  vbox_->addWidget(treeView_);

  /* Setup event handlers */

  connect(addImageSequenceAction, &QAction::triggered,
      treeView_, &QImageSequencesTreeView::onAddNewImageSequence);

  connect(addSourcesAction, &QAction::triggered,
      treeView_, &QImageSequencesTreeView::onAddSourcesToCurrentImageSequence);

  connect(deleteItemAction, &QAction::triggered,
      treeView_, &QImageSequencesTreeView::onDeleteSelectedItems);

//  connect(startStopStackingAction, &QAction::triggered,
//      treeView_, &QStackListTreeView::onStartStopStackingActionClicked);

  connect(stopAction, &QAction::triggered,
      this, &ThisClass::onStopPipelineClicked);

  connect(startAction, &QAction::triggered,
      this, &ThisClass::onStartPipelineClicked);

  connect(startAllAction, &QAction::triggered,
      this, &ThisClass::onStartAllPipelinesClicked);

  connect(showStackOptionsAction, &QAction::triggered,
      this, &ThisClass::onShowPipelineOptionsClicked);

  connect(treeView_, &QTreeWidget::customContextMenuRequested,
      this, &ThisClass::onCustomContextMenuRequested);

//  connect(treeView_, &QTreeWidget::itemChanged,
//      this, &ThisClass::onItemChanged);

  connect(treeView_, &QImageSequencesTreeView::imageSequenceNameChanged,
      this, &ThisClass::imageSequenceNameChanged);

  connect(treeView_, &QImageSequencesTreeView::imageSequenceSourcesChanged,
      this, &ThisClass::imageSequenceSourcesChanged);

  connect(treeView_, &QTreeWidget::currentItemChanged,
      this, &ThisClass::onCurrentItemChanged);

  connect(treeView_, &QTreeWidget::itemDoubleClicked,
      this, &ThisClass::onItemDoubleClicked);

//  connect(treeView_, &QTreeWidget::itemPressed,
//      this, &ThisClass::onItemDoubleClicked);

  connect(QPipelineThread::instance(), &QPipelineThread::starting,
      this, &ThisClass::onPipelineThreadStarting);

  connect(QPipelineThread::instance(), &QPipelineThread::started,
      this, &ThisClass::onPipelineThreadStarted);

  connect(QPipelineThread::instance(), &QPipelineThread::finishing,
      this, &ThisClass::onPipelineThreadFinishing);

  connect(QPipelineThread::instance(), &QPipelineThread::finished,
      this, &ThisClass::onPipelineThreadFinished);

}

void QImageSequencesTree::loadSequences(const std::string & cfgfilename)
{
  treeView_->loadSequences(cfgfilename);
}

void QImageSequencesTree::saveSequences(const std::string & cfgfilename)
{
  treeView_->saveSequences(cfgfilename);
}

const QList<QAction*>& QImageSequencesTree::toolbarActions() const
{
  return toolbarActions_;
}

c_input_source::sptr QImageSequencesTree::getCurrentInputSource(c_image_sequence::sptr * parent_sequence) const
{
  QTreeWidgetItem *currentItem =
      treeView_->currentItem();

  if( currentItem ) {

    switch (currentItem->type()) {

      case QInputSourceTreeItemType: {

        c_input_source::sptr inputSource;

        QInputSourceTreeItem *inputSourceItem =
            dynamic_cast<QInputSourceTreeItem*>(currentItem);

        if( inputSourceItem ) {

          inputSource = inputSourceItem->input_source();

          if( parent_sequence ) {

            QImageSequenceTreeItem *sequenceItem =
                dynamic_cast<QImageSequenceTreeItem*>(inputSourceItem->parent());

            if( !sequenceItem ) {
              parent_sequence->reset();
            }
            else {
              *parent_sequence = sequenceItem->input_sequence();
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
            *parent_sequence = sequenceItem->input_sequence();
          }
        }

        return nullptr;
    }
  }

  if( parent_sequence ) {
    parent_sequence->reset();
  }

  return nullptr;
}

void QImageSequencesTree::getSelectedSequences(std::vector<c_image_sequence::sptr> * sequences) const
{
  if( sequences ) {

    for( int i = 0, n = treeView_->topLevelItemCount(); i < n; ++i ) {

      QImageSequenceTreeItem *item =
          dynamic_cast<QImageSequenceTreeItem*>(treeView_->topLevelItem(i));

      if( item && item->isSelected() ) {
        sequences->emplace_back(item->input_sequence());
      }
    }
  }
}

void QImageSequencesTree::updateImageSequenceName(const c_image_sequence::sptr & image_sequece)
{
  treeView_->updateImageSequenceName(image_sequece);
}

void QImageSequencesTree::addNewImageSequence()
{
  treeView_->onAddNewImageSequence();
}

void QImageSequencesTree::addSourcesToCurrentImageSequence()
{
  treeView_->onAddSourcesToCurrentImageSequence();
}

void QImageSequencesTree::deleteSelectedItems()
{
  treeView_->onDeleteSelectedItems();
}

bool QImageSequencesTree::eventFilter(QObject * watched, QEvent * event)
{
  if( watched == treeView_ && event->type() == QEvent::KeyPress ) {

    const QKeyEvent *e = (const QKeyEvent*) event;
    if( e->key() == Qt::Key_Return && treeView_->state() != QAbstractItemView::EditingState ) {

      QTreeWidgetItem *item = treeView_->currentItem();
      if( item ) {

        QImageSequenceTreeItem *sequenceItem = nullptr;
        QInputSourceTreeItem *inputSourceItem = nullptr;

        switch (item->type()) {

          case QImageSequenceTreeItemType:
            sequenceItem = dynamic_cast<QImageSequenceTreeItem*>(item);
            break;

          case QInputSourceTreeItemType:
            inputSourceItem = dynamic_cast<QInputSourceTreeItem*>(item);
            sequenceItem = dynamic_cast<QImageSequenceTreeItem*>(item->parent());
            break;
        }

        Q_EMIT itemDoubleClicked(sequenceItem ? sequenceItem->input_sequence() : nullptr,
            inputSourceItem ? inputSourceItem->input_source() : nullptr);

        return true;
      }

    }
  }

  return false;
}

void QImageSequencesTree::onCurrentItemChanged(QTreeWidgetItem * current, QTreeWidgetItem * previous)
{

  if( !current ) {
    deleteItemAction->setEnabled(false);
    startMenuAction->setEnabled(false);
    stopAction->setEnabled(false);
    showStackOptionsAction->setEnabled(false);
  }
  else {

    QImageSequenceTreeItem *sequenceItem = nullptr;
    QInputSourceTreeItem *inputSourceItem = nullptr;

    deleteItemAction->setEnabled(true);
    showStackOptionsAction->setEnabled(true);

    switch (current->type()) {

      case QImageSequenceTreeItemType:
        sequenceItem = dynamic_cast<QImageSequenceTreeItem*>(current);
        addSourcesAction->setEnabled(true);
        startMenuAction->setEnabled(!QPipelineThread::isRunning());
        break;

      case QInputSourceTreeItemType:
        inputSourceItem = dynamic_cast<QInputSourceTreeItem*>(current);
        sequenceItem = dynamic_cast<QImageSequenceTreeItem*>(current->parent());
        startMenuAction->setEnabled(!QPipelineThread::isRunning());
        break;

      default:
        addSourcesAction->setEnabled(false);
        startMenuAction->setEnabled(!QPipelineThread::isRunning());
        break;
    }

    Q_EMIT currentItemChanged(sequenceItem ? sequenceItem->input_sequence() : nullptr,
        inputSourceItem ? inputSourceItem->input_source() : nullptr);
  }
}

void QImageSequencesTree::onItemDoubleClicked(QTreeWidgetItem * item, int /*column*/)
{
  QImageSequenceTreeItem *sequenceItem = nullptr;
  QInputSourceTreeItem *inputSourceItem = nullptr;

  switch (item->type()) {

    case QImageSequenceTreeItemType:
      sequenceItem = dynamic_cast<QImageSequenceTreeItem*>(item);
      break;

    case QInputSourceTreeItemType:
      inputSourceItem = dynamic_cast<QInputSourceTreeItem*>(item);
      sequenceItem = dynamic_cast<QImageSequenceTreeItem*>(item->parent());
      break;
  }

  Q_EMIT itemDoubleClicked(sequenceItem ? sequenceItem->input_sequence() : nullptr,
      inputSourceItem ? inputSourceItem->input_source() : nullptr);
}

void QImageSequencesTree::onCustomContextMenuRequested(const QPoint & pos)
{
  QMenu menu;
  QList<QTreeWidgetItem*> contextItems;
  QAction *action;

  if( true ) {

    QModelIndexList selectedIndexes =
        treeView_->selectionModel()->selectedRows();

    if( !selectedIndexes.empty() ) {
      for( int i = 0, n = selectedIndexes.size(); i < n; ++i ) {
        if( selectedIndexes[i].isValid() ) {
          contextItems.append(treeView_->itemFromIndex(selectedIndexes[i]));
        }
      }
    }
    else {
      QModelIndex index = treeView_->indexAt(pos);
      if( index.isValid() ) {
        contextItems.append(treeView_->itemFromIndex(index));
      }
    }
  }

  if( contextItems.size() == 1 ) {

    QTreeWidgetItem *item =
        contextItems[0];

    switch (item->type()) {
      case QImageSequenceTreeItemType: {

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

      case QInputSourceTreeItemType: {

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

  if( contextItems.size() > 0 ) {
    menu.addAction(deleteItemAction);
  }

  if( !menu.isEmpty() ) {
    menu.exec(treeView_->mapToGlobal(pos));
  }

}

void QImageSequencesTree::onShowPipelineOptionsClicked()
{
  QImageSequenceTreeItem *sequenceItem =
      treeView_->getImageSequenceItem(treeView_->currentItem());

  if( sequenceItem ) {
    Q_EMIT showImageSequenceOptionsClicked(sequenceItem->input_sequence());
  }
}

void QImageSequencesTree::onStartPipelineClicked()
{
  if( !QPipelineThread::isRunning() ) {

    QImageSequenceTreeItem *sequenceItem =
        treeView_->getImageSequenceItem(treeView_->currentItem());

    if( sequenceItem && sequenceItem->input_sequence() ) {

      const c_image_processing_pipeline::sptr & current_pipeline =
          sequenceItem->input_sequence()->current_pipeline();

      if ( current_pipeline ) {
        currentProcessingMode_ = ProcessSingleStack;
        QPipelineThread::start(current_pipeline);
      }
    }
  }
}

void QImageSequencesTree::onStartAllPipelinesClicked()
{
  if( !QPipelineThread::isRunning() ) {
    currentProcessingMode_ = ProcessBatch;
    if( !startNextStacking() ) {
      currentProcessingMode_ = ProcessIdle;
    }
  }
}

void QImageSequencesTree::onStopPipelineClicked()
{
  if( QPipelineThread::isRunning() ) {
    currentProcessingMode_ = ProcessIdle;
    QPipelineThread::cancel();
  }

}

bool QImageSequencesTree::startNextStacking()
{
  if( !QPipelineThread::isRunning() && currentProcessingMode_ == ProcessBatch ) {

    for( int i = 0, n = treeView_->topLevelItemCount(); i < n; ++i ) {

      QImageSequenceTreeItem *item =
          dynamic_cast<QImageSequenceTreeItem*>(
          treeView_->topLevelItem(i));

      if( item && item->checkState(0) == Qt::Checked && item->input_sequence() ) {
        if( item->input_sequence()->current_pipeline() ) {
          return QPipelineThread::start(item->input_sequence()->current_pipeline());
        }
      }
    }
  }

  return false;
}

void QImageSequencesTree::onPipelineThreadStarting()
{

}

void QImageSequencesTree::onPipelineThreadStarted()
{
  startMenu->setEnabled(false);
  stopAction->setEnabled(true);

  c_image_processing_pipeline::sptr currentPipeline =
      QPipelineThread::currentPipeline();

  if( currentPipeline ) {

    QImageSequenceTreeItem *sequenceItem =
        treeView_->findImageSequenceItem(currentPipeline->input_sequence());

    if( sequenceItem ) {

      c_update_controls_lock lock(treeView_);

      for( int j = 0, m = sequenceItem->childCount(); j < m; ++j ) {

        QInputSourceTreeItem *sourceItem =
            dynamic_cast<QInputSourceTreeItem*>(sequenceItem->child(j));

        if( sourceItem ) {
          sourceItem->setFlags(sourceItem->flags() & ~Qt::ItemIsEnabled);
        }
      }
    }
  }
}

void QImageSequencesTree::onPipelineThreadFinishing()
{
  stopAction->setEnabled(false);
}

void QImageSequencesTree::onPipelineThreadFinished()
{

  c_image_processing_pipeline::sptr currentPipeline =
      QPipelineThread::currentPipeline();

  if( currentPipeline ) {

    QImageSequenceTreeItem *sequenceItem =
        treeView_->findImageSequenceItem(currentPipeline->input_sequence());

    if( sequenceItem ) {

      c_update_controls_lock lock(treeView_);

      for( int j = 0, m = sequenceItem->childCount(); j < m; ++j ) {

        QInputSourceTreeItem *sourceItem =
            dynamic_cast<QInputSourceTreeItem*>(sequenceItem->child(j));

        if( sourceItem ) {
          sourceItem->setFlags(sourceItem->flags() | Qt::ItemIsEnabled);
        }
      }

      if( currentProcessingMode_ == ProcessBatch ) {
        sequenceItem->setCheckState(0, Qt::Unchecked);
      }
    }

    if( currentProcessingMode_ != ProcessBatch || !startNextStacking() ) {
      currentProcessingMode_ = ProcessIdle;
      startMenu->setEnabled(true);
      stopAction->setEnabled(true);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QImageSequenceTreeDock::QImageSequenceTreeDock(const QString & title, QWidget * parent) :
    Base(title, parent)
{
  Base::setWidget(treeView_ = new QImageSequencesTree(this));

  const QList<QAction*> actions = treeView_->toolbarActions();
  for( int i = 0, n = actions.size(); i < n; ++i ) {
    titleBar()->addButton(actions[n - i - 1]);
  }

}

QImageSequencesTree* QImageSequenceTreeDock::treeView() const
{
  return treeView_;
}

QImageSequenceTreeDock* addImageSequenceTreeDock(QMainWindow * parent,
    Qt::DockWidgetArea area,
    const QString & dockName,
    const QString & title,
    QMenu * viewMenu)
{
  QImageSequenceTreeDock *dock = new QImageSequenceTreeDock(title, parent);
  dock->setObjectName(dockName);
  parent->addDockWidget(area, dock);

  if( viewMenu ) {
    viewMenu->addAction(dock->toggleViewAction());
  }

  return dock;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

