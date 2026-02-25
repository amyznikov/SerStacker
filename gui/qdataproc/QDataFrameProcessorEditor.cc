/*
 * QDataFrameProcessorEditor.cc
 *
 *  Created on: Dec 16, 2023
 *      Author: amyznikov
 */

#include "QDataFrameProcessorEditor.h"
#include "QAddDataProcRoutineDialogBox.h"
#include <gui/widgets/QWaitCursor.h>
#include <gui/widgets/createAction.h>
#include <gui/widgets/style.h>
#include <core/ssprintf.h>
#include <core/debug.h>

#define ICON_double_arrow_down    ":/qimproc/icons/double-arrow-down"
#define ICON_double_arrow_right   ":/qimproc/icons/double-arrow-right"
#define ICON_move_down            ":/qimproc/icons/move-down2"
#define ICON_move_up              ":/qimproc/icons/move-up2"
#define ICON_delete               ":/qimproc/icons/delete"
#define ICON_add                  ":/qimproc/icons/add"
#define ICON_menu                 ":/qimproc/icons/menu"
#define ICON_rename               ":/qimproc/icons/rename"
#define ICON_copy                 ":/qimproc/icons/copy"
#define ICON_config               ":/qimproc/icons/config"


namespace  {

class QRoutineItem:
    public QTreeWidgetItem
{
public:
  typedef QRoutineItem ThisClass;
  typedef QTreeWidgetItem Base;

  QRoutineItem(const c_data_frame_processor_routine::sptr & routine) :
      routine_(routine)
  {
    setText(0, QString::fromStdString(routine->display_name()));
    setToolTip(0, routine->tooltip().c_str());
    setCheckState(0, routine->enabled() ? Qt::Checked : Qt::Unchecked);
  }

  const c_data_frame_processor_routine::sptr& routine() const
  {
    return routine_;
  }

protected:
  c_data_frame_processor_routine::sptr routine_;
};

class QOptionsItem:
    public QTreeWidgetItem
{
public:
  typedef QOptionsItem ThisClass;
  typedef QTreeWidgetItem Base;

  QOptionsItem(const c_data_frame_processor_routine::sptr & routine) :
      routine_(routine)
  {
  }

  const c_data_frame_processor_routine::sptr& routine() const
  {
    return routine_;
  }

protected:
  c_data_frame_processor_routine::sptr routine_;
};

QRoutineItem* currentRoutineItem(QTreeWidget * tree_ctl)
{
  QRoutineItem * processorItem = nullptr;

  QTreeWidgetItem * currentItem =
      tree_ctl->currentItem();

  if( currentItem ) {

    processorItem =
        dynamic_cast<QRoutineItem*>(currentItem);

    if( !processorItem ) {
      processorItem =
          dynamic_cast<QRoutineItem*>(
          currentItem->parent());
    }
  }

  return processorItem;
}

}  // namespace

QDataFrameProcessorEditor::QDataFrameProcessorEditor(QWidget * parent) :
  Base(parent)
{
  setContentsMargins(0, 0, 0, 0);

  lv_ = new QVBoxLayout(this);
  lv_->setContentsMargins(0, 0, 0, 0);

  addAction(moveDownAction_ =
      createAction(getIcon(ICON_move_down),
          "Move Down",
          "Move selected processor down",
          this,
          &ThisClass::onMoveCurrentProcessorDown));

  addAction(moveUpAction_ =
      createAction(getIcon(ICON_move_up),
          "Move Up",
          "Move selected processor up",
          this,
          &ThisClass::onMoveCurrentProcessorUp));

  addAction(addProcAction_ =
      createAction(getIcon(ICON_add),
          "Add processor ...",
          "Add image processor",
          this,
          &ThisClass::onAddImageProcessor));

  addAction(removeProcAction_ =
      createAction(getIcon(ICON_delete),
          "Remove selected processor",
          "Remove selected image processor",
          this,
          &ThisClass::onRemoveCurrentProcessor));

  ///////////////////////////////////////////////////////////////////
  lv_->addWidget(tree_ctl = new QTreeWidget(this));
  tree_ctl->setHeaderHidden(true);
  tree_ctl->setColumnCount(1);
  tree_ctl->setSelectionBehavior(QAbstractItemView::SelectionBehavior::SelectRows);
  tree_ctl->setSelectionMode(QAbstractItemView::SelectionMode::SingleSelection);
  tree_ctl->setContextMenuPolicy(Qt::CustomContextMenu);

  connect(tree_ctl, &QTreeWidget::itemChanged,
      this, &ThisClass::onTreeItemChanged);

  connect(tree_ctl, &QTreeWidget::currentItemChanged,
      this, &ThisClass::onCurrentTreeItemChanged);

  connect(tree_ctl, &QTreeWidget::itemExpanded,
      this, &ThisClass::updateItemSizeHint);

  connect(tree_ctl, &QTreeWidget::customContextMenuRequested,
      [this](
          const QPoint & pos) {
            QRoutineItem *item = dynamic_cast<QRoutineItem*>(tree_ctl->itemAt(pos));
            if ( item ) {

              QMenu menu;

              menu.addAction("Change Display Name...",
                  [this, item]() {

                    const QString oldDisplayName =
                        QString::fromStdString(item->routine()->display_name());

                    const QString newDisplayName =
                      QInputDialog::getText(this, "Enter new display name", "name:",
                          QLineEdit::EchoMode::Normal,
                          oldDisplayName);

                    if ( !newDisplayName.isEmpty() && newDisplayName != oldDisplayName ) {

                      item->routine()->set_display_name(newDisplayName.toStdString());
                      item->setText(0, QString::fromStdString(item->routine()->display_name()));

                      Q_EMIT parameterChanged();
                    }
                  });

              menu.exec(tree_ctl->mapToGlobal(QPoint(pos.x()-4, pos.y()-4)));
            }
          });


  ///////////////////////////////////////////////////////////////////
  updateControls();
}

void QDataFrameProcessorEditor::updateItemSizeHint(QTreeWidgetItem * item)
{
  if ( item ) {

    QTreeWidgetItem * subitem =
        item->child(0);

    if ( subitem ) {

      QScrollArea * sa =
          dynamic_cast<QScrollArea * >(tree_ctl->itemWidget(subitem, 0));

      if ( sa ) {

        QWidget * w =
            sa->widget();

        if ( w ) {

          const QSize s =
              w->sizeHint();

          subitem->setSizeHint(0, QSize(s.width(), std::min(s.height(),
              8 * tree_ctl->viewport()->height() / 10)));
        }

      }

    }
  }
}

void QDataFrameProcessorEditor::setCurrentProcessor(const c_data_frame_processor::sptr & p)
{
  currentProcessor_ = p;
  updateControls();
}

const c_data_frame_processor::sptr & QDataFrameProcessorEditor::currentProcessor() const
{
  return currentProcessor_;
}

void QDataFrameProcessorEditor::onupdatecontrols()
{
  if ( !currentProcessor_ ) {
    setEnabled(false);
    tree_ctl->clear();
  }
  else {

    tree_ctl->clear();

    for( const c_data_frame_processor_routine::sptr &routine : currentProcessor_->routines() ) {
      if( routine ) {
        insertRoutine(tree_ctl->topLevelItemCount(), routine);
      }
    }

    setEnabled(true);
  }
}

QTreeWidgetItem * QDataFrameProcessorEditor::insertRoutine(int index, const c_data_frame_processor_routine::sptr & routine)
{
  QRoutineItem * item =
      new QRoutineItem(routine);

  tree_ctl->insertTopLevelItem(index, item);

  QDataFrameRoutineOptionsControl * ctrl =
      QDataFrameRoutineOptionsControl::create(routine,
          this);


  if ( ctrl ) {

    //ctrl->setAutoFillBackground(true);

    QOptionsItem * subitem =
        new QOptionsItem(routine);

    item->addChild(subitem);

    tree_ctl->setItemWidget(subitem, 0, createScrollableWrap(ctrl));

    connect(ctrl, &QDataFrameRoutineOptionsControl::parameterChanged,
        this, &ThisClass::parameterChanged,
        Qt::QueuedConnection);

  }


  return item;
}

void QDataFrameProcessorEditor::onTreeItemChanged(QTreeWidgetItem * item, int column)
{
  if( item && column == 0 ) {

    QRoutineItem *improcItem =
        dynamic_cast<QRoutineItem*>(item);

    if( improcItem || (improcItem = dynamic_cast<QRoutineItem*>(item->parent())) ) {

      const bool checked =
          improcItem->checkState(0) == Qt::Checked;

      if( improcItem->routine()->enabled() != checked ) {

        improcItem->routine()->set_enabled(checked);

        Q_EMIT parameterChanged();
      }
    }
  }
}

void QDataFrameProcessorEditor::onCurrentTreeItemChanged(QTreeWidgetItem * current, QTreeWidgetItem * previous)
{
  if( !current ) {
    removeProcAction_->setEnabled(false);
    moveDownAction_->setEnabled(false);
    moveUpAction_->setEnabled(false);
  }
  else {
    removeProcAction_->setEnabled(true);
    moveDownAction_->setEnabled(true);
    moveUpAction_->setEnabled(true);

    current->setSelected(true);
  }
}

void QDataFrameProcessorEditor::onMoveCurrentProcessorDown()
{
  if( currentProcessor_ ) {

    QRoutineItem *currentItem =
        currentRoutineItem(tree_ctl);

    if( currentItem ) {

      const int index =
          tree_ctl->indexOfTopLevelItem(currentItem);

      if( index < tree_ctl->topLevelItemCount() - 1 ) {

        QWaitCursor wait(this);

        c_data_frame_processor::iterator pos =
            currentProcessor_->find(currentItem->routine());

        if( pos + 1 < currentProcessor_->end() ) {

          const c_data_frame_processor_routine::sptr routine = *pos;

          if( true ) {
            c_data_frame_processor::edit_lock lock(currentProcessor_);
            currentProcessor_->erase(pos);
            currentProcessor_->insert(pos + 1, routine);
          }

          const bool isExpanded =
              currentItem->isExpanded();

          delete currentItem;

          QTreeWidgetItem * newItem =
              insertRoutine(index + 1, routine);

          tree_ctl->setCurrentItem(newItem);
          if ( isExpanded ) {
            tree_ctl->expandItem(newItem);
          }

          Q_EMIT parameterChanged();
        }
      }
    }
  }
}

void QDataFrameProcessorEditor::onMoveCurrentProcessorUp()
{
  if( currentProcessor_ ) {

    QRoutineItem *currentItem =
        currentRoutineItem(tree_ctl);

    if( currentItem ) {

      const int index =
          tree_ctl->indexOfTopLevelItem(currentItem);


      if( index > 0 ) {

        QWaitCursor wait(this);

        c_data_frame_processor::iterator pos =
            currentProcessor_->find(currentItem->routine());

        if ( pos != currentProcessor_->begin() && pos != currentProcessor_->end() ) {

          const c_data_frame_processor_routine::sptr routine = *pos;

          if( true ) {
            c_data_frame_processor::edit_lock lock(currentProcessor_);
            currentProcessor_->erase(pos);
            currentProcessor_->insert(pos - 1, routine);
          }

          const bool isExpanded =
              currentItem->isExpanded();

          delete currentItem;

          QTreeWidgetItem * newItem =
              insertRoutine(index - 1, routine);

          tree_ctl->setCurrentItem(newItem);
          if ( isExpanded ) {
            tree_ctl->expandItem(newItem);
          }

          Q_EMIT parameterChanged();
        }
      }
    }
  }
}

void QDataFrameProcessorEditor::onAddImageProcessor()
{
  static QAddDataProcRoutineDialogBox *dlgbox = nullptr;

  c_data_frame_processor_routine::sptr current_routine, new_routine;

  if( !currentProcessor_ ) {
    return;
  }

  if( !dlgbox ) {
    dlgbox = new QAddDataProcRoutineDialogBox(this);
  }
  else {
    dlgbox->setParent(this);
  }

  if( dlgbox->exec() != QDialog::Accepted || !dlgbox->selectedClassFactory() ) {
    return;
  }

  QRoutineItem * insertAfter =
      currentRoutineItem(tree_ctl);

  if( insertAfter && !(current_routine = insertAfter->routine()) ) {
    QMessageBox::critical(this, "APP BUG", QString("insertAfter->routine() is NULL.\n"
        "Fix this code please"));
    return;
  }

  new_routine = c_data_frame_processor_routine::create(dlgbox->selectedClassFactory()->class_name);
  if( !new_routine ) {
    QMessageBox::critical(this, "ERROR", QString("c_image_processor_routine::create(%1) fails").
        arg(QString(dlgbox->selectedClassFactory()->class_name.c_str())));
    return;
  }

  QWaitCursor wait(this);

  if( true ) {
    c_data_frame_processor::edit_lock lock(currentProcessor_);

    if( !current_routine ) {
      currentProcessor_->insert(currentProcessor_->begin(),
          new_routine);
    }
    else {
      currentProcessor_->insert(currentProcessor_->find(current_routine) + 1,
          new_routine);
    }
  }

  QTreeWidgetItem * newItem =
      insertRoutine(tree_ctl->indexOfTopLevelItem(insertAfter) + 1,
          new_routine);

  tree_ctl->setCurrentItem(newItem);

  tree_ctl->expandItem(newItem);

  Q_EMIT parameterChanged();
}

void QDataFrameProcessorEditor::onRemoveCurrentProcessor()
{
  if ( currentProcessor_ ) {

    QRoutineItem * currentItem =
        currentRoutineItem(tree_ctl);

    if( currentItem ) {

      QWaitCursor wait(this);

      c_data_frame_processor::iterator pos =
          currentProcessor_->find(currentItem->routine());

      if( pos != currentProcessor_->end() ) {

        if( true ) {
          c_data_frame_processor::edit_lock lock(currentProcessor_);
          currentProcessor_->erase(pos);
        }

        delete currentItem;

        Q_EMIT parameterChanged();
      }
    }
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
QDataFrameRoutineOptionsControl * QDataFrameRoutineOptionsControl::create(const c_data_frame_processor_routine::sptr & processor,
    QWidget * parent)
{
  QDataFrameRoutineOptionsControl * widget = new QDataFrameRoutineOptionsControl(processor, parent);
  widget->setupControls();
  return widget;
}

QDataFrameRoutineOptionsControl::QDataFrameRoutineOptionsControl(const c_data_frame_processor_routine::sptr & processor, QWidget * parent) :
    Base(parent),
    _routine(processor)
{
  setFrameShape(QFrame::Shape::Box);
}

void QDataFrameRoutineOptionsControl::setupControls()
{
  if( !_routine->classfactory()->tooltip.empty() ) {
    QLabel *label = new QLabel(this);
    label->setTextFormat(Qt::RichText);
    label->setWordWrap(true);
    label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    label->setText(_routine->classfactory()->tooltip.c_str());
    form->addRow(label);
  }

  c_ctlist<c_data_frame_processor_routine> controls;
  _routine->getcontrols(controls);
  ::setupControls(this, controls);

  //  QObject::connect(this, &ThisClass::parameterChanged,
  //      [this]() {
  //        if ( _routine ) {
  //          _routine->parameter_changed();
  //        }
  //      });

  setOpts(_routine.get());
  updateControls();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QDataFrameProcessorSelector::QDataFrameProcessorSelector(QWidget * parent) :
    Base(parent)
{
  Q_INIT_RESOURCE(qimproc_resources);

  setFrameShape(QFrame::Shape::NoFrame);

  if( QDataFrameProcessorsCollection::empty() ) {
    QDataFrameProcessorsCollection::load();
  }

  lv_ = new QVBoxLayout(this);

  lv_->addWidget(toolbar_ctl = new QToolBar(this));
  toolbar_ctl->setToolButtonStyle(Qt::ToolButtonStyle::ToolButtonIconOnly);
  toolbar_ctl->setIconSize(QSize(16, 16));

  toolbar_ctl->addWidget(enabled_ctl = new QCheckBox(this));
  enabled_ctl->setToolTip("Enable / Disable image processing");

  toolbar_ctl->addWidget(selector_ctl = new QComboBox(this));
  selector_ctl->setEditable(false);
  selector_ctl->setMinimumContentsLength(12);
  selector_ctl->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

  toolbar_ctl->addWidget(selectorMenu_ctl = new QToolButton(this));
  selectorMenu_ctl->setIcon(getIcon(ICON_menu));
  selectorMenu_ctl->setToolTip("Popup menu");


  //lv_->addWidget(createScrollableWrap(processor_ctl = new QDataFrameProcessorEditor(this), this));
  lv_->addWidget(processor_ctl = new QDataFrameProcessorEditor(this));


  connect(enabled_ctl, &QCheckBox::stateChanged,
      this, &ThisClass::parameterChanged);


  connect(selectorMenu_ctl, &QToolButton::clicked,
      [this] () {

        QMenu menu;
        QAction * add_processor_action = nullptr;
        QAction * rename_processor_action = nullptr;
        QAction * copy_processor_action = nullptr;
        QAction * delete_processor_action = nullptr;

        menu.addAction(add_processor_action =
            new QAction(getIcon(ICON_add),
                "Add chain ..."));

        if ( currentProcessor_ ) {

          // Depp copy is not implemented yet
          //  menu.addAction(copy_processor_action =
          //    new QAction(getIcon(ICON_copy),
          //      "Add copy ..."));

          menu.addAction(rename_processor_action =
              new QAction(getIcon(ICON_rename),
                  "Rename chain ..."));

          menu.addAction(delete_processor_action =
              new QAction(getIcon(ICON_delete),
                  "Delete chain ..."));
        }


        QAction * selectedAction =
            menu.exec(selectorMenu_ctl->
                mapToGlobal(QPoint(selectorMenu_ctl->width()-4,
                    selectorMenu_ctl->height()-4)));

        if ( selectedAction ) {
          if ( selectedAction == add_processor_action ) {
            addProcessor();
          }
          else if ( selectedAction == copy_processor_action ) {
            addCopyOfCurrentProcessor();
          }
          else if ( selectedAction == rename_processor_action ) {
            renameCurrentProcessor();
          }
          else if ( selectedAction == delete_processor_action ) {
            deleteCurrentProcessor();
          }
        }

      });


  connect(selector_ctl, SIGNAL(currentIndexChanged(int)),
      this, SLOT(onProcessorSelectorCurrentIndexChanged(int)) );

  connect(processor_ctl, &QDataFrameProcessorEditor::parameterChanged,
      [this]() {

        if ( enabled_ctl->isChecked() ) {
          Q_EMIT parameterChanged();
        }

        const c_data_frame_processor::sptr & processor =
            processor_ctl->currentProcessor();
        if ( processor ) {
          processor->save();
        }

      });

  const QList<QAction*> chain_ctl_actions =
      processor_ctl->actions();

  for( QAction *action : chain_ctl_actions ) {
    toolbar_ctl->addAction(action);
  }

  updateControls();
}

c_data_frame_processor::sptr QDataFrameProcessorSelector::currentProcessor() const
{
  return enabled_ctl->isChecked() ? currentProcessor_ : nullptr;
}

QString QDataFrameProcessorSelector::selected_processor() const
{
  return selector_ctl->currentText();
}

void QDataFrameProcessorSelector::set_selected_processor(const QString & name)
{
  const int index =
      selector_ctl->findText(name);

  if ( index >= 0 ) {
    selector_ctl->setCurrentIndex(index);
  }
}

bool QDataFrameProcessorSelector::imageProcessingEnabled() const
{
  return enabled_ctl->isChecked();
}

void QDataFrameProcessorSelector::onupdatecontrols()
{
  selector_ctl->clear();

  for ( size_t i = 0, n = QDataFrameProcessorsCollection::size(); i < n; ++i ) {
    const c_data_frame_processor::sptr & processor = QDataFrameProcessorsCollection::item(i);
    if ( processor ) {
      selector_ctl->addItem(processor->cname(), QVariant((int) (i)));
      selector_ctl->setItemData(i, processor->cfilename(), Qt::ToolTipRole);
    }
  }

  if ( selector_ctl->count() > 0 ) {

    if ( !currentProcessor_ ) {
      selector_ctl->setCurrentIndex(0);
    }
    else {
      const int index = selector_ctl->findText(currentProcessor_->cname());
      selector_ctl->setCurrentIndex(index >= 0 ? index : 0);
    }
  }

  setEnabled(true);

  updatecurrentprocessor();
}

void QDataFrameProcessorSelector::onProcessorSelectorCurrentIndexChanged(int)
{
  if ( !updatingControls() ) {
    updatecurrentprocessor();
    if ( enabled_ctl->isChecked() ) {
      Q_EMIT parameterChanged();
    }
  }
}

void QDataFrameProcessorSelector::updatecurrentprocessor()
{
  c_data_frame_processor::sptr selected_processor;

  if( !QDataFrameProcessorsCollection::empty() ) {

    QString processorName = selector_ctl->currentText();
    if ( !processorName.isEmpty() ) {
      const int pos = QDataFrameProcessorsCollection::indexof(processorName);
      if ( pos >= 0 ) {
        selected_processor = QDataFrameProcessorsCollection::item(pos);
      }
    }
  }

  if ( currentProcessor_ != selected_processor  ) {
    currentProcessor_ = selected_processor;
  }

  if ( currentProcessor_ != processor_ctl->currentProcessor() ) {
    processor_ctl->setCurrentProcessor(currentProcessor_);
  }
}

void QDataFrameProcessorSelector::addProcessor()
{
  while ( 42 ) {

    const QString processorName =
        QInputDialog::getText(this,
            "Add image processor...",
            "Processor name:",
            QLineEdit::EchoMode::Normal);

    if ( processorName.isEmpty() ) {
      return;
    }


    if ( QDataFrameProcessorsCollection::indexof(processorName) >=0 ) {
      QMessageBox::warning(this, "ERROR", "Processor with this name is already exists.\n"
          "Enter another name.");
      continue;
    }

    c_data_frame_processor::sptr processor =
        c_data_frame_processor::create(processorName.toStdString());

    if ( !processor ) {
      QMessageBox::critical(this, "FATAL ERROR", "Unexpected error:\n"
          "c_data_frame_processor::create() fails.");
      return;
    }

    QDataFrameProcessorsCollection::add(currentProcessor_ = processor);
    currentProcessor_->save();
    updateControls();

    return;
  }

}

void QDataFrameProcessorSelector::deleteCurrentProcessor()
{
  if ( !currentProcessor_ ) {
    return;
  }

  const int reply = QMessageBox::question(this,
      "Confirmation required",
      QString("Are you sure to deletec current processor '%1' ?").
          arg(currentProcessor_->cname()),
      QMessageBox::Yes | QMessageBox::No);


  if ( reply != QMessageBox::Yes ) {
    return;
  }

  const int pos = QDataFrameProcessorsCollection::indexof(currentProcessor_);
  if( pos < 0 ) {
    return;
  }

  QDataFrameProcessorsCollection::remove_at(pos);

  if ( !currentProcessor_->filename().empty() ) {

    if ( QFile::exists(currentProcessor_->cfilename()) ) {

      if ( !QFile::remove(currentProcessor_->cfilename()) ) {

        QMessageBox::warning(this, "WARNING",
            QString("Failed to remove the file from disk:\n"
                "%1\n"
                "Check the path and file protection."));
      }
    }
  }

  currentProcessor_.reset();
  updateControls();
}

void QDataFrameProcessorSelector::renameCurrentProcessor()
{
  if ( !currentProcessor_ ) {
    return;
  }

  QString processorName = currentProcessor_->cname();

  while ( 42 ) {

    processorName =
        QInputDialog::getText(this,
            "Rename current processor...",
            "New processor name:",
            QLineEdit::EchoMode::Normal,
            processorName);

    if ( processorName.isEmpty() || processorName.compare(currentProcessor_->cname()) == 0 ) {
      return;
    }

    if ( !QDataFrameProcessorsCollection::empty() ) {
      if ( QDataFrameProcessorsCollection::indexof(processorName) >=0 ) {
        QMessageBox::warning(this, "ERROR", "Processor with this name is already exists.\n"
            "Enter another name.");
        continue;
      }
    }

    currentProcessor_->set_name(processorName.toStdString());

    if ( !currentProcessor_->filename().empty() ) {

      QFile::remove(currentProcessor_->cfilename());

      const QString abspath = QFileInfo(currentProcessor_->cfilename()).absolutePath();
      if ( !abspath.isEmpty() ) {
        currentProcessor_->set_filename(QString("%1/%2.cfg").arg(abspath).arg(processorName).toStdString());
      }
    }

    currentProcessor_->save();

    updateControls();

    return;
  }

}

void QDataFrameProcessorSelector::addCopyOfCurrentProcessor()
{
  // not implemented yet
}


///////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////

QDataFrameProcessorSelectionCombo::QDataFrameProcessorSelectionCombo(QWidget * parent) :
    Base(parent)
{
  setEditable(false);
  setMinimumContentsLength(12);
  setSizeAdjustPolicy(QComboBox::AdjustToContents);
  setFocusPolicy(Qt::StrongFocus);


  connect(QDataFrameProcessorsCollection::instance(), &QDataFrameProcessorsCollection::collectionChanged,
      this, &ThisClass::refresh);

  refresh();
}

c_data_frame_processor::sptr QDataFrameProcessorSelectionCombo::processor(int index) const
{
  if ( index < 1 || (index = QDataFrameProcessorsCollection::indexof(currentText()) ) < 0 ) {
    return nullptr;
  }

  return QDataFrameProcessorsCollection::item(index);
}

bool QDataFrameProcessorSelectionCombo::setCurrentProcessor(const c_data_frame_processor::sptr & processor)
{
  int current_index = 0; // "None" in refresh()
  if ( processor && (current_index = findText(processor->cname())) < 0) {
    current_index = 0;
  }

  setCurrentIndex(current_index);

  return currentIndex() > 0;
}

c_data_frame_processor::sptr QDataFrameProcessorSelectionCombo::currentProcessor() const
{
  return processor(currentIndex());
}

void QDataFrameProcessorSelectionCombo::refresh()
{
  const bool wasEnabled =
      isEnabled();

  if ( wasEnabled ) {
    setEnabled(false);
  }

  const QString current_processor_name =
      currentText();

  clear();
  addItem("None");

  for ( int i = 0, n = QDataFrameProcessorsCollection::size(); i < n; ++i ) {

    const c_data_frame_processor::sptr processor =
        QDataFrameProcessorsCollection::item(i);

    if( processor ) {
      addItem(processor->cname(), processor->cfilename());
    }
  }

  if ( !current_processor_name.isEmpty() ) {
    const int index =
        findText(current_processor_name);

    if ( index >= 0 ) {
      setCurrentIndex(index);
    }
  }

  if ( wasEnabled ) {
    setEnabled(true);
  }

  if ( currentText() != current_processor_name ) {
    Q_EMIT currentIndexChanged(currentIndex());
  }
}


///////////////////////////////////////////////////////////////////////////////

QDataFrameProcessorsCollection QDataFrameProcessorsCollection::instance_;

//c_data_frame_processor_collection::ptr QDataFrameProcessorsCollection::processors_ =
//    c_data_frame_processor_collection::create();

QDataFrameProcessorsCollection::QDataFrameProcessorsCollection()
{
}

QDataFrameProcessorsCollection::~QDataFrameProcessorsCollection()
{
}

// for QObject::connect()
QDataFrameProcessorsCollection * QDataFrameProcessorsCollection::instance()
{
  return &instance_;
}

bool QDataFrameProcessorsCollection::load()
{
  c_data_frame_processor_collection::default_instance()->clear();
  c_data_frame_processor_collection::default_instance()->load(c_data_frame_processor_collection::default_processor_collection_path());
  if ( c_data_frame_processor_collection::default_instance()->empty() ) {
    c_data_frame_processor_collection::default_instance()->emplace_back(c_data_frame_processor::create("Default"));
  }
  Q_EMIT instance()->collectionChanged();
  return true;
}

bool QDataFrameProcessorsCollection::save()
{
  return c_data_frame_processor_collection::default_instance()->save();
}

int QDataFrameProcessorsCollection::size()
{
  return c_data_frame_processor_collection::default_instance()->size();
}

bool QDataFrameProcessorsCollection::empty()
{
  return c_data_frame_processor_collection::default_instance()->empty();
}

const c_data_frame_processor::sptr & QDataFrameProcessorsCollection::item(int pos)
{
  return c_data_frame_processor_collection::default_instance()->at(pos);
}

void QDataFrameProcessorsCollection::add(const c_data_frame_processor::sptr & p, bool emit_notify)
{
  c_data_frame_processor_collection::default_instance()->emplace_back(p);

  if ( emit_notify ) {
    Q_EMIT instance_.collectionChanged();
  }
}

bool QDataFrameProcessorsCollection::insert(int pos, const c_data_frame_processor::sptr & p, bool emit_notify)
{
  if( pos < 0 || pos >= size() ) {
    CF_FATAL("POSSIBLE APP BUG: attempt to insert into position %d when array size=%d", pos, size());
    return false;
  }

  c_data_frame_processor_collection::default_instance()->insert(c_data_frame_processor_collection::default_instance()->begin() + pos, p);

  if ( emit_notify ) {
    Q_EMIT instance_.collectionChanged();
  }

  return true;
}

bool QDataFrameProcessorsCollection::remove(const c_data_frame_processor::sptr & p, bool emit_notify)
{
  c_data_frame_processor_collection::iterator pos =
      c_data_frame_processor_collection::default_instance()->find(p);

  if( pos != c_data_frame_processor_collection::default_instance()->end() ) {
    c_data_frame_processor_collection::default_instance()->erase(pos);

    if ( emit_notify ) {
      Q_EMIT instance_.collectionChanged();
    }

    return true;
  }

  return false;
}

bool QDataFrameProcessorsCollection::remove_at(int pos, bool emit_notify)
{
  if( pos < 0 || pos >= size() ) {
    CF_FATAL("POSSIBLE APP BUG: attempt to remove from position %d when array size=%d", pos, size());
    return false;
  }

  c_data_frame_processor_collection::default_instance()->erase(c_data_frame_processor_collection::default_instance()->begin() + pos);
  if ( emit_notify ) {
    Q_EMIT instance_.collectionChanged();
  }

  return true;
}

int QDataFrameProcessorsCollection::indexof(const c_data_frame_processor::sptr & p)
{
  c_data_frame_processor_collection::iterator pos =
      c_data_frame_processor_collection::default_instance()->find(p);

  return pos != c_data_frame_processor_collection::default_instance()->end() ?
      pos - c_data_frame_processor_collection::default_instance()->begin() : -1;
}

int QDataFrameProcessorsCollection::indexof(const std::string & name)
{
  c_data_frame_processor_collection::iterator pos =
      c_data_frame_processor_collection::default_instance()->find(name);

  return pos != c_data_frame_processor_collection::default_instance()->end() ?
      pos - c_data_frame_processor_collection::default_instance()->begin() : -1;
}

int QDataFrameProcessorsCollection::indexof(const QString & name)
{
  c_data_frame_processor_collection::iterator pos =
      c_data_frame_processor_collection::default_instance()->find(name.toStdString());

  return pos != c_data_frame_processor_collection::default_instance()->end() ?
      pos - c_data_frame_processor_collection::default_instance()->begin() : -1;
}


