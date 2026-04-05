/*
 * QImageProcessorChainEditor.cc
 *
 *  Created on: Aug 5, 2021
 *      Author: amyznikov
 */

#include "QImageProcessorChainEditor.h"
#include "QAddRoutineDialog.h"
#include "QRadialPolySharpSettings.h"
#include <gui/qfeature2d/QFeature2dOptions.h>
#include <gui/widgets/QBrowsePathCombo.h>
#include <gui/widgets/QSliderSpinBox.h>
#include <gui/widgets/QWaitCursor.h>
#include <gui/widgets/style.h>
#include <core/debug.h>

#define ICON_double_arrow_down    ":/qimproc/icons/double-arrow-down"
#define ICON_double_arrow_right   ":/qimproc/icons/double-arrow-right"
#define ICON_move_down            ":/qimproc/icons/move-down2"
#define ICON_move_up              ":/qimproc/icons/move-up2"
#define ICON_delete               ":/qimproc/icons/delete"
#define ICON_add                  ":/qimproc/icons/add"
#define ICON_menu                 ":/qimproc/icons/menu"

namespace  {

class QImageProcessorItem :
    public QTreeWidgetItem
{
public:
  typedef QImageProcessorItem ThisClass;
  typedef QTreeWidgetItem Base;

  QImageProcessorItem(const c_image_processor_routine::ptr &routine) :
    _routine(routine)
  {
    setText(0, routine->display_name().c_str());
    setToolTip(0, routine->tooltip().c_str());
    setCheckState(0, routine->enabled() ? Qt::Checked : Qt::Unchecked);
  }

  const c_image_processor_routine::ptr & routine() const
  {
    return _routine;
  }

protected:
  c_image_processor_routine::ptr _routine;
};

class QImageProcessorOptionsItem :
    public QTreeWidgetItem
{
public:
  typedef QImageProcessorOptionsItem ThisClass;
  typedef QTreeWidgetItem Base;

  QImageProcessorOptionsItem(const c_image_processor_routine::ptr &routine) :
    _routine(routine)
  {
  }

  const c_image_processor_routine::ptr & routine() const
  {
    return _routine;
  }

protected:
  c_image_processor_routine::ptr _routine;
};


QImageProcessorItem * currentImageProcessorItem(QTreeWidget * tree_ctl)
{
  QImageProcessorItem * processorItem = nullptr;

  QTreeWidgetItem * currentItem =
      tree_ctl->currentItem();

  if ( currentItem ) {

    processorItem =
        dynamic_cast<QImageProcessorItem * >(currentItem);

    if ( !processorItem ) {
      processorItem =
          dynamic_cast<QImageProcessorItem * >(
              currentItem->parent());
    }
  }


  return processorItem;
}

QWidget* addStretch(QToolBar * toolbar)
{
  QWidget *stretch = new QWidget();
  stretch->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  toolbar->addWidget(stretch);
  return stretch;
}

template<class Obj, typename Func1>
QAction * createAction(const QIcon &icon, const QString &text, const QString &tooltip, const Obj *object, Func1 slot)
{
  QAction * action = new QAction(icon, text);
  action->setToolTip(tooltip);
  QObject::connect(action, &QAction::triggered, object, slot);
  return action;
}


QScrollArea* createScrollableWrap(QWidget * w, QWidget * parent = nullptr)
{
  QScrollArea *scrollArea = new QScrollArea(parent ? parent : w->parentWidget());
  scrollArea->setWidgetResizable(true);
  scrollArea->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
  scrollArea->setFrameShape(QFrame::NoFrame);
  scrollArea->setWidget(w);
  return scrollArea;
}

}  // namespace


QImageProcessorChainEditor::QImageProcessorChainEditor(QWidget * parent) :
    Base(parent)
{
  setContentsMargins(0, 0, 0, 0);

  _lv = new QVBoxLayout(this);
  _lv->setContentsMargins(0, 0, 0, 0);

  ///////////////////////////////////////////////////////////////////
//  lv_->addWidget(toolbar_ctl = new QToolBar(this));
//  toolbar_ctl->setToolButtonStyle(Qt::ToolButtonStyle::ToolButtonIconOnly);
//  toolbar_ctl->setIconSize(QSize(16, 16));
//
//  addStretch(toolbar_ctl);

  addAction(_moveDownAction =
      createAction(getIcon(ICON_move_down),
          "Move Down",
          "Move selected processor down",
          this,
          &ThisClass::onMoveCurrentProcessorDown));

  addAction(_moveUpAction =
      createAction(getIcon(ICON_move_up),
          "Move Up",
          "Move selected processor up",
          this,
          &ThisClass::onMoveCurrentProcessorUp));

  addAction(_addProcAction =
      createAction(getIcon(ICON_add),
          "Add processor ...",
          "Add image processor",
          this,
          &ThisClass::onAddImageProcessor));

  addAction(_removeProcAction =
      createAction(getIcon(ICON_delete),
          "Remove selected processor",
          "Remove selected image processor",
          this,
          &ThisClass::onRemoveCurrentProcessor));

  ///////////////////////////////////////////////////////////////////
  _lv->addWidget(tree_ctl = new QTreeWidget(this));
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
            QImageProcessorItem *item = dynamic_cast<QImageProcessorItem*>(tree_ctl->itemAt(pos));
            if ( item ) {

              QMenu menu;

              menu.addAction("Change Display Name...",
                  [this, item]() {

                    const QString oldDisplayName =
                        QString::fromStdString(item->routine()->display_name());

                    const QString newDisplayName =
                        QInputDialog::getText(this, "Enter new display name",  "name:",
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

void QImageProcessorChainEditor::updateItemSizeHint(QTreeWidgetItem * item)
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

void QImageProcessorChainEditor::setCurrentProcessor(const c_image_processor::sptr & p)
{
  _currentProcessor = p;
  updateControls();
}

const c_image_processor::sptr & QImageProcessorChainEditor::currentProcessor() const
{
  return _currentProcessor;
}

void QImageProcessorChainEditor::onupdatecontrols()
{
  if ( !_currentProcessor ) {
    setEnabled(false);
    tree_ctl->clear();
  }
  else {

    tree_ctl->clear();

    for( const c_image_processor_routine::ptr &routine : *_currentProcessor ) {
      if( routine ) {
        insertProcessorItem(tree_ctl->topLevelItemCount(), routine);
      }
    }

    setEnabled(true);
  }
}

QTreeWidgetItem * QImageProcessorChainEditor::insertProcessorItem(int index, const c_image_processor_routine::ptr & routine)
{
  QImageProcessorItem * item = new QImageProcessorItem(routine);
  tree_ctl->insertTopLevelItem(index, item);

  QImageProcessorSettingsControl2 * ctrl = QImageProcessorSettingsControl2::create(routine, this);
  if ( ctrl ) {

    QImageProcessorOptionsItem * subitem = new QImageProcessorOptionsItem(routine);
    subitem->setFlags(subitem->flags() & ~Qt::ItemIsSelectable);
    item->addChild(subitem);

    tree_ctl->setItemWidget(subitem, 0, createScrollableWrap(ctrl));

    connect(ctrl, &QImageProcessorSettingsControl2::parameterChanged,
        this, &ThisClass::parameterChanged,
        Qt::QueuedConnection);

  }

  return item;
}

void QImageProcessorChainEditor::onTreeItemChanged(QTreeWidgetItem * item, int column)
{
  if( item && column == 0 ) {

    QImageProcessorItem *improcItem =
        dynamic_cast<QImageProcessorItem*>(item);

    if( improcItem || (improcItem = dynamic_cast<QImageProcessorItem*>(item->parent())) ) {

      const bool checked =
          improcItem->checkState(0) == Qt::Checked;

      if( improcItem->routine()->enabled() != checked ) {

        improcItem->routine()->set_enabled(checked);

        Q_EMIT parameterChanged();
      }
    }
  }
}

void QImageProcessorChainEditor::onCurrentTreeItemChanged(QTreeWidgetItem * current, QTreeWidgetItem * previous)
{
  if( !current ) {
    _removeProcAction->setEnabled(false);
    _moveDownAction->setEnabled(false);
    _moveUpAction->setEnabled(false);
  }
  else {
    _removeProcAction->setEnabled(true);
    _moveDownAction->setEnabled(true);
    _moveUpAction->setEnabled(true);

    current->setSelected(true);
  }
}

void QImageProcessorChainEditor::onMoveCurrentProcessorDown()
{
  if( _currentProcessor ) {

    QImageProcessorItem *currentItem =
        currentImageProcessorItem(tree_ctl);

    if( currentItem ) {

      const int index =
          tree_ctl->indexOfTopLevelItem(currentItem);

      if( index < tree_ctl->topLevelItemCount() - 1 ) {

        QWaitCursor wait(this);

        c_image_processor::iterator pos =
            _currentProcessor->find(currentItem->routine());

        if( pos + 1 < _currentProcessor->end() ) {

          const c_image_processor_routine::ptr routine = *pos;

          if( true ) {
            c_image_processor::edit_lock lock(_currentProcessor);
            _currentProcessor->erase(pos);
            _currentProcessor->insert(pos + 1, routine);
          }

          const bool isExpanded =
              currentItem->isExpanded();

          delete currentItem;

          QTreeWidgetItem * newItem =
              insertProcessorItem(index + 1, routine);

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

void QImageProcessorChainEditor::onMoveCurrentProcessorUp()
{
  if( _currentProcessor ) {

    QImageProcessorItem *currentItem =
        currentImageProcessorItem(tree_ctl);

    if( currentItem ) {

      const int index =
          tree_ctl->indexOfTopLevelItem(currentItem);


      if( index > 0 ) {

        QWaitCursor wait(this);

        c_image_processor::iterator pos =
            _currentProcessor->find(currentItem->routine());

        if ( pos != _currentProcessor->begin() && pos != _currentProcessor->end() ) {

          const c_image_processor_routine::ptr routine = *pos;

          if( true ) {
            c_image_processor::edit_lock lock(_currentProcessor);
            _currentProcessor->erase(pos);
            _currentProcessor->insert(pos - 1, routine);
          }

          const bool isExpanded =
              currentItem->isExpanded();

          delete currentItem;

          QTreeWidgetItem * newItem =
              insertProcessorItem(index - 1, routine);

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

void QImageProcessorChainEditor::onAddImageProcessor()
{
  static QAddRoutineDialog *dlgbox = nullptr;

  c_image_processor_routine::ptr current_routine, new_routine;

  if( !_currentProcessor ) {
    return;
  }

  if( !dlgbox ) {
    dlgbox = new QAddRoutineDialog(this);
  }
  else {
    dlgbox->setParent(this);
  }

  if( dlgbox->exec() != QDialog::Accepted || !dlgbox->selectedClassFactory() ) {
    return;
  }

  QImageProcessorItem * insertAfter =
      currentImageProcessorItem(tree_ctl);

  if( insertAfter && !(current_routine = insertAfter->routine()) ) {
    QMessageBox::critical(this, "APP BUG", QString("insertAfter->routine() is NULL.\n"
        "Fix this code please"));
    return;
  }

  new_routine = c_image_processor_routine::create(dlgbox->selectedClassFactory()->class_name);
  if( !new_routine ) {
    QMessageBox::critical(this, "ERROR", QString("c_image_processor_routine::create(%1) fails").
        arg(QString(dlgbox->selectedClassFactory()->class_name.c_str())));
    return;
  }

  QWaitCursor wait(this);

  if( true ) {
    c_image_processor::edit_lock lock(_currentProcessor);

    if( !current_routine ) {
      _currentProcessor->insert(_currentProcessor->begin(),
          new_routine);
    }
    else {
      _currentProcessor->insert(_currentProcessor->find(current_routine) + 1,
          new_routine);
    }
  }

  QTreeWidgetItem * newItem =
      insertProcessorItem(tree_ctl->indexOfTopLevelItem(insertAfter) + 1,
          new_routine);

  tree_ctl->setCurrentItem(newItem);

  tree_ctl->expandItem(newItem);

  Q_EMIT parameterChanged();
}

void QImageProcessorChainEditor::onRemoveCurrentProcessor()
{
  if ( _currentProcessor ) {

    QImageProcessorItem * currentItem =
        currentImageProcessorItem(tree_ctl);

    if( currentItem ) {

      QWaitCursor wait(this);

      c_image_processor::iterator pos =
          _currentProcessor->find(currentItem->routine());

      if( pos != _currentProcessor->end() ) {

        if( true ) {
          c_image_processor::edit_lock lock(_currentProcessor);
          _currentProcessor->erase(pos);
        }

        delete currentItem;

        Q_EMIT parameterChanged();
      }
    }
  }
}

QImageProcessorSettingsControl2::QImageProcessorSettingsControl2(const c_image_processor_routine::ptr & processor, QWidget * parent) :
    Base(parent),
    _processor(processor)
{
  setFrameShape(QFrame::Shape::Box);
}

QImageProcessorSettingsControl2 * QImageProcessorSettingsControl2::create(const c_image_processor_routine::ptr & processor, QWidget * parent)
{
  QImageProcessorSettingsControl2 * widget = new QImageProcessorSettingsControl2(processor, parent);
  widget->setupControls();
  return widget;
}

void QImageProcessorSettingsControl2::setupControls()
{
  if( !_processor->classfactory()->tooltip.empty() ) {
    QLabel *label = new QLabel(this);
    label->setTextFormat(Qt::RichText);
    label->setWordWrap(true);
    label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    label->setText(_processor->classfactory()->tooltip.c_str());
    form->addRow(label);
  }

  c_ctlist<c_image_processor_routine> controls;

  _processor->getcontrols(controls);
  ::setupControls(this, controls);

  QObject::connect(this, &ThisClass::parameterChanged,
      [this]() {
        if ( _processor ) {
          _processor->parameter_changed();
        }
      });

  setOpts(_processor.get());

  updateControls();
}
