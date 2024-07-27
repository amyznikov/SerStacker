/*
 * QImageProcessorChainEditor.cc
 *
 *  Created on: Aug 5, 2021
 *      Author: amyznikov
 */

#include "QImageProcessorChainEditor.h"
#include "QAddRoutineDialog.h"
#include "QRadialPolySharpSettings.h"
#include "QMtfSettings.h"
#include <gui/qmathexpression/QInputMathExpression.h>
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
    routine_(routine)
  {
    setText(0, routine->display_name().c_str());
    setToolTip(0, routine->tooltip().c_str());
    setCheckState(0, routine->enabled() ? Qt::Checked : Qt::Unchecked);
  }

  const c_image_processor_routine::ptr & routine() const
  {
    return routine_;
  }

protected:
  c_image_processor_routine::ptr routine_;
};

class QImageProcessorOptionsItem :
    public QTreeWidgetItem
{
public:
  typedef QImageProcessorOptionsItem ThisClass;
  typedef QTreeWidgetItem Base;

  QImageProcessorOptionsItem(const c_image_processor_routine::ptr &routine) :
    routine_(routine)
  {
  }

  const c_image_processor_routine::ptr & routine() const
  {
    return routine_;
  }

protected:
  c_image_processor_routine::ptr routine_;
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

  lv_ = new QVBoxLayout(this);
  lv_->setContentsMargins(0, 0, 0, 0);

  ///////////////////////////////////////////////////////////////////
//  lv_->addWidget(toolbar_ctl = new QToolBar(this));
//  toolbar_ctl->setToolButtonStyle(Qt::ToolButtonStyle::ToolButtonIconOnly);
//  toolbar_ctl->setIconSize(QSize(16, 16));
//
//  addStretch(toolbar_ctl);

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

  connect(tree_ctl, &QTreeWidget::itemChanged,
      this, &ThisClass::onTreeItemChanged);

  connect(tree_ctl, &QTreeWidget::currentItemChanged,
      this, &ThisClass::onCurrentTreeItemChanged);

  connect(tree_ctl, &QTreeWidget::itemExpanded,
      this, &ThisClass::updateItemSizeHint);

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

void QImageProcessorChainEditor::set_current_processor(const c_image_processor::sptr & p)
{
  current_processor_ = p;
  updateControls();
}

const c_image_processor::sptr & QImageProcessorChainEditor::current_processor() const
{
  return current_processor_;
}

void QImageProcessorChainEditor::onupdatecontrols()
{
  if ( !current_processor_ ) {
    setEnabled(false);
    tree_ctl->clear();
  }
  else {

    tree_ctl->clear();

    for( const c_image_processor_routine::ptr &routine : *current_processor_ ) {
      if( routine ) {
        insertProcessorItem(tree_ctl->topLevelItemCount(), routine);
      }
    }

    setEnabled(true);
  }
}

QTreeWidgetItem * QImageProcessorChainEditor::insertProcessorItem(int index, const c_image_processor_routine::ptr & routine)
{
  QImageProcessorItem * item =
      new QImageProcessorItem(routine);

  tree_ctl->insertTopLevelItem(index, item);

  QImageProcessorSettingsControl * ctrl =
      QImageProcessorSettingsControl::create(routine,
          this);


  if ( ctrl ) {

    //ctrl->setAutoFillBackground(true);

    QImageProcessorOptionsItem * subitem =
        new QImageProcessorOptionsItem(routine);

    item->addChild(subitem);

    tree_ctl->setItemWidget(subitem, 0, createScrollableWrap(ctrl));

    connect(ctrl, &QImageProcessorSettingsControl::parameterChanged,
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

void QImageProcessorChainEditor::onMoveCurrentProcessorDown()
{
  if( current_processor_ ) {

    QImageProcessorItem *currentItem =
        currentImageProcessorItem(tree_ctl);

    if( currentItem ) {

      const int index =
          tree_ctl->indexOfTopLevelItem(currentItem);

      if( index < tree_ctl->topLevelItemCount() - 1 ) {

        QWaitCursor wait(this);

        c_image_processor::iterator pos =
            current_processor_->find(currentItem->routine());

        if( pos + 1 < current_processor_->end() ) {

          const c_image_processor_routine::ptr routine = *pos;

          if( true ) {
            c_image_processor::edit_lock lock(current_processor_);
            current_processor_->erase(pos);
            current_processor_->insert(pos + 1, routine);
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
  if( current_processor_ ) {

    QImageProcessorItem *currentItem =
        currentImageProcessorItem(tree_ctl);

    if( currentItem ) {

      const int index =
          tree_ctl->indexOfTopLevelItem(currentItem);


      if( index > 0 ) {

        QWaitCursor wait(this);

        c_image_processor::iterator pos =
            current_processor_->find(currentItem->routine());

        if ( pos != current_processor_->begin() && pos != current_processor_->end() ) {

          const c_image_processor_routine::ptr routine = *pos;

          if( true ) {
            c_image_processor::edit_lock lock(current_processor_);
            current_processor_->erase(pos);
            current_processor_->insert(pos - 1, routine);
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

  if( !current_processor_ ) {
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
    c_image_processor::edit_lock lock(current_processor_);

    if( !current_routine ) {
      current_processor_->insert(current_processor_->begin(),
          new_routine);
    }
    else {
      current_processor_->insert(current_processor_->find(current_routine) + 1,
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
  if ( current_processor_ ) {

    QImageProcessorItem * currentItem =
        currentImageProcessorItem(tree_ctl);

    if( currentItem ) {

      QWaitCursor wait(this);

      c_image_processor::iterator pos =
          current_processor_->find(currentItem->routine());

      if( pos != current_processor_->end() ) {

        if( true ) {
          c_image_processor::edit_lock lock(current_processor_);
          current_processor_->erase(pos);
        }

        delete currentItem;

        Q_EMIT parameterChanged();
      }
    }
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
QImageProcessorSettingsControl * QImageProcessorSettingsControl::create(const c_image_processor_routine::ptr & processor,
    QWidget * parent)
{
  QImageProcessorSettingsControl * widget = nullptr;

  if( processor->classfactory() == c_radial_polysharp_routine::class_factory_instance() ) {
    widget = new QRadialPolySharpSettings(std::dynamic_pointer_cast<c_radial_polysharp_routine>(processor), parent);
  }
//  else if( processor->classfactory() == c_fit_jovian_ellipse_routine::class_factory_instance() ) {
//    widget = new QJovianEllipseSettings(std::dynamic_pointer_cast<c_fit_jovian_ellipse_routine>(processor), parent);
//  }
  else if( processor->classfactory() == c_mtf_routine::class_factory_instance() ) {
    widget = new QMtfSettings(std::dynamic_pointer_cast<c_mtf_routine>(processor), parent);
  }
  else {
    widget = new QImageProcessorSettingsControl(processor, parent);
  }

  widget->setupControls();

  return widget;
}

QImageProcessorSettingsControl::QImageProcessorSettingsControl(const c_image_processor_routine::ptr & processor, QWidget * parent) :
    Base("QImageProcessorSettingsControl", parent),
    processor_(processor)
{
  setFrameShape(QFrame::Shape::Box);
}

void QImageProcessorSettingsControl::setupControls()
{
  if( !processor_->classfactory()->tooltip.empty() ) {
    QLabel *label = new QLabel(this);
    label->setTextFormat(Qt::RichText);
    label->setWordWrap(true);
    label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    label->setText(processor_->classfactory()->tooltip.c_str());
    form->addRow(label);
  }

  std::vector<c_ctrl_bind> params;
  processor_->get_parameters(&params);
  Base::setup_controls(params);

  connect(this, &ThisClass::parameterChanged,
      [this]() {
        if ( processor_ ) {
          processor_->parameter_changed();
        }
      });

  updateControls();
}

void QImageProcessorSettingsControl::onupdatecontrols()
{
  if ( !processor_ ) {
    setEnabled(false);
  }
  else {
    Q_EMIT populatecontrols();
    setEnabled(true);
  }
}

