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
  lv_ = new QVBoxLayout(this);

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
          &ThisClass::onRemoveCurrentImageProcessor));

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


  ///////////////////////////////////////////////////////////////////

  updateControls();
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

void QImageProcessorChainEditor::onRemoveCurrentImageProcessor()
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
    label->setText(processor_->classfactory()->tooltip.c_str());
    form->addRow(label);
  }

  std::vector<struct c_image_processor_routine_ctrl> params;

  std::vector<QExpandableGroupBox*> groups;
  std::vector<QSettingsWidget*> gsettings;

  QExpandableGroupBox * group = nullptr;
  QSettingsWidget * groupSettings = nullptr;

  processor_->get_parameters(&params);

  for( const struct c_image_processor_routine_ctrl &p : params ) {

    switch (p.ctl_type) {

      case c_image_processor_ctl_begin_group: {

        if ( group ) {
          groups.emplace_back(group);
          gsettings.emplace_back(groupSettings);
        }

        group =
            add_expandable_groupbox(p.name.c_str(),
                groupSettings = new QSettingsWidget(""),
                100);

        group->expand();

        break;
      }

      case c_image_processor_ctl_end_group: {

        if( groups.empty() ) {
          group = nullptr;
          groupSettings = nullptr;
        }
        else {
          group = groups.back();
          groupSettings = gsettings.back();
          groups.pop_back();
          gsettings.pop_back();
        }

        break;
      }

      case c_image_processor_ctl_flags_chkbox: {

        QFlagsEditBoxBase * ctl =
            new QFlagsEditBoxBase();

        ctl->setToolTip(p.tooltip.c_str());

        if( p.get_enum_members ) {
          ctl->setupItems(p.get_enum_members());
        }

        if( groupSettings ) {
          groupSettings->addRow(p.name.c_str(), ctl);
        }
        else {
          this->addRow(p.name.c_str(), ctl);
        }

        if( p.set_value ) {

          QObject::connect(ctl, &QFlagsEditBoxBase::flagsChanged,
              [this, ctl, p](int flags) {
                if ( !updatingControls() ) {
                  p.set_value(toString(flags));
                  Q_EMIT parameterChanged();
                }
              });
        }

        if( p.get_value ) {

          QObject::connect(this, &ThisClass::populatecontrols,
              [ctl, p]() {
                int flags = ctl->flags();
                if ( fromString(p.get_value(), &flags)) {
                  ctl->setFlags(flags);
                }
              });
        }

        break;

      }

      case c_image_processor_ctl_numeric_box: {

        QNumericBox *ctl =
            new QNumericBox();

        ctl->setToolTip(p.tooltip.c_str());

        if( groupSettings ) {
          groupSettings->addRow(p.name.c_str(), ctl);
        }
        else {
          this->addRow(p.name.c_str(), ctl);
        }

        if( p.set_value ) {

            QObject::connect(ctl, &QNumericBox::textChanged,
                [this, ctl, p]() {
                  if ( !updatingControls() ) {
                    p.set_value(ctl->text().toStdString());
                    Q_EMIT parameterChanged();
                  }
                });

        }

        if( p.get_value ) {
            QObject::connect(this, &ThisClass::populatecontrols,
                [ctl, p]() {
                  ctl->setText(p.get_value().c_str());
                });
        }

        break;
      }

      case c_image_processor_ctl_enum_combobox: {

        QEnumComboBoxBase *ctl =
            new QEnumComboBoxBase();

        ctl->setToolTip(p.tooltip.c_str());

        if( p.get_enum_members ) {
          ctl->setupItems(p.get_enum_members());
        }

        if( groupSettings ) {
          groupSettings->addRow(p.name.c_str(), ctl);
        }
        else {
          this->addRow(p.name.c_str(), ctl);
        }

        if( p.set_value ) {

            QObject::connect(ctl, &QEnumComboBoxBase::currentItemChanged,
                [this, ctl, p]() {
                  if ( !updatingControls() ) {
                    p.set_value(ctl->currentText().toStdString());
                    Q_EMIT parameterChanged();
                  }
                });

        }

        if( p.get_value ) {

            QObject::connect(this, &ThisClass::populatecontrols,
                [ctl, p]() {
                  ctl->setCurrentText(p.get_value().c_str());
                });

        }

        break;
      }

      case c_image_processor_ctl_check_box: {

        QCheckBox *ctl =
            new QCheckBox();

        ctl->setToolTip(p.tooltip.c_str());

        if( groupSettings ) {
          groupSettings->addRow(p.name.c_str(), ctl);
        }
        else {
          this->addRow(p.name.c_str(), ctl);
        }

        if( p.set_value ) {

            QObject::connect(ctl, &QCheckBox::stateChanged,
                [this, ctl, p](int state) {
                  if ( !updatingControls() ) {
                    p.set_value(state == Qt::Checked ? "1" : "0");
                    Q_EMIT parameterChanged();
                  }
                });

        }

        if( p.get_value ) {

            QObject::connect(this, &ThisClass::populatecontrols,
                [ctl, p]() {
                  bool checked = false;
                  if ( fromString(p.get_value(), &checked) ) {
                    ctl->setChecked(checked);
                  }
                });

        }
        break;
      }


      case c_image_processor_ctl_browse_for_existing_file: {

        QBrowsePathCombo *ctl =
            new QBrowsePathCombo("", QFileDialog::AcceptMode::AcceptOpen,
                QFileDialog::ExistingFile);

        ctl->setToolTip(p.tooltip.c_str());

        if( groupSettings ) {
          groupSettings->addRow(p.name.c_str(), ctl);
        }
        else {
          this->addRow(p.name.c_str(), ctl);
        }

        if( p.set_value ) {

          QObject::connect(ctl, &QBrowsePathCombo::pathChanged,
              [this, ctl, p]() {
                if ( !updatingControls() ) {
                  p.set_value(ctl->currentPath().toStdString());
                  Q_EMIT parameterChanged();
                }
              });

        }

        if( p.get_value ) {
          QObject::connect(this, &ThisClass::populatecontrols,
              [ctl, p]() {
                ctl->setCurrentPath(p.get_value().c_str(), false);
              });
        }

        break;
      }

      case c_image_processor_ctl_browse_for_directory: {

        QBrowsePathCombo *ctl =
            new QBrowsePathCombo("", QFileDialog::AcceptMode::AcceptOpen,
                QFileDialog::Directory);

        ctl->setToolTip(p.tooltip.c_str());

        if( groupSettings ) {
          groupSettings->addRow(p.name.c_str(), ctl);
        }
        else {
          this->addRow(p.name.c_str(), ctl);
        }

        if( p.set_value ) {

          QObject::connect(ctl, &QBrowsePathCombo::pathChanged,
              [this, ctl, p]() {
                if ( !updatingControls() ) {
                  p.set_value(ctl->currentPath().toStdString());
                  Q_EMIT parameterChanged();
                }
              });

        }

        if( p.get_value ) {
          QObject::connect(this, &ThisClass::populatecontrols,
              [ctl, p]() {
                ctl->setCurrentPath(p.get_value().c_str(), false);
              });
        }

        break;
      }


      case c_image_processor_ctl_spinbox: {

        QSpinBox * ctl =
            new QSpinBox(this);

        ctl->setToolTip(p.tooltip.c_str());
        ctl->setKeyboardTracking(false);
        ctl->setFocusPolicy(Qt::StrongFocus);
        ctl->setAccelerated(true);
        ctl->setButtonSymbols(QSpinBox::ButtonSymbols::UpDownArrows);
        ctl->setStepType(QSpinBox::DefaultStepType);
        ctl->setRange(p.min, p.max);
        ctl->setSingleStep(p.step);

        if( groupSettings ) {
          groupSettings->addRow(p.name.c_str(), ctl);
        }
        else {
          this->addRow(p.name.c_str(), ctl);
        }

        if( p.set_value ) {
          QObject::connect(ctl, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
              [this, p](int value) {
                if ( !updatingControls() ) {
                  p.set_value(toString(value));
                  Q_EMIT parameterChanged();
                }
              });
        }

        if( p.get_value ) {
          QObject::connect(this, &ThisClass::populatecontrols,
              [ctl, p]() {
                int value;
                if ( fromString(p.get_value(), &value) ) {
                  ctl->setValue(value);
                }
              });
        }

        break;
      }

      case c_image_processor_ctl_double_slider : {

        QDoubleSliderSpinBox * ctl =
            new QDoubleSliderSpinBox(this);

        ctl->setToolTip(p.tooltip.c_str());
        //ctl->setKeyboardTracking(false);
        //ctl->setFocusPolicy(Qt::StrongFocus);
        //ctl->setAccelerated(true);
        //ctl->setButtonSymbols(QSpinBox::ButtonSymbols::UpDownArrows);
        //ctl->setStepType(QSpinBox::DefaultStepType);
        ctl->setRange(p.min, p.max);
        ctl->setSingleStep(p.step);

        CF_DEBUG("p: min=%g max=%g step=%g", p.min, p.max, p.step);
        CF_DEBUG("ctl: min=%g max=%g", ctl->minimum(), ctl->maximum());

        if( groupSettings ) {
          groupSettings->addRow(p.name.c_str(), ctl);
        }
        else {
          this->addRow(p.name.c_str(), ctl);
        }

        if( p.set_value ) {
          QObject::connect(ctl, &QDoubleSliderSpinBox::valueChanged,
              [this, p](double value) {
                if ( !updatingControls() ) {
                  p.set_value(toString(value));
                  Q_EMIT parameterChanged();
                }
              });
        }

        if( p.get_value ) {
          QObject::connect(this, &ThisClass::populatecontrols,
              [ctl, p]() {
                double value;
                if ( fromString(p.get_value(), &value) ) {
                  ctl->setValue(value);
                }
              });
        }

        break;
      }


      case c_image_processor_ctl_sparse_feature_detector : {

        QSparseFeatureDetectorOptions * ctl =
            new QSparseFeatureDetectorOptions(this);

        ctl->setToolTip(p.tooltip.c_str());

        if( groupSettings ) {
          groupSettings->addRow(p.name.c_str(), ctl);
        }
        else {
          this->addRow(p.name.c_str(), ctl);
        }

        if( p.sparse_feature_detector ) {

          QObject::connect(this, &ThisClass::populatecontrols,
              [ctl, p]() {
                ctl->set_feature_detector_options(p.sparse_feature_detector());
              });
        }

        QObject::connect(ctl, &QSparseFeatureDetectorOptions::parameterChanged,
            [this]() {
              if ( processor_ && !updatingControls() ) {
                processor_->parameter_changed();
                Q_EMIT parameterChanged();
              }
            });

        break;
      }

      case c_image_processor_ctl_math_expression : {

        QInputMathExpressionWidget * ctl =
            new QInputMathExpressionWidget(this);

        ctl->setToolTip(p.tooltip.c_str());

        if( groupSettings ) {
          groupSettings->addRow(p.name.c_str(), ctl);
        }
        else {
          this->addRow(p.name.c_str(), ctl);
        }

        if( p.get_value ) {

          QObject::connect(this, &ThisClass::populatecontrols,
              [ctl, p]() {
                ctl->setText(p.get_value().c_str());
              });
        }

        if( p.set_value ) {

          QObject::connect(ctl, &QInputMathExpressionWidget::apply,
              [this, ctl, p]() {
                if ( !updatingControls() ) {
                  p.set_value(ctl->text().toStdString());
                  Q_EMIT parameterChanged();
                }
              });
        }

        break;
      }

      default:
        break;
    }
  }

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

