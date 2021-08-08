/*
 * QImageProcessorChainEditor.cc
 *
 *  Created on: Aug 5, 2021
 *      Author: amyznikov
 */

#include "QImageProcessorChainEditor.h"
#include "QAddRoutineDialog.h"
#include <gui/widgets/QWaitCursor.h>

#define ICON_double_arrow_down    "double-arrow-down"
#define ICON_double_arrow_right   "double-arrow-right"
#define ICON_move_down            "move-down"
#define ICON_move_up              "move-up"
#define ICON_delete               "delete"
#define ICON_add                  "add"
#define ICON_menu                 "menu"


static QIcon getIcon(const QString & name)
{
  return QIcon(QString(":/qimproc/icons/%1").arg(name));
}


QImageProcessorChainEditor::QImageProcessorChainEditor(QWidget * parent)
  : Base("QImageProcessorChainEditor", parent)
{
}

void QImageProcessorChainEditor::set_current_processor(const c_image_processor::ptr & p)
{
  current_processor_ = p;

  for ( int i = form->rowCount() - 1; i >= 0; --i ) {
    form->removeRow(i);
  }

  if ( current_processor_ ) {

    if ( true ) { // stub for empty processors

      static QMenu menu;
      static QAction * add_routine_action = Q_NULLPTR;
      if ( menu.isEmpty() ) {
        add_routine_action = menu.addAction(getIcon(ICON_add), "Add ...");
      }

      static const char borderless_style[] = ""
          "QToolButton { border: none; }";

      QToolButton * menu_ctl = new QToolButton();
      menu_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
      menu_ctl->setIconSize(QSize(16, 16));
      menu_ctl->setStyleSheet(borderless_style);
      menu_ctl->setIcon(getIcon(ICON_menu));
      menu_ctl->setText("Options...");
      menu_ctl->setToolTip("Options...");
      form->addRow(menu_ctl);

      connect(menu_ctl, &QToolButton::clicked,
          [this, menu_ctl] () {
            if ( current_processor_ ) {

              QAction * selectedAction =
                  menu.exec(menu_ctl->mapToGlobal(QPoint(
                      menu_ctl->width()-2,
                      menu_ctl->height()-4)));

              if ( selectedAction == add_routine_action ) {
                addRoutine();
              }
            }
          });
    }

    for ( const c_image_processor_routine::ptr & routine : *current_processor_ ) {
      if ( routine ) {

        QImageProcessorRoutineSettingsBase * ctl =
            QImageProcessorRoutineSettingsBase::create(routine);

        if ( !ctl ) {
          form->addRow(routine->class_name().c_str(), new QSettingsWidget(""));
        }
        else {

          form->addRow(ctl);

          connect(ctl, &QImageProcessorRoutineSettingsBase::addRoutineRequested,
              this, &ThisClass::addRoutine);

          connect(ctl, &QImageProcessorRoutineSettingsBase::removeRoutineRequested,
              this, &ThisClass::removeRoutine);

          connect(ctl, &QImageProcessorRoutineSettingsBase::moveUpRequested,
              this, &ThisClass::moveUpRoutine);

          connect(ctl, &QImageProcessorRoutineSettingsBase::moveDownRequested,
              this, &ThisClass::moveDownRoutine);

          connect(ctl, &QSettingsWidget::parameterChanged,
              [this]() {
                if ( current_processor_ ) {
                  current_processor_->save();
                }
                emit parameterChanged();
          });
        }
      }
    }

  }

  emit parameterChanged();

  updateControls();
}

const c_image_processor::ptr & QImageProcessorChainEditor::current_processor() const
{
  return current_processor_;
}

void QImageProcessorChainEditor::onupdatecontrols()
{
  if ( !current_processor_ ) {
    setEnabled(false);
  }
  else {

    QLayoutItem *item;
    QSettingsWidget * ctl;

    for ( int i = 1, n = form->rowCount(); i < n; ++i ) {

      if ( (item = form->itemAt(i, QFormLayout::ItemRole::FieldRole)) ) {

        if ( (ctl = dynamic_cast<QSettingsWidget *>(item->widget()))  ) {

          ctl->updateControls();

        }
      }
    }

    setEnabled(true);
  }

  Base::onupdatecontrols();
}


void QImageProcessorChainEditor::addRoutine(QImageProcessorRoutineSettingsBase * insertAfter)
{
  static QAddRoutineDialog * dlgbox = Q_NULLPTR;
  c_image_processor_routine::ptr current_routine, new_routine;

  if ( !current_processor_ ) {
    return;
  }

  if ( !dlgbox ) {
    dlgbox = new QAddRoutineDialog(this);
  }
  else {
    dlgbox->setParent(this);
  }

  if ( dlgbox->exec() != QDialog::Accepted || !dlgbox->selectedClassFactory() ) {
    return;
  }

  if ( insertAfter && !(current_routine = insertAfter->current_routine()) ) {
    QMessageBox::critical(this, "APP BUG", QString("insertAfter->current_routine() is NULL.\n"
        "Fix this code please"));
    return;
  }

  new_routine = c_image_processor_routine::create(dlgbox->selectedClassFactory()->class_name);
  if ( !new_routine ) {
    QMessageBox::critical(this, "ERROR", QString("c_image_processor_routine::create(%1) fails").
        arg(dlgbox->selectedClassFactory()->class_name.c_str()));
    return;
  }

  QWaitCursor wait(this);

  if ( !current_routine ) {
    current_processor_->insert(current_processor_->begin(), new_routine);
  }
  else {
    current_processor_->insert(current_processor_->find(current_routine) + 1, new_routine);
  }

  // force reload config widgets
  current_processor_->save();
  set_current_processor(current_processor_);
}

void QImageProcessorChainEditor::removeRoutine(QImageProcessorRoutineSettingsBase * w)
{
  if ( w && current_processor_ ) {

    QWaitCursor wait(this);

    c_image_processor::iterator pos = current_processor_->find(w->current_routine());
    if ( pos != current_processor_->end() ) {

      current_processor_->erase(pos);
      current_processor_->save();

      // force reload config widgets
      set_current_processor(current_processor_);
    }
  }
}

void QImageProcessorChainEditor::moveUpRoutine(QImageProcessorRoutineSettingsBase * w)
{
  if ( w && current_processor_ ) {

    QWaitCursor wait(this);

    c_image_processor::iterator pos = current_processor_->find(w->current_routine());
    if ( pos != current_processor_->begin() && pos != current_processor_->end() ) {

      const c_image_processor_routine::ptr routine = *pos;

      current_processor_->erase(pos);
      current_processor_->insert(pos - 1, routine);
      current_processor_->save();

      // force reload config widgets
      set_current_processor(current_processor_);
    }
  }

}

void QImageProcessorChainEditor::moveDownRoutine(QImageProcessorRoutineSettingsBase * w)
{
  if ( w && current_processor_ ) {

    QWaitCursor wait(this);

    c_image_processor::iterator pos = current_processor_->find(w->current_routine());
    if ( pos + 1 < current_processor_->end()  ) {

      const c_image_processor_routine::ptr routine = *pos;

      current_processor_->erase(pos);
      current_processor_->insert(pos + 1, routine);
      current_processor_->save();

      // force reload config widgets
      set_current_processor(current_processor_);
    }
  }

}
