/*
 * QFlagsEditBox.cc
 *
 *  Created on: Feb 26, 2023
 *      Author: amyznikov
 */

#include "QFlagsEditBox.h"

QFlagsEditBoxBase::QFlagsEditBoxBase(QWidget * parent) :
    ThisClass(nullptr, parent)
{
}

QFlagsEditBoxBase::QFlagsEditBoxBase(const c_enum_member * membs, QWidget * parent) :
    Base(parent)
{
  setContentsMargins(0, 0, 0, 0);

  layout_ = new QHBoxLayout(this);
  layout_->setContentsMargins(0, 0, 0, 0);
  layout_->addWidget(editBox_ = new QLineEdit(this));
  layout_->addWidget(menuButton_ = new QToolButton(this));

  editBox_->setReadOnly(true);

  menuButton_->setIconSize(QSize(16,16));
  menuButton_->setText("Edit...");

  connect(menuButton_, &QToolButton::clicked,
      this, &ThisClass::onMenuButtonClicked);

  if ( membs ) {
    setupItems(membs);
    updateLabelText();
  }
}

int QFlagsEditBoxBase::flags() const
{
  return flags_;
}

void QFlagsEditBoxBase::setFlags(int value)
{
  flags_ = value;
  updateControls();
}

void QFlagsEditBoxBase::setupItems(const c_enum_member * membs)
{
  for( QAction *action : actions_ ) {
    delete action;
  }

  actions_.clear();

  if( membs ) {

    QAction *action;

    action = new QAction("Edit...", this);
    actions_.append(action);

    connect(action, &QAction::triggered,
        [this, action, membs]() {

          bool fOk = false;

          QString text = QInputDialog::getText(this,
              "QFlagsEditBox",
              "Enter flags:",
              QLineEdit::Normal,
              flagsToString(flags_, membs).c_str(),
              &fOk).trimmed();

          if ( !text.isEmpty() ) {

            const int oldflags =
                flags_;

            if ( text.startsWith("0x", Qt::CaseInsensitive ) ) {
              flags_ = text.toInt(&fOk, 16);
            }
            else if ( text[0].isDigit() ) {
              flags_ = text.toInt(&fOk, 10);
            }
            else {
              flags_ = flagsFromString(text.toStdString(), membs);
            }

            updateControls();

            if ( flags_ != oldflags ) {
              Q_EMIT flagsChanged(flags_);
            }
          }
        });

    action = new QAction("", this);
    action->setSeparator(true);
    actions_.append(action);

    while (membs->name && *membs->name) {

      action =
          new QAction(membs->name, this);

      action->setData(membs->value);
      action->setToolTip(QString(membs->comment));
      action->setCheckable(true);
      action->setChecked((flags_ & membs->value));
      actions_.append(action);

      connect(action, &QAction::triggered,
          [this, action](bool checked) {

            if ( !updatingControls_ ) {

              const int value =
                  action->data().toInt();

              if ( checked ) {
                flags_ |= value;
              }
              else {
                flags_ &= ~value;
              }

              updateLabelText();

              Q_EMIT flagsChanged(flags_);
            }
          });

      ++membs;
    }

  }
}

void QFlagsEditBoxBase::updateLabelText()
{
  editBox_->setText(qsprintf("0x%0X", flags_));
}

void QFlagsEditBoxBase::updateControls()
{
  updatingControls_ = true;
  updateLabelText();

  for( QAction *action : actions_ ) {

    const int value =
        action->data().toInt();

    action->setChecked((flags_ & value));
  }

  updatingControls_ = false;
}

void QFlagsEditBoxBase::onMenuButtonClicked()
{
  const QPoint pos(menuButton_->width() / 2,
      menuButton_->height() / 2);

  QMenu::exec(actions_,
      menuButton_->mapToGlobal(pos));
}

