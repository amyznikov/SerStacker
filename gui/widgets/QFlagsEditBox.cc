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

  _layout = new QHBoxLayout(this);
  _layout->setContentsMargins(0, 0, 0, 0);
  _layout->addWidget(_editBox = new QLineEdit(this));
  _layout->addWidget(_menuButton = new QToolButton(this));

  _editBox->setReadOnly(true);

  _menuButton->setIconSize(QSize(16,16));
  _menuButton->setText("Edit...");

  connect(_menuButton, &QToolButton::clicked,
      this, &ThisClass::onMenuButtonClicked);

  if ( membs ) {
    setupItems(_membs = membs);
    updateLabelText();
  }
}

int QFlagsEditBoxBase::flags() const
{
  return _flags;
}

void QFlagsEditBoxBase::setFlags(int value)
{
  _flags = value;
  updateControls();
}

void QFlagsEditBoxBase::setupItems(const c_enum_member * membs)
{
  QSignalBlocker block(this);

  for( QAction *action : _actions ) {
    delete action;
  }

  _actions.clear();

  if( membs ) {

    QAction *action;

    action = new QAction("Edit...", this);
    _actions.append(action);

    connect(action, &QAction::triggered,
        [this, action, membs]() {

          bool fOk = false;

          QString text = QInputDialog::getText(this,
              "QFlagsEditBox",
              "Enter flags:",
              QLineEdit::Normal,
              flagsToString(_flags, membs).c_str(),
              &fOk).trimmed();

          if ( !text.isEmpty() ) {

            const int oldflags =
                _flags;

            if ( text.startsWith("0x", Qt::CaseInsensitive ) ) {
              _flags = text.toInt(&fOk, 16);
            }
            else if ( text[0].isDigit() ) {
              _flags = text.toInt(&fOk, 10);
            }
            else {
              _flags = flagsFromString(text.toStdString(), membs);
            }

            updateControls();

            if ( _flags != oldflags ) {
              Q_EMIT flagsChanged(_flags);
            }
          }
        });

    action = new QAction("", this);
    action->setSeparator(true);
    _actions.append(action);

    while (!membs->name.empty()) {

      action =
          new QAction(membs->name.c_str(), this);

      action->setData(membs->value);
      action->setToolTip(membs->comment.c_str());
      action->setCheckable(true);
      action->setChecked((_flags & membs->value));
      _actions.append(action);

      QObject::connect(action, &QAction::triggered,
          [this, action](bool checked) {

            //if ( !updatingControls_ )
            {

              const int value =
                  action->data().toInt();

              if ( checked ) {
                _flags |= value;
              }
              else {
                _flags &= ~value;
              }

              updateLabelText();

              Q_EMIT flagsChanged(_flags);
            }
          });

      ++membs;
    }

  }
}

void QFlagsEditBoxBase::updateLabelText()
{
  _editBox->setText(qsprintf("0x%0X %s", _flags, _membs ? flagsToString(_flags, _membs).c_str() : "" ));
}

void QFlagsEditBoxBase::updateControls()
{
  QSignalBlocker block(this);
  //updatingControls_ = true;
  updateLabelText();

  for( QAction *action : _actions ) {

    const int value =
        action->data().toInt();

    action->setChecked((_flags & value));
  }

  //updatingControls_ = false;
}

void QFlagsEditBoxBase::onMenuButtonClicked()
{
  const QPoint pos(_menuButton->width() / 2,
      _menuButton->height() / 2);

  QMenu::exec(_actions,
      _menuButton->mapToGlobal(pos));
}

