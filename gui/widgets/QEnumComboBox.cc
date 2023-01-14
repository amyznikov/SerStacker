/*
 * QEnumComboBox.cc
 *
 *  Created on: Jan 14, 2023
 *      Author: amyznikov
 */
#include "QEnumComboBox.h"
#include <core/debug.h>

QEnumComboBoxBase::QEnumComboBoxBase(QWidget * parent) :
    ThsClass(nullptr, parent)
{
}

QEnumComboBoxBase::QEnumComboBoxBase(const c_enum_member * membs, QWidget * parent) :
    Base(parent)
{
  setupItems(membs);

  connect(this, SIGNAL(currentIndexChanged(int)),
      this, SIGNAL(currentItemChanged(int)));

  setEditable(true);
  setInsertPolicy(QComboBox::NoInsert);

  completer_ = new QCompleter(this);
  completer_->setModel(this->model());
  completer_->setCompletionMode(QCompleter::PopupCompletion);
  completer_->setFilterMode(Qt::MatchContains);
  completer_->setCaseSensitivity(Qt::CaseInsensitive);
  setCompleter(completer_);

  connect(this, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
      [this](int index) {

        if ( index < 0 ) {
          setToolTip(whatsThis());
        }
        else {
          const QString s1 = whatsThis();
          const QString s2 = itemData(index, Qt::WhatsThisRole).toString();
          if ( !s1.isEmpty() && !s2.isEmpty() ) {
            setToolTip(QString("%1\n  %2").arg(s1).arg(s2));
          }
          else if ( !s1.isEmpty() ) {
            setToolTip(s1);
          }
          else if ( !s2.isEmpty() ) {
            setToolTip(s2);
          }
          else {
            setToolTip("");
          }
        }
      });

  connect(lineEdit(), &QLineEdit::editingFinished,
      [this]() {
        setCurrentText(itemText(currentIndex()));
      });

}

void QEnumComboBoxBase::setupItems(const c_enum_member * membs)
{
  if( membs ) {
    while (membs->name && *membs->name) {
      Base::addItem(membs->name, (int) (membs->value));
      if( membs->comment ) {
        Base::setItemData(count() - 1, QString(membs->comment), Qt::WhatsThisRole);
      }
      ++membs;
    }
  }
}

