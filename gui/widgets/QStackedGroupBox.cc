/*
 * QStackedGroupBox.cc
 *
 *  Created on: Oct 8, 2026
 *      Author: amyznikov
 */

#include "QStackedGroupBox.h"
#include<core/debug.h>

QStackedGroupBox::QStackedGroupBox(const QString & label, QWidget * parent) :
  ThisClass(label, nullptr, parent)
{
}

QStackedGroupBox::QStackedGroupBox(const QString & label, const c_enum_member * membs, QWidget * parent) :
  Base(parent)
{
  _vbox = new QVBoxLayout(this);
  _vbox->addLayout(_hbox = new QHBoxLayout(), 1);
  _vbox->addWidget(_stack = new QStackedWidget(this), 100);
  _hbox->addWidget(new QLabel(label, this));
  _hbox->addWidget(_combo = new QEnumComboBoxBase(this));

  setupComboboxItems(membs);

  QObject::connect(_combo, &QEnumComboBoxBase::currentItemChanged,
      this, &ThisClass::onComboboxCurrentItemChanged);

}

void QStackedGroupBox::setupComboboxItems(const c_enum_member * membs)
{
  _combo->setupItems(membs);
}

void QStackedGroupBox::addWidget(int selectorValue, QWidget * widget)
{
  widget->setProperty("QStackedGroupBoxSelectorValue", selectorValue);
  _stack->addWidget(widget);
}

void QStackedGroupBox::updateSelection()
{
  onComboboxCurrentItemChanged(_combo->currentIndex());
}

void QStackedGroupBox::setCurrentSelection(int selectorValue)
{
  _combo->setCurrentIndex(_combo->findData(selectorValue));
}

void QStackedGroupBox::onComboboxCurrentItemChanged(int currentIndex)
{
  int selectorValue = -1;
  bool widgetFound = false;

  if( currentIndex >= 0 ) {
    selectorValue = _combo->itemData(currentIndex).toInt();
    for( int i = 0, n = _stack->count(); i < n; ++i ) {
      QWidget * widget = _stack->widget(i);
      if( widget && widget->property("QStackedGroupBoxSelectorValue").toInt() == selectorValue ) {
        widgetFound = true;
        widget->show();
        _stack->setCurrentIndex(i);
       break;
      }
    }
  }

  if ( !widgetFound ) {
    _stack->setCurrentIndex(-1);
    QWidget * currentWidget = _stack->currentWidget();
    if ( currentWidget ) {
      currentWidget->hide();
    }
  }

  Q_EMIT currentSelectionChanged(selectorValue);
}
