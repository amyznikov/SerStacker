/*
 * QMenuWidgetAction.h
 *
 *  Created on: Nov 12, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMenuWidgetAction_h__
#define __QMenuWidgetAction_h__

#include <QtWidgets/QtWidgets>

template <class CtrlType>
class QMenuWidgetAction:
    public QWidgetAction
{
public:
  typedef QMenuWidgetAction ThisClass;
  typedef QWidgetAction Base;

  QMenuWidgetAction(const QString & label, QObject * parent = nullptr) :
    Base(parent)
  {
    hbox_ = new QHBoxLayout(widget_ = new QWidget());
    hbox_->setContentsMargins(0,0,0,0);
    hbox_->addWidget(label_ = new QLabel(label), 0, Qt::AlignLeft);
    hbox_->addWidget(ctrl_ = new CtrlType(), 0, Qt::AlignRight);
    label_->setAlignment(Qt::AlignLeft);
    setDefaultWidget(widget_);
  }

  CtrlType * control() const
  {
    return ctrl_;
  }

  QLabel * label() const
  {
    return label_;
  }

  QLabel * icon() const
  {
    if ( !icon_ ) {
      icon_ = new QLabel();
      icon_->setContentsMargins(0,0,0,0);
      icon_->setAlignment(Qt::AlignLeft);
      hbox_->insertWidget(0, icon_, 0, Qt::AlignLeft);
    }
    return icon_;
  }

protected:
  QHBoxLayout * hbox_ = nullptr;
  QWidget * widget_ = nullptr;
  mutable QLabel * icon_ = nullptr;
  QLabel * label_ = nullptr;
  CtrlType * ctrl_ = nullptr;
};

template<class CtrlType, class Fn>
inline QMenuWidgetAction<CtrlType> * createMenuWidgetAction(const QString & label, QObject * parent, Fn && setupCtrl)
{
  QMenuWidgetAction<CtrlType> * action =
      new QMenuWidgetAction<CtrlType>(label, parent);

  setupCtrl(action);

  return action;
}


#endif /* __QMenuWidgetAction_h__ */
