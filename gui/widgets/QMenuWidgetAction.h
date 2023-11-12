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


class QMenuWidgetAction :
    public QWidgetAction
{
public:
  typedef QMenuWidgetAction ThisClass;
  typedef QWidgetAction Base;

  QMenuWidgetAction(const QString & label, QWidget * ctrl, QObject * parent = nullptr) :
    Base(parent),
    ctrl_(ctrl)
  {
    hbox_ = new QHBoxLayout(widget_ = new QWidget());
    hbox_->addWidget(label_ = new QLabel(label), 0, Qt::AlignLeft);
    hbox_->addWidget(ctrl, 0, Qt::AlignRight);
    setDefaultWidget(widget_);
  }

  QWidget * control() const
  {
    return ctrl_;
  }

  QLabel * label() const
  {
    return label_;
  }

protected:
  QHBoxLayout * hbox_ = nullptr;
  QWidget * widget_ = nullptr;
  QLabel * label_ = nullptr;
  QWidget * ctrl_ = nullptr;
};


template <class CtrlType>
class QMenuWidgetActionTemplate:
    public QWidgetAction
{
public:
  typedef QMenuWidgetActionTemplate ThisClass;
  typedef QWidgetAction Base;

  QMenuWidgetActionTemplate(const QString & label, QObject * parent = nullptr) :
    Base(parent)
  {
    hbox_ = new QHBoxLayout(widget_ = new QWidget());
    hbox_->addWidget(label_ = new QLabel(label), 0, Qt::AlignLeft);
    hbox_->addWidget(ctrl_ = new CtrlType(), 0, Qt::AlignRight);
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

protected:
  QHBoxLayout * hbox_ = nullptr;
  QWidget * widget_ = nullptr;
  QLabel * label_ = nullptr;
  CtrlType * ctrl_ = nullptr;
};

template<class CtrlType, class Fn>
inline QMenuWidgetActionTemplate<CtrlType> * createMenuWidgetAction(const QString & label, QObject * parent, Fn && setupCtrl)
{
  QMenuWidgetActionTemplate<CtrlType> * action =
      new QMenuWidgetActionTemplate<CtrlType>(label, parent);

  setupCtrl(action);

  return action;
}


#endif /* __QMenuWidgetAction_h__ */
