/*
 * createAction.h
 *
 *  Created on: Nov 13, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __createAction_h__
#define __createAction_h__

#include <QtWidgets/QtWidgets>

template<class Obj, typename Fn>
QAction* createAction(const QIcon & icon, const QString & text, const QString & tooltip,
    Obj * receiver, Fn fn,
    QShortcut * shortcut = nullptr)
{
  QAction *action = new QAction(icon, text);
  action->setToolTip(tooltip);

  QObject::connect(action, &QAction::triggered, receiver, fn);

  if( shortcut ) {
    QObject::connect(shortcut, &QShortcut::activated,
        action, &QAction::trigger);
  }

  return action;
}

template<typename Slot>
QAction* createAction(const QIcon & icon, const QString & text, const QString & tooltip, Slot && slot,
    QShortcut * shortcut = nullptr)
{
  QAction *action = new QAction(icon, text);
  action->setToolTip(tooltip);

  QObject::connect(action, &QAction::triggered, slot);

  if( shortcut ) {
    QObject::connect(shortcut, &QShortcut::activated,
        action, &QAction::trigger);
  }

  return action;
}

template<class Obj, typename Fn>
QAction* createCheckableAction(const QIcon & icon, const QString & text, const QString & tooltip,
    bool checked,
    Obj * receiver, Fn fn,
    QShortcut * shortcut = nullptr)
{
  QAction *action = new QAction(icon, text);
  action->setToolTip(tooltip);
  action->setCheckable(true);
  action->setChecked(checked);

  QObject::connect(action, &QAction::triggered, receiver, fn);

  if( shortcut ) {
    QObject::connect(shortcut, &QShortcut::activated,
        action, &QAction::trigger);
  }

  return action;
}

template<typename Slot>
QAction* createCheckableAction(const QIcon & icon, const QString & text, const QString & tooltip,
    bool checked,
    Slot && slot,
    QShortcut * shortcut = nullptr)
{
  QAction *action = new QAction(icon, text);
  action->setToolTip(tooltip);
  action->setCheckable(true);
  action->setChecked(checked);

  QObject::connect(action, &QAction::triggered, slot);

  if( shortcut ) {
    QObject::connect(shortcut, &QShortcut::activated,
        action, &QAction::trigger);
  }

  return action;
}


template<class Obj, typename Fn>
QToolButton* createToolButton(const QIcon & icon, const QString & text, const QString & tooltip,
    Obj * receiver, Fn onclick,
    QShortcut * shortcut = nullptr)
{
  QToolButton * tb = new QToolButton();
  tb->setToolButtonStyle(Qt::ToolButtonIconOnly);
  tb->setIcon(icon);
  tb->setText(text);
  tb->setToolTip(tooltip);

  QObject::connect(tb, &QToolButton::clicked,
      receiver, onclick);

  if( shortcut ) {
    QObject::connect(shortcut, &QShortcut::activated,
        tb, &QToolButton::click);
  }

  return tb;
}

template<class Fn>
QToolButton* createToolButton(const QIcon & icon, const QString & text, const QString & tooltip, Fn && onclicked)
{
  QToolButton *tb = new QToolButton();
  tb->setToolButtonStyle(Qt::ToolButtonIconOnly);
  tb->setIcon(icon);
  tb->setText(text);
  tb->setToolTip(tooltip);

  QObject::connect(tb, &QToolButton::clicked,
      [tb, onclicked]() {
        onclicked(tb);
      });

  return tb;
}

inline QWidget* addStretch(QToolBar * toolbar)
{
  QWidget *stretch = new QWidget();
  stretch->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  toolbar->addWidget(stretch);
  return stretch;
}


inline QScrollArea* createScrollableWrap(QWidget * w, QWidget * parent = nullptr)
{
  QScrollArea *scrollArea = new QScrollArea(parent ? parent : w->parentWidget());
  scrollArea->setWidgetResizable(true);
  scrollArea->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
  scrollArea->setFrameShape(QFrame::NoFrame);
  scrollArea->setWidget(w);
  return scrollArea;
}


#endif /* __createAction_h__ */
