/*
 * QCustomDock.cc
 *
 *  Created on: Mar 19, 2019
 *      Author: amyznikov
 */

#include "QCustomDock.h"
#include <core/debug.h>

#define ICON_dock       "dock"
#define ICON_dock_close "close"


static const char borderless_style[] = ""
    "QToolButton { border: none; }"
    "QToolButton:checked { background-color: darkgray; border: none; }"
    ;


static QIcon getIcon(const QString & name)
{
  return QIcon(QString(":/qcustomdock/icons/%1").arg(name));
}

///////////////////////////////////////////////////////////////////////////////

QCustomDockTitleBarLabel::QCustomDockTitleBarLabel(const QString & title)
    : Base(title)
{
}

void QCustomDockTitleBarLabel::mousePressEvent(QMouseEvent *event)
{
  event->ignore();
}
void QCustomDockTitleBarLabel::mouseReleaseEvent(QMouseEvent *event)
{
  event->ignore();
}

void QCustomDockTitleBarLabel::mouseDoubleClickEvent(QMouseEvent *event)
{
  event->ignore();
}

void QCustomDockTitleBarLabel::mouseMoveEvent(QMouseEvent *event)
{
  event->ignore();
}

#if QT_CONFIG(wheelevent)
void QCustomDockTitleBarLabel::wheelEvent(QWheelEvent *event)
{
  event->ignore();
}
#endif



QCustomDockTitleBar::QCustomDockTitleBar(const QString & title)
{
  Q_INIT_RESOURCE(qcustomdock_resources);

  layout_ = new QHBoxLayout(this);

  layout_->addWidget(title_ = new QCustomDockTitleBarLabel(title), 1000, Qt::AlignLeft);
  title_->setTextFormat(Qt::RichText);

  layout_->addWidget(float_button_ = new QToolButton(), 1, Qt::AlignRight);
  float_button_->setFocusPolicy(Qt::NoFocus);
  float_button_->setIconSize(QSize(16,16));
  float_button_->setIcon(getIcon(ICON_dock));
  float_button_->setStyleSheet(borderless_style);

  connect(float_button_, &QToolButton::clicked,
      [this]() {
        QDockWidget * parent = qobject_cast<QDockWidget * >(parentWidget());
        if ( parent ) {
          parent->setFloating(!parent->isFloating());
        }
      });


  layout_->addWidget(close_button_ = new QToolButton(), 1, Qt::AlignRight);
  close_button_->setFocusPolicy(Qt::NoFocus);
  close_button_->setIconSize(QSize(16,16));
  close_button_->setIcon(getIcon(ICON_dock_close));
  close_button_->setStyleSheet(borderless_style);

  connect(close_button_, &QToolButton::clicked,
      [this]() {
        QDockWidget * parent = qobject_cast<QDockWidget * >(parentWidget());
        if ( parent ) {
          parent->close();
        }
      });
}


QCustomDockTitleBarLabel * QCustomDockTitleBar::titleLabel() const
{
  return title_;
}


QToolButton * QCustomDockTitleBar::addButton(QToolButton * button)
{
  button->setIconSize(QSize(16,16));

  if ( button->styleSheet().isEmpty() ) {
    button->setStyleSheet(borderless_style);
  }

  button->setFocusPolicy(Qt::NoFocus);
  layout_->insertWidget(1, button, 1, Qt::AlignRight);

  return button;
}

QToolButton* QCustomDockTitleBar::addButton(QAction * action)
{
  QToolButton *button = nullptr;

  if( action ) {
    QMenu *menu = action->menu();
    if( !menu ) {

      button = new QToolButton();
      button->setIconSize(QSize(16, 16));
      button->setFocusPolicy(Qt::NoFocus);
      button->setCheckable(action->isCheckable());
      button->setChecked(action->isChecked());
      button->setStyleSheet(borderless_style);
      button->setDefaultAction(action);
      layout_->insertWidget(1, button, 1, Qt::AlignRight);
    }
    else {

      static const char borderless_style[] = ""
          "QToolButton { border: none; } "
          "QToolButton::menu-indicator { image: none; }"
          "";

      button = new QToolButton();
      button->setToolButtonStyle(Qt::ToolButtonStyle::ToolButtonIconOnly);
      button->setIconSize(QSize(16, 16));
      button->setFocusPolicy(Qt::NoFocus);
      button->setStyleSheet(borderless_style);
      button->setPopupMode(QToolButton::InstantPopup);
      button->setMenu(menu);
      button->setDefaultAction(menu->defaultAction());
      button->setEnabled(action->isEnabled());
      button->setIcon(action->icon());

      layout_->insertWidget(1, button, 1, Qt::AlignRight);

      connect(button, &QToolButton::triggered,
          button, &QToolButton::setDefaultAction);

      connect(action, &QAction::changed,
          [action, button]() {
            button->setEnabled(action->isEnabled());
          });

    }
  }

  return button;
}


QToolButton * QCustomDockTitleBar::addButton(const QIcon & icon, const QString & tooltip)
{
  QToolButton * tb = new QToolButton();
  tb->setIcon(icon);
  if ( !tooltip.isEmpty() ) {
    tb->setToolTip(tooltip);
  }
  addButton(tb);
  return tb;
}


QToolButton * QCustomDockTitleBar::addButton(const QString & icon, const QString & tooltip)
{
  return addButton(getIcon(icon), tooltip);
}


QSize QCustomDockTitleBar::minimumSizeHint() const
{
  return layout_->minimumSize();
}

QSize QCustomDockTitleBar::sizeHint() const
{
  return layout_->sizeHint();
}



QCustomDockWidget::QCustomDockWidget(const QString &title, QWidget *parent, QWidget * view, Qt::WindowFlags flags)
    : Base(title, parent, flags)
{
  Base::setTitleBarWidget(new QCustomDockTitleBar(title));
  Base::setWidget(view);

  setAllowedAreas(Qt::AllDockWidgetAreas);

  QObject::connect(toggleViewAction(), &QAction::triggered,
      [this] (bool checked) {
        if ( checked ) {
          raise();
        }
      });


}

QCustomDockTitleBar * QCustomDockWidget::titleBar() const
{
  return qobject_cast<QCustomDockTitleBar*>(titleBarWidget());
}


