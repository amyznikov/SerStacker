/*
 * QCustomDock.cc
 *
 *  Created on: Mar 19, 2019
 *      Author: amyznikov
 */

#include "QCustomDock.h"
#include <gui/widgets/style.h>
#include <core/debug.h>

#define ICON_dock       ":/qcustomdock/icons/dock.png"
#define ICON_close      ":/qcustomdock/icons/close.png"

static const char * borderless_style()
{
  static const char borderless_style_light[] = ""
      "QToolButton { border: none; }"
      "QToolButton:checked { background-color: black; border: none; }"
      ;

  static const char borderless_style_dark[] = ""
      "QToolButton { border: none; }"
      "QToolButton:checked { background-color: #C0C0C0; }"
      "QToolButton[popupMode=\"1\"] { /* only for MenuButtonPopup */"
      "    padding: 0px;"
      "    padding-right: 14px;"
      "}"
      ;

  //

  return iconStyleSelector().contains("light", Qt::CaseInsensitive) ?
    borderless_style_light : borderless_style_dark;

}

//
//static QWidget* addStretch(QToolBar * toolbar)
//{
//  QWidget *stretch = new QWidget();
//  stretch->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
//  toolbar->addWidget(stretch);
//  return stretch;
//}
//

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

//  QVBoxLayout * l = new QVBoxLayout(this);
//  l->setContentsMargins(0, 0, 0, 0);
//
//  tb_ = new  QToolBar(this);
//  tb_->setStyleSheet(borderless_style());
//  tb_->setIconSize(QSize(16, 16));
//  tb_->setContentsMargins(0, 0, 0, 0);
//
//  l->addWidget(tb_);

  layout_ = new QHBoxLayout(this);

  layout_->addWidget(title_ = new QCustomDockTitleBarLabel(title), 1000, Qt::AlignLeft);
  //tb_->addWidget(title_ = new QCustomDockTitleBarLabel(title));
  title_->setTextFormat(Qt::RichText);

//  stretch_ = addStretch(tb_);
//  separator_ = tb_->addSeparator();

  layout_->addWidget(float_button_ = new QToolButton(), 1, Qt::AlignRight);
  //tb_->addWidget(float_button_ = new QToolButton());
  float_button_->setStyleSheet(borderless_style());
  float_button_->setFocusPolicy(Qt::NoFocus);
  float_button_->setIconSize(QSize(16,16));
  float_button_->setIcon(getIcon(ICON_dock));

  connect(float_button_, &QToolButton::clicked,
      [this]() {
        QDockWidget * parent = qobject_cast<QDockWidget * >(parentWidget());
        if ( parent ) {
          parent->setFloating(!parent->isFloating());
        }
      });


  layout_->addWidget(close_button_ = new QToolButton(), 1, Qt::AlignRight);
  //tb_->addWidget(close_button_ = new QToolButton());
  close_button_->setStyleSheet(borderless_style());
  close_button_->setFocusPolicy(Qt::NoFocus);
  close_button_->setIconSize(QSize(16,16));
  close_button_->setIcon(getIcon(ICON_close));

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

QToolButton* QCustomDockTitleBar::addButton(QToolButton * button)
{
  if( button->styleSheet().isEmpty() ) {
    button->setStyleSheet(borderless_style());
  }

  if( button->popupMode() == QToolButton::ToolButtonPopupMode::MenuButtonPopup ) {
    button->setIconSize(QSize(32, 16));
  }
  else {
    button->setIconSize(QSize(16, 16));
  }

  button->setFocusPolicy(Qt::NoFocus);
  layout_->insertWidget(1, button, 1, Qt::AlignRight);
  //tb_->insertWidget(separator_, button);

  return button;
}

QToolButton* QCustomDockTitleBar::addButton(QAction * action)
{
  QToolButton *button = nullptr;

  if( action ) {
    QMenu *menu = action->menu();
    if( !menu ) {

      button = new QToolButton();
      button->setStyleSheet(borderless_style());
      button->setIconSize(QSize(16, 16));
      button->setFocusPolicy(Qt::NoFocus);
      button->setCheckable(action->isCheckable());
      button->setChecked(action->isChecked());
      button->setDefaultAction(action);
      layout_->insertWidget(1, button, 1, Qt::AlignRight);
      //tb_->insertWidget(separator_, button);
    }
    else {

      static const char borderless_style[] = ""
          "QToolButton { border: none; } "
          "QToolButton::menu-indicator { image: none; }"
          "";

      button = new QToolButton();
      button->setStyleSheet(borderless_style);
      button->setToolButtonStyle(Qt::ToolButtonStyle::ToolButtonIconOnly);
      button->setIconSize(QSize(32, 16));
      button->setFocusPolicy(Qt::NoFocus);
      button->setPopupMode(QToolButton::InstantPopup);
      button->setMenu(menu);
      button->setDefaultAction(menu->defaultAction());
      button->setEnabled(action->isEnabled());
      button->setIcon(action->icon());

      layout_->insertWidget(1, button, 1, Qt::AlignRight);
      //tb_->insertWidget(separator_, button);

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
  //return tb_->minimumSize();
}

QSize QCustomDockTitleBar::sizeHint() const
{
  return layout_->sizeHint();
  //return tb_->sizeHint();
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


