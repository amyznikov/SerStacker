/*
 * QScaleSelectionButton.cc
 *
 *  Created on: Mar 8, 2020
 *      Author: amyznikov
 */

#include "QScaleSelectionButton.h"

#define ICON_zoom "zoom"

static QIcon getIcon(const QString & name)
{
  return QIcon(QString(":/gui/icons/%1").arg(name));
}

class QScaleSelectionButton::QPopupSlider
  : public QDialog
{
public:
  typedef QPopupSlider ThisClass;
  typedef QDialog Base;

  QPopupSlider(QScaleSelectionButton * parent = Q_NULLPTR);

  void setRange(int min, int max);

  void setValue(int);
  int value() const;

  void showPopup(QPoint pos);


protected:
  void onValueChanged(int v);

protected:
  QVBoxLayout * vbox_ = Q_NULLPTR;
  QSlider * slider_ = Q_NULLPTR;
  QLabel * lb_ = Q_NULLPTR;
  bool enableFeedback_ = true;
};


QScaleSelectionButton::QPopupSlider::QPopupSlider(QScaleSelectionButton * parent)
    : Base(parent)
{
  setWindowFlags(Qt::Window | Qt::FramelessWindowHint | Qt::Popup);

  vbox_ = new QVBoxLayout(this);

  lb_ = new QLabel();
  slider_ = new QSlider();
  slider_->setOrientation(Qt::Vertical);

  vbox_->addWidget(lb_);
  vbox_->addWidget(slider_);

  connect(slider_, &QSlider::valueChanged,
      parent, &QScaleSelectionButton::scaleChanged);
}

void QScaleSelectionButton::QPopupSlider::onValueChanged(int value)
{
  lb_->setText(QString("%1").arg(value));
  enableFeedback_ = false;
  emit ((QScaleSelectionButton*) parent())->scaleChanged(value);
  enableFeedback_ = true;
}

void QScaleSelectionButton::QPopupSlider::setRange(int min, int max)
{
  slider_->setRange(min, max);
}

void QScaleSelectionButton::QPopupSlider::setValue(int value)
{
  if ( enableFeedback_ ) {
    slider_->setValue(value);
  }
}

int QScaleSelectionButton::QPopupSlider::value() const
{
  return slider_->value();
}

void QScaleSelectionButton::QPopupSlider::showPopup(QPoint pos)
{
#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
  QScreen * currentScreen = QGuiApplication::screenAt(pos);
  if ( !currentScreen && !(currentScreen = QGuiApplication::primaryScreen())) {
    return;
  }
#else
  QScreen * currentScreen = QGuiApplication::primaryScreen();
  if ( !currentScreen ) {
    return;
  }
#endif

  QRect rc = currentScreen->geometry();
  QSize s = sizeHint();

  if ( pos.x() + s.width() > rc.right() ) {
    pos.setX(rc.right() - s.width() - 8 );
  }
  if ( pos.x() < rc.left() ) {
    pos.setX(rc.left() + 8 );
  }

  if ( pos.y() + s.height() > rc.bottom() ) {
    pos.setY(rc.bottom() - s.height() - 8 );
  }
  if ( pos.y() < rc.top() ) {
    pos.setY(rc.top() + 8 );
  }

  Base::move(pos);
  Base::exec();
}





QScaleSelectionButton::QScaleSelectionButton(QWidget * parent)
  : Base(parent)
{
  Q_INIT_RESOURCE(gui_resources);

  setToolButtonStyle(Qt::ToolButtonIconOnly);
  setIconSize(QSize(16, 16));
  setIcon(getIcon(ICON_zoom));
  setText("Zoom");
  setToolTip("Scale image (Ctrl+'+', Ctrl+'-')");

  popup_ = new QPopupSlider(this);
  popup_->setMinimumWidth(32);
  popup_->setMinimumHeight(256);
  popup_->setRange(-20, 20);
  popup_->setValue(0);

  connect(this, &QToolButton::clicked, [this]() {
    popup_->showPopup(mapToGlobal(QPoint(0, this->height() + 2)));
  });

}

int QScaleSelectionButton::currentScale() const
{
  return popup_->value();
}

void QScaleSelectionButton::setScaleRange(int min, int max)
{
  return popup_->setRange(min, max);
}

void QScaleSelectionButton::setCurrentScale(int scale)
{
  return popup_->setValue(scale);
}
