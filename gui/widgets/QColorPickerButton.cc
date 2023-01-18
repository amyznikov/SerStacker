/*
 * QColorPickerButton.cc
 *
 *  Created on: Dec 19, 2022
 *      Author: amyznikov
 */

#include "QColorPickerButton.h"

QColorPickerButton::QColorPickerButton(QWidget * parent) :
  Base(parent)
{
  connect(this, &Base::clicked,
      this, &ThisClass::onClicked);
}

QColorPickerButton::QColorPickerButton(const QColor & color, QWidget * parent) :
  Base(parent),
  color_(color)
{
}

void QColorPickerButton::setColor(const QColor & color)
{
  this->color_ = color;
  updateIcon();
}

const QColor & QColorPickerButton::color() const
{
  return this->color_;
}

void QColorPickerButton::resizeEvent(QResizeEvent *event)
{
  Base::resizeEvent(event);
  updateIcon();
}

void QColorPickerButton::updateIcon()
{
  const QSize buttonSize =
      this->size();

  const QSize iconSize =
      QSize(buttonSize.width() - 4,
          buttonSize.height() - 4);

  if( iconSize.width() > 0 && iconSize.height() > 0 ) {

    QPixmap pixmap(iconSize);
    pixmap.fill(color_);

    QPainter p(&pixmap);
    p.setPen(QPen(Qt::gray));
    p.drawRect(1, 1, iconSize.width() - 2, iconSize.height() - 2);
    setIcon(QIcon(pixmap));
  }

  update();
}

void QColorPickerButton::onClicked()
{
  QColorDialog dialog(color_, this);

  if( dialog.exec() == QDialog::Accepted ) {

    QColor newColor =
        dialog.currentColor();

    if( newColor != color_ ) {

      color_ = newColor;

      updateIcon();

      Q_EMIT colorSelected();
    }
  }
}
