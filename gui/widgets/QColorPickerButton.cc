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
  setAutoDefault(false);
  setDefault(false);

  connect(this, &Base::clicked,
      this, &ThisClass::onClicked);
}

QColorPickerButton::QColorPickerButton(const QColor & color, QWidget * parent) :
  Base(parent),
  color_(color)
{
  setAutoDefault(false);
  setDefault(false);

  connect(this, &Base::clicked,
      this, &ThisClass::onClicked);

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
      QSize( std::min(32, buttonSize.width() - 2),
          std::min(32, buttonSize.height() - 2));

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
  dialog.setOption(QColorDialog::DontUseNativeDialog, true);

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
