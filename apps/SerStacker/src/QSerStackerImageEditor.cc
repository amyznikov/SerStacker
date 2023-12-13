/*
 * QSerStackerImageEditor.cc
 *
 *  Created on: May 2, 2022
 *      Author: amyznikov
 */

#include "QSerStackerImageEditor.h"

namespace serstacker {


QSerStackerImageEditor::QSerStackerImageEditor(QWidget * parent) :
  Base(parent),
  mtfDisplayFunction_(this, "QImageEditor")
{
  Base::setDisplayFunction(&mtfDisplayFunction_);

  connect(&mtfDisplayFunction_, &QMtfDisplay::displayTypeChanged,
      &mtfDisplayFunction_, &QMtfDisplay::parameterChanged);

  connect(&mtfDisplayFunction_, &QImageViewMtfDisplayFunction::parameterChanged,
      this, &ThisClass::updateDisplay);

  connect(this, &ThisClass::displayImageChanged,
      &mtfDisplayFunction_, &QMtfDisplay::displayImageChanged,
      Qt::QueuedConnection);

  scene()->setBackgroundBrush(Qt::darkGray);

  createRoiShape();
}

QImageViewMtfDisplayFunction * QSerStackerImageEditor::mtfDisplayFunction()
{
  return &mtfDisplayFunction_;
}

const QImageViewMtfDisplayFunction * QSerStackerImageEditor::mtfDisplayFunction() const
{
  return &mtfDisplayFunction_;
}

QGraphicsRectShape * QSerStackerImageEditor::roiShape() const
{
  return roiShape_;
}

void QSerStackerImageEditor::createRoiShape()
{
  if( !roiShape_ ) {

    QRectF rect;

    if( currentImage_.empty() ) {
      rect.setRect(0, 0, 256, 256);
    }
    else {

      rect.setRect(0, 0, currentImage_.cols, currentImage_.rows);

      if( rect.width() > 400 ) {
        rect.setX((rect.left() + rect.right()) / 2 - 200);
        rect.setWidth(400);
      }

      if( rect.height() > 400 ) {
        rect.setY((rect.top() + rect.bottom()) / 2 - 200);
        rect.setHeight(400);
      }
    }

    roiShape_ = new QGraphicsRectShape(rect);
    roiShape_->setResizable(true);
    roiShape_->setSnapToPixelGrid(true);
    roiShape_->setFlag(QGraphicsItem::ItemIsMovable, true);
    roiShape_->setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
    roiShape_->setCosmeticPen(Qt::red);
    roiShape_->setVisible(false);
    // roiShape_->setFixOnSceneCenter(true);

    roiShape_->setToolTip("ROI rectangle:\n"
        "Shift + LeftMouseButton for move\n"
        "Ctrl + LeftMouseButton for resize\n");

    scene_->addItem(roiShape_);
  }

}


} /* namespace serstacker */
