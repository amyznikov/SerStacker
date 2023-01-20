/*
 * QImageEditor.cc
 *
 *  Created on: May 2, 2022
 *      Author: amyznikov
 */

#include "QImageEditor.h"

namespace qserstacker {


QImageEditor::QImageEditor(QWidget * parent) :
  Base(parent),
  mtfDisplayFunction_(this, "QImageEditor")
{
  Base::setDisplayFunction(&mtfDisplayFunction_);

  connect(&mtfDisplayFunction_, &QImageViewMtfDisplayFunction::parameterChanged,
      this, &ThisClass::updateDisplay);

  connect(this, &ThisClass::displayImageChanged,
      &mtfDisplayFunction_, &QMtfDisplay::displayImageChanged,
      Qt::QueuedConnection);

  scene()->setBackgroundBrush(Qt::darkGray);

  createRoiRectShape();

}

QImageViewMtfDisplayFunction * QImageEditor::mtfDisplayFunction()
{
  return &mtfDisplayFunction_;
}

const QImageViewMtfDisplayFunction * QImageEditor::mtfDisplayFunction() const
{
  return &mtfDisplayFunction_;
}

QGraphicsRectShape * QImageEditor::roiRectShape() const
{
  return roiRectShape_;
}

void QImageEditor::createRoiRectShape()
{
  if( !roiRectShape_ ) {

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

    roiRectShape_ = new QGraphicsRectShape(rect);
    roiRectShape_->setResizable(true);
    roiRectShape_->setFlag(QGraphicsItem::ItemIsMovable, true);
    roiRectShape_->setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
    roiRectShape_->setCosmeticPen(Qt::red);
    roiRectShape_->setVisible(false);
    roiRectShape_->setFixOnSceneCenter(true);
    scene_->addItem(roiRectShape_);
  }

}


} /* namespace qserstacker */
