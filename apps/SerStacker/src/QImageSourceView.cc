/*
 * QInputImageSourceView.cc
 *
 *  Created on: Dec 4, 2023
 *      Author: amyznikov
 */

#include "QImageSourceView.h"

namespace serstacker {



QImageSourceView::QImageSourceView(QWidget * parent) :
  Base(parent)
{
//  Base::setDisplayFunction(&mtfDisplayFunction_);
//
//  connect(&mtfDisplayFunction_, &QImageViewMtfDisplayFunction::parameterChanged,
//      this, &ThisClass::updateDisplay);
//
//  connect(this, &ThisClass::displayImageChanged,
//      &mtfDisplayFunction_, &QMtfDisplay::displayImageChanged,
//      Qt::QueuedConnection);

  scene()->setBackgroundBrush(Qt::darkGray);

  createRoiShape();
}
//
//QCloudViewMtfDisplayFunction * QInputImageSourceView::mtfDisplayFunction()
//{
//  return &mtfDisplayFunction_;
//}
//
//const QCloudViewMtfDisplayFunction * QInputImageSourceView::mtfDisplayFunction() const
//{
//  return &mtfDisplayFunction_;
//}

QGraphicsRectShape * QImageSourceView::roiShape() const
{
  return _roiShape;
}

void QImageSourceView::createRoiShape()
{
  if( !_roiShape ) {

    QRectF rect;

    if( _currentImage.empty() ) {
      rect.setRect(0, 0, 256, 256);
    }
    else {

      rect.setRect(0, 0, _currentImage.cols, _currentImage.rows);

      if( rect.width() > 400 ) {
        rect.setX((rect.left() + rect.right()) / 2 - 200);
        rect.setWidth(400);
      }

      if( rect.height() > 400 ) {
        rect.setY((rect.top() + rect.bottom()) / 2 - 200);
        rect.setHeight(400);
      }
    }

    _roiShape = new QGraphicsRectShape(rect);
    _roiShape->setResizable(true);
    _roiShape->setSnapToPixelGrid(true);
    _roiShape->setFlag(QGraphicsItem::ItemIsMovable, true);
    _roiShape->setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
    _roiShape->setCosmeticPen(Qt::red);
    _roiShape->setVisible(false);
    // roiShape_->setFixOnSceneCenter(true);

    _roiShape->setToolTip("ROI rectangle:\n"
        "Shift + LeftMouseButton for move\n"
        "Ctrl + LeftMouseButton for resize\n");

    _scene->addItem(_roiShape);
  }

}


} /* namespace serstacker */
