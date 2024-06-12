/*
 * QDSOImageView.cc
 *
 *  Created on: Jun 12, 2024
 *      Author: amyznikov
 */

#include "QDSOImageView.h"

namespace qdso {

QDSOImageView::QDSOImageView(QWidget * parent) :
    Base(parent)
{
  connect(this, &ThisClass::redrawImage,
      this, &ThisClass::updateImage,
      Qt::QueuedConnection);
}


void QDSOImageView::showImage(const cv::Mat & image, bool make_copy)
{
  if ( make_copy)  {
    image.copyTo(Base::inputImage_);
  }
  else {
    Base::inputImage_ = image;
  }

  Q_EMIT redrawImage();
}

QDSOImageViewDock::QDSOImageViewDock(const QString & title, QWidget * parent, QDSOImageView * view, Qt::WindowFlags flags) :
    Base(title, parent, view, flags)
{

}

QDSOImageView * QDSOImageViewDock::view() const
{
  return dynamic_cast<QDSOImageView * >(Base::widget());
}

} // namespace qdso
