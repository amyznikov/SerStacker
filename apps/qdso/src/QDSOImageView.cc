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
