/*
 * QFFMPEGCameraUrlWidget.cc
 *
 *  Created on: Mar 17, 2023
 *      Author: amyznikov
 */


#include "QFFMPEGCameraUrlWidget.h"

namespace serimager {

QFFMPEGCameraUrlWidget::QFFMPEGCameraUrlWidget(QWidget * parent) :
    Base(parent)
{
  hbox = new QHBoxLayout(this);
  hbox->setContentsMargins(0, 0, 0, 0);

  hbox->addWidget(url_ctl = new QLineEditBox());
  hbox->addWidget(browse_ctl = new QToolButton());

  url_ctl->setPlaceholderText("Stream URL");
  browse_ctl->setText("File...");

  connect(url_ctl, &QLineEditBox::textChanged,
      this, &ThisClass::urlChanged);

}

void QFFMPEGCameraUrlWidget::setUrl(const QString & v)
{
  url_ctl->setText(v);
}

QString QFFMPEGCameraUrlWidget::url() const
{
  return url_ctl->text();
}

} /* namespace serimager */
