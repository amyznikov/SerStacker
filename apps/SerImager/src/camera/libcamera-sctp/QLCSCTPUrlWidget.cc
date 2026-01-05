/*
 * QLCSCTPUrlWidget.cc
 *
 *  Created on: Jan 1, 2026
 *      Author: amyznikov
 */

#include "QLCSCTPUrlWidget.h"

namespace serimager {

QLCSCTPUrlWidget::QLCSCTPUrlWidget(QWidget * parent) :
    Base(parent)
{
  hbox = new QHBoxLayout(this);
  hbox->setContentsMargins(0, 0, 0, 0);

  hbox->addWidget(url_ctl = new QLineEditBox());
  //hbox->addWidget(browse_ctl = new QToolButton());

  url_ctl->setPlaceholderText("address:port");
  //browse_ctl->setText("File...");

  connect(url_ctl, &QLineEditBox::textChanged,
      this, &ThisClass::urlChanged);
}

void QLCSCTPUrlWidget::setUrl(const QString & v)
{
  url_ctl->setText(v);
}

QString QLCSCTPUrlWidget::url() const
{
  return url_ctl->text();
}

} /* namespace serimager */
