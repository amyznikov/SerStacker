/*
 * QMtfDialogBox.cc
 *
 *  Created on: Dec 11, 2020
 *      Author: amyznikov
 */

#include "QMtfDialogBox.h"

#define ICON_histogram                "histogram"

static QIcon getIcon(const QString & name)
{
  return QIcon(QString(":/qmtf/icons/%1").arg(name));
}

QMtfDialogBox::QMtfDialogBox(QWidget * parent)
  : Base(parent)
{
  Q_INIT_RESOURCE(qmtf_resources);

  setWindowIcon(getIcon(ICON_histogram));
  setWindowTitle("Adjust Display Levels ...");

  vbox_ = new QVBoxLayout(this);
  mtfControl_ = new QMtfControl(this);

  vbox_->addWidget(mtfControl_);
}

void QMtfDialogBox::showEvent(QShowEvent *e)
{
  if ( !lastWidnowSize_.isEmpty() ) {
    Base::move(lastWidnowPos_);
    Base::resize(lastWidnowSize_);
  }

  Base::showEvent(e);
  emit visibilityChanged(isVisible());
}

void QMtfDialogBox::hideEvent(QHideEvent *e)
{
  lastWidnowSize_ = this->size();
  lastWidnowPos_ = this->pos();
  Base::hideEvent(e);
  emit visibilityChanged(isVisible());
}

void QMtfDialogBox::setInputImage(cv::InputArray image, cv::InputArray mask)
{
  mtfControl_->setInputImage(image, mask);
}

void QMtfDialogBox::updateOutputHistogramLevels()
{
  mtfControl_->updateOutputHistogramLevels();
}

void QMtfDialogBox::setDisplayFunction(QMtfDisplayFunction * displayFunction)
{
  mtfControl_->setDisplayFunction(displayFunction);
}

QMtfDisplayFunction * QMtfDialogBox::displayFunction() const
{
  return mtfControl_->displayFunction();
}
