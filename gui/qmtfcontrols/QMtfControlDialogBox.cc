/*
 * QMtfControlDialogBox.cc
 *
 *  Created on: Dec 11, 2020
 *      Author: amyznikov
 */

#include "QMtfControlDialogBox.h"

#define ICON_histogram                "histogram"

static QIcon getIcon(const QString & name)
{
  return QIcon(QString(":/qmtfcontrols/icons/%1").arg(name));
}

QMtfControlDialogBox::QMtfControlDialogBox(QWidget * parent)
  : Base(parent)
{
  Q_INIT_RESOURCE(qmtfcontrols_resources);

  setWindowIcon(getIcon(ICON_histogram));
  setWindowTitle("Adjust Display Levels ...");

  vbox_ = new QVBoxLayout(this);
  mtfControl_ = new QMtfControl(this);

  vbox_->addWidget(mtfControl_);

  connect(mtfControl_, &QMtfControl::mtfChanged,
      this, &ThisClass::mtfChanged);
}

void QMtfControlDialogBox::showEvent(QShowEvent *e)
{
  if ( !lastWidnowSize_.isEmpty() ) {
    Base::move(lastWidnowPos_);
    Base::resize(lastWidnowSize_);
  }

  Base::showEvent(e);
  emit visibilityChanged(isVisible());
}

void QMtfControlDialogBox::hideEvent(QHideEvent *e)
{
  lastWidnowSize_ = this->size();
  lastWidnowPos_ = this->pos();
  Base::hideEvent(e);
  emit visibilityChanged(isVisible());
}

void QMtfControlDialogBox::setInputImage(cv::InputArray image, cv::InputArray mask)
{
  mtfControl_->setInputImage(image, mask);

}

void QMtfControlDialogBox::setDisplayImage(cv::InputArray image, cv::InputArray mask)
{
  mtfControl_->setOutputImage(image, mask);
}


void QMtfControlDialogBox::setMtf(const c_pixinsight_mtf::sptr & mtf)
{
  return mtfControl_->setMtf(mtf);
}

const c_pixinsight_mtf::sptr & QMtfControlDialogBox::mtf() const
{
  return mtfControl_->mtf();
}
