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

  vbox_ = new QVBoxLayout(this);
  imageLevelsConfigWidget_ = new QMtfControl(this);

  vbox_->addWidget(imageLevelsConfigWidget_);

  connect(imageLevelsConfigWidget_, &QMtfControl::mtfChanged,
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
  imageLevelsConfigWidget_->setInputImage(image, mask);

}

void QMtfControlDialogBox::setDisplayImage(cv::InputArray image, cv::InputArray mask)
{
  imageLevelsConfigWidget_->setOutputImage(image, mask);
}


void QMtfControlDialogBox::setMtf(const c_pixinsight_midtones_transfer_function::ptr & mtf)
{
  return imageLevelsConfigWidget_->setMtf(mtf);
}

const c_pixinsight_midtones_transfer_function::ptr & QMtfControlDialogBox::mtf() const
{
  return imageLevelsConfigWidget_->mtf();
}
