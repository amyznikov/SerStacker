/*
 * QSettingsWidget.cc
 *
 *  Created on: Jan 11, 2019
 *      Author: amyznikov
 */

#include "QSettingsWidget.h"

QSettingsWidget::QSettingsWidget(const QString & prefix, QWidget * parent)
  : Base(parent), PREFIX(prefix)
{
  setFrameShape(NoFrame);
  form = new QFormLayout(this);
}

void QSettingsWidget::setSettingsPrefix(const QString & v)
{
  PREFIX = v;
}

const QString & QSettingsWidget::settingsPrefix() const
{
  return PREFIX;
}


void QSettingsWidget::set_mutex(std::mutex * mtx)
{
  this->mtx_ = mtx;
}

std::mutex * QSettingsWidget::mutex()
{
  return this->mtx_;

}

void QSettingsWidget::lock()
{
  if ( mtx_ ) {
    mtx_->lock();
  }
}

void QSettingsWidget::unlock()
{
  if ( mtx_ ) {
    mtx_->unlock();
  }
}

bool QSettingsWidget::updatingControls()
{
  return updatingControls_ > 0;
}

void QSettingsWidget::setUpdatingControls(bool v)
{
  if ( v ) {
    ++updatingControls_;
  }
  else if ( updatingControls_ && --updatingControls_ < 0 ) {
    updatingControls_ = 0;
  }
}

void QSettingsWidget::updateControls()
{
  setUpdatingControls(true);
  onupdatecontrols();
  setUpdatingControls(false);
}

void QSettingsWidget::loadParameters()
{
  QSettings settings;
  loadSettings(settings);
}

void QSettingsWidget::loadSettings(QSettings & settings)
{
  onload(settings);
  updateControls();
}


void QSettingsWidget::onload(QSettings & /*settings*/)
{
}

void QSettingsWidget::onupdatecontrols()
{
  populatecontrols();
}
