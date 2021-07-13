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

void QSettingsWidget::LOCK()
{
  if ( mtx_ ) {
    mtx_->lock();
  }
}

void QSettingsWidget::UNLOCK()
{
  if ( mtx_ ) {
    mtx_->unlock();
  }
}

bool QSettingsWidget::updatingControls()
{
  return updatingControls_;
}

void QSettingsWidget::setUpdatingControls(bool v)
{
  updatingControls_ = v;
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
}
