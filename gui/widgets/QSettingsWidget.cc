/*
 * QSettingsWidget.cc
 *
 *  Created on: Jan 11, 2019
 *      Author: amyznikov
 */

#include "QSettingsWidget.h"
#include <gui/qfeature2d/QFeature2dOptions.h>

#define ICON_menu   ":/gui/icons/menu"

QSettingsWidget::QSettingsWidget(QWidget * parent) :
  Base(parent)
{
  setFrameShape(NoFrame);
  form = new QFormLayout(this);

  connect(this, &ThisClass::parameterChanged,
      this, &ThisClass::enablecontrols);
}

void QSettingsWidget::set_mutex(std::mutex * mtx)
{
  this->_mtx = mtx;
}

std::mutex * QSettingsWidget::mutex()
{
  return this->_mtx;

}

void QSettingsWidget::lock()
{
  if ( _mtx ) {
    _mtx->lock();
  }
}

void QSettingsWidget::unlock()
{
  if ( _mtx ) {
    _mtx->unlock();
  }
}

bool QSettingsWidget::updatingControls()
{
  return _updatingControls > 0;
}

void QSettingsWidget::setUpdatingControls(bool v)
{
  if ( v ) {
    ++_updatingControls;
  }
  else if ( _updatingControls && --_updatingControls < 0 ) {
    _updatingControls = 0;
  }
}

void QSettingsWidget::updateControls()
{
  setUpdatingControls(true);
  Q_EMIT populatecontrols();
  Q_EMIT enablecontrols();
  setUpdatingControls(false);
}

void QSettingsWidget::loadSettings(const QString & prefix)
{
  const QSettings settings;
  loadSettings(settings, prefix);
}

void QSettingsWidget::loadSettings(const QSettings & settings, const QString & prefix)
{
  onload(settings, prefix);
}

void QSettingsWidget::saveSettings(const QString & prefix)
{
  QSettings settings;
  saveSettings(settings, prefix);
}

void QSettingsWidget::saveSettings(QSettings & settings, const QString & prefix)
{
  onsave(settings, prefix);
}

void QSettingsWidget::onload(const QSettings & settings, const QString & prefix)
{
}

void QSettingsWidget::onsave(QSettings & settings, const QString & prefix)
{
}
