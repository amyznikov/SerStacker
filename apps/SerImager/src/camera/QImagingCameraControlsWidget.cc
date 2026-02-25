/*
 * QImagerSettingsWidget.cc
 *
 *  Created on: Dec 21, 2022
 *      Author: amyznikov
 */

#include "QImagingCameraControlsWidget.h"

#include <core/debug.h>

namespace serimager {

namespace {

QScrollArea* createScrollableWrap(QWidget * w, QWidget * parent = nullptr)
{
  QScrollArea *scrollArea = new QScrollArea(parent ? parent : w->parentWidget());
  scrollArea->setWidgetResizable(true);
  scrollArea->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
  scrollArea->setFrameShape(QFrame::NoFrame);
  scrollArea->setWidget(w);
  return scrollArea;
}


}

QImagingCameraControlsWidget::QImagingCameraControlsWidget(QWidget * parent) :
    Base(parent)
{
  _layout = new QVBoxLayout(this);

  _layout->addWidget(cameraSelection_ctl =
      new QCameraSelectionWidget(this));

  _layout->addWidget(createScrollableWrap(
      settings_ctl = new QSettingsWidget(this),
      this));

  settings_ctl->add_expandable_groupbox("Capture",
      captureSettings_ctl = new QCaptureSettingsWidget(this));

  connect(cameraSelection_ctl, &QCameraSelectionWidget::selectedCameraChanged,
      this, &ThisClass::onSelectedCameraChanged);

  updateControls();
}

const QImagingCamera::sptr &QImagingCameraControlsWidget::selectedCamera() const
{
  return cameraSelection_ctl->selectedCamera();
}

void QImagingCameraControlsWidget::setCameraWriter(QCameraWriter * writer)
{
  if( writer ) {
    writer->setCamera(selectedCamera());
  }
  captureSettings_ctl->setCameraWriter(writer);
}

QCameraWriter * QImagingCameraControlsWidget::cameraWriter() const
{
  return captureSettings_ctl->cameraWriter();
}

void QImagingCameraControlsWidget::loadSettings(const QString & prefix)
{
  const QSettings settings;
  loadSettings(settings, prefix);
}

void QImagingCameraControlsWidget::loadSettings(const QSettings & settings, const QString & prefix)
{
  captureSettings_ctl->loadSettings(settings, prefix);
}

void QImagingCameraControlsWidget::saveSettings(const QString & prefix)
{
  QSettings settings;
  saveSettings(settings, prefix);
}

void QImagingCameraControlsWidget::saveSettings(QSettings & settings, const QString & prefix)
{
  captureSettings_ctl->saveSettings(settings, prefix);
}

void QImagingCameraControlsWidget::onSelectedCameraChanged()
{
  if( cameraControls_ctl ) {
    settings_ctl->removeWidget(cameraControls_ctl);
    delete cameraControls_ctl;
    cameraControls_ctl = nullptr;
  }

  const QImagingCamera::sptr &selectedCamera =
      cameraSelection_ctl->selectedCamera();

  if( selectedCamera ) {
    if( (cameraControls_ctl = QCameraControlsWidget::create(selectedCamera, this)) ) {
      settings_ctl->insertRow(0, cameraControls_ctl);
    }
  }

  QCameraWriter *writer =
      captureSettings_ctl->cameraWriter();

  if( writer ) {
    writer->setCamera(selectedCamera);
  }

  Q_EMIT selectedCameraChanged();
}

QImagingCameraControlsDock::QImagingCameraControlsDock(const QString & title, QWidget * parent,
    QImagingCameraControlsWidget * view) :
    Base(title, parent, view)
{
}

} /* namespace serimager */
