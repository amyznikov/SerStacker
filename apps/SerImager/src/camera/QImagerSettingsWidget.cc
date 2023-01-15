/*
 * QImagerSettingsWidget.cc
 *
 *  Created on: Dec 21, 2022
 *      Author: amyznikov
 */

#include "QImagerSettingsWidget.h"
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

QImagerSettingsWidget::QImagerSettingsWidget(QWidget * parent) :
    Base(parent)
{
  layout_ = new QVBoxLayout(this);

  layout_->addWidget(cameraSelection_ctl =
      new QCameraSelectionWidget(this));

  layout_->addWidget(createScrollableWrap(
      settings_ctl = new QSettingsWidget("captureControls", this),
      this));

  settings_ctl->add_expandable_groupbox("Capture",
      captureSettings_ctl = new QCaptureSettingsWidget(this));

  connect(cameraSelection_ctl, &QCameraSelectionWidget::selectedCameraChanged,
      this, &ThisClass::onSelectedCameraChanged);

  updateControls();
}

const QImagingCamera::sptr &QImagerSettingsWidget::selectedCamera() const
{
  return cameraSelection_ctl->selectedCamera();
}

void QImagerSettingsWidget::setCameraWriter(QCameraWriter * writer)
{
  if( writer ) {
    writer->setCamera(selectedCamera());
  }
  captureSettings_ctl->setCameraWriter(writer);
}

QCameraWriter * QImagerSettingsWidget::cameraWriter() const
{
  return captureSettings_ctl->cameraWriter();
}

void QImagerSettingsWidget::onupdatecontrols()
{
}

void QImagerSettingsWidget::onSelectedCameraChanged()
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
      settings_ctl->insertWidget(0, cameraControls_ctl);
    }
  }

  QCameraWriter *writer =
      captureSettings_ctl->cameraWriter();

  if( writer ) {
    writer->setCamera(selectedCamera);
  }

  Q_EMIT selectedCameraChanged();
}

QCameraControlDock::QCameraControlDock(const QString & title, QWidget * parent, QImagerSettingsWidget * view) :
    Base(title, parent, view)
{
}

} /* namespace qserimager */
