/*
 * QCameraSelectionWidget.cc
 *
 *  Created on: Dec 21, 2022
 *      Author: amyznikov
 */

#include <gui/widgets/style.h>
#include "QCameraSelectionWidget.h"
#include <core/debug.h>
#include "v4l2/QV4L2Camera.h"
#include "zwo_asi/QASICamera.h"
// #include <gui/widgets/QWaitCursor.h>

namespace serimager {

///////////////////////////////////////////////////////////
namespace {

#define ICON_hourglass      ":/qserimager/icons/hourglass.png"
// #define ICON_start          ":/qserimager/icons/play.png"
#define ICON_start          ":/qserimager/icons/start.png"
#define ICON_stop           ":/qserimager/icons/stop.png"
#define ICON_info           ":/qserimager/icons/info.png"
#define ICON_connect        ":/qserimager/icons/connect.png"
#define ICON_disconnect     ":/qserimager/icons/disconnect.png"


static QIcon icon_hourglass;
static QIcon icon_start;
static QIcon icon_stop;
static QIcon icon_info;
static QIcon icon_connect;
static QIcon icon_disconnect;

void enableControl(QAbstractButton * w, bool enabled)
{
  if ( w->isEnabled() != enabled ) {
    w->setEnabled(enabled);
  }
}

void enableControl(QAbstractButton * w, bool enabled, const QIcon & icon)
{
  if ( w->isEnabled() != enabled ) {
    w->setEnabled(enabled);
  }
  if ( w->icon().cacheKey() != icon.cacheKey() ){
    w->setIcon(icon);
  }
}


static void init_resources()
{
  if( icon_hourglass.isNull() ) {
    icon_hourglass = getIcon(ICON_hourglass);
  }
  if( icon_start.isNull() ) {
    icon_start = getIcon(ICON_start);
  }
  if( icon_stop.isNull() ) {
    icon_stop = getIcon(ICON_stop);
  }
  if( icon_info.isNull() ) {
    icon_info = getIcon(ICON_info);
  }
  if( icon_connect.isNull() ) {
    icon_connect = getIcon(ICON_connect);
  }
  if( icon_disconnect.isNull() ) {
    icon_disconnect = getIcon(ICON_disconnect);
  }
}

} // namespace
///////////////////////////////////////////////////////////


QCameraSelectionWidget::QCameraSelectionWidget(QWidget * parent) :
    Base(parent)
{

  init_resources();

  static const auto create_toolbutton =
      [](const QIcon & icon, const QString & tooltip, QWidget * parent = nullptr) -> QToolButton* {

        QToolButton * tb = new QToolButton(parent);

        tb->setToolButtonStyle(Qt::ToolButtonIconOnly);
        // tb->setIconSize(QSize(22, 22));
        tb->setIcon(icon);
        tb->setToolTip(tooltip);

        return tb;
      };


  layout_ = new QHBoxLayout(this);

  layout_->addWidget(cameraSelection_ctl = new QComboBox(this));
  cameraSelection_ctl->setEditable(false);

  layout_->addWidget(connectionStatus_ctl =
      create_toolbutton(icon_connect,
          "Connect / Disconnect selected camera",
          this));

  layout_->addWidget(startStop_ctl =
      create_toolbutton(icon_start,
          "Start / Stop camera capture",
          this));

  layout_->addWidget(cameraInfo_ctl =
      create_toolbutton(icon_info,
          "Show selected camera info",
          this));

  connect(cameraSelection_ctl, SIGNAL(currentIndexChanged(int)),
      this, SLOT(onCameraSelectionCurrentIndexChanged(int)));

  connect(connectionStatus_ctl, &QToolButton::clicked,
      this, &ThisClass::onConnectionStatusCtrlClicked);

  connect(startStop_ctl, &QToolButton::clicked,
      this, &ThisClass::onStartStopCtrlClicked);

  connect(cameraInfo_ctl, &QToolButton::clicked,
      this, &ThisClass::onCameraInfoCtrlClicked);

  updateControls();

  refreshCamerasTimerId_ =
      startTimer(1500);
}

const QImagingCamera::sptr & QCameraSelectionWidget::selectedCamera() const
{
  return selectedCamera_;
}

QImagingCamera::sptr QCameraSelectionWidget::getSelectedCamera() const
{
  const int cursel = cameraSelection_ctl->currentIndex();
  return cursel >= 0 ? cameraSelection_ctl->itemData(cursel).value<QImagingCamera::sptr>() : nullptr;
}

void QCameraSelectionWidget::onCameraSelectionCurrentIndexChanged(int index)
{
  QImagingCamera::sptr newSelectedCamera =
      getSelectedCamera();

  if( selectedCamera_ && selectedCamera_ != newSelectedCamera ) {

    selectedCamera_->disconnect();

    disconnect(selectedCamera_.get(), nullptr,
        this, nullptr);
  }

  if( (selectedCamera_ = newSelectedCamera) ) {

    connect(selectedCamera_.get(), &QImagingCamera::stateChanged,
        this, &ThisClass::onUpdateControls,
        Qt::QueuedConnection);

  }

  updateControls();

  Q_EMIT selectedCameraChanged();
}

void QCameraSelectionWidget::onConnectionStatusCtrlClicked()
{
  if( selectedCamera_ ) {

    switch (selectedCamera_->state()) {
      case QImagingCamera::State_disconnected:
        selectedCamera_->connect();
        break;
      case QImagingCamera::State_connecting:
        selectedCamera_->disconnect();
        break;
      case QImagingCamera::State_connected:
        selectedCamera_->disconnect();
        break;
      case QImagingCamera::State_starting:
        selectedCamera_->disconnect();
        break;
      case QImagingCamera::State_started:
        selectedCamera_->disconnect();
        break;
      case QImagingCamera::State_stopping:
        selectedCamera_->disconnect();
        break;
      case QImagingCamera::State_disconnecting:
        break;
    }
  }

}

void QCameraSelectionWidget::onStartStopCtrlClicked()
{
  if( selectedCamera_ ) {
    switch (selectedCamera_->state()) {
      case QImagingCamera::State_disconnected:
        break;
      case QImagingCamera::State_connecting:
        break;
      case QImagingCamera::State_connected:
        selectedCamera_->start();
        break;
      case QImagingCamera::State_starting:
        break;
      case QImagingCamera::State_started:
        selectedCamera_->stop();
        break;
      case QImagingCamera::State_stopping:
        break;
      case QImagingCamera::State_disconnecting:
        break;
    }
  }
}

void QCameraSelectionWidget::onCameraInfoCtrlClicked()
{

}

void QCameraSelectionWidget::timerEvent(QTimerEvent *event)
{
  if( selectedCamera_ && selectedCamera_->state() != QImagingCamera::State_disconnected ) {
    /*if( !selectedCamera_->check_status() ) {
      refreshCameras();
    }*/
  }
  else {
    refreshCameras();
  }
}

void QCameraSelectionWidget::onUpdateControls()
{
  updateControls();
}

void QCameraSelectionWidget::onupdatecontrols()
{
  if ( cameraSelection_ctl->count() < 1 ) {
    setEnabled(false);
    return;
  }

  setEnabled(true);

  if ( !selectedCamera_ ) {
    cameraSelection_ctl->setEnabled(cameraSelection_ctl->count() > 0);
    enableControl(connectionStatus_ctl, false, icon_hourglass);
    enableControl(startStop_ctl, false, icon_hourglass);
    enableControl(cameraInfo_ctl, false);
  }
  else {

    switch (selectedCamera_->state()) {
      case QImagingCamera::State_disconnected:
        cameraSelection_ctl->setEnabled(true);
        enableControl(connectionStatus_ctl, true, icon_connect);
        enableControl(startStop_ctl, false, icon_start);
        enableControl(cameraInfo_ctl, false);
        break;
      case QImagingCamera::State_connecting:
        cameraSelection_ctl->setEnabled(false);
        enableControl(connectionStatus_ctl, true, icon_hourglass);
        enableControl(startStop_ctl, false, icon_start);
        enableControl(cameraInfo_ctl, false);
        break;
      case QImagingCamera::State_connected:
        cameraSelection_ctl->setEnabled(false);
        enableControl(connectionStatus_ctl, true, icon_disconnect);
        enableControl(startStop_ctl, true, icon_start);
        enableControl(cameraInfo_ctl, true);
        break;
      case QImagingCamera::State_starting:
        cameraSelection_ctl->setEnabled(false);
        enableControl(connectionStatus_ctl, true, icon_disconnect);
        enableControl(startStop_ctl, true, icon_stop);
        enableControl(cameraInfo_ctl, true);
        break;
      case QImagingCamera::State_started:
        cameraSelection_ctl->setEnabled(false);
        enableControl(connectionStatus_ctl, true, icon_disconnect);
        enableControl(startStop_ctl, true, icon_stop);
        enableControl(cameraInfo_ctl, true);
        break;
      case QImagingCamera::State_stopping:
        cameraSelection_ctl->setEnabled(false);
        enableControl(connectionStatus_ctl, true, icon_disconnect);
        enableControl(startStop_ctl, false, icon_hourglass);
        enableControl(cameraInfo_ctl, true);
        break;
      case QImagingCamera::State_disconnecting:
        cameraSelection_ctl->setEnabled(false);
        enableControl(connectionStatus_ctl, true, icon_hourglass);
        enableControl(startStop_ctl, false, icon_connect);
        enableControl(cameraInfo_ctl, false);
        break;
    }
  }
}

void QCameraSelectionWidget::refreshCameras()
{
  QList<QImagingCamera::sptr> detectedCameras;

  detectedCameras.append(QASICamera::detectCameras());
  detectedCameras.append(QV4L2Camera::detectCameras());

  //
  // Remove disappeared cameras
  //
  for( int i = 0; i < cameraSelection_ctl->count(); ++i ) {

    const QImagingCamera::sptr c2 =
        cameraSelection_ctl->itemData(i).value<QImagingCamera::sptr>();

    if( !c2 ) {
      CF_ERROR("APP BUG: c2 is null for item %d", i);
      continue;
    }

    bool disapeared = true;

    for( const QImagingCamera::sptr &c1 : detectedCameras ) {
      if( c1->is_same_camera(c2) ) {
        disapeared = false;
        break;
      }
    }

    if( disapeared ) {
      CF_DEBUG("Disconnected '%s'", c2->display_name().toUtf8().constData());
      cameraSelection_ctl->removeItem(i--);
    }
  }


  //
  // Append appeared cameras
  //

  for( const QImagingCamera::sptr &c1 : detectedCameras ) {

    bool missing = true;

    for( int i = 0; i < cameraSelection_ctl->count(); ++i ) {

      const QImagingCamera::sptr c2 =
          cameraSelection_ctl->itemData(i).value<QImagingCamera::sptr>();

      if( !c2 ) {
        CF_ERROR("APP BUG: c2 is null for item %d", i);
        continue;
      }

      if( c2->is_same_camera(c1) ) {
        missing = false;
        break;
      }
    }

    if( missing ) {

      CF_DEBUG("Connected '%s'", c1->display_name().toUtf8().constData());
      cameraSelection_ctl->addItem(c1->display_name(),
          QVariant::fromValue(c1));
    }
  }

}


} /* namespace qserimager */
