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
#include "ffmpeg/QFFMPEGCamera.h"
#include "ffmpeg/QFFStreams.h"


namespace serimager {

///////////////////////////////////////////////////////////
namespace {

#define ICON_hourglass      ":/serimager/icons/hourglass.png"
// #define ICON_start          ":/serimager/icons/play.png"
#define ICON_start          ":/serimager/icons/start.png"
#define ICON_stop           ":/serimager/icons/stop.png"
#define ICON_info           ":/serimager/icons/info.png"
#define ICON_connect        ":/serimager/icons/connect.png"
#define ICON_disconnect     ":/serimager/icons/disconnect.png"
#define ICON_menu           ":/serimager/icons/menu.png"


static QIcon icon_hourglass;
static QIcon icon_start;
static QIcon icon_stop;
static QIcon icon_info;
static QIcon icon_connect;
static QIcon icon_disconnect;
static QIcon icon_menu;


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
  if( icon_menu.isNull() ) {
    icon_menu = getIcon(ICON_menu);
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

  layout_->addWidget(menu_ctl =
      create_toolbutton(icon_menu,
          "Options...",
          this));

  connect(cameraSelection_ctl, SIGNAL(currentIndexChanged(int)),
      this, SLOT(onCameraSelectionCurrentIndexChanged(int)));

  connect(connectionStatus_ctl, &QToolButton::clicked,
      this, &ThisClass::onConnectionStatusCtrlClicked);

  connect(startStop_ctl, &QToolButton::clicked,
      this, &ThisClass::onStartStopCtrlClicked);

  connect(menu_ctl, &QToolButton::clicked,
      this, &ThisClass::onMenuCtrlClicked);


  updateControls();

  refreshCamerasTimerId_ =
      startTimer(1500);
}

const QImagingCamera::sptr & QCameraSelectionWidget::selectedCamera() const
{
  return selectedCamera_;
}

QImagingCamera::sptr QCameraSelectionWidget::getSelectedCamera()
{
  const int cursel =
      cameraSelection_ctl->currentIndex();

  if( cursel < 0 ) {
    return nullptr;
  }

  return cameraSelection_ctl->itemData(cursel).value<QImagingCamera::sptr>();
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

void QCameraSelectionWidget::onMenuCtrlClicked()
{
  QMenu menu;
  QAction * action;

  static QFFStreamsDialogBox *ffStreamsDialogBox = nullptr;

  /////////////

  menu.addAction(action = new QAction("FFmpeg streams..."));

  action->setCheckable(true);
  action->setChecked(ffStreamsDialogBox && ffStreamsDialogBox->isVisible());

  connect(action, &QAction::triggered,
      [this](bool checked) {

        if ( !checked ) {
          if ( ffStreamsDialogBox && ffStreamsDialogBox->isVisible() ) {
            ffStreamsDialogBox->hide();
          }
        }
        else {
          if( !ffStreamsDialogBox ) {
            ffStreamsDialogBox = new QFFStreamsDialogBox(this);
          }
          else if( ffStreamsDialogBox->parent() != this ) {
            ffStreamsDialogBox->setParent(this);
          }

          if( !ffStreamsDialogBox->isVisible() ) {
            ffStreamsDialogBox->show();
          }

          ffStreamsDialogBox->setFocus();
        }

      });

  /////////////

  menu.exec(menu_ctl->mapToGlobal(QPoint(menu_ctl->width() / 2, menu_ctl->height() / 2)));



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

  enableControl(menu_ctl, true);

  if ( !selectedCamera_ ) {
    cameraSelection_ctl->setEnabled(cameraSelection_ctl->count() > 0);
    enableControl(connectionStatus_ctl, false, icon_hourglass);
    enableControl(startStop_ctl, false, icon_hourglass);
  }
  else {

    switch (selectedCamera_->state()) {
      case QImagingCamera::State_disconnected:
        cameraSelection_ctl->setEnabled(true);
        enableControl(connectionStatus_ctl, true, icon_connect);
        enableControl(startStop_ctl, false, icon_start);
        break;
      case QImagingCamera::State_connecting:
        cameraSelection_ctl->setEnabled(false);
        enableControl(connectionStatus_ctl, true, icon_hourglass);
        enableControl(startStop_ctl, false, icon_start);
        break;
      case QImagingCamera::State_connected:
        cameraSelection_ctl->setEnabled(false);
        enableControl(connectionStatus_ctl, true, icon_disconnect);
        enableControl(startStop_ctl, true, icon_start);
        break;
      case QImagingCamera::State_starting:
        cameraSelection_ctl->setEnabled(false);
        enableControl(connectionStatus_ctl, true, icon_disconnect);
        enableControl(startStop_ctl, true, icon_stop);
        break;
      case QImagingCamera::State_started:
        cameraSelection_ctl->setEnabled(false);
        enableControl(connectionStatus_ctl, true, icon_disconnect);
        enableControl(startStop_ctl, true, icon_stop);
        break;
      case QImagingCamera::State_stopping:
        cameraSelection_ctl->setEnabled(false);
        enableControl(connectionStatus_ctl, true, icon_disconnect);
        enableControl(startStop_ctl, false, icon_hourglass);
        break;
      case QImagingCamera::State_disconnecting:
        cameraSelection_ctl->setEnabled(false);
        enableControl(connectionStatus_ctl, true, icon_hourglass);
        enableControl(startStop_ctl, false, icon_connect);
        break;
    }
  }
}

void QCameraSelectionWidget::refreshCameras()
{
  QList<QImagingCamera::sptr> detectedCameras;

  detectedCameras.append(QASICamera::detectCameras());
  detectedCameras.append(QV4L2Camera::detectCameras());
  detectedCameras.append(QFFStreams::streams());

  //
  // Remove disappeared cameras
  //
  for( int i = 0; i < cameraSelection_ctl->count(); ++i ) {

    const QImagingCamera::sptr c2 =
        cameraSelection_ctl->itemData(i).value<QImagingCamera::sptr>();

    if( !c2 ) {
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
        continue;
      }

      if( c2->is_same_camera(c1) ) {

        missing = false;

        const QString displayName =
            c1->display_name();

        if( cameraSelection_ctl->itemText(i) != displayName ) {
          cameraSelection_ctl->setItemText(i, displayName);
        }

        break;
      }
    }

    if( missing ) {

      CF_DEBUG("Connected '%s'", c1->display_name().toUtf8().constData());
      cameraSelection_ctl->addItem(c1->display_name(),
          QVariant::fromValue(c1));
    }
  }


//  int index = cameraSelection_ctl->findData(QVariant::fromValue((int) ADD_FFMPEG_STREAM));
//  if( index >= 0 && index != cameraSelection_ctl->count() - 1 ) {
//    cameraSelection_ctl->removeItem(index);
//  }
//  if( index < 0 ) {
//    cameraSelection_ctl->addItem("FFMPEG stream ...",
//        QVariant::fromValue((int) ADD_FFMPEG_STREAM));
//  }


}


} /* namespace serimager */
