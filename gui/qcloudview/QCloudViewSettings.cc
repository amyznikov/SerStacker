/*
 * QCloudViewSettings.cc
 *
 *  Created on: May 18, 2021
 *      Author: amyznikov
 */

#include "QCloudViewSettings.h"
#include <gui/widgets/settings.h>
//#include "widgets/addctrl.h"
#include <core/debug.h>



QCloudViewSettings::QCloudViewSettings(QWidget * parent) :
  Base("QCloudViewSettings", parent)
{
  nearPlane_ctl =
      add_numeric_box<double>("NearPlane",
          [this](double v) -> bool {
            if ( cloudViewer_ && v != cloudViewer_->cloudView()->nearPlane() ) {
              cloudViewer_->cloudView()->setNearPlane(v);
              save_parameter(PREFIX, "nearPlane", cloudViewer_->cloudView()->nearPlane());
              return true;
            }
            return false;
          });

  farPlane_ctl =
      add_numeric_box<double>("FarPlane:",
          [this](double v) -> bool {
            if ( cloudViewer_ && v != cloudViewer_->cloudView()->farPlane() ) {
              cloudViewer_->cloudView()->setFarPlane(v);
              save_parameter(PREFIX, "farPlane", v);
              return true;
            }
            return false;
          });

  sceneTarget_ctl =
      add_numeric_box<QVector3D>("Target:",
          [this](const QVector3D & v) -> bool {
            if ( cloudViewer_ ) {
              cloudViewer_->cloudView()->setViewTargetPoint(v);
              save_parameter(PREFIX, "target", cloudViewer_->cloudView()->viewTargetPoint());
              return true;
            }
            return false;
          });

  upDirection_ctl =
      add_numeric_box<QVector3D>("Up:",
          [this](const QVector3D & v) -> bool {
            if ( cloudViewer_ ) {
              cloudViewer_->cloudView()->setUpDirection(v);
              save_parameter(PREFIX, "upDirection", cloudViewer_->cloudView()->upDirection());
              return true;
            }
            return false;
          });

  sceneOrigin_ctl =
      add_numeric_box<QVector3D>("Origin:",
          [this](const QVector3D & v) -> bool {
            if ( cloudViewer_ ) {
              cloudViewer_->cloudView()->setSceneOrigin(v);
              save_parameter(PREFIX, "SceneOrigin", cloudViewer_->cloudView()->sceneOrigin());
              return true;
            }
            return false;
          });

  pointSize_ctl =
      add_numeric_box<double>("pointSize",
          [this](double v) -> bool {
            if ( cloudViewer_ && v > 0 && v != cloudViewer_->cloudView()->pointSize() ) {
              cloudViewer_->cloudView()->setPointSize(v);
              save_parameter(PREFIX, "pointSize", v);
              return true;
            }
            return false;
          });

  pointBrightness_ctl =
      add_numeric_box<double>("pointBrightness",
          [this](double v) -> bool {
            if ( cloudViewer_ && v > 0 && v != cloudViewer_->cloudView()->pointBrightness() ) {
              cloudViewer_->cloudView()->setPointBrightness(v);
              save_parameter(PREFIX, "pointBrightness", cloudViewer_->cloudView()->pointBrightness());
              return true;
            }
            return false;
          });




//  add_expandable_groupbox(form, "Clouds",
//      cloudsSettings_ctl = new QPointCloudsSettingsControl(this));
}

void QCloudViewSettings::setCloudViewer(QCloudViewer * v)
{
  cloudViewer_ = v;
  updateControls();
}

QCloudViewer * QCloudViewSettings::cloudViewer() const
{
  return cloudViewer_;
}

void QCloudViewSettings::onload(QSettings & settings)
{
  if ( cloudViewer_ ) {

//    double sceneRadius = cloudViewer_->sceneRadius();
//    if ( load_parameter(settings, PREFIX, "sceneRadius", &sceneRadius) ) {
//      cloudViewer_->setSceneRadius(sceneRadius);
//    }

    QGLCloudViewer * cloudView =
        cloudViewer_->cloudView();


    double nearPlane = cloudView->nearPlane();
    if ( load_parameter(settings, PREFIX, "nearPlane", &nearPlane) ) {
      cloudView->setNearPlane(nearPlane);
    }

    double farPlane = cloudView->farPlane();
    if ( load_parameter(settings, PREFIX, "farPlane", &farPlane) ) {
      cloudView->setFarPlane(farPlane);
    }

    QVector3D target = cloudView->viewTargetPoint();
    if ( load_parameter(settings, PREFIX, "target", &target) ) {
      cloudView->setViewTargetPoint(target);
    }

    QVector3D upDirection = cloudView->upDirection();
    if ( load_parameter(settings, PREFIX, "upDirection", &upDirection) ) {
      cloudView->setUpDirection(upDirection);
    }

    QVector3D sceneOrigin = cloudView->sceneOrigin();
    if ( load_parameter(settings, PREFIX, "sceneOrigin", &sceneOrigin) ) {
      cloudView->setSceneOrigin(sceneOrigin);
    }

    double pointSize = cloudView->pointSize();
    if ( load_parameter(settings, PREFIX, "pointSize", &pointSize) ) {
      cloudView->setPointSize(pointSize);
    }

    double pointBrightness = cloudView->pointBrightness();
    if ( load_parameter(settings, PREFIX, "pointBrightness", &pointBrightness) ) {
      cloudView->setPointBrightness(pointBrightness);
    }

  }

}

void QCloudViewSettings::onupdatecontrols()
{
  //cloudsSettings_ctl->setCloudViewer(cloudViewer_);

  if ( !cloudViewer_ ) {
    setEnabled(false);
  }
  else {

    QGLCloudViewer * cloudView =
        cloudViewer_->cloudView();


    farPlane_ctl->setValue(toQString(cloudView->farPlane()));
    nearPlane_ctl->setValue(toQString(cloudView->nearPlane()));
    sceneTarget_ctl->setValue(toQString(cloudView->viewTargetPoint()));
    upDirection_ctl->setValue(toQString(cloudView->upDirection()));
    sceneOrigin_ctl->setValue(toQString(cloudView->sceneOrigin()));
    pointSize_ctl->setValue(cloudView->pointSize());
    pointBrightness_ctl->setValue(cloudView->pointBrightness());

    refreshCloudList();

    setEnabled(true);
  }
}

void QCloudViewSettings::refreshCloudList()
{
  //cloudsSettings_ctl->refreshCloudList();
}


///////////////////////////////////////////////////////////////////////////////
QCloudViewSettingsDialogBox::QCloudViewSettingsDialogBox(QWidget * parent)
  : Base(parent)
{
  vbox_ = new QVBoxLayout(this);
  vbox_->addWidget(cloudViewSettings_ = new QCloudViewSettings(this));
}

void QCloudViewSettingsDialogBox::setCloudViewer(QCloudViewer * v)
{
  cloudViewSettings_->setCloudViewer(v);
}

QCloudViewer * QCloudViewSettingsDialogBox::cloudViewer() const
{
  return cloudViewSettings_->cloudViewer();
}

void QCloudViewSettingsDialogBox::showEvent(QShowEvent *e)
{
  Base::showEvent(e);
  emit visibilityChanged(isVisible());
}

void QCloudViewSettingsDialogBox::hideEvent(QHideEvent *e)
{
  Base::hideEvent(e);
  emit visibilityChanged(isVisible());
}
