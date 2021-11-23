/*
 * QCloudViewSettings.cc
 *
 *  Created on: May 18, 2021
 *      Author: amyznikov
 */

#include "QCloudViewSettings.h"

#if HAVE_QGLViewer // Should come from CMakeLists.txt

#include <gui/widgets/settings.h>
//#include "widgets/addctrl.h"
#include <core/debug.h>



QCloudViewSettings::QCloudViewSettings(QWidget * parent)
  : Base("QCloudViewSettings", parent)
{

  sceneRadius_ctl = add_numeric_box<double>("sceneRadius",
      [this](double v) -> bool {
        if ( cloudViewer_ && v > 0 ) {
          cloudViewer_->setSceneRadius(v);
          save_parameter(PREFIX, "sceneRadius", v);
          return true;
        }
        return false;
      });


  sceneOrigin_ctl = add_numeric_box<QGLVector>("sceneOrigin",
      [this](const QGLVector & v) -> bool {
        if ( cloudViewer_ && v != cloudViewer_->sceneOrigin() ) {
          cloudViewer_->setSceneOrigin(v);
          save_parameter(PREFIX, "SceneOrigin", v);
          return true;
        }
        return false;
      });

  pointSize_ctl = add_numeric_box<double>("pointSize",
      [this](double v) -> bool {
        if ( cloudViewer_ && v > 0 ) {
          cloudViewer_->setPointSize(v);
          save_parameter(PREFIX, "pointSize", v);
          return true;
        }
        return false;
      });

  pointBrightness_ctl = add_numeric_box<double>("pointBrightness",
      [this](double v) -> bool {
        if ( cloudViewer_ && v > 0 ) {
          cloudViewer_->setPointBrightness(v);
          save_parameter(PREFIX, "pointBrightness", v);
          return true;
        }
        return false;
      });


  sceneCenter_ctl = add_numeric_box<QGLVector>("sceneCenter",
      [this](const QGLVector & v) -> bool {
        if ( cloudViewer_ && v != cloudViewer_->sceneCenter() ) {
          cloudViewer_->setSceneCenter(v);
          save_parameter(PREFIX, "sceneCenter", v);
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

    double sceneRadius = cloudViewer_->sceneRadius();
    if ( load_parameter(settings, PREFIX, "sceneRadius", &sceneRadius) ) {
      cloudViewer_->setSceneRadius(sceneRadius);
    }

    QGLVector sceneOrigin = cloudViewer_->sceneOrigin();
    if ( load_parameter(settings, PREFIX, "sceneOrigin", &sceneOrigin) ) {
      cloudViewer_->setSceneOrigin(sceneOrigin);
    }

    double pointSize = cloudViewer_->pointSize();
    if ( load_parameter(settings, PREFIX, "pointSize", &pointSize) ) {
      cloudViewer_->setPointSize(pointSize);
    }

    double pointBrightness = cloudViewer_->pointBrightness();
    if ( load_parameter(settings, PREFIX, "pointBrightness", &pointBrightness) ) {
      cloudViewer_->setPointBrightness(pointBrightness);
    }


    QGLVector sceneCenter = cloudViewer_->sceneCenter();
    if ( load_parameter(settings, PREFIX, "sceneCenter", &sceneCenter) ) {
      cloudViewer_->setSceneCenter(sceneCenter);
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

    sceneRadius_ctl->setValue(cloudViewer_->sceneRadius());
    sceneOrigin_ctl->setValue(toString(cloudViewer_->sceneOrigin()));
    pointSize_ctl->setValue(cloudViewer_->pointSize());
    pointBrightness_ctl->setValue(cloudViewer_->pointBrightness());
    sceneCenter_ctl->setValue(toString(cloudViewer_->sceneCenter()));

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

#endif // HAVE_QGLViewer
