/*
 * QCloudViewSettings.cc
 *
 *  Created on: May 18, 2021
 *      Author: amyznikov
 */

#include "QCloudViewSettings.h"
#include <gui/widgets/settings.h>
#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////////////////////////

template<class Fn>
static QToolButton * createToolButton(QWidget * parent,
    const QIcon & icon, const QString & text, const QString & tooltip,
    Fn && clickFunc)
{

  QToolButton * tb = new QToolButton(parent);
  tb->setIconSize(QSize(16, 16));
  tb->setIcon(icon);
  tb->setText(text);
  tb->setToolTip(tooltip);

  QObject::connect(tb, &QToolButton::clicked,
      clickFunc);

  return tb;
}


///////////////////////////////////////////////////////////////////////////////////////////////////

QCloudViewSettings::QCloudViewSettings(QWidget * parent) :
  Base("QCloudViewSettings", parent)
{
  nearPlane_ctl =
      add_numeric_box<double>("NearPlane:",
          "",
          [this](double v) {
            if ( cloudViewer_ && v != cloudViewer_->cloudView()->nearPlane() ) {
              cloudViewer_->cloudView()->setNearPlane(v);
              save_parameter(PREFIX, "nearPlane", cloudViewer_->cloudView()->nearPlane());
            }
          },
          [this](double * v) {
            if ( cloudViewer_ ) {
              * v = cloudViewer_->cloudView()->nearPlane();
              return true;
            }
            return false;
          });

  farPlane_ctl =
      add_numeric_box<double>("FarPlane:",
          "",
          [this](double v) {
            if ( cloudViewer_ && v != cloudViewer_->cloudView()->farPlane() ) {
              cloudViewer_->cloudView()->setFarPlane(v);
              save_parameter(PREFIX, "farPlane", v);
            }
          },
          [this](double * v) {
            if ( cloudViewer_ ) {
              * v = cloudViewer_->cloudView()->farPlane();
              return true;
            }
            return false;
          });

  sceneTarget_ctl =
      add_numeric_box<QVector3D>("View Target:",
          "",
          [this](const QVector3D & v) {
            if ( cloudViewer_ ) {
              cloudViewer_->cloudView()->setViewTargetPoint(v);
              save_parameter(PREFIX, "target", cloudViewer_->cloudView()->viewTargetPoint());
            }
          },
          [this](QVector3D * v) {
            if ( cloudViewer_ ) {
              * v = cloudViewer_->cloudView()->viewTargetPoint();
              return true;
            }
            return false;
          });

  upDirection_ctl =
      add_numeric_box<QVector3D>("Up:",
          "",
          [this](const QVector3D & v) {
            if ( cloudViewer_ ) {
              cloudViewer_->cloudView()->setUpDirection(v);
              save_parameter(PREFIX, "upDirection", cloudViewer_->cloudView()->upDirection());
            }
          },
          [this](QVector3D * v) {
            if ( cloudViewer_ ) {
              * v = cloudViewer_->cloudView()->upDirection();
              return true;
            }
            return false;
          });

  sceneOrigin_ctl =
      add_numeric_box<QVector3D>("Origin:",
          "",
          [this](const QVector3D & v) {
            if ( cloudViewer_ ) {
              cloudViewer_->cloudView()->setSceneOrigin(v);
              save_parameter(PREFIX, "SceneOrigin", cloudViewer_->cloudView()->sceneOrigin());
            }
          },
          [this](QVector3D * v) {
            if ( cloudViewer_ ) {
              * v = cloudViewer_->cloudView()->sceneOrigin();
              return true;
            }
            return false;
          });

  autoShowViewTarget_ctl =
      add_checkbox("Auto Show View Target",
          "",
          [this](bool checked) {
            if ( cloudViewer_ && cloudViewer_->cloudView()->autoShowViewTarget() != checked ) {
              cloudViewer_->cloudView()->setAutoShowViewTarget(checked);
              save_parameter(PREFIX, "autoShowViewTarget", cloudViewer_->cloudView()->autoShowViewTarget());
            }
          },
          [this](bool * v) {
            if ( cloudViewer_ ) {
              * v = cloudViewer_->cloudView()->autoShowViewTarget();
              return true;
            }
            return false;
          });


  pointSize_ctl =
      add_numeric_box<double>("pointSize:",
          "",
          [this](double v) {
            if ( cloudViewer_ && v > 0 && v != cloudViewer_->cloudView()->pointSize() ) {
              cloudViewer_->cloudView()->setPointSize(v);
              save_parameter(PREFIX, "pointSize", v);
            }
          },
          [this](double * v) {
            if ( cloudViewer_ ) {
              * v = cloudViewer_->cloudView()->pointSize();
              return true;
            }
            return false;
          });

  pointBrightness_ctl =
      add_numeric_box<double>("pointBrightness",
          "",
          [this](double v) {
            if ( cloudViewer_ && v != cloudViewer_->cloudView()->pointBrightness() ) {
              cloudViewer_->cloudView()->setPointBrightness(v);
              save_parameter(PREFIX, "pointBrightness", cloudViewer_->cloudView()->pointBrightness());
            }
          },
          [this](double * v) {
            if ( cloudViewer_ ) {
              * v = cloudViewer_->cloudView()->pointBrightness();
              return true;
            }
            return false;
          });



  bgColor_ctl =
      add_widget<QColorPickerButton>("Background Color");

  connect(bgColor_ctl, &QColorPickerButton::colorSelected,
      [this]() {
        if ( cloudViewer_ && cloudViewer_->cloudView()->backgroundColor() != bgColor_ctl->color() ) {
          cloudViewer_->cloudView()->setBackgroundColor(bgColor_ctl->color());
          save_parameter(PREFIX, "BackgroundColor", cloudViewer_->cloudView()->backgroundColor());
        }
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

    bgColor_ctl->setColor(cloudView->backgroundColor());

//
//
//    farPlane_ctl->setValue(toQString(cloudView->farPlane()));
//    nearPlane_ctl->setValue(toQString(cloudView->nearPlane()));
//    sceneTarget_ctl->setValue(toQString(cloudView->viewTargetPoint()));
//    upDirection_ctl->setValue(toQString(cloudView->upDirection()));
//    sceneOrigin_ctl->setValue(toQString(cloudView->sceneOrigin()));
//    pointSize_ctl->setValue(cloudView->pointSize());
//    autoShowViewTarget_ctl->setChecked(cloudView->autoShowViewTarget());
//    pointBrightness_ctl->setValue(cloudView->pointBrightness());

    Base::onupdatecontrols();

    refreshCloudList();

    setEnabled(true);
  }
}

void QCloudViewSettings::refreshCloudList()
{
  //cloudsSettings_ctl->refreshCloudList();
}


///////////////////////////////////////////////////////////////////////////////


QCloudViewSettingsWidget::QCloudViewSettingsWidget(QWidget * parent) :
    Base(parent)
{
  QHBoxLayout * layout =
      new QHBoxLayout(this);

  QVBoxLayout * buttonBox =
      new QVBoxLayout();

  buttonBox->setAlignment(Qt::AlignTop);

  buttonBox->addWidget(rotateCameraToShowCloud_ctl =
      createToolButton(this, QIcon(),
          "Auto rotate",
          "Rotate camera to show point cloud",
          [this]() {
            if ( cloudViewer_ ) {
              cloudViewer_->rotateToShowCloud();
            }
          }));

//  buttonBox->addWidget(moveCameraToShowCloud_ctl=
//      createToolButton(this, QIcon(),
//          "Show entire cloud",
//          "Rotate and Move camera to show entire point cloud",
//          [this]() {
//            if ( cloudViewer_ ) {
//            }
//
//          }));

  buttonBox->addWidget(showKeyBindings_ctl=
      createToolButton(this, QIcon(),
          "Help for keys...",
          "Show Mouse and keyboard bindings...",
          [this]() {
            if ( cloudViewer_ ) {
              cloudViewer_->showKeyBindings();
            }

          }));

  layout->addWidget(settings_ctl = new QCloudViewSettings(this) );
  layout->addLayout(buttonBox);

}

void QCloudViewSettingsWidget::setCloudViewer(QCloudViewer * v)
{
  settings_ctl->setCloudViewer(cloudViewer_ = v);
  setEnabled(cloudViewer_ != nullptr);
}

QCloudViewer * QCloudViewSettingsWidget::cloudViewer() const
{
  return cloudViewer_;
}


///////////////////////////////////////////////////////////////////////////////
QCloudViewSettingsDialogBox::QCloudViewSettingsDialogBox(QWidget * parent) :
    Base(parent)
{
  vbox_ = new QVBoxLayout(this);
  vbox_->addWidget(cloudViewSettingsWidget_ = new QCloudViewSettingsWidget(this));
}

void QCloudViewSettingsDialogBox::setCloudViewer(QCloudViewer * v)
{
  cloudViewSettingsWidget_->setCloudViewer(v);
}

QCloudViewer * QCloudViewSettingsDialogBox::cloudViewer() const
{
  return cloudViewSettingsWidget_->cloudViewer();
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
