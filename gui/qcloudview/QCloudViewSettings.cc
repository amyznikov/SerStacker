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
  projection_ctl =
      add_enum_combobox<QGLView::Projection>("Projection",
          "Select projection type",
          [this](QGLView::Projection v) {
            if ( cloudViewer_ && v != cloudViewer_->cloudView()->projection() ) {
              cloudViewer_->cloudView()->setProjection(v);
            }
          },
          [this](QGLView::Projection * v) {
            if ( cloudViewer_ ) {
              * v = cloudViewer_->cloudView()->projection();
              return true;
            }
            return false;
          });

  nearPlane_ctl =
      add_numeric_box<double>("NearPlane:",
          "",
          [this](double v) {
            if ( cloudViewer_ && v != cloudViewer_->cloudView()->nearPlane() ) {
              cloudViewer_->cloudView()->setNearPlane(v);
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
            }
          },
          [this](double * v) {
            if ( cloudViewer_ ) {
              * v = cloudViewer_->cloudView()->farPlane();
              return true;
            }
            return false;
          });

  fov_ctl =
      add_numeric_box<double>("FOV [deg]:",
          "Field of View for Perspective in degrees",
          [this](double v) {
            if ( cloudViewer_ && v != cloudViewer_->cloudView()->fov() ) {
              cloudViewer_->cloudView()->setFOV(v);
            }
          },
          [this](double * v) {
            if ( cloudViewer_ ) {
              * v = cloudViewer_->cloudView()->fov();
              return true;
            }
            return false;
          });

  rect_ctl  =
      add_numeric_box<QRectF>("Rect:",
          "Rectangle for Orto and Frustrum views",
          [this](const QRectF & v) {
            if ( cloudViewer_ && v != cloudViewer_->cloudView()->viewRect() ) {
              cloudViewer_->cloudView()->setViewRect(v);
            }
          },
          [this](QRectF * v) {
            if ( cloudViewer_ ) {
              * v = cloudViewer_->cloudView()->viewRect();
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
        }
      });

  connect(this, &ThisClass::populatecontrols,
      [this]() {
        if ( cloudViewer_ ) {
          bgColor_ctl->setColor(cloudViewer_->cloudView()->backgroundColor());
        }
      });


  updateControls();
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


void QCloudViewSettings::onupdatecontrols()
{
  if ( !cloudViewer_ ) {
    setEnabled(false);
  }
  else {
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
