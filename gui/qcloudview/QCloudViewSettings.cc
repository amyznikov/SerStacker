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
  Base(parent)
{
  projection_ctl =
      add_enum_combobox<QGLView::Projection>("Projection",
          "Select projection type",
          [this](QGLView::Projection v) {
            if ( _cloudViewer && v != _cloudViewer->cloudView()->projection() ) {
              _cloudViewer->cloudView()->setProjection(v);
            }
          },
          [this](QGLView::Projection * v) {
            if ( _cloudViewer ) {
              * v = _cloudViewer->cloudView()->projection();
              return true;
            }
            return false;
          });

  nearPlane_ctl =
      add_numeric_box<double>("NearPlane:",
          "",
          [this](double v) {
            if ( _cloudViewer && v != _cloudViewer->cloudView()->nearPlane() ) {
              _cloudViewer->cloudView()->setNearPlane(v);
            }
          },
          [this](double * v) {
            if ( _cloudViewer ) {
              * v = _cloudViewer->cloudView()->nearPlane();
              return true;
            }
            return false;
          });

  farPlane_ctl =
      add_numeric_box<double>("FarPlane:",
          "",
          [this](double v) {
            if ( _cloudViewer && v != _cloudViewer->cloudView()->farPlane() ) {
              _cloudViewer->cloudView()->setFarPlane(v);
            }
          },
          [this](double * v) {
            if ( _cloudViewer ) {
              * v = _cloudViewer->cloudView()->farPlane();
              return true;
            }
            return false;
          });

  fov_ctl =
      add_numeric_box<double>("FOV [deg]:",
          "Field of View for Perspective in degrees",
          [this](double v) {
            if ( _cloudViewer && v != _cloudViewer->cloudView()->fov() ) {
              _cloudViewer->cloudView()->setFOV(v);
            }
          },
          [this](double * v) {
            if ( _cloudViewer ) {
              * v = _cloudViewer->cloudView()->fov();
              return true;
            }
            return false;
          });

  showMainAxes_ctl =
      add_checkbox("Show Main Axes",
          "Set true to show main XYZ coordinate axes",
          [this](bool checked) {
            if ( _cloudViewer && _cloudViewer->cloudView()->showMainAxes() != checked ) {
              _cloudViewer->cloudView()->setShowMainAxes(checked);
            }
          },
          [this](bool * v) {
            if ( _cloudViewer ) {
              * v = _cloudViewer->cloudView()->showMainAxes();
              return true;
            }
            return false;
          });

  mainAxesLength_ctl =
      add_numeric_box<double>("Main Axes size:",
          "Set to 0 for auto, -1 for disable",
          [this](double v) {
            if ( _cloudViewer && v != _cloudViewer->cloudView()->mainAxesLength() ) {
              _cloudViewer->cloudView()->setMainAxesLength(v);
            }
          },
          [this](double * v) {
            if ( _cloudViewer ) {
              * v = _cloudViewer->cloudView()->mainAxesLength();
              return true;
            }
            return false;
          });

  sceneTarget_ctl =
      add_numeric_box<QVector3D>("View Target:",
          "",
          [this](const QVector3D & v) {
            if ( _cloudViewer ) {
              _cloudViewer->cloudView()->setViewTargetPoint(v);
            }
          },
          [this](QVector3D * v) {
            if ( _cloudViewer ) {
              * v = _cloudViewer->cloudView()->viewTargetPoint();
              return true;
            }
            return false;
          });

  upDirection_ctl =
      add_numeric_box<QVector3D>("Up:",
          "",
          [this](const QVector3D & v) {
            if ( _cloudViewer ) {
              _cloudViewer->cloudView()->setUpDirection(v);
            }
          },
          [this](QVector3D * v) {
            if ( _cloudViewer ) {
              * v = _cloudViewer->cloudView()->upDirection();
              return true;
            }
            return false;
          });

  sceneOrigin_ctl =
      add_numeric_box<QVector3D>("Origin:",
          "",
          [this](const QVector3D & v) {
            if ( _cloudViewer ) {
              _cloudViewer->cloudView()->setSceneOrigin(v);
            }
          },
          [this](QVector3D * v) {
            if ( _cloudViewer ) {
              * v = _cloudViewer->cloudView()->sceneOrigin();
              return true;
            }
            return false;
          });

  autoShowViewTarget_ctl =
      add_checkbox("Auto Show View Target",
          "",
          [this](bool checked) {
            if ( _cloudViewer && _cloudViewer->cloudView()->autoShowViewTarget() != checked ) {
              _cloudViewer->cloudView()->setAutoShowViewTarget(checked);
            }
          },
          [this](bool * v) {
            if ( _cloudViewer ) {
              * v = _cloudViewer->cloudView()->autoShowViewTarget();
              return true;
            }
            return false;
          });


  pointSize_ctl =
      add_numeric_box<double>("pointSize:",
          "",
          [this](double v) {
            if ( _cloudViewer && v > 0 && v != _cloudViewer->cloudView()->pointSize() ) {
              _cloudViewer->cloudView()->setPointSize(v);
            }
          },
          [this](double * v) {
            if ( _cloudViewer ) {
              * v = _cloudViewer->cloudView()->pointSize();
              return true;
            }
            return false;
          });

  pointBrightness_ctl =
      add_numeric_box<double>("pointBrightness",
          "",
          [this](double v) {
            if ( _cloudViewer && v != _cloudViewer->cloudView()->pointBrightness() ) {
              _cloudViewer->cloudView()->setPointBrightness(v);
            }
          },
          [this](double * v) {
            if ( _cloudViewer ) {
              * v = _cloudViewer->cloudView()->pointBrightness();
              return true;
            }
            return false;
          });


  bgColor_ctl =
      add_widget<QColorPickerButton>("Background Color");

  connect(bgColor_ctl, &QColorPickerButton::colorSelected,
      [this]() {
        if ( _cloudViewer && _cloudViewer->cloudView()->backgroundColor() != bgColor_ctl->color() ) {
          _cloudViewer->cloudView()->setBackgroundColor(bgColor_ctl->color());
        }
      });

  connect(this, &ThisClass::populatecontrols,
      [this]() {
        if ( _cloudViewer ) {
          bgColor_ctl->setColor(_cloudViewer->cloudView()->backgroundColor());
          refreshCloudList();
        }
      });

  connect(this, &ThisClass::enablecontrols,
      [this]() {
        setEnabled(_cloudViewer != nullptr);
      });

  updateControls();
}

void QCloudViewSettings::setCloudViewer(QCloudViewer * v)
{
  _cloudViewer = v;
  updateControls();
}

QCloudViewer * QCloudViewSettings::cloudViewer() const
{
  return _cloudViewer;
}

//
//bool QCloudViewSettings::enable_controls() const
//{
//  return _cloudViewer != nullptr;
//}

//void QCloudViewSettings::onupdatecontrols()
//{
//  Base::onupdatecontrols();
//  if ( _cloudViewer ) {
//    refreshCloudList();
//  }
//}

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
  setWindowTitle("Cloud View Settings");

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
