/*
 * QInputPointCloudSourceView.cc
 *
 *  Created on: Dec 9, 2023
 *      Author: amyznikov
 */

#include "QPointCloudSourceView.h"
#include <core/debug.h>

namespace serstacker {

///////////////////////////////////////////////////////////////////////////////////////////////////

QGLPointCloudView::QGLPointCloudView(QWidget * parent) :
    Base(parent)
{
}

void QGLPointCloudView::setDisplayFunction(QCloudViewDisplayFunction * displayFunc)
{
  displayFunction_ = displayFunc;
  update_display_points_ = true;
  update();
}

QCloudViewDisplayFunction * QGLPointCloudView::displayFunction() const
{
  return displayFunction_;
}

void QGLPointCloudView::setPointSize(double v)
{
  pointSize_ = v;
  update();
}

double QGLPointCloudView::pointSize() const
{
  return pointSize_;
}

void QGLPointCloudView::setPointBrightness(double v)
{
  pointBrightness_ = v;
  updateDisplayColors();
}

double QGLPointCloudView::pointBrightness() const
{
  return pointBrightness_;
}


void QGLPointCloudView::setSceneOrigin(const QVector3D & v)
{
  sceneOrigin_ = v;
  updateDisplayPoints();
}

QVector3D QGLPointCloudView::sceneOrigin() const
{
  return sceneOrigin_;
}


void QGLPointCloudView::rotateToShowCloud()
{
  const int mn =
      displayPoints_.size();

  if( mn > 0 ) {

    cv::Vec3f mv;

    for( const cv::Vec3f & p : displayPoints_ ) {
      mv += p;
    }

    setViewTargetPoint(QVector3D(mv[0] / mn, mv[1] / mn, mv[2] / mn));
  }
}


void QGLPointCloudView::setPoints(cv::InputArray points, cv::InputArray colors, cv::InputArray mask, bool make_copy)
{
  if ( make_copy ) {
    points.getMat().copyTo(currentPoints_);
    colors.getMat().copyTo(currentColors_);
    mask.getMat().copyTo(currentMask_);
  }
  else {
    currentPoints_ = points.getMat();
    currentColors_ = colors.getMat();
    currentMask_ = mask.getMat();
  }

  updateDisplayPoints();
}

void QGLPointCloudView::clearPoints()
{
}

const cv::Mat & QGLPointCloudView::currentPoints() const
{
  return currentPoints_;
}

const cv::Mat & QGLPointCloudView::currentColors() const
{
  return currentColors_;
}

const cv::Mat & QGLPointCloudView::currentMask() const
{
  return currentMask_;
}

const std::vector<cv::Vec3f> & QGLPointCloudView::displayPoints() const
{
  return displayPoints_;
}

const cv::Mat & QGLPointCloudView::mtfColors() const
{
  return mtfColors_;
}

const std::vector<cv::Vec3b> & QGLPointCloudView::displayColors() const
{
  return displayColors_;
}

void QGLPointCloudView::updateDisplayPoints()
{
  update_display_points_ = true;
  update();
}

void QGLPointCloudView::updateDisplayColors()
{
  update_display_colors_ = true;
  update();
}

void QGLPointCloudView::glInit()
{
  Base::glInit();
}

void QGLPointCloudView::glPreDraw()
{
  Base::glPreDraw();

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_ACCUM_BUFFER_BIT);

  glDisable(GL_LIGHTING);
  glDisable(GL_COLOR_MATERIAL);

  glEnable(GL_PROGRAM_POINT_SIZE);
  glEnable(GL_DEPTH_TEST);

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void QGLPointCloudView::glDraw()
{
  computeDisplayPoints();

  if ( !displayPoints_.empty() ) {

    glPointSize(pointSize_);
    glColor3ub(255, 255, 255);

    // activate vertex arrays before array drawing
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, displayPoints_.data());

    if ( !displayColors_.empty() ) {
      glEnableClientState(GL_COLOR_ARRAY);
      glColorPointer(3, GL_UNSIGNED_BYTE, 3, displayColors_.data());
    }

    glDrawArrays(GL_POINTS, 0, displayPoints_.size());

    if ( !displayColors_.empty() ) {
      glDisableClientState(GL_COLOR_ARRAY);
    }
    glDisableClientState(GL_VERTEX_ARRAY);
  }
}


void QGLPointCloudView::computeDisplayPoints()
{
  if( update_display_points_ || update_display_colors_ ) {


    if ( displayFunction_ ) {

      displayFunction_->createDisplayPoints(currentPoints_,
          currentColors_,
          currentMask_,
          displayPoints_,
          mtfColors_,
          displayColors_);
    }

    update_display_points_ = false;
    update_display_colors_ = false;
  }


//  if( update_display_points_ || display_points_.empty() ) {
//
//    int total_points_to_display = 0;
//
//    for( const auto &cloud : clouds_ ) {
//      if( cloud->visible ) {
//        total_points_to_display +=
//            cloud->points.rows;
//      }
//    }
//
//    display_points_.clear();
//
//    if( total_points_to_display > 0 ) {
//
//      display_points_.reserve(total_points_to_display);
//
//      for( const auto &cloud : clouds_ ) {
//        if( cloud->visible && cloud->points.rows > 0 ) {
//
//          const cv::Mat3f &points =
//              cloud->points;
//
//          const double Sx = cloud->Scale.x();
//          const double Sy = cloud->Scale.y();
//          const double Sz = cloud->Scale.z();
//
//          const double Tx = cloud->Translation.x();
//          const double Ty = cloud->Translation.y();
//          const double Tz = cloud->Translation.z();
//
//          const double Rx = cloud->Rotation.x();
//          const double Ry = cloud->Rotation.y();
//          const double Rz = cloud->Rotation.z();
//
//          for( int i = 0; i < points.rows; ++i ) {
//
//            const cv::Vec3f & srcp =
//                points[i][0];
//
//            display_points_.emplace_back(srcp[0] * Sx - Tx - sceneOrigin_.x(),
//                srcp[1] * Sy - Ty - sceneOrigin_.y(),
//                srcp[2] * Sz - Tz - sceneOrigin_.z());
//
//          }
//        }
//      }
//    }
//
//    update_display_colors_ = true;
//  }
//
//  if( update_display_colors_ || display_colors_.size() != display_points_.size() ) {
//
//    display_colors_.clear();
//
//    QMtfDisplay::DisplayParams & opts =
//        mtfDisplay_.displayParams();
//
//    c_pixinsight_mtf &mtf =
//        opts.mtf;
//
//    double imin, imax;
//
//    mtf.get_input_range(&imin, &imax);
//
//    if( imin >= imax ) {
//      mtf.set_input_range(0, 255);
//    }
//
//    for( const auto &cloud : clouds_ ) {
//      if( cloud->visible && cloud->points.rows > 0 ) {
//
//        if ( cloud->colors.rows != cloud->points.rows ) {
//
//          int gray = mtf.apply(255);
//
//          for( int i = 0, n = cloud->points.rows; i < n; ++i ) {
//            display_colors_.emplace_back(gray, gray, gray);
//          }
//
//        }
//        else {
//
//          const cv::Mat & colors =
//              cloud->colors;
//
//          const int channels =
//              colors.channels();
//
//
//          for( int i = 0; i < colors.rows; ++i ) {
//
//            const cv::Scalar color =
//                compute_point_color(colors, i, mtf);
//
//            if ( channels == 1) {
//
//              const int gray =
//                  std::max(0, std::min(255,
//                      (int) (color[0] + pointBrightness_)));
//
//              display_colors_.emplace_back(gray, gray, gray);
//            }
//            else {
//
//              const int red =
//                  std::max(0, std::min(255,
//                      (int) (color[2] + pointBrightness_)));
//
//              const int green =
//                  std::max(0, std::min(255,
//                      (int) (color[1] + pointBrightness_)));
//
//              const int blue =
//                  std::max(0, std::min(255,
//                      (int) (color[0] + pointBrightness_)));
//
//              display_colors_.emplace_back(red, green, blue);
//            }
//          }
//        }
//      }
//    }
//
//    if( imin >= imax ) {
//      mtf.set_input_range(imin, imax);
//    }
//  }
//
//  if ( update_display_colors_ ) {
//    Q_EMIT mtfDisplay_.displayImageChanged();
//  }
//
//  update_display_points_ = false;
//  update_display_colors_ = false;

}

void QGLPointCloudView::onLoadParameters(QSettings & settings)
{
  Base::onLoadParameters(settings);

  sceneOrigin_ =
      settings.value("QGLCloudViewer/sceneOrigin_",
          sceneOrigin_).value<decltype(sceneOrigin_)>();

  pointSize_ =
      settings.value("QGLCloudViewer/pointSize_",
          pointSize_).value<decltype(pointSize_)>();

  pointBrightness_ =
      settings.value("QGLCloudViewer/pointBrightness_",
          pointBrightness_).value<decltype(pointBrightness_)>();

}

void QGLPointCloudView::onSaveParameters(QSettings & settings)
{
  Base::onSaveParameters(settings);

  settings.setValue("QGLCloudViewer/sceneOrigin_", sceneOrigin_);
  settings.setValue("QGLCloudViewer/pointSize_", pointSize_);
  settings.setValue("QGLCloudViewer/pointBrightness_", pointBrightness_);
}


///////////////////////////////////////////////////////////////////////////////////////////////////

QPointCloudSourceView::QPointCloudSourceView(QWidget * parent) :
    Base(parent)
{

}



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

QPointCloudViewSettings::QPointCloudViewSettings(QWidget * parent) :
  Base("QPointCloudViewSettings", parent)
{

  projection_ctl =
      add_enum_combobox<QGLView::Projection>("Projection",
          "Select projection type",
          [this](QGLView::Projection v) {
            if ( cloudView_ && v != cloudView_->projection() ) {
              cloudView_->setProjection(v);
            }
          },
          [this](QGLView::Projection * v) {
            if ( cloudView_ ) {
              * v = cloudView_->projection();
              return true;
            }
            return false;
          });

  nearPlane_ctl =
      add_numeric_box<double>("NearPlane:",
          "",
          [this](double v) {
            if ( cloudView_ && v != cloudView_->nearPlane() ) {
              cloudView_->setNearPlane(v);
            }
          },
          [this](double * v) {
            if ( cloudView_ ) {
              * v = cloudView_->nearPlane();
              return true;
            }
            return false;
          });

  farPlane_ctl =
      add_numeric_box<double>("FarPlane:",
          "",
          [this](double v) {
            if ( cloudView_ && v != cloudView_->farPlane() ) {
              cloudView_->setFarPlane(v);
            }
          },
          [this](double * v) {
            if ( cloudView_ ) {
              * v = cloudView_->farPlane();
              return true;
            }
            return false;
          });

  fov_ctl =
      add_numeric_box<double>("FOV [deg]:",
          "Field of View for Perspective in degrees",
          [this](double v) {
            if ( cloudView_ && v != cloudView_->fov() ) {
              cloudView_->setFOV(v);
            }
          },
          [this](double * v) {
            if ( cloudView_ ) {
              * v = cloudView_->fov();
              return true;
            }
            return false;
          });

  showMainAxes_ctl =
       add_checkbox("Show Main Axes",
           "Set true to show main XYZ coordinate axes",
           [this](bool checked) {
             if ( cloudView_ && cloudView_->showMainAxes() != checked ) {
               cloudView_->setShowMainAxes(checked);
             }
           },
           [this](bool * v) {
             if ( cloudView_ ) {
               * v = cloudView_->showMainAxes();
               return true;
             }
             return false;
           });

  mainAxesLength_ctl =
      add_numeric_box<double>("Main Axes size:",
          "Set to 0 for auto, -1 for disable",
          [this](double v) {
            if ( cloudView_ && v != cloudView_->mainAxesLength() ) {
              cloudView_->setMainAxesLength(v);
            }
          },
          [this](double * v) {
            if ( cloudView_ ) {
              * v = cloudView_->mainAxesLength();
              return true;
            }
            return false;
          });

  sceneTarget_ctl =
      add_numeric_box<QVector3D>("View Target:",
          "",
          [this](const QVector3D & v) {
            if ( cloudView_ ) {
              cloudView_->setViewTargetPoint(v);
            }
          },
          [this](QVector3D * v) {
            if ( cloudView_ ) {
              * v = cloudView_->viewTargetPoint();
              return true;
            }
            return false;
          });

  upDirection_ctl =
      add_numeric_box<QVector3D>("Up:",
          "",
          [this](const QVector3D & v) {
            if ( cloudView_ ) {
              cloudView_->setUpDirection(v);
            }
          },
          [this](QVector3D * v) {
            if ( cloudView_ ) {
              * v = cloudView_->upDirection();
              return true;
            }
            return false;
          });

  sceneOrigin_ctl =
      add_numeric_box<QVector3D>("Origin:",
          "",
          [this](const QVector3D & v) {
            if ( cloudView_ ) {
              cloudView_->setSceneOrigin(v);
            }
          },
          [this](QVector3D * v) {
            if ( cloudView_ ) {
              * v = cloudView_->sceneOrigin();
              return true;
            }
            return false;
          });

  autoShowViewTarget_ctl =
      add_checkbox("Auto Show View Target",
          "",
          [this](bool checked) {
            if ( cloudView_ && cloudView_->autoShowViewTarget() != checked ) {
              cloudView_->setAutoShowViewTarget(checked);
            }
          },
          [this](bool * v) {
            if ( cloudView_ ) {
              * v = cloudView_->autoShowViewTarget();
              return true;
            }
            return false;
          });


  pointSize_ctl =
      add_numeric_box<double>("pointSize:",
          "",
          [this](double v) {
            if ( cloudView_ && v > 0 && v != cloudView_->pointSize() ) {
              cloudView_->setPointSize(v);
            }
          },
          [this](double * v) {
            if ( cloudView_ ) {
              * v = cloudView_->pointSize();
              return true;
            }
            return false;
          });

  pointBrightness_ctl =
      add_numeric_box<double>("pointBrightness",
          "",
          [this](double v) {
            if ( cloudView_ && v != cloudView_->pointBrightness() ) {
              cloudView_->setPointBrightness(v);
            }
          },
          [this](double * v) {
            if ( cloudView_ ) {
              * v = cloudView_->pointBrightness();
              return true;
            }
            return false;
          });


  bgColor_ctl =
      add_widget<QColorPickerButton>("Background Color");

  connect(bgColor_ctl, &QColorPickerButton::colorSelected,
      [this]() {
        if ( cloudView_ && cloudView_->backgroundColor() != bgColor_ctl->color() ) {
          cloudView_->setBackgroundColor(bgColor_ctl->color());
        }
      });

  connect(this, &ThisClass::populatecontrols,
      [this]() {
        if ( cloudView_ ) {
          bgColor_ctl->setColor(cloudView_->backgroundColor());
        }
      });


  updateControls();
}

void QPointCloudViewSettings::setCloudView(QPointCloudSourceView * v)
{
  cloudView_ = v;
  updateControls();
}

QPointCloudSourceView * QPointCloudViewSettings::cloudView() const
{
  return cloudView_;
}


void QPointCloudViewSettings::onupdatecontrols()
{
  if ( !cloudView_ ) {
    setEnabled(false);
  }
  else {
    Base::onupdatecontrols();
    refreshCloudList();
    setEnabled(true);
  }
}

void QPointCloudViewSettings::refreshCloudList()
{
  //cloudsSettings_ctl->refreshCloudList();
}


///////////////////////////////////////////////////////////////////////////////


QPointCloudViewSettingsWidget::QPointCloudViewSettingsWidget(QWidget * parent) :
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
            if ( cloudView_ ) {
              cloudView_->rotateToShowCloud();
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
            if ( cloudView_ ) {
              cloudView_->showKeyBindings();
            }

          }));

  layout->addWidget(settings_ctl = new QPointCloudViewSettings(this) );
  layout->addLayout(buttonBox);

}

void QPointCloudViewSettingsWidget::setCloudView(QPointCloudSourceView * v)
{
  settings_ctl->setCloudView(cloudView_ = v);
  setEnabled(cloudView_ != nullptr);
}

QPointCloudSourceView * QPointCloudViewSettingsWidget::cloudView() const
{
  return cloudView_;
}


///////////////////////////////////////////////////////////////////////////////
QPointCloudViewSettingsDialogBox::QPointCloudViewSettingsDialogBox(QWidget * parent) :
    Base(parent)
{
  setWindowTitle("Cloud View Settings");

  vbox_ = new QVBoxLayout(this);
  vbox_->addWidget(cloudViewSettingsWidget_ = new QPointCloudViewSettingsWidget(this));
}

void QPointCloudViewSettingsDialogBox::setCloudViewer(QPointCloudSourceView * v)
{
  cloudViewSettingsWidget_->setCloudView(v);
}

QPointCloudSourceView * QPointCloudViewSettingsDialogBox::cloudViewer() const
{
  return cloudViewSettingsWidget_->cloudView();
}

void QPointCloudViewSettingsDialogBox::showEvent(QShowEvent *e)
{
  Base::showEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QPointCloudViewSettingsDialogBox::hideEvent(QHideEvent *e)
{
  Base::hideEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}
} /* namespace serstacker */
