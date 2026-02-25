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
  _displayFunction = displayFunc;
  _update_display_points = true;
  update();
}

QCloudViewDisplayFunction * QGLPointCloudView::displayFunction() const
{
  return _displayFunction;
}

void QGLPointCloudView::setPointSize(double v)
{
  _pointSize = v;
  update();
}

double QGLPointCloudView::pointSize() const
{
  return _pointSize;
}

void QGLPointCloudView::setPointBrightness(double v)
{
  _pointBrightness = v;
  updateDisplayColors();
}

double QGLPointCloudView::pointBrightness() const
{
  return _pointBrightness;
}


void QGLPointCloudView::setSceneOrigin(const QVector3D & v)
{
  _sceneOrigin = v;
  updateDisplayPoints();
}

QVector3D QGLPointCloudView::sceneOrigin() const
{
  return _sceneOrigin;
}


void QGLPointCloudView::rotateToShowCloud()
{
  const int mn =
      _displayPoints.size();

  if( mn > 0 ) {

    cv::Vec3f mv;

    for( const cv::Vec3f & p : _displayPoints ) {
      mv += p;
    }

    setViewTargetPoint(QVector3D(mv[0] / mn, mv[1] / mn, mv[2] / mn));
  }
}


void QGLPointCloudView::setPoints(cv::InputArray points, cv::InputArray colors, cv::InputArray mask, bool make_copy)
{
  _currentPids.clear();
  _currentPid2PosMapping.clear();

  if ( make_copy ) {
    points.getMat().copyTo(_currentPoints);
    colors.getMat().copyTo(_currentColors);
    mask.getMat().copyTo(_currentMask);
  }
  else {
    _currentPoints = points.getMat();
    _currentColors = colors.getMat();
    _currentMask = mask.getMat();
  }

  updateDisplayPoints();
}

void QGLPointCloudView::setPoints(cv::Mat && points, cv::Mat && colors, cv::Mat && mask, std::vector<uint64_t> && pids)
{
  _currentPoints = points;
  _currentColors = colors;
  _currentMask = mask;
  _currentPids = pids;

  _currentPid2PosMapping.clear();
  if (!_currentPids.empty()) {
    for (size_t i = 0, n = _currentPids.size(); i < n; ++i) {
      _currentPid2PosMapping.emplace(_currentPids[i], i);
    }
  }

  updateDisplayPoints();
}


void QGLPointCloudView::clearPoints()
{
  _currentPoints.release();
  _currentColors.release();
  _currentMask.release();
  _currentPids.clear();
  updateDisplayPoints();
}

const cv::Mat & QGLPointCloudView::currentPoints() const
{
  return _currentPoints;
}

const cv::Mat & QGLPointCloudView::currentColors() const
{
  return _currentColors;
}

const cv::Mat & QGLPointCloudView::currentMask() const
{
  return _currentMask;
}

const std::vector<uint64_t> & QGLPointCloudView::currentPids() const
{
  return _currentPids;
}

const QGLPointCloudView::PID2POSMapping & QGLPointCloudView::currentPid2PosMapping() const
{
  return _currentPid2PosMapping;
}

const std::vector<cv::Vec3f> & QGLPointCloudView::displayPoints() const
{
  return _displayPoints;
}

const cv::Mat & QGLPointCloudView::mtfColors() const
{
  return _mtfColors;
}

const std::vector<cv::Vec3b> & QGLPointCloudView::displayColors() const
{
  return _displayColors;
}

void QGLPointCloudView::updateDisplayPoints()
{
  _update_display_points = true;
  update();
}

void QGLPointCloudView::updateDisplayColors()
{
  _update_display_colors = true;
  update();
}

//void QGLPointCloudView::setRulerLineEnabled(bool v)
//{
//  if (!(ruler_line_enabled_ = v) && _rulerLine.isVisible()) {
//    _rulerLine.setVisible(false);
//    update();
//  }
//}
//
//bool QGLPointCloudView::isRulerLineEnabled() const
//{
//  return ruler_line_enabled_;
//}

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

  if ( !_displayPoints.empty() ) {

    glPointSize(_pointSize);
    glColor3ub(255, 255, 255);

    // activate vertex arrays before array drawing
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, _displayPoints.data());

    if ( !_displayColors.empty() ) {
      glEnableClientState(GL_COLOR_ARRAY);
      glColorPointer(3, GL_UNSIGNED_BYTE, 3, _displayColors.data());
    }

    glDrawArrays(GL_POINTS, 0, _displayPoints.size());

    if ( !_displayColors.empty() ) {
      glDisableClientState(GL_COLOR_ARRAY);
    }
    glDisableClientState(GL_VERTEX_ARRAY);
  }
}


void QGLPointCloudView::computeDisplayPoints()
{
  if( _update_display_points || _update_display_colors ) {


    if ( _displayFunction ) {

      _displayFunction->createDisplayPoints(_mtfColors,
          _displayPoints,
          _displayColors);
    }

    _update_display_points = false;
    _update_display_colors = false;
  }


}


bool QGLPointCloudView::findPointID(double objX, double objY, double objZ, uint64_t * pid) const
{
  if (!_displayPoints.empty() && _currentPids.size() == _displayPoints.size() ) {

    static const auto distance =
        [](double x1, double y1, double z1, double x2, double y2, double z2) -> double {
          return sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2) );
        };

    int best_point_index = -1;
    double best_distance = DBL_MAX;

    for (int point_index = 0, num_points = _displayPoints.size(); point_index < num_points; ++point_index) {

      const cv::Vec3f &p =
          _displayPoints[point_index];

      const double d =
          distance(p[0], p[1], p[2], objX, objY, objZ);

      if (d < best_distance) {
        best_distance = d;
        best_point_index = point_index;
      }

    }

    if (best_point_index >= 0 && best_point_index < _currentPids.size()) {

      *pid = _currentPids[best_point_index];

      return true;
    }

  }

  return false;
}

//bool QGLPointCloudView::findSelectionPid(const QPointF &click_pos, double objX, double objY, double objZ,
//    uint64_t *pid) const
//{
//  static const auto distance =
//      [](double x1, double y1, double z1, double x2, double y2, double z2) -> double {
//        return sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2) );
//      };
//
//  if (!displayPoints_.empty()) {
//
//    int best_point_index = -1;
//    double best_distance = DBL_MAX;
//
//    for (int point_index = 0, num_points = displayPoints_.size(); point_index < num_points; ++point_index) {
//
//      const cv::Vec3f &p =
//          displayPoints_[point_index];
//
//      const double d =
//          distance(p[0], p[1], p[2], objX, objY, objZ);
//
//      if (d < best_distance) {
//        best_distance = d;
//        best_point_index = point_index;
//      }
//
//    }
//
//    if (best_point_index >= 0 && best_point_index < currentPids_.size()) {
//
//      *pid = currentPids_[best_point_index];
//
//      return true;
//    }
//
//  }
//
//  return false;
//}

void QGLPointCloudView::glMouseEvent(const QPointF & mousePos, QEvent::Type mouseEventType,
    Qt::MouseButtons mouseButtons, Qt::KeyboardModifiers keyboardModifiers,
    bool objHit, double objX, double objY, double objZ)
{
  Q_EMIT glPointMouseEvent(mousePos, mouseEventType,
        mouseButtons, keyboardModifiers,
        objHit, objX, objY, objZ);
}

void QGLPointCloudView::onload(const QSettings & settings, const QString & _prefix)
{
  const QString PREFIX = _prefix.isEmpty() ? "QGLPointCloudView" : _prefix;
  const auto PARAM =
      [PREFIX](const QString & name) {
        return QString("%1/%2").arg(PREFIX).arg(name);
      };

  Base::onload(settings, PREFIX);

  _sceneOrigin = settings.value(PARAM("sceneOrigin"), _sceneOrigin).value<decltype(_sceneOrigin)>();
  _pointSize = settings.value(PARAM("pointSize"),  _pointSize).value<decltype(_pointSize)>();
  _pointBrightness = settings.value(PARAM("pointBrightness"),  _pointBrightness).value<decltype(_pointBrightness)>();
  _viewPoint = settings.value(PARAM("viewPoint"), _viewPoint).value<decltype(_viewPoint)>();
  _viewTarget = settings.value(PARAM("viewTarget"), _viewTarget).value<decltype(_viewTarget)>();
  _viewUpDirection = settings.value(PARAM("viewUpDirection"), _viewUpDirection).value<decltype(_viewUpDirection)>();
}

void QGLPointCloudView::onsave(QSettings & settings, const QString & _prefix)
{
  const QString PREFIX = _prefix.isEmpty() ? "QGLPointCloudView" : _prefix;
  const auto PARAM =
      [PREFIX](const QString & name) {
        return QString("%1/%2").arg(PREFIX).arg(name);
      };

  Base::onsave(settings, PREFIX);

  settings.setValue(PARAM("sceneOrigin"), _sceneOrigin);
  settings.setValue(PARAM("pointSize"),  _pointSize);
  settings.setValue(PARAM("pointBrightness"),  _pointBrightness);
  settings.setValue(PARAM("viewPoint"), _viewPoint);
  settings.setValue(PARAM("viewTarget"), _viewTarget);
  settings.setValue(PARAM("viewUpDirection"), _viewUpDirection);
}


///////////////////////////////////////////////////////////////////////////////////////////////////

QPointCloudSourceView::QPointCloudSourceView(QWidget * parent) :
    Base(parent)
{

}


void QPointCloudSourceView::keyPressEvent(QKeyEvent *e)
{
  if( e->key() == Qt::Key_A && e->modifiers() == Qt::KeyboardModifier::ControlModifier ) {

    setEnableSelection(!enableSelection());

    CF_DEBUG("enableSelection=%d", enableSelection());

    e->ignore();
    return;
  }

  return Base::keyPressEvent(e) ;
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
  Base(parent)
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

//
//void QPointCloudViewSettings::onupdatecontrols()
//{
//  if ( !cloudView_ ) {
//    setEnabled(false);
//  }
//  else {
//    Base::onupdatecontrols();
//    refreshCloudList();
//    setEnabled(true);
//  }
//}

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

void QPointCloudViewSettingsDialogBox::setCloudView(QPointCloudSourceView * v)
{
  cloudViewSettingsWidget_->setCloudView(v);
}

QPointCloudSourceView * QPointCloudViewSettingsDialogBox::cloudView() const
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
