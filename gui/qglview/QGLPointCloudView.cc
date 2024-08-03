/*
 * QGLPointCloudView.cc
 *
 *  Created on: Jun 12, 2024
 *      Author: amyznikov
 */

#include "QGLPointCloudView.h"
#include <gui/widgets/createAction.h>
#include <gui/widgets/style.h>
#include <core/proc/pose.h>
#include <core/debug.h>

#define ICON_add              ":/gui/icons/add.png"
#define ICON_delete           ":/gui/icons/delete2.png"

static int get_items_count(cv::InputArrayOfArrays a)
{
  switch (a.kind()) {
    case cv::_InputArray::MAT:
    case cv::_InputArray::MATX:
    case cv::_InputArray::UMAT:
    case cv::_InputArray::CUDA_GPU_MAT:
    case cv::_InputArray::STD_VECTOR:
    case cv::_InputArray::STD_BOOL_VECTOR:
#if OPENCV_ABI_COMPATIBILITY < 500
    case cv::_InputArray::STD_ARRAY:
#endif
      return 1;

    case cv::_InputArray::STD_VECTOR_VECTOR:
    case cv::_InputArray::STD_VECTOR_MAT:
    case cv::_InputArray::STD_VECTOR_UMAT:
    case cv::_InputArray::STD_VECTOR_CUDA_GPU_MAT:
    case cv::_InputArray::STD_ARRAY_MAT:
      return a.total();
  }

  return 0;
}

static cv::Mat getItem(cv::InputArrayOfArrays a, int i)
{
  if ( !a.empty() ) {

    switch (a.kind()) {
      case cv::_InputArray::MAT:
      case cv::_InputArray::MATX:
      case cv::_InputArray::UMAT:
      case cv::_InputArray::CUDA_GPU_MAT:
      case cv::_InputArray::STD_VECTOR:
      case cv::_InputArray::STD_ARRAY:
      case cv::_InputArray::STD_BOOL_VECTOR:
        return a.getMat(-1);

      case cv::_InputArray::STD_VECTOR_VECTOR:
      case cv::_InputArray::STD_VECTOR_MAT:
      case cv::_InputArray::STD_VECTOR_UMAT:
      case cv::_InputArray::STD_VECTOR_CUDA_GPU_MAT:
      case cv::_InputArray::STD_ARRAY_MAT:
        return a.getMat(i);
    }
  }

  return cv::Mat();
}


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

std::vector<QGLPointCloudView::CloudSettings> & QGLPointCloudView::cloudSettings()
{
  return _cloudSettings;
}

const std::vector<QGLPointCloudView::CloudSettings> & QGLPointCloudView::cloudSettings() const
{
  return _cloudSettings;
}


void QGLPointCloudView::rotateToShowCloud()
{
  int mn = 0;
  cv::Vec3f mv;

  for( size_t i = 0, n = _displayPoints.size(); i < n; ++i ) {
    mn += _displayPoints[i].size();
    for( const cv::Vec3f & p : _displayPoints[i] ) {
      mv += p;
    }
  }

  if( mn > 0 ) {
    setViewTargetPoint(QVector3D(mv[0] / mn, mv[1] / mn, mv[2] / mn));
  }
}


void QGLPointCloudView::setPoints(cv::InputArrayOfArrays points, cv::InputArrayOfArrays colors, cv::InputArrayOfArrays masks, bool make_copy)
{
  _currentPoints.clear();
  _currentColors.clear();
  _currentMasks.clear();

  const int num_clouds =
      get_items_count(points);

  const int num_colors =
      get_items_count(colors);

//  CF_DEBUG("kind=%s num_clouds=%d num_colors=%d", toCString(points.kind()),  num_clouds, num_colors);

  if ( num_clouds > 0 ) {

    for ( int i = 0; i < num_clouds; ++i ) {

      if ( !make_copy ) {
        _currentPoints.emplace_back(getItem(points, i));
        _currentColors.emplace_back(getItem(colors, i));
        _currentMasks.emplace_back(getItem(masks, i));
      }
      else {

        _currentPoints.emplace_back(getItem(points, i).clone());
        _currentColors.emplace_back(getItem(colors, i).clone());
        _currentMasks.emplace_back(getItem(masks, i).clone());
      }
    }
  }

  updateDisplayPoints();
}

void QGLPointCloudView::clearPoints()
{
}

const std::vector<cv::Mat> & QGLPointCloudView::currentPoints() const
{
  return _currentPoints;
}

const std::vector<cv::Mat> & QGLPointCloudView::currentColors() const
{
  return _currentColors;
}

const std::vector<cv::Mat> & QGLPointCloudView::currentMasks() const
{
  return _currentMasks;
}

const cv::Mat & QGLPointCloudView::currentPoints(uint32_t index) const
{
  return _currentPoints[index];
}

const cv::Mat & QGLPointCloudView::currentColors(uint32_t index) const
{
  return _currentColors[index];
}

const cv::Mat & QGLPointCloudView::currentMasks(uint32_t index) const
{
  return _currentMasks[index];
}

const std::vector<std::vector<cv::Vec3f>> & QGLPointCloudView::displayPoints() const
{
  return _displayPoints;
}

const std::vector<std::vector<cv::Vec3b>> & QGLPointCloudView::displayColors() const
{
  return _displayColors;
}

const std::vector<cv::Mat> & QGLPointCloudView::mtfColors() const
{
  return _mtfColors;
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

  if( !_displayPoints.empty() ) {

    glPointSize(_pointSize);
    glColor3ub(255, 255, 255);

    // activate vertex arrays before array drawing
    glEnableClientState(GL_VERTEX_ARRAY);
    if( !_displayColors.empty() ) {
      glEnableClientState(GL_COLOR_ARRAY);
    }

    for( size_t i = 0, n = _displayPoints.size(); i < n; ++i ) {
      if( !_displayPoints[i].empty() ) {

        glVertexPointer(3, GL_FLOAT, 0, _displayPoints[i].data());

        if( !_displayColors[i].empty() ) {
          glColorPointer(3, GL_UNSIGNED_BYTE, 3, _displayColors[i].data());
        }

        glDrawArrays(GL_POINTS, 0, _displayPoints[i].size());
      }
    }

    if( !_displayColors.empty() ) {
      glDisableClientState(GL_COLOR_ARRAY);
    }
    glDisableClientState(GL_VERTEX_ARRAY);
  }
}


void QGLPointCloudView::computeDisplayPoints()
{
  if( _display_lock.try_lock() ) {

   // CF_DEBUG("_update_display_points=%d _update_display_colors=%d", _update_display_points, _update_display_colors);

    if( _update_display_points || _update_display_colors ) {

      if( _displayFunction ) {

        const size_t num_clouds =
            _currentPoints.size();

        _displayPoints.resize(num_clouds);
        _displayColors.resize(num_clouds);
        _mtfColors.resize(num_clouds);

        for( size_t i = 0; i < num_clouds; ++i ) {

          const CloudSettings * opts =
              i < _cloudSettings.size() ? &_cloudSettings[i] :
                  nullptr;

          if( _currentPoints[i].empty() || (opts && !opts->visible) ) {
            _displayPoints[i].clear();
            _displayColors[i].clear();
            _mtfColors[i].release();
          }
          else {

            _displayFunction->createDisplayPoints(_currentPoints[i],
                _currentColors[i],
                _currentMasks[i],
                _displayPoints[i],
                _mtfColors[i],
                _displayColors[i]);

            if ( opts ) {

              if ( opts->override_color ) {
                for ( cv::Vec3b & v : _displayColors[i] ) {
                  v = opts->color;
                }
              }

              const cv::Vec3f S(opts->scale[0] == 0 ? 1 : opts->scale[0],
                  opts->scale[1] == 0 ? 1 : opts->scale[1],
                  opts->scale[2] == 0 ? 1 : opts->scale[2]);

              if ( opts->scale != cv::Vec3f(1,1,1) ) {
                for( cv::Vec3f & v : _displayPoints[i] ) {
                  v[0] *= S[0];
                  v[1] *= S[1];
                  v[2] *= S[2];
                }
              }

              if( opts->rotation != cv::Vec3f(0, 0, 0) ) {

                const cv::Matx33f R =
                    build_rotation(opts->rotation * CV_PI / 180);

                for ( cv::Vec3f & v : _displayPoints[i] ) {
                  v = R * v;
                }
              }

              if ( opts->translation != cv::Vec3f(0,0,0) ) {

                for ( cv::Vec3f & v : _displayPoints[i] ) {
                  v += opts->translation;
                }
              }
            }

          }
        }

      }

      _update_display_points = false;
      _update_display_colors = false;
    }

    _display_lock.unlock();
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

  _sceneOrigin =
      settings.value("QGLCloudViewer/sceneOrigin_",
          _sceneOrigin).value<decltype(_sceneOrigin)>();

  _pointSize =
      settings.value("QGLCloudViewer/pointSize_",
          _pointSize).value<decltype(_pointSize)>();

  _pointBrightness =
      settings.value("QGLCloudViewer/pointBrightness_",
          _pointBrightness).value<decltype(_pointBrightness)>();


  const int num_cloud_settings =
      std::min(64, settings.value("QGLCloudViewer/cloudSettings_size", 0).value<int>());

  _cloudSettings.resize(num_cloud_settings);


  for( int i = 0; i < num_cloud_settings; ++i ) {

    CloudSettings & opts =
            _cloudSettings[i];

    opts.visible =
        settings.value(qsprintf("QGLCloudViewer/cloudSettings%zu_visible", i),
            opts.visible).value<decltype(opts.visible)>();

    opts.override_color =
        settings.value(qsprintf("QGLCloudViewer/cloudSettings%zu_override_color", i),
            opts.override_color).value<decltype(opts.override_color)>();

    opts.translation[0] =
        settings.value(qsprintf("QGLCloudViewer/cloudSettings%zu_translation0", i),
            opts.translation[0]).value<float>();

    opts.translation[1] =
        settings.value(qsprintf("QGLCloudViewer/cloudSettings%zu_translation1", i),
            opts.translation[1]).value<float>();

    opts.translation[2] =
        settings.value(qsprintf("QGLCloudViewer/cloudSettings%zu_translation2", i),
            opts.translation[2]).value<float>();

    opts.rotation[0] =
        settings.value(qsprintf("QGLCloudViewer/cloudSettings%zu_rotation0", i),
            opts.rotation[0]).value<float>();

    opts.rotation[1] =
        settings.value(qsprintf("QGLCloudViewer/cloudSettings%zu_rotation1", i),
            opts.rotation[1]).value<float>();

    opts.rotation[2] =
        settings.value(qsprintf("QGLCloudViewer/cloudSettings%zu_rotation2", i),
            opts.rotation[2]).value<float>();

    opts.scale[0] =
        settings.value(qsprintf("QGLCloudViewer/cloudSettings%zu_scale0", i),
            opts.scale[0]).value<float>();

    opts.scale[1] =
        settings.value(qsprintf("QGLCloudViewer/cloudSettings%zu_scale1", i),
            opts.scale[1]).value<float>();

    opts.scale[2] =
        settings.value(qsprintf("QGLCloudViewer/cloudSettings%zu_scale2", i),
            opts.scale[2]).value<float>();

    opts.color[0] =
        settings.value(qsprintf("QGLCloudViewer/cloudSettings%zu_color0", i),
            opts.color[0]).value<float>();

    opts.color[1] =
        settings.value(qsprintf("QGLCloudViewer/cloudSettings%zu_color1", i),
            opts.color[1]).value<float>();

    opts.color[2] =
        settings.value(qsprintf("QGLCloudViewer/cloudSettings%zu_color2", i),
            opts.color[2]).value<float>();

  }


}

void QGLPointCloudView::onSaveParameters(QSettings & settings)
{
  Base::onSaveParameters(settings);

  settings.setValue("QGLCloudViewer/sceneOrigin_", _sceneOrigin);
  settings.setValue("QGLCloudViewer/pointSize_", _pointSize);
  settings.setValue("QGLCloudViewer/pointBrightness_", _pointBrightness);


  settings.setValue("QGLCloudViewer/cloudSettings_size", (int)_cloudSettings.size());
  for( size_t i = 0, n = _cloudSettings.size(); i < n; ++i ) {

    const CloudSettings & opts =
        _cloudSettings[i];

    settings.setValue(qsprintf("QGLCloudViewer/cloudSettings%zu_visible", i), opts.visible);
    settings.setValue(qsprintf("QGLCloudViewer/cloudSettings%zu_override_color", i), opts.override_color);
    settings.setValue(qsprintf("QGLCloudViewer/cloudSettings%zu_translation0", i), opts.translation[0]);
    settings.setValue(qsprintf("QGLCloudViewer/cloudSettings%zu_translation1", i), opts.translation[1]);
    settings.setValue(qsprintf("QGLCloudViewer/cloudSettings%zu_translation2", i), opts.translation[2]);
    settings.setValue(qsprintf("QGLCloudViewer/cloudSettings%zu_rotation0", i), opts.rotation[0]);
    settings.setValue(qsprintf("QGLCloudViewer/cloudSettings%zu_rotation1", i), opts.rotation[1]);
    settings.setValue(qsprintf("QGLCloudViewer/cloudSettings%zu_rotation2", i), opts.rotation[2]);
    settings.setValue(qsprintf("QGLCloudViewer/cloudSettings%zu_scale0", i), opts.scale[0]);
    settings.setValue(qsprintf("QGLCloudViewer/cloudSettings%zu_scale1", i), opts.scale[1]);
    settings.setValue(qsprintf("QGLCloudViewer/cloudSettings%zu_scale2", i), opts.scale[2]);
    settings.setValue(qsprintf("QGLCloudViewer/cloudSettings%zu_color0", i), opts.color[0]);
    settings.setValue(qsprintf("QGLCloudViewer/cloudSettings%zu_color1", i), opts.color[1]);
    settings.setValue(qsprintf("QGLCloudViewer/cloudSettings%zu_color2", i), opts.color[2]);

  }

}


///////////////////////////////////////////////////////////////////////////////////////////////////



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

QGlPointCloudViewSettings::QGlPointCloudViewSettings(QWidget * parent) :
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

void QGlPointCloudViewSettings::setCloudView(QGLPointCloudView * v)
{
  cloudView_ = v;
  updateControls();
}

QGLPointCloudView * QGlPointCloudViewSettings::cloudView() const
{
  return cloudView_;
}


void QGlPointCloudViewSettings::onupdatecontrols()
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

void QGlPointCloudViewSettings::refreshCloudList()
{
  //cloudsSettings_ctl->refreshCloudList();
}


///////////////////////////////////////////////////////////////////////////////


QGlPointCloudViewSettingsWidget::QGlPointCloudViewSettingsWidget(QWidget * parent) :
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

  layout->addWidget(settings_ctl = new QGlPointCloudViewSettings(this) );
  layout->addLayout(buttonBox);

}

void QGlPointCloudViewSettingsWidget::setCloudView(QGLPointCloudView * v)
{
  settings_ctl->setCloudView(cloudView_ = v);
  setEnabled(cloudView_ != nullptr);
}

QGLPointCloudView * QGlPointCloudViewSettingsWidget::cloudView() const
{
  return cloudView_;
}


///////////////////////////////////////////////////////////////////////////////
QGlPointCloudViewSettingsDialogBox::QGlPointCloudViewSettingsDialogBox(QWidget * parent) :
    Base(parent)
{
  setWindowTitle("Cloud View Settings");

  vbox_ = new QVBoxLayout(this);
  vbox_->addWidget(cloudViewSettingsWidget_ = new QGlPointCloudViewSettingsWidget(this));
}

void QGlPointCloudViewSettingsDialogBox::setCloudViewer(QGLPointCloudView * v)
{
  cloudViewSettingsWidget_->setCloudView(v);
}

QGLPointCloudView * QGlPointCloudViewSettingsDialogBox::cloudViewer() const
{
  return cloudViewSettingsWidget_->cloudView();
}

void QGlPointCloudViewSettingsDialogBox::showEvent(QShowEvent *e)
{
  Base::showEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QGlPointCloudViewSettingsDialogBox::hideEvent(QHideEvent *e)
{
  Base::hideEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}


///////////////////////////////////////////////////////////////////////////////



QGlPointCloudSettingsWidget::QGlPointCloudSettingsWidget(QWidget * parent) :
    Base("", parent)
{
  using CloudSettings = QGLPointCloudView::CloudSettings;

  toolbar_ctl = new QToolBar(this);
  toolbar_ctl->setIconSize(QSize(16, 16));
  toolbar_ctl->setToolButtonStyle(Qt::ToolButtonStyle::ToolButtonIconOnly);

  toolbar_ctl->addWidget(itemSelectionLb_ctl = new QLabel("Cloud:  ", this));

  toolbar_ctl->addWidget(itemSelection_ctl = new QComboBox(this));
  itemSelection_ctl->setEditable(false);

  toolbar_ctl->addAction(addItem_action =
      createAction(getIcon(ICON_add),
          "Add", "Add cloud settings item",
          this,
          &ThisClass::onAddSettingsItem));

  toolbar_ctl->addAction(removeItem_action =
      createAction(getIcon(ICON_delete),
          "Remove", "Delete selected cloud settings item",
          this,
          &ThisClass::onDeleteSettingsItem));

  addRow(toolbar_ctl);

  itemVisible_ctl =
      add_checkbox("Visible", "Show / Hide selected cloud",
          [this](bool checked) {
            CloudSettings * item = currentItem();
            if ( item && item->visible != checked ) {
              item->visible = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) -> bool {
            const CloudSettings * item = currentItem();
            if ( item ) {
              * checked = item->visible;
              return true;
            }
            return false;
          });

  overrideColor_ctl =
      add_checkbox("Override Color", "Override points color for selected cloud",
          [this](bool checked) {
              CloudSettings * item = currentItem();
              if ( item && item->override_color != checked ) {
                item->override_color = checked;
                Q_EMIT parameterChanged();
              }
          },
          [this](bool * checked) -> bool {
            const CloudSettings * item = currentItem();
            if ( item ) {
              * checked = item->override_color;
              return true;
            }
            return false;
          });

  itemColor_ctl =
      add_numeric_box<cv::Vec3b>("Point color:", "Specify point color",
          [this](const cv::Vec3b & v) {
            CloudSettings * item = currentItem();
            if ( item && item->color != v ) {
              item->color = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Vec3b * v) -> bool {
            const CloudSettings * item = currentItem();
            if ( item ) {
              * v = item->color;
              return true;
            }
            return false;
          });

  itemTranslation_ctl =
      add_numeric_box<cv::Vec3f>("Translation:", "Specify Cloud Translation Vector Tx;Ty;Tz",
          [this](const cv::Vec3f & v) {
            CloudSettings * item = currentItem();
            if ( item && item->translation != v ) {
              item->translation = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Vec3f * v) -> bool {
            const CloudSettings * item = currentItem();
            if ( item ) {
              * v = item->translation;
              return true;
            }
            return false;
          });

  itemRotation_ctl =
      add_numeric_box<cv::Vec3f>("Rotation:", "Specify Cloud Rotation Euler Angles Rx;Ry;Rz in degrees",
          [this](const cv::Vec3f & v) {
            CloudSettings * item = currentItem();
            if ( item && item->rotation != v ) {
              item->rotation = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Vec3f * v) -> bool {
            const CloudSettings * item = currentItem();
            if ( item ) {
              * v = item->rotation;
              return true;
            }
            return false;
          });

  itemScale_ctl =
      add_numeric_box<cv::Vec3f>("Scale:", "Specify Cloud Scale Sx;Sy;Sz",
          [this](const cv::Vec3f & v) {
            CloudSettings * item = currentItem();
            if ( item && item->scale != v ) {
              item->scale = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Vec3f * v) -> bool {
            const CloudSettings * item = currentItem();
            if ( item ) {
              * v = item->scale;
              return true;
            }
            return false;
          });

  connect(itemSelection_ctl, (void (QComboBox::*)(int)) (&QComboBox::currentIndexChanged),
      this, &ThisClass::onCurrentItemIndexChanged);


  connect(this, &ThisClass::parameterChanged,
      [this]() {
        if ( cloudView_ ) {
          cloudView_->updateDisplayPoints();
          cloudView_->saveParameters();
        }
      });

  updateControls();
}

QGLPointCloudView * QGlPointCloudSettingsWidget::cloudView() const
{
  return cloudView_;
}

void QGlPointCloudSettingsWidget::setCloudView(QGLPointCloudView * cloudView)
{
  cloudView_ = cloudView;
  populatecombobox();
  updateControls();
}


QGLPointCloudView::CloudSettings * QGlPointCloudSettingsWidget::currentItem() const
{
  if ( cloudView_ ) {

    std::vector<QGLPointCloudView::CloudSettings> & items =
        cloudView_->cloudSettings();

    const int cursel =
        itemSelection_ctl->currentIndex();

    if( cursel >= 0 && cursel < (int) (items.size()) ) {
      return &items[cursel];
    }

  }

  return nullptr;
}


void QGlPointCloudSettingsWidget::onupdatecontrols()
{
  if ( !cloudView_ ) {
    setEnabled(false);
  }
  else {
    Base::populatecontrols();
    updatecontrolstates();
    setEnabled(true);
  }
}

void QGlPointCloudSettingsWidget::populatecombobox()
{
  itemSelection_ctl->clear();

  if ( cloudView_ ) {

    const std::vector<QGLPointCloudView::CloudSettings> & items =
        cloudView_->cloudSettings();

    itemSelection_ctl->setUpdatesEnabled(false);

    for ( size_t i = 0, n = items.size(); i < n; ++i ) {
      itemSelection_ctl->addItem(qsprintf("%zu", i));
    }

    itemSelection_ctl->setUpdatesEnabled(true);
  }

}

void QGlPointCloudSettingsWidget::updatecontrolstates()
{
  const int cursel =
      itemSelection_ctl->currentIndex();

  const bool enableControls =
      cursel >= 0;

  removeItem_action->setEnabled(enableControls);
  itemVisible_ctl->setEnabled(enableControls);
  overrideColor_ctl->setEnabled(enableControls);
  itemColor_ctl->setEnabled(enableControls);
  itemTranslation_ctl->setEnabled(enableControls);
  itemRotation_ctl->setEnabled(enableControls);
  itemScale_ctl->setEnabled(enableControls);
}


void QGlPointCloudSettingsWidget::onAddSettingsItem()
{
  if( cloudView_ ) {

    std::vector<QGLPointCloudView::CloudSettings> & items =
        cloudView_->cloudSettings();

    items.emplace_back();

    itemSelection_ctl->addItem(qsprintf("%zu", items.size() - 1));
    itemSelection_ctl->setCurrentIndex(items.size() - 1);
  }
}

void QGlPointCloudSettingsWidget::onDeleteSettingsItem()
{
  if( cloudView_ ) {

    const int cursel =
        itemSelection_ctl->currentIndex();

    std::vector<QGLPointCloudView::CloudSettings> & items =
        cloudView_->cloudSettings();

    if( cursel >= 0 && cursel < (int) (items.size()) ) {

      const int n =
          itemSelection_ctl->count();

      for( int i = cursel + 1; i < n; ++i ) {
        itemSelection_ctl->setItemText(i, qsprintf("%d", i - 1));
      }

      items.erase(items.begin() + cursel);

      itemSelection_ctl->removeItem(cursel);
    }
  }
}

void QGlPointCloudSettingsWidget::onCurrentItemIndexChanged()
{
  Base::populatecontrols();
  updatecontrolstates();



  const int cursel =
      itemSelection_ctl->currentIndex();

  const std::vector<QGLPointCloudView::CloudSettings> & items =
      cloudView_->cloudSettings();

  if( cursel >= 0 && cursel < (int) (items.size()) ) {
  }


  updatecontrolstates();
}

///////////////////////////////////////////////////////////////////////////////
QGlPointCloudSettingsDialogBox::QGlPointCloudSettingsDialogBox(QWidget * parent) :
    Base(parent)
{
  setWindowTitle("Cloud View Settings");

  vbox_ = new QVBoxLayout(this);
  vbox_->addWidget(cloudSettingsWidget_ = new QGlPointCloudSettingsWidget(this));
}

void QGlPointCloudSettingsDialogBox::setCloudViewer(QGLPointCloudView * v)
{
  cloudSettingsWidget_->setCloudView(v);
}

QGLPointCloudView * QGlPointCloudSettingsDialogBox::cloudViewer() const
{
  return cloudSettingsWidget_->cloudView();
}

void QGlPointCloudSettingsDialogBox::showEvent(QShowEvent *e)
{
  Base::showEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QGlPointCloudSettingsDialogBox::hideEvent(QHideEvent *e)
{
  Base::hideEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

