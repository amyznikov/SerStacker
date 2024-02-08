/*
 * QCloudViewer.cc
 *
 *  Created on: May 18, 2021
 *      Author: amyznikov
 */

#include "QCloudViewer.h"
#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////

bool fromString(const QString & s, QVector3D * vec)
{
  double v[3];

  if( sscanf(s.toUtf8().data(), "%lf ; %lf ; %lf", &v[0], &v[1], &v[2]) == 3 ) {

    vec->setX(v[0]);
    vec->setY(v[1]);
    vec->setZ(v[2]);

    return true;
  }
  return false;
}

QString toQString(const QVector3D & v)
{
  return QString("%1;%2;%3").arg(v.x()).arg(v.y()).arg(v.z());
}

bool load_parameter(const QSettings & settings, const QString & prefix, const char * name,  QVector3D * v)
{
  return fromString(settings.value(QString("%1/%2").arg(prefix).arg(QString(name)), "").toString(), v);
}

void save_parameter(const QString & prefix, const char * name, const QVector3D & value )
{
  QSettings settings;
  settings.setValue(QString("%1/%2").arg(prefix).arg(QString(name)), toQString(value));
}


///////////////////////////////////////////////////////////////////////////////

namespace {

enum DISPLAY_TYPE {
  DISPLAY_PIXEL_VALUE,
};

template<class T>
static bool compute_colors_(const c_pixinsight_mtf &mtf,
    const cv::Mat3f & points, const cv::Mat & src_colors,
    cv::Mat3b & display_colors)
{
  const int src_rows =
      src_colors.rows;

  const int src_cols  =
      src_colors.cols;

  const int src_channels =
      src_colors.channels();

  if ( src_channels == 1 ) {

    display_colors.create(src_rows, 1);

    const cv::Mat_<T> src =
        src_colors;

    for( int i = 0; i < src_rows; ++i ) {

      const int gray =
          mtf.apply(src[i][0]);

      display_colors[i][0][0] = gray;
      display_colors[i][0][1] = gray;
      display_colors[i][0][2] = gray;
    }

  }
  else if ( src_channels == 3 ) {

    display_colors.create(src_rows, 1);

    const cv::Mat_<cv::Vec<T, 3>> src =
        src_colors;

    for( int i = 0; i < src_rows; ++i ) {

      display_colors[i][0][0] = mtf.apply(src[i][0][0]);
      display_colors[i][0][1] = mtf.apply(src[i][0][1]);
      display_colors[i][0][2] = mtf.apply(src[i][0][2]);
    }

  }
  else {
    CF_ERROR("Unsupported number of src_channels=%d",
        src_channels);
    return false;
  }

  return true;
}

static bool compute_colors(const c_pixinsight_mtf &mtf,
    const cv::Mat3f & points, const cv::Mat & src_colors,
    cv::Mat3b & display_colors)
{
  switch (src_colors.depth()) {
    case CV_8U:
      return compute_colors_<uint8_t>(mtf, points, src_colors, display_colors);
    case CV_8S:
      return compute_colors_<int8_t>(mtf, points, src_colors, display_colors);
    case CV_16U:
      return compute_colors_<uint16_t>(mtf, points, src_colors, display_colors);
    case CV_16S:
      return compute_colors_<int16_t>(mtf, points, src_colors, display_colors);
    case CV_32S:
      return compute_colors_<int32_t>(mtf, points, src_colors, display_colors);
    case CV_32F:
      return compute_colors_<float>(mtf, points, src_colors, display_colors);
    case CV_64F:
      return compute_colors_<double>(mtf, points, src_colors, display_colors);
  }

  display_colors.release();
  return false;
}


cv::Scalar get_point_color(const cv::Mat & src, int r)
{
  cv::Scalar s;

  const int channels =
      src.channels();

  switch (src.depth()) {
    case CV_8U: {
      const uint8_t *p = src.ptr<const uint8_t>(r);
      for( int c = 0; c < channels; ++c ) {
        s[c] = p[c];
      }
      break;
    }
    case CV_8S: {
      const int8_t *p = src.ptr<const int8_t>(r);
      for( int c = 0; c < channels; ++c ) {
        s[c] = p[c];
      }
      break;
    }
    case CV_16U: {
      const uint16_t *p = src.ptr<const uint16_t>(r);
      for( int c = 0; c < channels; ++c ) {
        s[c] = p[c];
      }
      break;
    }
    case CV_16S: {
      const int16_t *p = src.ptr<const int16_t>(r);
      for( int c = 0; c < channels; ++c ) {
        s[c] = p[c];
      }
      break;
    }
    case CV_32S: {
      const int32_t *p = src.ptr<const int32_t>(r);
      for( int c = 0; c < channels; ++c ) {
        s[c] = p[c];
      }
      break;
    }
    case CV_32F: {
      const float *p = src.ptr<const float>(r);
      for( int c = 0; c < channels; ++c ) {
        s[c] = p[c];
      }
      break;
    }
    case CV_64F: {
      const double *p = src.ptr<const double>(r);
      for( int c = 0; c < channels; ++c ) {
        s[c] = p[c];
      }
      break;
    }
    default:
      break;
  }

  return s;
}

cv::Scalar compute_point_color(const cv::Mat & src, int r, const c_pixinsight_mtf & mtf)
{
  cv::Scalar s;

  const int channels =
      src.channels();

  switch (src.depth()) {
    case CV_8U: {
      const uint8_t *p = src.ptr<const uint8_t>(r);
      for( int c = 0; c < channels; ++c ) {
        s[c] = mtf.apply(p[c]);
      }
      break;
    }
    case CV_8S: {
      const int8_t *p = src.ptr<const int8_t>(r);
      for( int c = 0; c < channels; ++c ) {
        s[c] = mtf.apply(p[c]);
      }
      break;
    }
    case CV_16U: {
      const uint16_t *p = src.ptr<const uint16_t>(r);
      for( int c = 0; c < channels; ++c ) {
        s[c] = mtf.apply(p[c]);
      }
      break;
    }
    case CV_16S: {
      const int16_t *p = src.ptr<const int16_t>(r);
      for( int c = 0; c < channels; ++c ) {
        s[c] = mtf.apply(p[c]);
      }
      break;
    }
    case CV_32S: {
      const int32_t *p = src.ptr<const int32_t>(r);
      for( int c = 0; c < channels; ++c ) {
        s[c] = mtf.apply(p[c]);
      }
      break;
    }
    case CV_32F: {
      const float *p = src.ptr<const float>(r);
      for( int c = 0; c < channels; ++c ) {
        s[c] = mtf.apply(p[c]);
      }
      break;
    }
    case CV_64F: {
      const double *p = src.ptr<const double>(r);
      for( int c = 0; c < channels; ++c ) {
        s[c] = mtf.apply(p[c]);
      }
      break;
    }
    default:
      break;
  }

  return s;
}

} // namespace

template<>
const c_enum_member* members_of<DISPLAY_TYPE>()
{
  static const c_enum_member members[] = {
      { DISPLAY_PIXEL_VALUE, "PIXEL_VALUE" },
      { DISPLAY_PIXEL_VALUE }
  };

  return members;
}



QCloudViewMtfDisplay::QCloudViewMtfDisplay(QGLCloudViewer * cloudView) :
    Base("", cloudView),
    cloudView_(cloudView)
{
  QMtfDisplay::displayChannel_ = "PIXEL_VALUE";
  QMtfDisplay::addDisplay(QMtfDisplay::displayChannel_, 0, 255);
}

QGLCloudViewer * QCloudViewMtfDisplay::cloudView() const
{
  return cloudView_;
}

QStringList QCloudViewMtfDisplay::displayChannels() const
{
  QStringList sl;

  for ( const auto & p : displays_ ) {
    sl.append(p.first);
  }

  return sl;

  //return members_of<DISPLAY_TYPE>();
}

void QCloudViewMtfDisplay::getInputDataRange(double * minval, double * maxval) const
{
  *minval = HUGE_VAL;
  *maxval = 0;

  if( cloudView_ ) {

    double minv, maxv;

    for( const QPointCloud::sptr &cloud : cloudView_->clouds() ) {
      if( cloud->visible && cloud->colors.rows > 0 ) {

        cv::minMaxLoc(cloud->colors, &minv, &maxv);

        if( minv < *minval ) {
          *minval = minv;
        }
        if( maxv > *maxval ) {
          *maxval = maxv;
        }
      }
    }
  }
}

void QCloudViewMtfDisplay::getInputHistogramm(cv::OutputArray H, double * imin, double * imax)
{
  H.release();

  if( !cloudView_ || cloudView_->clouds().empty() ) {
    *imin = *imax = 0;
    return;
  }

  int max_channels = 0;

  for( const QPointCloud::sptr &cloud : cloudView_->clouds() ) {
    if( cloud->visible && cloud->colors.rows > 0 ) {

      const int channels =
          cloud->colors.channels();

      if( channels > max_channels ) {
        max_channels = channels;
      }
    }
  }

  if( max_channels > 0 ) {

    c_histogram_builder builder;

    getInputDataRange(imin, imax);

    builder.set_input_range(*imin, *imax);
    builder.set_channels(max_channels);
    builder.set_bins(256);

    for( const QPointCloud::sptr &cloud : cloudView_->clouds() ) {
      if( cloud->visible && cloud->colors.rows > 0 ) {

        for( int i = 0; i < cloud->colors.rows; ++i ) {
          builder.add_pixel(get_point_color(cloud->colors, i));
        }

      }
    }

    builder.compute(H);
  }
}

void QCloudViewMtfDisplay::getOutputHistogramm(cv::OutputArray H, double * omin, double * omax)
{
  displayParams().mtf.get_output_range(omin, omax);
  H.release();

  if( !cloudView_ || cloudView_->display_colors_.empty() ) {
    return;
  }

  c_histogram_builder builder;

  builder.set_input_range(0, 255);
  builder.set_channels(3);
  builder.set_bins(256);

  for ( const cv::Vec3b & color : cloudView_->display_colors_ ) {
    builder.add_pixel(color);
  }

  builder.compute(H);
}


///////////////////////////////////////////////////////////////////////////////


QGLCloudViewer::QGLCloudViewer(QWidget* parent) :
  Base(parent),
  mtfDisplay_(this)
{
  connect(&mtfDisplay_, &QMtfDisplay::displayTypeChanged,
      &mtfDisplay_, &QMtfDisplay::parameterChanged);

  connect(&mtfDisplay_, &QCloudViewMtfDisplay::parameterChanged,
      this, &ThisClass::updateDisplayColors);
}

QCloudViewMtfDisplay & QGLCloudViewer::mtfDisplay()
{
  return mtfDisplay_;
}

const QCloudViewMtfDisplay & QGLCloudViewer::mtfDisplay() const
{
  return mtfDisplay_;
}

const std::vector<QPointCloud::sptr> & QGLCloudViewer::clouds() const
{
  return clouds_;
}

const QPointCloud::sptr & QGLCloudViewer::cloud(int index) const
{
  return clouds_[index];
}

void QGLCloudViewer::add(const QPointCloud::sptr & cloud)
{
  clouds_.emplace_back(cloud);
  updateDisplayPoints();
}


void QGLCloudViewer::clear()
{
  clouds_.clear();
  updateDisplayPoints();
}

void QGLCloudViewer::rotateToShowCloud()
{
  cv::Vec3f mv;
  int mn = 0;

  for( const QPointCloud::sptr &cloud : clouds_ ) {

    const cv::Mat3f &points =
        cloud->points;

    for( int i = 0; i < points.rows; ++i ) {
      mv += points[i][0];
      mn += 1;
    }
  }

  if( mn > 0 ) {
    setViewTargetPoint(QVector3D(mv[0] / mn, mv[1] / mn, mv[2] / mn));
  }
}

void QGLCloudViewer::setSceneOrigin(const QVector3D & v)
{
  sceneOrigin_ = v;
  updateDisplayPoints();
}

QVector3D QGLCloudViewer::sceneOrigin() const
{
  return sceneOrigin_;
}

void QGLCloudViewer::setPointSize(double v)
{
  pointSize_ = v;
  update();
}

double QGLCloudViewer::pointSize() const
{
  return pointSize_;
}

void QGLCloudViewer::setPointBrightness(double v)
{
  pointBrightness_ = v;
  updateDisplayColors();
}

double QGLCloudViewer::pointBrightness() const
{
  return pointBrightness_;
}


void QGLCloudViewer::glInit()
{
  Base::glInit();
  // setBackgroundColor(QColor(32, 32, 32));
}

void QGLCloudViewer::glPreDraw()
{
  Base::glPreDraw();

  glDisable(GL_LIGHTING);
  glDisable(GL_COLOR_MATERIAL);

  glEnable(GL_PROGRAM_POINT_SIZE);
  glEnable(GL_DEPTH_TEST);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_ACCUM_BUFFER_BIT);
}

void QGLCloudViewer::glPostDraw()
{
  if ( showMainAxes_ ) {
    glColor3ub(200, 200, 200);
    drawMainAxes();
  }

  Base::glPostDraw();
}

void QGLCloudViewer::glDraw()
{
  computeDisplayPoints();

  if ( !display_points_.empty() ) {

    glPointSize(pointSize_);
    glColor3ub(255, 255, 255);

    // activate vertex arrays before array drawing
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, display_points_.data());

    if ( !display_colors_.empty() ) {
      glEnableClientState(GL_COLOR_ARRAY);
      glColorPointer(3, GL_UNSIGNED_BYTE, 3, display_colors_.data());
    }

    glDrawArrays(GL_POINTS, 0, display_points_.size());

    if ( !display_colors_.empty() ) {
      glDisableClientState(GL_COLOR_ARRAY);
    }
    glDisableClientState(GL_VERTEX_ARRAY);
  }

}

void QGLCloudViewer::updateDisplayPoints()
{
  update_display_points_ = true;
  update();
}

void QGLCloudViewer::updateDisplayColors()
{
  update_display_colors_ = true;
  update();
}

void QGLCloudViewer::computeDisplayPoints()
{
  if( update_display_points_ || display_points_.empty() ) {

    int total_points_to_display = 0;

    for( const auto &cloud : clouds_ ) {
      if( cloud->visible ) {
        total_points_to_display +=
            cloud->points.rows;
      }
    }

    display_points_.clear();

    if( total_points_to_display > 0 ) {

      display_points_.reserve(total_points_to_display);

      for( const auto &cloud : clouds_ ) {
        if( cloud->visible && cloud->points.rows > 0 ) {

          const cv::Mat3f &points =
              cloud->points;

          const double Sx = cloud->Scale.x();
          const double Sy = cloud->Scale.y();
          const double Sz = cloud->Scale.z();

          const double Tx = cloud->Translation.x();
          const double Ty = cloud->Translation.y();
          const double Tz = cloud->Translation.z();

          const double Rx = cloud->Rotation.x();
          const double Ry = cloud->Rotation.y();
          const double Rz = cloud->Rotation.z();

          for( int i = 0; i < points.rows; ++i ) {

            const cv::Vec3f & srcp =
                points[i][0];

            display_points_.emplace_back(srcp[0] * Sx - Tx - sceneOrigin_.x(),
                srcp[1] * Sy - Ty - sceneOrigin_.y(),
                srcp[2] * Sz - Tz - sceneOrigin_.z());

          }
        }
      }
    }

    update_display_colors_ = true;
  }

  if( update_display_colors_ || display_colors_.size() != display_points_.size() ) {

    display_colors_.clear();

    QMtfDisplay::DisplayParams & opts =
        mtfDisplay_.displayParams();

    c_pixinsight_mtf &mtf =
        opts.mtf;

    double imin, imax;

    mtf.get_input_range(&imin, &imax);

    if( imin >= imax ) {
      mtf.set_input_range(0, 255);
    }

    for( const auto &cloud : clouds_ ) {
      if( cloud->visible && cloud->points.rows > 0 ) {

        if ( cloud->colors.rows != cloud->points.rows ) {

          int gray = mtf.apply(255);

          for( int i = 0, n = cloud->points.rows; i < n; ++i ) {
            display_colors_.emplace_back(gray, gray, gray);
          }

        }
        else {

          const cv::Mat & colors =
              cloud->colors;

          const int channels =
              colors.channels();


          for( int i = 0; i < colors.rows; ++i ) {

            const cv::Scalar color =
                compute_point_color(colors, i, mtf);

            if ( channels == 1) {

              const int gray =
                  std::max(0, std::min(255,
                      (int) (color[0] + pointBrightness_)));

              display_colors_.emplace_back(gray, gray, gray);
            }
            else {

              const int red =
                  std::max(0, std::min(255,
                      (int) (color[2] + pointBrightness_)));

              const int green =
                  std::max(0, std::min(255,
                      (int) (color[1] + pointBrightness_)));

              const int blue =
                  std::max(0, std::min(255,
                      (int) (color[0] + pointBrightness_)));

              display_colors_.emplace_back(red, green, blue);
            }
          }
        }
      }
    }

    if( imin >= imax ) {
      mtf.set_input_range(imin, imax);
    }
  }

  if ( update_display_colors_ ) {
    Q_EMIT mtfDisplay_.displayImageChanged();
  }

  update_display_points_ = false;
  update_display_colors_ = false;
}

bool QGLCloudViewer::openPlyFile(const QString & pathFileName)
{
  QPointCloud::sptr cloud =
      QPointCloud::create();

  if ( !loadPlyFile(pathFileName, cloud.get()) ) {
    CF_ERROR("loadPlyFile('%s') fails",
        pathFileName.toStdString().c_str());
    return false;
  }

  add(cloud);

  return true;
}


void QGLCloudViewer::onLoadParameters(QSettings & settings)
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

void QGLCloudViewer::onSaveParameters(QSettings & settings)
{
  Base::onSaveParameters(settings);

  settings.setValue("QGLCloudViewer/sceneOrigin_", sceneOrigin_);
  settings.setValue("QGLCloudViewer/pointSize_", pointSize_);
  settings.setValue("QGLCloudViewer/pointBrightness_", pointBrightness_);
}


///////////////////////////////////////////////////////////////////////////////

QCloudViewer::QCloudViewer(QWidget* parent) :
    Base(parent)
{
  layout_ = new QVBoxLayout(this);
  layout_->setContentsMargins(0, 0, 0, 0);

  glViewer_ = new QGLCloudViewer(this);

  layout_->addWidget(glViewer_, 100);

  connect(glViewer_, &QGLCloudViewer::displayImageChanged,
      this, &ThisClass::displayImageChanged);
}

void QCloudViewer::loadParameters()
{
  glViewer_->loadParameters();
}

void QCloudViewer::saveParameters()
{
  glViewer_->saveParameters();
}

void QCloudViewer::showEvent(QShowEvent * e)
{
  Base::showEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QCloudViewer::hideEvent(QHideEvent * e)
{
  Base::hideEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

QCloudViewMtfDisplay & QCloudViewer::mtfDisplay()
{
  return glViewer_->mtfDisplay();
}

const QCloudViewMtfDisplay & QCloudViewer::mtfDisplay() const
{
  return glViewer_->mtfDisplay();
}

QToolBar* QCloudViewer::toolbar()
{
  if( !toolbar_ ) {
    toolbar_ = new QToolBar(this);
    toolbar_->setToolButtonStyle(Qt::ToolButtonIconOnly);
    toolbar_->setOrientation(Qt::Horizontal);
    toolbar_->setIconSize(QSize(16, 16));
    layout_->insertWidget(0, toolbar_, 1, Qt::AlignTop);
  }

  return toolbar_;
}

QGLCloudViewer * QCloudViewer::cloudView() const
{
  return glViewer_;
}

void QCloudViewer::setAutoShowViewTarget(bool v)
{
  glViewer_->setAutoShowViewTarget(v);
}

bool QCloudViewer::autoShowViewTarget() const
{
  return glViewer_->autoShowViewTarget();
}

void QCloudViewer::setPointSize(double v)
{
  glViewer_->setPointSize(v);
}

double QCloudViewer::pointSize() const
{
  return glViewer_->pointSize();
}

void QCloudViewer::setPointBrightness(double v)
{
  glViewer_->setPointBrightness(v);
}

double QCloudViewer::pointBrightness() const
{
  return glViewer_->pointBrightness();
}

void QCloudViewer::setBackgroundColor(const QColor &color)
{
  glViewer_->setBackgroundColor(color);
}

const QColor & QCloudViewer::backgroundColor() const
{
  return glViewer_->backgroundColor();
}

void QCloudViewer::setSceneOrigin(const QVector3D & v)
{
  glViewer_->setSceneOrigin(v);
}

QVector3D QCloudViewer::sceneOrigin() const
{
  return glViewer_->sceneOrigin();
}

void QCloudViewer::add(const QPointCloud::sptr & cloud)
{
  glViewer_->add(cloud);
}

void QCloudViewer::clear()
{
  glViewer_->clear();
  Q_EMIT currentFileNameChanged();
}

void QCloudViewer::updateDisplayPoints()
{
  glViewer_->updateDisplayPoints();
}

void QCloudViewer::updateDisplayColors()
{
  glViewer_->updateDisplayColors();
}

void QCloudViewer::redraw()
{
  glViewer_->update();
}

bool QCloudViewer::openPlyFile(const QString & pathFileName)
{
  glViewer_->clear();

  bool fOk =
      glViewer_->openPlyFile(
          pathFileName);

  currentFileName_ =
        pathFileName;

  Q_EMIT currentFileNameChanged();

  return fOk;
}

void QCloudViewer::setCurrentFileName(const QString & v)
{
  currentFileName_ = v;
  Q_EMIT currentFileNameChanged();
}

const QString & QCloudViewer::currentFileName() const
{
  return currentFileName_;
}

void QCloudViewer::rotateToShowCloud()
{
  glViewer_->rotateToShowCloud();
}

void QCloudViewer::showKeyBindings()
{
  glViewer_->showKeyBindings();
}

bool QCloudViewer::copyViewportToClipboard()
{
  return glViewer_->copyViewportToClipboard();
}

QPixmap QCloudViewer::grabViewportPixmap()
{
  return glViewer_->grab();
}

