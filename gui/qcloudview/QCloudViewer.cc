/*
 * QCloudViewer.cc
 *
 *  Created on: May 18, 2021
 *      Author: amyznikov
 */

#include "QCloudViewer.h"
#include <core/debug.h>

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
  QMtfDisplay::_displayChannel = "PIXEL_VALUE";
  QMtfDisplay::addDisplay(QMtfDisplay::_displayChannel, 0, 255);
}

QGLCloudViewer * QCloudViewMtfDisplay::cloudView() const
{
  return cloudView_;
}

QStringList QCloudViewMtfDisplay::displayChannels() const
{
  QStringList sl;

  for ( const auto & p : _displays ) {
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

//void QCloudViewMtfDisplay::getInputHistogramm(cv::OutputArray H, double * imin, double * imax)
//{
//  H.release();
//
//  if( !cloudView_ || cloudView_->clouds().empty() ) {
//    *imin = *imax = 0;
//    return;
//  }
//
//  int max_channels = 0;
//
//  for( const QPointCloud::sptr &cloud : cloudView_->clouds() ) {
//    if( cloud->visible && cloud->colors.rows > 0 ) {
//
//      const int channels =
//          cloud->colors.channels();
//
//      if( channels > max_channels ) {
//        max_channels = channels;
//      }
//    }
//  }
//
//  if( max_channels > 0 ) {
//
//    c_histogram_builder builder;
//
//    getInputDataRange(imin, imax);
//
//    builder.set_input_range(*imin, *imax);
//    builder.set_channels(max_channels);
//    builder.set_bins(256);
//
//    for( const QPointCloud::sptr &cloud : cloudView_->clouds() ) {
//      if( cloud->visible && cloud->colors.rows > 0 ) {
//
//        for( int i = 0; i < cloud->colors.rows; ++i ) {
//          builder.add_pixel(get_point_color(cloud->colors, i));
//        }
//
//      }
//    }
//
//    builder.compute(H);
//  }
//}

void QCloudViewMtfDisplay::getOutputHistogramm(cv::OutputArray H, double * omin, double * omax)
{
  displayParams().mtf.get_output_range(omin, omax);
  H.release();

  if( !cloudView_ || cloudView_->_display_colors.empty() ) {
    return;
  }

  c_histogram_builder builder;

  builder.set_input_range(0, 255);
  builder.set_channels(3);
  builder.set_bins(256);

  for ( const cv::Vec3b & color : cloudView_->_display_colors ) {
    builder.add_pixel(color);
  }

  builder.compute(H);
}


///////////////////////////////////////////////////////////////////////////////


QGLCloudViewer::QGLCloudViewer(QWidget* parent) :
  Base(parent),
  _mtfDisplay(this)
{
  connect(&_mtfDisplay, &QMtfDisplay::displayChannelsChanged,
      &_mtfDisplay, &QMtfDisplay::parameterChanged);

  connect(&_mtfDisplay, &QCloudViewMtfDisplay::parameterChanged,
      this, &ThisClass::updateDisplayColors);
}

QCloudViewMtfDisplay & QGLCloudViewer::mtfDisplay()
{
  return _mtfDisplay;
}

const QCloudViewMtfDisplay & QGLCloudViewer::mtfDisplay() const
{
  return _mtfDisplay;
}

const std::vector<QPointCloud::sptr> & QGLCloudViewer::clouds() const
{
  return _clouds;
}

const QPointCloud::sptr & QGLCloudViewer::cloud(int index) const
{
  return _clouds[index];
}

void QGLCloudViewer::add(const QPointCloud::sptr & cloud)
{
  _clouds.emplace_back(cloud);
  updateDisplayPoints();
}


void QGLCloudViewer::clear()
{
  _clouds.clear();
  updateDisplayPoints();
}

void QGLCloudViewer::rotateToShowCloud()
{
  cv::Vec3f mv;
  int mn = 0;

  for( const QPointCloud::sptr &cloud : _clouds ) {

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
  _sceneOrigin = v;
  updateDisplayPoints();
}

QVector3D QGLCloudViewer::sceneOrigin() const
{
  return _sceneOrigin;
}

void QGLCloudViewer::setPointSize(double v)
{
  _pointSize = v;
  update();
}

double QGLCloudViewer::pointSize() const
{
  return _pointSize;
}

void QGLCloudViewer::setPointBrightness(double v)
{
  _pointBrightness = v;
  updateDisplayColors();
}

double QGLCloudViewer::pointBrightness() const
{
  return _pointBrightness;
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
  if ( showMainAxes() ) {
    glColor3ub(200, 200, 200);
    drawMainAxes();
  }

  Base::glPostDraw();
}

void QGLCloudViewer::glDraw()
{
  computeDisplayPoints();

  if ( !_display_points.empty() ) {

    glPointSize(_pointSize);
    glColor3ub(255, 255, 255);

    // activate vertex arrays before array drawing
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, _display_points.data());

    if ( !_display_colors.empty() ) {
      glEnableClientState(GL_COLOR_ARRAY);
      glColorPointer(3, GL_UNSIGNED_BYTE, 3, _display_colors.data());
    }

    glDrawArrays(GL_POINTS, 0, _display_points.size());

    if ( !_display_colors.empty() ) {
      glDisableClientState(GL_COLOR_ARRAY);
    }
    glDisableClientState(GL_VERTEX_ARRAY);
  }

}

void QGLCloudViewer::updateDisplayPoints()
{
  _update_display_points = true;
  update();
}

void QGLCloudViewer::updateDisplayColors()
{
  _update_display_colors = true;
  update();
}

void QGLCloudViewer::computeDisplayPoints()
{
  if( _update_display_points || _display_points.empty() ) {

    int total_points_to_display = 0;

    for( const auto &cloud : _clouds ) {
      if( cloud->visible ) {
        total_points_to_display +=
            cloud->points.rows;
      }
    }

    _display_points.clear();

    if( total_points_to_display > 0 ) {

      _display_points.reserve(total_points_to_display);

      for( const auto &cloud : _clouds ) {
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

            _display_points.emplace_back(srcp[0] * Sx - Tx - _sceneOrigin.x(),
                srcp[1] * Sy - Ty - _sceneOrigin.y(),
                srcp[2] * Sz - Tz - _sceneOrigin.z());

          }
        }
      }
    }

    _update_display_colors = true;
  }

  if( _update_display_colors || _display_colors.size() != _display_points.size() ) {

    _display_colors.clear();

    QMtfDisplay::DisplayParams & opts =
        _mtfDisplay.displayParams();

    c_pixinsight_mtf &mtf =
        opts.mtf;

    double imin, imax;

    mtf.get_input_range(&imin, &imax);

    if( imin >= imax ) {
      mtf.set_input_range(0, 255);
    }

    for( const auto &cloud : _clouds ) {
      if( cloud->visible && cloud->points.rows > 0 ) {

        if ( cloud->colors.rows != cloud->points.rows ) {

          int gray = mtf.apply(255);

          for( int i = 0, n = cloud->points.rows; i < n; ++i ) {
            _display_colors.emplace_back(gray, gray, gray);
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
                      (int) (color[0] + _pointBrightness)));

              _display_colors.emplace_back(gray, gray, gray);
            }
            else {

              const int red =
                  std::max(0, std::min(255,
                      (int) (color[2] + _pointBrightness)));

              const int green =
                  std::max(0, std::min(255,
                      (int) (color[1] + _pointBrightness)));

              const int blue =
                  std::max(0, std::min(255,
                      (int) (color[0] + _pointBrightness)));

              _display_colors.emplace_back(red, green, blue);
            }
          }
        }
      }
    }

    if( imin >= imax ) {
      mtf.set_input_range(imin, imax);
    }
  }

  if ( _update_display_colors ) {
    Q_EMIT _mtfDisplay.displayImageChanged();
  }

  _update_display_points = false;
  _update_display_colors = false;
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


void QGLCloudViewer::onload(const QSettings & settings, const QString & _prefix)
{
  const QString PREFIX = _prefix.isEmpty() ? "QGLCloudViewer" : _prefix;
  const auto PARAM =
      [PREFIX](const QString & name) {
        return QString("%1/%2").arg(PREFIX).arg(name);
      };

  Base::onload(settings, PREFIX);

  _sceneOrigin = settings.value(PARAM("sceneOrigin"), _sceneOrigin).value<decltype(_sceneOrigin)>();
  _pointSize = settings.value(PARAM("pointSize"), _pointSize).value<decltype(_pointSize)>();
  _pointBrightness = settings.value(PARAM("pointBrightness"), _pointBrightness).value<decltype(_pointBrightness)>();
}

void QGLCloudViewer::onsave(QSettings & settings, const QString & _prefix)
{
  const QString PREFIX = _prefix.isEmpty() ? "QGLCloudViewer" : _prefix;
  const auto PARAM =
      [PREFIX](const QString & name) {
        return QString("%1/%2").arg(PREFIX).arg(name);
      };

  Base::onsave(settings, PREFIX);

  settings.setValue(PARAM("sceneOrigin"), _sceneOrigin);
  settings.setValue(PARAM("pointSize"), _pointSize);
  settings.setValue(PARAM("pointBrightness"), _pointBrightness);
}


///////////////////////////////////////////////////////////////////////////////

QCloudViewer::QCloudViewer(QWidget* parent) :
    Base(parent)
{
  _layout = new QVBoxLayout(this);
  _layout->setContentsMargins(0, 0, 0, 0);

  _glViewer = new QGLCloudViewer(this);

  _layout->addWidget(_glViewer, 100);

  connect(_glViewer, &QGLCloudViewer::displayImageChanged,
      this, &ThisClass::displayImageChanged);
}

void QCloudViewer::loadSettings(const QString & prefix)
{
  _glViewer->loadSettings(prefix);
}

void QCloudViewer::loadSettings(const QSettings & settings, const QString & prefix)
{
  _glViewer->loadSettings(settings, prefix);
}

void QCloudViewer::saveSettings(const QString & prefix)
{
  _glViewer->saveSettings(prefix);
}

void QCloudViewer::saveSettings(QSettings & settings, const QString & prefix)
{
  _glViewer->saveSettings(settings, prefix);
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
  return _glViewer->mtfDisplay();
}

const QCloudViewMtfDisplay & QCloudViewer::mtfDisplay() const
{
  return _glViewer->mtfDisplay();
}

QToolBar* QCloudViewer::toolbar()
{
  if( !_toolbar ) {
    _toolbar = new QToolBar(this);
    _toolbar->setToolButtonStyle(Qt::ToolButtonIconOnly);
    _toolbar->setOrientation(Qt::Horizontal);
    _toolbar->setIconSize(QSize(16, 16));
    _layout->insertWidget(0, _toolbar, 1, Qt::AlignTop);
  }

  return _toolbar;
}

QGLCloudViewer * QCloudViewer::cloudView() const
{
  return _glViewer;
}

void QCloudViewer::setAutoShowViewTarget(bool v)
{
  _glViewer->setAutoShowViewTarget(v);
}

bool QCloudViewer::autoShowViewTarget() const
{
  return _glViewer->autoShowViewTarget();
}

void QCloudViewer::setPointSize(double v)
{
  _glViewer->setPointSize(v);
}

double QCloudViewer::pointSize() const
{
  return _glViewer->pointSize();
}

void QCloudViewer::setPointBrightness(double v)
{
  _glViewer->setPointBrightness(v);
}

double QCloudViewer::pointBrightness() const
{
  return _glViewer->pointBrightness();
}

void QCloudViewer::setBackgroundColor(const QColor &color)
{
  _glViewer->setBackgroundColor(color);
}

const QColor & QCloudViewer::backgroundColor() const
{
  return _glViewer->backgroundColor();
}

void QCloudViewer::setSceneOrigin(const QVector3D & v)
{
  _glViewer->setSceneOrigin(v);
}

QVector3D QCloudViewer::sceneOrigin() const
{
  return _glViewer->sceneOrigin();
}

void QCloudViewer::add(const QPointCloud::sptr & cloud)
{
  _glViewer->add(cloud);
}

void QCloudViewer::clear()
{
  _glViewer->clear();
  Q_EMIT currentFileNameChanged();
}

void QCloudViewer::updateDisplayPoints()
{
  _glViewer->updateDisplayPoints();
}

void QCloudViewer::updateDisplayColors()
{
  _glViewer->updateDisplayColors();
}

void QCloudViewer::redraw()
{
  _glViewer->update();
}

bool QCloudViewer::openPlyFile(const QString & pathFileName)
{
  _glViewer->clear();

  bool fOk =
      _glViewer->openPlyFile(
          pathFileName);

  _currentFileName =
        pathFileName;

  Q_EMIT currentFileNameChanged();

  return fOk;
}

void QCloudViewer::setCurrentFileName(const QString & v)
{
  _currentFileName = v;
  Q_EMIT currentFileNameChanged();
}

const QString & QCloudViewer::currentFileName() const
{
  return _currentFileName;
}

void QCloudViewer::rotateToShowCloud()
{
  _glViewer->rotateToShowCloud();
}

void QCloudViewer::showKeyBindings()
{
  _glViewer->showKeyBindings();
}

bool QCloudViewer::copyViewportToClipboard()
{
  return _glViewer->copyViewportToClipboard();
}

QPixmap QCloudViewer::grabViewportPixmap()
{
  return _glViewer->grab();
}

