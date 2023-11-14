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
}

template<>
const c_enum_member* members_of<DISPLAY_TYPE>()
{
  static constexpr c_enum_member members[] = {
      { DISPLAY_PIXEL_VALUE, "PIXEL_VALUE" },
      { DISPLAY_PIXEL_VALUE }
  };

  return members;
}



QCloudViewMtfDisplay::QCloudViewMtfDisplay(QGLCloudViewer * cloudView) :
    Base("", cloudView),
    cloudView_(cloudView)
{
  Base::displayType_ = DISPLAY_PIXEL_VALUE;

  addDisplay(DISPLAY_PIXEL_VALUE, 0, 255);
}

QGLCloudViewer * QCloudViewMtfDisplay::cloudView() const
{
  return cloudView_;
}

const c_enum_member * QCloudViewMtfDisplay::displayTypes() const
{
  return members_of<DISPLAY_TYPE>();
}

void QCloudViewMtfDisplay::getInputDataRange(double * minval, double * maxval) const
{
  *minval = HUGE_VAL;
  *maxval = 0;

  if( cloudView_ ) {

    int v;

    for( const QPointCloud::ptr &cloud : cloudView_->clouds() ) {
      for( const QColor &color : cloud->colors ) {

        if( (v = color.red()) < *minval ) {
          *minval = v;
        }
        if( v > *maxval ) {
          *maxval = v;
        }

        if( (v = color.green()) < *minval ) {
          *minval = v;
        }
        if( v > *maxval ) {
          *maxval = v;
        }

        if( (v = color.blue()) < *minval ) {
          *minval = v;
        }
        if( v > *maxval ) {
          *maxval = v;
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

  getInputDataRange(imin, imax);

  c_histogram_builder builder;

  builder.set_input_range(*imin, *imax);
  builder.set_channels(3);
  builder.set_bins(256);

  for( const QPointCloud::ptr &cloud : cloudView_->clouds() ) {
    for( const QColor &color : cloud->colors ) {
      builder.add_pixel(cv::Scalar(color.blue(), color.green(), color.red()));
    }
  }

  builder.compute(H);
}

void QCloudViewMtfDisplay::getOutputHistogramm(cv::OutputArray H, double * omin, double * omax)
{
  displayParams().mtf.get_output_range(omin, omax);
  H.release();

  if( !cloudView_ || cloudView_->clouds().empty() ) {
    return;
  }

  c_histogram_builder builder;

  builder.set_input_range(0, 255);
  builder.set_channels(3);
  builder.set_bins(256);

  for( const QPointCloud::ptr &cloud : cloudView_->clouds() ) {

    const std::vector<QColor> &display_colors =
        cloud->display_colors.empty() ? cloud->colors :
            cloud->display_colors;

    for( const QColor &color : display_colors ) {
      builder.add_pixel(cv::Scalar(color.blue(), color.green(), color.red()));
    }
  }

  builder.compute(H);
}



void QCloudViewMtfDisplay::computeDisplayColors(const std::vector<QPoint3D> & points,
    const std::vector<QColor> & src_colors,
    std::vector<QColor> & display_colors)
{
  DisplayParams &opts =
      displayParams();

  c_pixinsight_mtf &mtf =
      opts.mtf;

  double imin, imax;

  mtf.get_input_range(&imin, &imax);

  if ( imin >= imax  ) {
    mtf.set_input_range(0, 255);
  }

  display_colors.clear();

  for( const QColor &color : src_colors ) {

    const int red = mtf.apply(color.red());
    const int green = mtf.apply(color.green());
    const int blue = mtf.apply(color.blue());

    display_colors.emplace_back(QColor(red, green, blue));
  }

  if ( imin >= imax  ) {
    mtf.set_input_range(imin, imax);
  }
}

///////////////////////////////////////////////////////////////////////////////


QGLCloudViewer::QGLCloudViewer(QWidget* parent) :
  Base(parent),
  mtfDisplay_(this)
{
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

std::vector<QPointCloud::ptr> & QGLCloudViewer::clouds()
{
  return clouds_;
}

const std::vector<QPointCloud::ptr> & QGLCloudViewer::clouds() const
{
  return clouds_;
}

const QPointCloud::ptr & QGLCloudViewer::cloud(int index) const
{
  return clouds_[index];
}

void QGLCloudViewer::clear()
{
  clouds_.clear();
  update();
}

void QGLCloudViewer::rotateToShowCloud()
{
  double mx = 0, my = 0, mz = 0;
  int mn = 0;

  for (const QPointCloud::ptr & cloud : clouds_ ) {
    for (const QPoint3D & p : cloud->points ) {
      mx += p.x;
      my += p.y;
      mz += p.z;
      mn += 1;
    }
  }

  if ( mn > 0 ) {
    setViewTargetPoint(QVector3D(mx / mn, my / mn, mz / mn));
  }
}

void QGLCloudViewer::setSceneOrigin(const QVector3D & v)
{
  sceneOrigin_ = v;
  update();
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
  update();
}

double QGLCloudViewer::pointBrightness() const
{
  return pointBrightness_;
}


void QGLCloudViewer::glInit()
{
  Base::glInit();

  setBackgroundColor(QColor(32, 32, 32));
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
  glColor3ub(200, 200, 200);
  drawMainAxes();

  Base::glPostDraw();
}

void QGLCloudViewer::glDraw()
{
  // CF_DEBUG("glIsEnabled(GL_DEPTH_TEST)=%d", glIsEnabled(GL_DEPTH_TEST));

  glPointSize(pointSize_);

  for ( int i = 0, n = clouds_.size(); i < n; ++i ) {

    const QPointCloud::ptr & cloud =
        clouds_[i];

    if ( cloud->visible ) {

      const std::vector<QPoint3D> & points =
          cloud->points;

      if( cloud->display_colors.size() != cloud->colors.size() ) {
        mtfDisplay_.computeDisplayColors(points, cloud->colors,
            cloud->display_colors);
      }

      const std::vector<QColor> &colors =
          cloud->display_colors;

      const bool haveColors = colors.size() == points.size();

      const double Sx = cloud->Scale.x;
      const double Sy = cloud->Scale.y;
      const double Sz = cloud->Scale.z;

      const double Tx = cloud->Translation.x;
      const double Ty = cloud->Translation.y;
      const double Tz = cloud->Translation.z;

      const double Rx = cloud->Rotation.x;
      const double Ry = cloud->Rotation.y;
      const double Rz = cloud->Rotation.z;


      if ( !haveColors ) {
        glColor3ub(255, 255, 255);
      }

      glBegin(GL_POINTS);

      for ( int j = 0, m = points.size(); j < m; ++j ) {

        if ( haveColors ) {
          const QColor & c = colors[j];
          glColor3ub(std::max(0, std::min(255, (int) (c.red() + pointBrightness_))),
              std::max(0, std::min(255, (int) (c.green() + pointBrightness_))),
              std::max(0, std::min(255, (int) (c.blue() + pointBrightness_))));
        }

        const QPoint3D & p =
            points[j];

        glVertex3d(p.x *  Sx - Tx - sceneOrigin_.x(),
            p.y * Sy - Ty - sceneOrigin_.y(),
            p.z * Sz - Tz - sceneOrigin_.z());
      }

      glEnd(/*GL_POINTS*/);
    }
  }

}


bool QGLCloudViewer::openPlyFile(const QString & pathFileName)
{
  QPointCloud::ptr cloud = QPointCloud::create();

  if ( !loadPlyFile(pathFileName, cloud.get()) ) {
    CF_ERROR("loadPlyFile('%s') fails",
        pathFileName.toStdString().c_str());
    return false;
  }

  clouds_.emplace_back(cloud);
  return true;
}


void QGLCloudViewer::updateDisplayColors()
{
  if( !clouds_.empty() ) {
    for( const QPointCloud::ptr &cloud : clouds_ ) {
      mtfDisplay_.computeDisplayColors(cloud->points, cloud->colors, cloud->display_colors);
    }
    update();
    Q_EMIT mtfDisplay_.displayImageChanged();
  }

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

void QCloudViewer::setSceneOrigin(const QVector3D & v)
{
  glViewer_->setSceneOrigin(v);
}

QVector3D QCloudViewer::sceneOrigin() const
{
  return glViewer_->sceneOrigin();
}

void QCloudViewer::clear()
{
  glViewer_->clear();
  Q_EMIT currentFileNameChanged();
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

