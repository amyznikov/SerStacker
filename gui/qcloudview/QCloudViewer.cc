/*
 * QCloudViewer.cc
 *
 *  Created on: May 18, 2021
 *      Author: amyznikov
 */

#include "QCloudViewer.h"
#if HAVE_QGLViewer // Should come from CMakeLists.txt

#include "QPointCloud.h"
#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////

bool fromString(const QString & s, QGLVector * v)
{
  return sscanf(s.toUtf8().data(), "%lf ; %lf ; %lf", &v->x, &v->y, &v->z) == 3;
}

QString toQString(const QGLVector & v)
{
  return QString("%1;%2;%3").arg(v.x).arg(v.y).arg(v.z);
}

bool load_parameter(const QSettings & settings, const QString & prefix, const char * name,  QGLVector * v)
{
  return fromString(settings.value(QString("%1/%2").arg(prefix).arg(name), "").toString(), v);
}

void save_parameter(const QString & prefix, const char * name, const QGLVector & value )
{
  QSettings settings;
  settings.setValue(QString("%1/%2").arg(prefix).arg(name), toQString(value));
}


///////////////////////////////////////////////////////////////////////////////


QGLCloudViewer::QGLCloudViewer(QWidget* parent)
  : Base(parent)
{
  //init();
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

void QGLCloudViewer::setSceneRadius(qreal radius)
{
  Base::setSceneRadius(radius);
  update();
}

void QGLCloudViewer::setSceneOrigin(const QGLVector & v)
{
  sceneOrigin_ = v;
  update();
}

QGLVector QGLCloudViewer::sceneOrigin() const
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


void QGLCloudViewer::init()
{
  setSceneRadius(100);
  setSceneCenter(QGLVector(0.0, 0.0, 0.0));
  setBackgroundColor(QColor(0, 0, 0));

  glDisable(GL_LIGHTING);
  glDisable(GL_COLOR_MATERIAL);
  glEnable(GL_PROGRAM_POINT_SIZE);

  Base::init();
}


void QGLCloudViewer::draw()
{
//  glPushMatrix();

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClearColor(0.0, 0.0, 0.0, 1.0);

  glPointSize(pointSize_);

  for ( int i = 0, n = clouds_.size(); i < n; ++i ) {

    const QPointCloud::ptr & cloud = clouds_[i];

    if ( cloud->visible ) {

      const std::vector<QPoint3D> & points = cloud->points;
      const std::vector<QColor> & colors = cloud->colors;
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
          glColor3ub(std::min(255, c.red() + (int) pointBrightness_),
              std::min(255, c.green() + (int) pointBrightness_),
              std::min(255, c.blue() + (int) pointBrightness_));
        }

        const QPoint3D & p = points[j];

        glVertex3d(p.x *  Sx - Tx - sceneOrigin_.x,
            p.y * Sy - Ty - sceneOrigin_.y,
            p.z * Sz - Tz - sceneOrigin_.z);
      }

      glEnd(/*GL_POINTS*/);
    }
  }

//  glPopMatrix();
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

///////////////////////////////////////////////////////////////////////////////

QCloudViewer::QCloudViewer(QWidget* parent)
  : Base(parent)
{
  layout_ = new QVBoxLayout(this);
  layout_->setContentsMargins(0, 0, 0, 0);

  toolbar_ = new QToolBar(this);
  toolbar_->setToolButtonStyle(Qt::ToolButtonIconOnly);
  toolbar_->setOrientation(Qt::Horizontal);
  toolbar_->setIconSize(QSize(16,16));

  glViewer_ = new QGLCloudViewer(this);

  layout_->addWidget(toolbar_, 1);
  layout_->addWidget(glViewer_, 100);
}

QToolBar * QCloudViewer::toolbar() const
{
  return toolbar_;
}


void QCloudViewer::setSceneRadius(qreal radius)
{
  glViewer_->setSceneRadius(radius);
}

qreal QCloudViewer::sceneRadius() const
{
  return glViewer_->sceneRadius();
}


void QCloudViewer::setSceneCenter(const QGLVector &center)
{
  return glViewer_->setSceneCenter(center);
}

QGLVector QCloudViewer::sceneCenter() const
{
  return glViewer_->sceneCenter();
}


void QCloudViewer::setSceneOrigin(const QGLVector & v)
{
  glViewer_->setSceneOrigin(v);
}

QGLVector QCloudViewer::sceneOrigin() const
{
  return glViewer_->sceneOrigin();
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

void QCloudViewer::clear()
{
  glViewer_->clear();
  emit currentFileNameChanged();
}

bool QCloudViewer::openPlyFile(const QString & pathFileName)
{
  glViewer_->clear();

  bool fOk =
      glViewer_->openPlyFile(
          pathFileName);

  currentFileName_ =
        pathFileName;

  emit currentFileNameChanged();

  return fOk;
}


void QCloudViewer::setCurrentFileName(const QString & v)
{
  currentFileName_ = v;
  emit currentFileNameChanged();
}

const QString & QCloudViewer::currentFileName() const
{
  return currentFileName_;
}
#endif // HAVE_QGLViewer
