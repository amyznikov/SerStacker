/*
 * QGLView.cc
 *
 *  Created on: Feb 1, 2023
 *      Author: amyznikov
 *
 *  https://computergraphics.stackexchange.com/questions/151/how-to-implement-a-trackball-in-opengl
 *  https://stackoverflow.com/questions/8491247/c-opengl-convert-world-coords-to-screen2d-coords
 *  https://www.khronos.org/opengl/wiki/Object_Mouse_Trackball
 *
 *  https://registry.khronos.org/OpenGL-Refpages/gl4/html/glGet.xhtml
 *
 */

#include "QGLView.h"
#include <gui/widgets/settings.h>
#include <core/io/save_image.h>
#include <core/ssprintf.h>
#include <core/debug.h>


#ifdef __ssprintf_h__

template<>
const c_enum_member* members_of<QGLView::Projection>()
{
  static const c_enum_member members[] = {
      { QGLView::Perspective, "Perspective", "" },
      { QGLView::Orthographic, "Orthographic", "" },
      { QGLView::Perspective },
  };

  return members;
}

#endif // __ssprintf_h__


namespace {

void toSpherical(const QVector3D & v, double * r, double * phi, double * theta)
{
  const double x = v.x();
  const double y = v.y();
  const double z = v.z();

  *r = std::sqrt(x * x + y * y + z * z); //  v.length();
  *phi = std::acos(v.z() / *r);
  *theta = std::atan2(v.y(), v.x());
}

QVector3D toSpherical(const QVector3D & v)
{
  const double x = v.x();
  const double y = v.y();
  const double z = v.z();
  const double r = std::sqrt(x * x + y * y + z * z); //  v.length();

  return QVector3D(r, std::acos(z / r), std::atan2(y, x));
}

QVector3D fromSpherical(double r, double phi, double theta)
{
  return QVector3D(r * std::cos(theta) * std::sin(phi),
      r * std::sin(theta) * std::sin(phi),
      r * std::cos(phi));
}

QVector3D fromSpherical(const QVector3D & v)
{
  const double r = v[0];
  const double phi = v[1];
  const double theta = v[2];

  return QVector3D(r * std::cos(theta) * std::sin(phi),
      r * std::sin(theta) * std::sin(phi),
      r * std::cos(phi));
}

template<typename T>
inline int sign(T x)
{
  return x >= 0 ? 1 : -1;
}

} // namespace


/**
 * Fills m[4][4] with the OpenGL (column-major) representation of the QQuaternion rotation
 * Return pointer to m[][]
 * */
GLfloat * getMatrix(const QQuaternion & q, float m[4][4])
{
  const float x = q.x();
  const float y = q.y();
  const float z = q.z();
  const float w = q.scalar();

  const qreal q00 = 2.0 * x * x;
  const qreal q11 = 2.0 * y * y;
  const qreal q22 = 2.0 * z * z;

  const qreal q01 = 2.0 * x * y;
  const qreal q02 = 2.0 * x * z;
  const qreal q03 = 2.0 * x * w;

  const qreal q12 = 2.0 * y * z;
  const qreal q13 = 2.0 * y * w;

  const qreal q23 = 2.0 * z * w;

  m[0][0] = 1.0 - q11 - q22;
  m[1][0] = q01 - q23;
  m[2][0] = q02 + q13;

  m[0][1] = q01 + q23;
  m[1][1] = 1.0 - q22 - q00;
  m[2][1] = q12 - q03;

  m[0][2] = q02 - q13;
  m[1][2] = q12 + q03;
  m[2][2] = 1.0 - q11 - q00;

  m[0][3] = 0.0f;
  m[1][3] = 0.0f;
  m[2][3] = 0.0f;

  m[3][0] = 0.0f;
  m[3][1] = 0.0f;
  m[3][2] = 0.0f;
  m[3][3] = 1.0f;

  return (GLfloat *) m;
}


QGLView::QGLView(QWidget * parent) :
    Base(parent)
{
  setFocusPolicy(Qt::StrongFocus);
  setAttribute(Qt::WA_NoSystemBackground);
}

QGLView::~QGLView()
{
  // still non clear how to manage GL cleanup from destructor
  cleanupGL();
}

void QGLView::loadParameters()
{
  QSettings settings;
  onLoadParameters(settings);
}

void QGLView::onLoadParameters(QSettings & settings)
{
  _backgroundColor =
      settings.value("QGLView/backgroundColor_",
          _backgroundColor).value<QColor>();

  _foregroundColor =
      settings.value("QGLView/foregroundColor_",
          _foregroundColor).value<QColor>();

  _showMainAxes =
      settings.value("QGLView/showMainAxes",
          _showMainAxes).toBool();

  _mainAxesLength =
      settings.value("QGLView/mainAxesLength",
          _mainAxesLength).toDouble();

  fromString(settings.value("QGLView/projection",
      toQString(_viewParams.projection)).toString().toStdString(),
      &_viewParams.projection);

  _viewParams.fov = settings.value("QGLView/fov",
      _viewParams.fov).value<decltype(_viewParams.fov)>();

  _viewParams.nearPlane = settings.value("QGLView/nearPlane",
      _viewParams.nearPlane).value<decltype(_viewParams.nearPlane)>();

  _viewParams.farPlane = settings.value("QGLView/farPlane",
      _viewParams.farPlane).value<decltype(_viewParams.farPlane)>();

  const int numGrids =
      settings.value("QGLView/numGrids",
          0).value<int>();

  _grids.clear();

  for( int i = 0; i < numGrids; ++i ) {

    const QString keyPrefix =
        QString("QGLView/grid%1").arg(i);

    _grids.emplace_back();

    PlanarGridOptions & grid =
        _grids.back();

    grid.name =
        settings.value(QString("%1/name").arg(keyPrefix),
            grid.name).value<decltype(grid.name)>();

    grid.maxDistance =
        settings.value(QString("%1/max_distance").arg(keyPrefix),
            grid.maxDistance).value<decltype(grid.maxDistance)>();

    grid.step =
        settings.value(QString("%1/step").arg(keyPrefix),
            grid.step).value<decltype(grid.step)>();

    grid.Rotation =
        settings.value(QString("%1/Rotation").arg(keyPrefix),
            grid.Rotation).value<decltype(grid.Rotation)>();

    grid.Translation =
        settings.value(QString("%1/Translation").arg(keyPrefix),
            grid.Translation).value<decltype(grid.Translation)>();

    grid.gridColor =
        settings.value(QString("%1/gridColor").arg(keyPrefix),
            grid.gridColor).value<decltype(grid.gridColor)>();

    grid.fillColor =
        settings.value(QString("%1/fillColor").arg(keyPrefix),
            grid.fillColor).value<decltype(grid.fillColor)>();

    grid.gridOpaqueness =
        settings.value(QString("%1/gridOpaqueness").arg(keyPrefix),
            grid.gridOpaqueness).value<decltype(grid.gridOpaqueness)>();

    grid.fillOpaqueness =
        settings.value(QString("%1/fillOpaqueness").arg(keyPrefix),
            grid.fillOpaqueness).value<decltype(grid.fillOpaqueness)>();

    grid.visible =
        settings.value(QString("%1/visible").arg(keyPrefix),
            grid.visible).value<decltype(grid.visible)>();

  }

}

void QGLView::saveParameters()
{
  QSettings settings;
  onSaveParameters(settings);
}

void QGLView::onSaveParameters(QSettings & settings)
{
  settings.setValue("QGLView/backgroundColor_",
      _backgroundColor);

  settings.setValue("QGLView/foregroundColor_",
      _foregroundColor);

  settings.setValue("QGLView/showMainAxes",
      _showMainAxes);

  settings.setValue("QGLView/mainAxesLength",
      _mainAxesLength);

  settings.setValue("QGLView/projection",
      toQString(_viewParams.projection));

  settings.setValue("QGLView/fov",
      _viewParams.fov);

  settings.setValue("QGLView/nearPlane",
      _viewParams.nearPlane);

  settings.setValue("QGLView/farPlane",
      _viewParams.farPlane);

  settings.setValue("QGLView/numGrids",
      (int) (_grids.size()));

  for( int i = 0, n = _grids.size(); i < n; ++i ) {

    const PlanarGridOptions & grid =
        _grids[i];

    const QString keyPrefix =
        QString("QGLView/grid%1").arg(i);

    settings.setValue(QString("%1/name").arg(keyPrefix),
        grid.name);

    settings.setValue(QString("%1/max_distance").arg(keyPrefix),
        grid.maxDistance);

    settings.setValue(QString("%1/step").arg(keyPrefix),
        grid.step);

    settings.setValue(QString("%1/Rotation").arg(keyPrefix),
        grid.Rotation);

    settings.setValue(QString("%1/Translation").arg(keyPrefix),
        grid.Translation);

    settings.setValue(QString("%1/gridColor").arg(keyPrefix),
        grid.gridColor);

    settings.setValue(QString("%1/fillColor").arg(keyPrefix),
        grid.fillColor);

    settings.setValue(QString("%1/gridOpaqueness").arg(keyPrefix),
        grid.gridOpaqueness);

    settings.setValue(QString("%1/fillOpaqueness").arg(keyPrefix),
        grid.fillOpaqueness);

    settings.setValue(QString("%1/visible").arg(keyPrefix),
        grid.visible);


  }

}


void QGLView::setBackgroundColor(const QColor &color)
{
  _backgroundColor = color;
  _dirty = true;
  update();
}

const QColor & QGLView::backgroundColor() const
{
  return _backgroundColor;
}

void QGLView::setForegroundColor(const QColor &color)
{
  _foregroundColor = color;
  _dirty = true;
  update();
}

const QColor & QGLView::foregroundColor() const
{
  return _foregroundColor;
}

const QGLView::ViewParams & QGLView::viewParams() const
{
  return _viewParams;
}

const std::vector<QGLView::PlanarGridOptions> & QGLView::grids() const
{
  return _grids;
}

std::vector<QGLView::PlanarGridOptions> & QGLView::grids()
{
  return _grids;
}


QGLView::Projection QGLView::projection() const
{
  return _viewParams.projection;
}

void QGLView::setProjection(Projection v)
{
  _viewParams.projection = v;
  _dirty = true;
  update();
}

void QGLView::setFOV(double degrees)
{
  _viewParams.fov = degrees;
  _dirty = true;
  update();
}

double QGLView::fov() const
{
  return _viewParams.fov;
}

void QGLView::setNearPlane(double v)
{
  _viewParams.nearPlane = v;
  _dirty = true;
  update();


}

double QGLView::nearPlane() const
{
  return _viewParams.nearPlane;
}

void QGLView::setFarPlane(double v)
{
  _viewParams.farPlane = v;
  _dirty = true;
  update();
}

double QGLView::farPlane() const
{
  return _viewParams.farPlane;
}

void QGLView::setShowMainAxes(bool v)
{
  _showMainAxes = v;
  update();
}

bool QGLView::showMainAxes() const
{
  return _showMainAxes;
}

void QGLView::setMainAxesLength(double v)
{
  _mainAxesLength = v;
  update();
}

double QGLView::mainAxesLength() const
{
  return _mainAxesLength;
}

void QGLView::setViewPoint(const QVector3D & eye)
{
  _viewPoint = eye;
  _dirty = true;
  update();
}

const QVector3D & QGLView::viewPoint() const
{
  return _viewPoint;
}

void QGLView::setViewTargetPoint(const QVector3D & v)
{
  _viewTarget = v;
  _dirty = true;
  update();
}

const QVector3D & QGLView::viewTargetPoint() const
{
  return _viewTarget;
}


void QGLView::setUpDirection(const QVector3D & v)
{
  _viewUpDirection = v;
  _dirty = true;
  update();
}

const QVector3D & QGLView::upDirection() const
{
  return _viewUpDirection;
}

void QGLView::setPerspecitive(double fov, double nearPlane, double farPlane)
{
  _viewParams.projection = Perspective;
  _viewParams.fov = fov;
  _viewParams.nearPlane = nearPlane;
  _viewParams.farPlane = farPlane;
  _dirty = true;
  update();
}

const cv::Mat1f & QGLView::depthBuffer() const
{
  return _depthBuffer;
}

void QGLView::cameraTo(const QVector3D & eye_pos,
    const QVector3D & target_pos,
    const QVector3D & up_direction)
{
  _viewPoint = eye_pos;
  _viewTarget = target_pos;
  _viewUpDirection = up_direction;
  _dirty = true;
  update();
}

void QGLView::lookTo(const QVector3D &target)
{
  setViewTargetPoint(target);
}

bool QGLView::projectToScreen(const QVector3D & pos, QPointF * screen_pos) const
{
  // normalized device coordinates
  const QVector3D ndc =
      _mtotal.map(pos);

  if( ndc.z() < 1 && std::abs(ndc.x()) < 1 && std::abs(ndc.y()) < 1 ) {

    // apply view port to compute window coordinates
//    screen_pos->setX(((ndc.x() + 1.0) / 2.0) * viewport.w + viewport.x);
//    screen_pos->setY(((1.0 - ndc.y()) / 2.0) * viewport.h + viewport.y);

    screen_pos->setX((1 + ndc.x()) * viewport.w / 2 + viewport.x);
    screen_pos->setY((1 - ndc.y()) * viewport.h / 2 + viewport.y);

    return true;
  }

  return false;
}

bool QGLView::projectToScreen(const cv::Vec3f & pos, cv::Point2f * screen_pos) const
{
  // normalized device coordinates
  const QVector3D ndc =
      _mtotal.map(QVector3D(pos[0], pos[1], pos[2]));

  if( ndc.z() < 1 && std::abs(ndc.x()) < 1 && std::abs(ndc.y()) < 1 ) {

    // apply view port to compute window coordinates
    screen_pos->x = (1 + ndc.x()) * viewport.w / 2 + viewport.x;
    screen_pos->y = (1 - ndc.y()) * viewport.h / 2 + viewport.y;
    return true;
  }

  return false;
}

bool QGLView::projectToScreen(const cv::Vec3f & pos, cv::Point3f * screen_pos) const
{
  // normalized device coordinates
  const QVector3D ndc =
      _mtotal.map(QVector3D(pos[0], pos[1], pos[2]));

  if( ndc.z() < 1 && std::abs(ndc.x()) < 1 && std::abs(ndc.y()) < 1 ) {

    // apply view port to compute window coordinates
    screen_pos->x = (1 + ndc.x()) * viewport.w / 2 + viewport.x;
    screen_pos->y = (1 - ndc.y()) * viewport.h / 2 + viewport.y;
    screen_pos->z = ndc.z();
    return true;
  }

  return false;
}


void QGLView::initializeGL()
{
  Base::initializeGL();
  initializeOpenGLFunctions();

  connect(context(), &QOpenGLContext::aboutToBeDestroyed,
      this, &ThisClass::cleanupGL);

  glInit();
}

void QGLView::resizeGL(int w, int h)
{
  Base::resizeGL(w, h);

  // update and save current viewport
  if( w > 0 && h > 0 ) {
    glViewport(viewport.x = 0, viewport.y = 0, GLint(viewport.w = w), GLint(viewport.h = h));
    _dirty = true;
    update();
  }
}

void QGLView::paintGL()
{
  if( viewport.w > 1 && viewport.h > 1 ) {
    glPreDraw();
    glDraw();
    glPostDraw();
  }
}

void QGLView::cleanupGL()
{
  Base::makeCurrent();

  QOpenGLContext *ctx =
      context();

  if( ctx ) {
    disconnect(ctx, &QOpenGLContext::aboutToBeDestroyed,
        this, &ThisClass::cleanupGL);
  }

  glCleanup();

  Base::doneCurrent();
}

void QGLView::paintEvent(QPaintEvent *e)
{
  Base::paintEvent(e);

  Q_EMIT displayImageChanged();
}

void QGLView::glInit()
{
  // Default colors
  //setForegroundColor(QColor(200, 200, 200));
  // setBackgroundColor(QColor(32, 32, 32));
}

void QGLView::glPreDraw()
{
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  if( _dirty && viewport.w > 0 && viewport.h > 0 ) {

    _mview.setToIdentity();
    _mview.lookAt(_viewPoint, _viewTarget,
        _viewUpDirection);

    _mprojection.setToIdentity();

    switch (_viewParams.projection) {

      case Projection::Orthographic: {
        ///////////////

        // const qreal dist = orthoCoef_ * fabs(cameraCoordinatesOf(pivotPoint()).z);
        // halfWidth = dist * ((aspectRatio() < 1.0) ? 1.0 : aspectRatio());
        // halfHeight = dist * ((aspectRatio() < 1.0) ? 1.0 / aspectRatio() : 1.0);

        // the camera coordinates of pivot point around which the camera rotates
        const QVector3D pivotPoint = _viewTarget - _viewPoint;
        const double orthoCoef = tan(_viewParams.fov * M_PI / 360);
        const double dist = orthoCoef * pivotPoint.length();
        const double aspectRatio = viewport.w / (double) (viewport.h);
        const double w = dist * ((aspectRatio < 1.0) ? 1.0 : aspectRatio);
        const double h = dist * ((aspectRatio < 1.0) ? 1.0 / aspectRatio : 1.0);

        _mprojection.ortho(-w, w, -h, h,
            _viewParams.nearPlane,
            _viewParams.farPlane);

        break;
      }

      case Projection::Perspective:
      default: {

        _mprojection.perspective(_viewParams.fov,
            GLfloat(viewport.w) / viewport.h,
            _viewParams.nearPlane,
            _viewParams.farPlane);

        break;
      }
    }

    _mtotal = _mprojection * _mview;
    _dirty = false;
  }

  glMatrixMode(GL_PROJECTION);
  glLoadMatrixf(_mtotal.constData());

  glClearColor(_backgroundColor.redF(),
      _backgroundColor.greenF(),
      _backgroundColor.blueF(),
      _backgroundColor.alphaF());


  /*
    // In derived class set also optionally:

    glDisable(GL_LIGHTING);
    glDisable(GL_COLOR_MATERIAL);

    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_DEPTH_TEST);

    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable( GL_BLEND);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_ACCUM_BUFFER_BIT);

   */

}

void QGLView::glDraw()
{
}

void QGLView::glPostDraw()
{
  if( viewport.w < 3 || viewport.h < 3 ) {
    glFlush();
    return;
  }

  for( QGLShape * shape : _shapes ) {
    if ( shape && shape->isVisible() && !shape->isTopLevel() ) {
      shape->draw(this);
    }
  }

  if( _enableGLMouseEvents ) {

    _depthBuffer.create(viewport.h, viewport.w);

    glReadPixels(viewport.x, viewport.y, viewport.w, viewport.h,
        GL_DEPTH_COMPONENT, GL_FLOAT,
        _depthBuffer.data);

    glGetDoublev( GL_MODELVIEW_MATRIX, _smodelview);
    glGetDoublev( GL_PROJECTION_MATRIX, _sprojection);
    glGetIntegerv( GL_VIEWPORT, _sviewport);
  }

  if ( _showMainAxes ) {
    glColor3ub(200, 200, 200);
    drawMainAxes();
  }


  const bool showViewTarget =
      _hideViewTargetTimerId > 0;

  if ( showViewTarget ) {

    const qreal length =
        _mainAxesLength > 0 ? _mainAxesLength :
            0.0025 * std::abs(_viewParams.farPlane - _viewParams.nearPlane);

    const QVector3D arrow_start[3] = {
        QVector3D(_viewTarget.x() - length, _viewTarget.y(), _viewTarget.z()),
        QVector3D(_viewTarget.x(), _viewTarget.y() - length, _viewTarget.z()),
        QVector3D(_viewTarget.x(), _viewTarget.y(), _viewTarget.z() - length),
    };

    const QVector3D arrow_end[3] = {
        QVector3D(_viewTarget.x() + 2 * length, _viewTarget.y(), _viewTarget.z()),
        QVector3D(_viewTarget.x(), _viewTarget.y() + 2 * length, _viewTarget.z()),
        QVector3D(_viewTarget.x(), _viewTarget.y(), _viewTarget.z() + 2 * length),
    };

    glColor3ub(255, 255, 64);

    for ( int i = 0; i < 3; ++i ) {
      drawArrow(arrow_start[i], arrow_end[i], length / 64, 4);
    }
  }

  // For transparency (blend) to work it is important to draw the solid things first,
  // so the grids are drawn here in glPostDraw()
  if( !_grids.empty() ) {

    GLboolean gl_depth_writemask = GL_TRUE;
    int numGridsDrawn = 0;

    for( const PlanarGridOptions & grid : _grids ) {

      if( grid.visible ) {

        if( !numGridsDrawn++ ) {
          glGetBooleanv(GL_DEPTH_WRITEMASK, &gl_depth_writemask);
          glDepthMask(GL_FALSE);
        }

        const float max_distance =
            grid.maxDistance > 0 ? std::min(grid.maxDistance, (float) _viewParams.farPlane) :
                _viewParams.farPlane;

        const QQuaternion R =
            QQuaternion::fromEulerAngles(grid.Rotation.x(),
                grid.Rotation.y(),
                grid.Rotation.z());

        if( grid.fillOpaqueness > 0 ) {

          glColor4ub(grid.fillColor.red(), grid.fillColor.green(), grid.fillColor.blue(), grid.fillOpaqueness);

          glBegin(GL_QUADS);
            glVertex(R.rotatedVector(QVector3D(-max_distance, -max_distance, 0)) + grid.Translation);
            glVertex(R.rotatedVector(QVector3D(max_distance, -max_distance, 0)) + grid.Translation);
            glVertex(R.rotatedVector(QVector3D(max_distance, max_distance, 0)) + grid.Translation);
            glVertex(R.rotatedVector(QVector3D(-max_distance, max_distance, 0)) + grid.Translation);
          glEnd();
        }

        if( grid.gridOpaqueness > 0 ) {

          const float step =
              grid.step > 0 ? grid.step :
                  max_distance / 20;

          glColor4ub(grid.gridColor.red(), grid.gridColor.green(), grid.gridColor.blue(), grid.gridOpaqueness);
          glLineWidth(1);


          glBegin(GL_LINES);

          for( int i = 1;; ++i ) {

            const float s = i * step;
            if( s > max_distance ) {
              break;
            }

            // X
            glVertex(R.rotatedVector(QVector3D(s, -max_distance, 0)) + grid.Translation);
            glVertex(R.rotatedVector(QVector3D(s, max_distance, 0)) + grid.Translation);
            glVertex(R.rotatedVector(QVector3D(-s, -max_distance, 0)) + grid.Translation);
            glVertex(R.rotatedVector(QVector3D(-s, max_distance, 0)) + grid.Translation);

            // Y
            glVertex(R.rotatedVector(QVector3D(-max_distance, s, 0)) + grid.Translation);
            glVertex(R.rotatedVector(QVector3D(max_distance, s, 0)) + grid.Translation);
            glVertex(R.rotatedVector(QVector3D(-max_distance, -s, 0)) + grid.Translation);
            glVertex(R.rotatedVector(QVector3D(max_distance, -s, 0)) + grid.Translation);
          }

          glEnd(/*GL_LINES*/);
        }
      }
    }

    if ( numGridsDrawn  ) {
      glDepthMask(gl_depth_writemask);
    }

    /////////
  }



  bool haveTopLevelShapes =
      false;

  for( QGLShape * shape : _shapes ) {
    if ( shape && shape->isVisible() && shape->isTopLevel() ) {
      haveTopLevelShapes = true;
      break;
    }
  }

  if ( haveTopLevelShapes ) {

    GLboolean DEPTH_TEST_ENABLED =
        GL_FALSE;

    glGetBooleanv(GL_DEPTH_TEST,
        &DEPTH_TEST_ENABLED);

    if ( DEPTH_TEST_ENABLED ) {
      glDisable(GL_DEPTH_TEST);
    }

    for( QGLShape * shape : _shapes ) {
      if ( shape && shape->isVisible() && shape->isTopLevel() ) {
        shape->draw(this);
      }
    }

    if ( DEPTH_TEST_ENABLED ) {
      glEnable(GL_DEPTH_TEST);
    }
  }

  glFlush();

//  Q_EMIT displayImageChanged();
}

// still non clear how to manage GL cleanup from destructor
void QGLView::glCleanup()
{
}


void QGLView::drawText(const QPointF & pos, const QFont & font, const QString & text)
{
  // Retrieve last OpenGL color to use as a font color
  GLfloat glColor[4];
  glGetFloatv(GL_CURRENT_COLOR, glColor);

  QColor fontColor(255 * glColor[0], 255 * glColor[1],
      255 * glColor[2], 255 * glColor[3]);

  glPushAttrib(GL_ALL_ATTRIB_BITS);

  // Render text
  QPainter painter(this);
  painter.setPen(fontColor);
  painter.setFont(font);
  painter.drawText(pos, text);
  painter.end();

  glPopAttrib();
}

void QGLView::drawText(double x, double y, const QFont & font, const QString & text)
{
  drawText(QPointF(x, y), font, text);
}

void QGLView::drawText(const QVector3D & pos, const QFont &font, const QString &text)
{
  QPointF spos;
  if( projectToScreen(pos, &spos) ) {
    drawText(spos, font, text);
  }
}

void QGLView::drawText(double x, double y, double z, const QFont &font, const QString &text)
{
  QPointF spos;
  if( projectToScreen(QVector3D(x, y, z), &spos) ) {
    drawText(spos, font, text);
  }
}

void QGLView::glvaprintf(const QPointF & pos, const QFont &font, const char * format, va_list arglist)
{
#if QT_VERSION < QT_VERSION_CHECK(5, 14, 0)
  QString text;
  text.vsprintf(format, arglist);
  drawText(pos, font, text);
#else
  drawText(pos, font, QString::vasprintf(format, arglist));
#endif
}

void QGLView::glvaprintf(double x, double y, const QFont & font, const char * format, va_list arglist)
{
  glvaprintf(QPointF(x, y), font, format, arglist);
}

void QGLView::glvaprintf(const QVector3D & pos, const QFont & font, const char * format, va_list arglist)
{
  QPointF spos;
  if( projectToScreen(pos, &spos) ) {
    glvaprintf(spos, font, format, arglist);
  }
}

void QGLView::glvaprintf(double x, double y, double z, const QFont &font, const char * format, va_list arglist)
{
  QPointF spos;
  if( projectToScreen(QVector3D(x, y, z), &spos) ) {
    glvaprintf(spos, font, format, arglist);
  }
}

void QGLView::glprintf(const QPointF & pos, const QFont &font, const char * format, ...)
{
  va_list arglist;
  va_start(arglist, format);
  glvaprintf(pos, font, format, arglist);
  va_end(arglist);
}

void QGLView::glprintf(double x, double y, const QFont &font, const char * format, ...)
{
  va_list arglist;
  va_start(arglist, format);
  glvaprintf(QPointF(x,y), font, format, arglist);
  va_end(arglist);
}

void QGLView::glprintf(const QVector3D & pos, const QFont &font, const char * format, ...)
{
  QPointF spos;
  if( projectToScreen(pos, &spos) ) {
    va_list arglist;
    va_start(arglist, format);
    glvaprintf(spos, font, format, arglist);
    va_end(arglist);
  }
}

void QGLView::glprintf(double x, double y, double z, const QFont & font, const char * format, ...)
{
  QPointF spos;
  if( projectToScreen(QVector3D(x, y, z), &spos) ) {
    va_list arglist;
    va_start(arglist, format);
    glvaprintf(spos, font, format, arglist);
    va_end(arglist);
  }
}

void QGLView::drawArrow(qreal length, qreal radius, int nbSubdivisions)
{
  static GLUquadric *quadric =
      gluNewQuadric();

  if( radius < 0.0 ) {
    radius = 0.05 * length;
  }

  const qreal head = 2.5 * (radius / length) + 0.1;
  const qreal coneRadiusCoef = 4.0 - 5.0 * head;

  gluCylinder(quadric, radius, radius, length * (1.0 - head / coneRadiusCoef), nbSubdivisions, 1);
  glTranslated(0.0, 0.0, length * (1.0 - head));
  gluCylinder(quadric, coneRadiusCoef * radius, 0.0, head * length, nbSubdivisions, 1);
  glTranslated(0.0, 0.0, -length * (1.0 - head));

}

void QGLView::drawArrow(const QVector3D & start, const QVector3D & end, qreal radius, int nbSubdivisions)
{
  float m[4][4];

  const QVector3D direction =
      end - start;

  glPushMatrix();

  glTranslatef(start.x(), start.y(), start.z());

  glMultMatrixf(getMatrix(QQuaternion::fromDirection(direction,
      QVector3D(0, 0, 1)), m));

  drawArrow(direction.length(), radius, nbSubdivisions);

  glPopMatrix();
}


void QGLView::drawMainAxes()
{
  static const QFont font("Monospace", 12,
      QFont::Weight::DemiBold);

  const qreal length =
      _mainAxesLength > 0 ? _mainAxesLength :
          0.005 * std::abs(_viewParams.farPlane - _viewParams.nearPlane);

//  CF_DEBUG("length=%g", length);

  drawArrow(QVector3D(0, 0, 0), QVector3D(length, 0, 0), length / 100, 4);
  drawArrow(QVector3D(0, 0, 0), QVector3D(0, length, 0), length / 100, 4);
  drawArrow(QVector3D(0, 0, 0), QVector3D(0, 0, length), length / 100, 4);

  drawText(QVector3D(length, 0, 0), font, "X");
  drawText(QVector3D(0, length, 0), font, "Y");
  drawText(QVector3D(0, 0, length), font, "Z");
}

void QGLView::addShape(QGLShape * shape)
{
  if ( shape ) {
    _shapes.emplace_back(shape);
  }
}

void QGLView::removeShape(QGLShape * shape)
{
  auto ii = std::find(_shapes.begin(), _shapes.end(), shape);
  while (ii != _shapes.end()) {
    ii = std::find(_shapes.erase(ii), _shapes.end(), shape);
  }
}

void QGLView::glMouseEvent(QEvent::Type eventType, int keyOrMouseButtons,
    Qt::KeyboardModifiers keyboardModifiers, const QPointF & mousePos,
    bool objHit, double objX, double objY, double objZ)
{
  CF_DEBUG("QGLView: x=%g y=%g obj: X=%g Y=%g Z=%g",
      mousePos.x(), mousePos.y(), objX, objY, objZ);
}


void QGLView::onGLMouseEvent(QEvent::Type eventType, int keyOrMouseButtons,
    Qt::KeyboardModifiers keyboardModifiers,
    const QPointF & mousePos)
{
  if( _enableGLMouseEvents ) {

    const GLdouble X =
        mousePos.x();

    const GLdouble Y =
        _depthBuffer.rows - mousePos.y();

    const int iX =
        qRound(X);

    const int iY =
        qRound(Y);

    if( iX >= 0 && iY >= 0 && iX < _depthBuffer.cols && iY < _depthBuffer.rows ) {

      const GLdouble Z =
          _depthBuffer[iY][iX];

      bool objHit = false;
      GLdouble objX = 0, objY = 0, objZ = 0;

      if ( Z < 1 ) {
        objHit =
            gluUnProject(X, Y, Z,
                _smodelview, _sprojection, _sviewport,
                &objX, &objY, &objZ) == GL_TRUE;
      }

      glMouseEvent(eventType, keyOrMouseButtons,
          keyboardModifiers, mousePos,
          objHit, objX, objY, objZ);
    }
  }
}


void QGLView::keyPressEvent(QKeyEvent *event)
{
  Base::keyPressEvent(event);
  if (_enableGLMouseEvents ) {
  }
}

void QGLView::keyReleaseEvent(QKeyEvent *event)
{
  Base::keyReleaseEvent(event);
  if (_enableGLMouseEvents ) {
  }
}


void QGLView::mousePressEvent(QMouseEvent * e)
{
#if QT_VERSION >= QT_VERSION_CHECK(6,0,0)
  _prev_mouse_pos = e->position();
#else
  _prev_mouse_pos = e->localPos();
#endif

  if( _enableGLMouseEvents && e->modifiers() == Qt::ControlModifier ) {
    onGLMouseEvent(e->type(), e->buttons(), e->modifiers(), e->localPos());
  }
  else if( e->modifiers() == (Qt::ShiftModifier | Qt::ControlModifier) ) {
    setAutoShowViewTarget(!autoShowViewTarget());
    update();
  }

  e->ignore();
}

void QGLView::mouseReleaseEvent(QMouseEvent *e)
{
  if (_enableGLMouseEvents ) {
    onGLMouseEvent(e->type(), e->buttons(), e->modifiers(), e->localPos());
  }

  e->ignore();
}

void QGLView::mouseDoubleClickEvent(QMouseEvent *e)
{
  if (_enableGLMouseEvents ) {
    onGLMouseEvent(e->type(), e->buttons(), e->modifiers(), e->localPos());
  }
  e->ignore();
}

void QGLView::mouseMoveEvent(QMouseEvent * e)
{
  if( _enableGLMouseEvents && e->modifiers() == Qt::ControlModifier ) {
    onGLMouseEvent(e->type(), e->buttons(), e->modifiers(), e->localPos());
  }
  else if( e->buttons() ) {

#if QT_VERSION >= QT_VERSION_CHECK(6,0,0)
    const QPointF newpos = e->position();
#else
    const QPointF newpos = e->localPos();
#endif

    if( e->buttons() == Qt::RightButton ) { // Translate (shift) camera Up / Right

      const QPointF delta =
          newpos - _prev_mouse_pos;

      if( delta.x() || delta.y() ) {

        const QVector3D forward =
            _viewPoint - _viewTarget;

        const QMatrix4x4 minv =
            _mview.inverted();

        const QVector3D T0 =
            minv.map(QVector3D(0, 0, forward.length()));

        if( delta.y() ) {

          const QVector3D TU =
              minv.map(QVector3D(0, delta.y() / viewport.h, forward.length()));

          const QVector3D Up =
              (TU - T0) * std::max((float) _viewParams.nearPlane, forward.length());

          _viewTarget += Up;
          _viewPoint += Up;
        }

        if( delta.x() ) {

          const QVector3D TR =
              minv.map(QVector3D(-delta.x() / viewport.w, 0, forward.length()));

          const QVector3D Right =
              (TR - T0) * std::max((float) _viewParams.nearPlane, forward.length());

          _viewTarget += Right;
          _viewPoint += Right;
        }

        _dirty = true;

        showViewTarget(true);
        update();
        Q_EMIT viewPointChanged();
      }

      _prev_mouse_pos = newpos;
    }

    else if( e->buttons() == Qt::LeftButton ) {

      // Rotate camera

      const QPointF delta =
          newpos - _prev_mouse_pos;

      if( delta.x() || delta.y() ) {

        const QVector3D forward =
            _viewPoint - _viewTarget;

        const QMatrix4x4 minv =
            _mview.inverted();

        if( e->modifiers() == Qt::ShiftModifier ) { // Rotate around forward looking axis

          const int signy =
              newpos.x() > viewport.w / 2 ? +1 : -1;

          const int signx =
              newpos.y() < viewport.h / 2 ? +1 : -1;

          _viewUpDirection =
              QQuaternion::fromAxisAndAngle(forward,
                  0.2 * (signy * delta.y() + signx * delta.x()))
                  .rotatedVector(_viewUpDirection);

          _dirty = true;

          showViewTarget(true);
          update();
          Q_EMIT viewPointChanged();
        }

        else { // Rotate camera around of the Up / Right axes

          if( e->modifiers() & Qt::ControlModifier ) {

            const QVector3D T0 = minv.map(QVector3D(0, 0, forward.length()));
            const QVector3D TU = minv.map(QVector3D(0, 0.5, forward.length()));
            const QVector3D Up = (TU - T0).normalized();

            double dx = delta.x();
            double dy = delta.y();

            // if( e->modifiers() & Qt::ShiftModifier ) {
            if( std::abs(dx) >= std::abs(dy) ) {
              dy = 0;
            }
            else {
              dx = 0;
            }
//          }

            const QVector3D viewRotation(0, -0.05 * dy * M_PI / 180, -0.05 * dx * M_PI / 180);

            _viewPoint =
                fromSpherical(toSpherical(_viewPoint - _viewTarget) + viewRotation) + _viewTarget;

            _viewUpDirection =
                fromSpherical(toSpherical(Up) + viewRotation);

          }
          else {

            QVector3D T0 = minv.map(QVector3D(0, 0, forward.length()));
            QVector3D TU = minv.map(QVector3D(0, 0.5, forward.length()));
            QVector3D TR = minv.map(QVector3D(0.5, 0, forward.length()));
            QVector3D Up = (TU - T0).normalized();
            QVector3D Right = (TR - T0).normalized();

            double dx = 0.5 * delta.x();
            double dy = 0.5 * delta.y();

//            if( e->modifiers() == Qt::ControlModifier ) {
//              if( std::abs(dx) >= std::abs(dy) ) {
//                dy = 0;
//              }
//              else {
//                dx = 0;
//              }
//            }

            QVector3D newForward =
                QQuaternion::fromAxisAndAngle(-Up * dx - Right * dy, 0.2 * hypot(dx, dy))
                    .rotatedVector(forward);

            T0 = minv.map(QVector3D(0, 0, newForward.length()));
            TU = minv.map(QVector3D(0, 0.5, newForward.length()));
            Up = (TU - T0).normalized();

            _viewPoint = newForward + _viewTarget;
            _viewUpDirection = Up;
          }

          _dirty = true;
          showViewTarget(true);
          update();
          Q_EMIT viewPointChanged();
        }
      }

      _prev_mouse_pos = newpos;
    }
  }
  e->accept();
}

#if QT_CONFIG(wheelevent)
void QGLView::wheelEvent(QWheelEvent * e)
{
  const int delta =
      e->angleDelta().y();

  if( delta ) {

    if( !e->modifiers() ) {

      // Move both camera and viewTarget_ forward / backward

      const QVector3D forward =
          _viewTarget - _viewPoint;

      const QVector3D neweye =
          _viewPoint + 1e-4 * forward * delta;  // / forward.length();

      const QVector3D newforward =
          _viewTarget - neweye;

      if( newforward.length() < 3 * _viewParams.nearPlane ||
          QVector3D::dotProduct(forward, newforward)  < 0 )
      {
        _viewTarget += neweye - _viewPoint;
      }

      _viewPoint = neweye;
      _dirty = true;

      showViewTarget(true);
      update();
      Q_EMIT viewPointChanged();

    }
    else if( e->modifiers() == Qt::ControlModifier ) {

      // Move viewTarget_ only (the effect is mouse sensitivity adjustment)

      const QVector3D forward =
          _viewTarget - _viewPoint;

      const QVector3D newtarget =
          _viewTarget + 1e-3 * forward * delta; // / forward.length();

      const QVector3D newforward =
          newtarget - _viewPoint;

      const double p =
          QVector3D::dotProduct(newforward,
              forward);

      if ( p > 0 &&  newforward.length() > _viewParams.nearPlane ) {
        _viewTarget = newtarget;
        _dirty = true;
        showViewTarget(true);
        update();
        Q_EMIT viewPointChanged();
      }

    }
  }

  e->ignore();
}
#endif


void QGLView::setAutoShowViewTarget(bool v)
{
  _autoShowViewTarget = v;
}

bool QGLView::autoShowViewTarget() const
{
  return _autoShowViewTarget;
}

void QGLView::showViewTarget(bool show)
{
  if ( _hideViewTargetTimerId > 0 ) {
    Base::killTimer(_hideViewTargetTimerId);
    _hideViewTargetTimerId = 0;
  }

  if( show && _autoShowViewTarget ) {
    _hideViewTargetTimerId =
        Base::startTimer(500, Qt::CoarseTimer);
  }
}

void QGLView::setEnableSelection(bool v)
{
  if( (_enableGLMouseEvents = v) ) {
    update(); // force repaint to get depth buffer
  }
  else {
    _depthBuffer.release();
  }
}

bool QGLView::enableSelection() const
{
  return _enableGLMouseEvents;
}

void QGLView::timerEvent(QTimerEvent * e)
{
  if ( e->timerId() == _hideViewTargetTimerId)  {
    showViewTarget(false);
    e->ignore();
    update();
    return;
  }

  Base::timerEvent(e);
}

void QGLView::showKeyBindings()
{
  static QDialog * helpWidget;

  if( !helpWidget ) {

    static const QString text =
        " <p><strong>LeftButton</strong> : Rotate camera around of the Up / Right axes </p>"
        " <p><strong>Shift + LeftButton</strong> : Rotate camera around of the Forward Looking axis </p>"
        " <p><strong>Ctrl + Shift + LeftButton</strong> : Toggle auto show target point </p>"
        ""
        " <p><strong>RightButton</strong> : Translate (shift) camera Up / Right  </p>"
        ""
        " <p><strong>Wheel</strong> : Move both camera and View Target Point forward / backward  </p>"
        " <p><strong>Ctrl + Wheel</strong> : Move View Target Point only (the effect is mouse sensitivity adjustment)  </p>"
        ;

    helpWidget =
        new QDialog(qApp->activeWindow());

    helpWidget->setWindowTitle("QGLView Key Bindings");

    QVBoxLayout * layout = new QVBoxLayout(helpWidget);
    QLabel * label = new QLabel(helpWidget);

    label->setTextFormat(Qt::RichText);
    label->setTextInteractionFlags(Qt::TextSelectableByMouse | Qt::TextSelectableByKeyboard);
    label->setText(text);
    layout->addWidget(label);

  }

  if ( helpWidget->isVisible() ) {
    helpWidget->raise();
  }
  else {
    helpWidget->setParent(qApp->activeWindow());
    helpWidget->show();
  }

}

bool QGLView::copyViewportToClipboard()
{
  QClipboard * clipboard =
      QApplication::clipboard();

  if ( !clipboard ) {
    return false;
  }

  clipboard->setPixmap(Base::grab());

  return true;
}

