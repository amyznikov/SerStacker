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
 */

#include "QGLView.h"
#include <core/debug.h>

namespace qgltest {

namespace {

void toSpherical(const QVector3D & v, double * r, double * phi, double * theta)
{
  *r = v.length();
  *phi = acos(v.z() / *r);
  *theta = atan2(v.y(), v.x());
}

QVector3D fromSpherical(double r, double phi, double theta)
{
  return QVector3D(r * cos(theta) * sin(phi),
      r * sin(theta) * sin(phi),
      r * cos(phi));
}

template<typename T>
inline int sign(T x)
{
  return x >= 0 ? 1 : -1;
}

} // namespace

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


void QGLView::setBackgroundColor(const QColor &color)
{
  backgroundColor_ = color;
  dirty_ = true;
  update();
}

const QColor & QGLView::backgroundColor() const
{
  return backgroundColor_;
}

void QGLView::setForegroundColor(const QColor &color)
{
  foregroundColor_ = color;
  dirty_ = true;
  update();
}

const QColor & QGLView::foregroundColor() const
{
  return foregroundColor_;
}

void QGLView::setFOV(double radians)
{
  fov_ = radians;
  dirty_ = true;
  update();
}

double QGLView::fov() const
{
  return fov_;
}

void QGLView::setNearPlane(double v)
{
  nearPlane_ = v;
  dirty_ = true;
  update();
}

double QGLView::nearPlane() const
{
  return nearPlane_;
}

void QGLView::setFarPlane(double v)
{
  farPlane_ = v;
  dirty_ = true;
  update();
}

double QGLView::farPlane() const
{
  return farPlane_;
}

void QGLView::setPerspecitive(double fov, double nearPlane, double farPlane)
{
  fov_ = fov;
  nearPlane_ = nearPlane;
  farPlane_ = farPlane;
  dirty_ = true;
  update();
}

void QGLView::cameraTo(const QVector3D & eye_pos,
    const QVector3D & target_pos,
    const QVector3D & up_direction)
{
  eye_ = eye_pos;
  target_ = target_pos;
  updirection_ = up_direction;
  dirty_ = true;
  update();
}

QPointF QGLView::projectToScreen(const QVector3D & pos) const
{
  // view port
  const int x = 0;
  const int y = 0;
  const int w = width();
  const int h = height();

  // normalized device coordinates
  const QVector3D ndc =
      matrix_ * pos;

  // window coordinates
  return QPointF(((ndc.x() + 1.0) / 2.0) * w + x,
      ((1.0 - ndc.y()) / 2.0) * h + y);
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
  dirty_ = true;
}

void QGLView::paintGL()
{
  glPreDraw();
  glDraw();
  glPostDraw();
}

void QGLView::cleanupGL()
{
  Base::makeCurrent();

  disconnect(context(), &QOpenGLContext::aboutToBeDestroyed,
      this, &ThisClass::cleanupGL);

  glCleanup();

  Base::doneCurrent();
}

void QGLView::glInit()
{
  // Default colors
  setForegroundColor(QColor(180, 180, 180));
  setBackgroundColor(QColor(100, 100, 100));
}

void QGLView::glPreDraw()
{
  glDisable(GL_LIGHTING);
  glDisable(GL_COLOR_MATERIAL);
  glEnable(GL_PROGRAM_POINT_SIZE);
  glEnable(GL_DEPTH_TEST);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glClearColor(backgroundColor_.redF(),
      backgroundColor_.greenF(),
      backgroundColor_.blueF(),
      backgroundColor_.alphaF());

  glPushMatrix();
  glMatrixMode(GL_PROJECTION);

  const int w = width();
  const int h = height();

  if( dirty_ && w > 0 && h > 0 ) {

    glViewport(0, 0, GLint(w), GLint(h));

    matrix_.setToIdentity();
    matrix_.perspective(fov_ * 180 / M_PI, GLfloat(w) / h, 1.f, 100.0f);
    matrix_.lookAt(eye_, target_, updirection_);

    dirty_ = false;
  }

  glLoadMatrixf(matrix_.constData());
}

void QGLView::glDraw()
{
}

void QGLView::glPostDraw()
{
  glPopMatrix();
  glFlush();
}

// still non clear how to manage GL cleanup from destructor
void QGLView::glCleanup()
{
}


void QGLView::drawText(const QPointF & pos, const QFont & font, const QString & text)
{
  // Retrieve last OpenGL color to use as a font color
  GLdouble glColor[4];
  glGetDoublev(GL_CURRENT_COLOR, glColor);

  QColor fontColor(255 * glColor[0], 255 * glColor[1],
      255 * glColor[2], 255 * glColor[3]);

  // Render text
  QPainter painter(this);
  painter.setPen(fontColor);
  painter.setFont(font);
  painter.drawText(pos, text);
  painter.end();
}

void QGLView::drawText(double x, double y, const QFont & font, const QString & text)
{
  drawText(QPointF(x, y), font, text);
}

void QGLView::drawText(const QVector3D & pos, const QFont &font, const QString &text)
{
  drawText(projectToScreen(pos), font, text);
}

void QGLView::drawText(double x, double y, double z, const QFont &font, const QString &text)
{
  drawText(projectToScreen(QVector3D(x, y, z)), font, text);
}

void QGLView::vaprintf(const QPointF & pos, const QFont &font, const char * format, va_list arglist)
{
#if QT_VERSION < QT_VERSION_CHECK(5, 14, 0)
  QString text;
  text.vsprintf(format, arglist);
  drawText(pos, font, text);
#else
  drawText(pos, font, QString::vasprintf(format, arglist));
#endif
}

void QGLView::vaprintf(double x, double y, const QFont & font, const char * format, va_list arglist)
{
  vaprintf(QPointF(x, y), font, format, arglist);
}

void QGLView::vasprintf(const QVector3D & pos, const QFont &font, const char * format, va_list arglist)
{
  vaprintf(projectToScreen(pos), font, format, arglist);
}

void QGLView::vasprintf(double x, double y, double z, const QFont &font, const char * format, va_list arglist)
{
  vaprintf(projectToScreen(QVector3D(x,y, z)), font, format, arglist);
}

void QGLView::printf(const QPointF & pos, const QFont &font, const char * format, ...)
{
  va_list arglist;
  va_start(arglist, format);
  vaprintf(pos, font, format, arglist);
  va_end(arglist);
}

void QGLView::printf(double x, double y, const QFont &font, const char * format, ...)
{
  va_list arglist;
  va_start(arglist, format);
  vaprintf(QPointF(x,y), font, format, arglist);
  va_end(arglist);
}

void QGLView::printf(const QVector3D & pos, const QFont &font, const char * format, ...)
{
  va_list arglist;
  va_start(arglist, format);
  vaprintf(projectToScreen(pos), font, format, arglist);
  va_end(arglist);
}

void QGLView::printf(double x, double y, double z, const QFont & font, const char * format, ...)
{
  va_list arglist;
  va_start(arglist, format);
  vaprintf(projectToScreen(QVector3D(x, y, z)), font, format, arglist);
  va_end(arglist);
}


void QGLView::mousePressEvent(QMouseEvent *e)
{
  prev_mouse_pos_ = e->localPos();
  e->ignore();
}

void QGLView::mouseReleaseEvent(QMouseEvent *e)
{
  e->ignore();
}

void QGLView::mouseDoubleClickEvent(QMouseEvent *e)
{
  e->ignore();
}


// temporary very stupid mouse move management,
// may be consider also deformed Mouse Trackball algorithm instead ?
void QGLView::mouseMoveEvent(QMouseEvent * e)
{
  if( e->buttons() == Qt::LeftButton ) {

    const QPointF newpos = e->localPos();
    const QPointF delta = newpos - prev_mouse_pos_;

    if( e->modifiers() == Qt::ShiftModifier ) {

      if( delta.x() || delta.y() ) {

        QMatrix4x4 minv =
            matrix_.inverted();

        const QVector3D v0 =
            minv.map(QVector3D(0.0, 0.0, 1.0));

        if( delta.y() ) {

          QVector3D vv =
              minv.map(QVector3D(0.0, 0.1, 1.0)) - v0;

          vv *= 1e-2 * delta.y() / vv.length();
          target_ += vv;
          eye_ += vv;
        }

        if( delta.x() ) {

          QVector3D vh =
              minv.map(QVector3D(0.1, 0.0, 1.0)) - v0;

          vh *= 1e-2 * delta.x() / vh.length();

          target_ -= vh;
          eye_ -= vh;
        }

        dirty_ = true;
        update();
      }
    }
    else {

      double r, phi, theta, newphi, newtheta;
      toSpherical(eye_ - target_, &r, &phi, &theta);

      newphi = phi - 5e-3 * sign(updirection_.z()) * delta.y();
      newtheta = theta - 5e-3 * sign(updirection_.z()) * delta.x();

      if( newphi < 0 ) {
        newphi = -newphi;
        newtheta += M_PI;
        updirection_ = -updirection_;
      }
      else if( newphi >= M_PI ) {
        newphi = 2 * M_PI - newphi;
        newtheta += M_PI;
        updirection_ = -updirection_;
      }

      eye_ = fromSpherical(r, newphi, newtheta) + target_;

      dirty_ = true;

      Q_EMIT eyeChanged();

      update();
    }

    prev_mouse_pos_ = newpos;
  }

  e->accept();

}

#if QT_CONFIG(wheelevent)
void QGLView::wheelEvent(QWheelEvent *e)
{
  Base::wheelEvent(e);

  QPoint delta = e->pixelDelta();

  double L = eye_.length();
  if( (L -= delta.y() / 100) > 0.5 ) {
    eye_ *= L / eye_.length();
    dirty_ = true;
    update();
  }

}
#endif

} /* namespace qgltest */
