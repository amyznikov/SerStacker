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

void QGLView::setCameraPos(const QVector3D & eye)
{
  eye_ = eye;
  dirty_ = true;
  update();
}

const QVector3D & QGLView::cameraPos() const
{
  return eye_;
}

void QGLView::setTarget(const QVector3D & v)
{
  target_ = v;
  dirty_ = true;
  update();
}

const QVector3D & QGLView::target() const
{
  return target_;
}


void QGLView::setUpDirection(const QVector3D & v)
{
  updirection_ = v;
  dirty_ = true;
  update();
}

const QVector3D & QGLView::upDirection() const
{
  return updirection_;
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

void QGLView::lookTo(const QVector3D &target)
{
  setTarget(target);
}

bool QGLView::projectToScreen(const QVector3D & pos, QPointF * screen_pos) const
{
  // normalized device coordinates
  const QVector3D ndc =
      mtotal_.map(pos);

  if( ndc.z() < 1 && fabsf(ndc.x()) < 1 && fabsf(ndc.y()) < 1 ) {

    // apply view port to compute window coordinates
    screen_pos->setX(((ndc.x() + 1.0) / 2.0) * vp.w + vp.x);
    screen_pos->setY(((1.0 - ndc.y()) / 2.0) * vp.h + vp.y);
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
    glViewport(vp.x = 0, vp.y = 0, GLint(vp.w = w), GLint(vp.h = h));
    dirty_ = true;
    update();
  }
}

void QGLView::paintGL()
{
  if( vp.w > 1 && vp.h > 1 ) {
    glPreDraw();
    glDraw();
    glPostDraw();
  }
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
  setForegroundColor(QColor(200, 200, 200));
  setBackgroundColor(QColor(32, 32, 32));
}

void QGLView::glPreDraw()
{
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  if( dirty_ && vp.w > 0 && vp.h > 0 ) {

    mview_.setToIdentity();
    mview_.lookAt(eye_, target_, updirection_);

    mperspective_.setToIdentity();
    mperspective_.perspective(fov_ * 180 / M_PI, GLfloat(vp.w) / vp.h, nearPlane_, farPlane_);

    mtotal_ = mperspective_ * mview_;

    dirty_ = false;
  }

  glMatrixMode(GL_PROJECTION);
  glLoadMatrixf(mtotal_.constData());

  glClearColor(backgroundColor_.redF(),
      backgroundColor_.greenF(),
      backgroundColor_.blueF(),
      backgroundColor_.alphaF());
}

void QGLView::glDraw()
{
}

void QGLView::glPostDraw()
{
  glFlush();
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

  qreal length = eye_.length() / 3;

  drawArrow(QVector3D(0, 0, 0), QVector3D(length, 0, 0), length / 100, 8);
  drawArrow(QVector3D(0, 0, 0), QVector3D(0, length, 0), length / 100, 8);
  drawArrow(QVector3D(0, 0, 0), QVector3D(0, 0, length), length / 100, 8);

  drawText(QVector3D(length, 0, 0), font, "X");
  drawText(QVector3D(0, length, 0), font, "Y");
  drawText(QVector3D(0, 0, length), font, "Z");
}

void QGLView::mousePressEvent(QMouseEvent *e)
{
#if QT_VERSION >= QT_VERSION_CHECK(6,0,0)
  prev_mouse_pos_ = e->position();
#else
  prev_mouse_pos_ = e->localPos();
#endif
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
  if( e->buttons() ) {

#if QT_VERSION >= QT_VERSION_CHECK(6,0,0)
    const QPointF newpos = e->position();
#else
    const QPointF newpos = e->localPos();
#endif

    if( e->buttons() == Qt::RightButton ) { // Translate (shift) camera Up / Right

      const QPointF delta =
          newpos - prev_mouse_pos_;

      if( delta.x() || delta.y() ) {

        const QVector3D forward =
            eye_ - target_;

        const QMatrix4x4 minv =
            mview_.inverted();

        const QVector3D T0 =
            minv.map(QVector3D(0, 0, forward.length()));

        if( delta.y() ) {

          const QVector3D TU =
              minv.map(QVector3D(0, delta.y() / vp.h, forward.length()));

          const QVector3D Up =
              (TU - T0) * std::max( (float)nearPlane_, forward.length());

          target_ += Up;
          eye_ += Up;
        }

        if( delta.x() ) {

          const QVector3D TR =
              minv.map(QVector3D(-delta.x() / vp.w, 0, forward.length()));

          const QVector3D Right =
              (TR - T0) * std::max((float) nearPlane_, forward.length());

          target_ += Right;
          eye_ += Right;
        }

        dirty_ = true;

        Q_EMIT eyeChanged();
        update();
      }

      prev_mouse_pos_ = newpos;
    }

    else if( e->buttons() == Qt::LeftButton ) { // Rotate camera

      const QPointF delta =
          newpos - prev_mouse_pos_;

      if( delta.x() || delta.y() ) {

        const QVector3D forward =
            eye_ - target_;

        const QMatrix4x4 minv =
            mview_.inverted();

        if( e->modifiers() == Qt::ShiftModifier ) { // Rotate around forward looking axis

          const int signy =
              newpos.x() > vp.w / 2 ? +1 : -1;

          const int signx =
              newpos.y() < vp.h / 2 ? +1 : -1;

          updirection_ =
              QQuaternion::fromAxisAndAngle(forward, 0.2 * (signy * delta.y() + signx * delta.x()))
                  .rotatedVector(updirection_);

          dirty_ = true;

          Q_EMIT eyeChanged();
          update();
        }

        else { // Rotate camera around of the Up / Right axes

          QVector3D T0 = minv.map(QVector3D(0, 0, forward.length()));
          QVector3D TU = minv.map(QVector3D(0, 0.5, forward.length()));
          QVector3D TR = minv.map(QVector3D(0.5, 0, forward.length()));
          QVector3D Up = (TU - T0).normalized();
          QVector3D Right = (TR - T0).normalized();

          QVector3D newForward =
              QQuaternion::fromAxisAndAngle(-Up * delta.x() - Right * delta.y(), 0.2 * hypot(delta.x(), delta.y()))
                  .rotatedVector(forward);

          T0 = minv.map(QVector3D(0, 0, newForward.length()));
          TU = minv.map(QVector3D(0, 0.5, newForward.length()));
          Up = (TU - T0).normalized();

          eye_ = newForward + target_;
          updirection_ = Up;
          dirty_ = true;

          Q_EMIT eyeChanged();
          update();
        }
      }

      prev_mouse_pos_ = newpos;
    }
  }
  e->accept();
}

#if QT_CONFIG(wheelevent)
void QGLView::wheelEvent(QWheelEvent * e)
{
  const double delta =
      e->pixelDelta().y();

  if( delta ) {

    if( !e->modifiers() ) {

      // Move both camera and target forward / backward

      const QVector3D forward =
          target_ - eye_;

      const QVector3D neweye =
          eye_ + 1e-2 * forward * delta / forward.length();

      target_ += neweye - eye_;
      eye_ = neweye;
      dirty_ = true;

      Q_EMIT eyeChanged();
      update();

    }
    else if( e->modifiers() == Qt::ControlModifier ) {

      // Move camera forward / backward

      const QVector3D forward =
          target_ - eye_;

      const QVector3D neweye =
          eye_ + 1e-2 * forward * delta / forward.length();

      const QVector3D newforward =
          target_ - neweye;

      const double p =
          QVector3D::dotProduct(newforward,
              forward);

      if( p > 0 ) {
        // just move camera
        eye_ = neweye;
        dirty_ = true;

        Q_EMIT eyeChanged();
        update();
      }
      else if( p < 0 ) {
        // don't allow camera to jump target

        target_ += neweye - eye_;
        eye_ = neweye;
        dirty_ = true;

        Q_EMIT eyeChanged();
        update();
      }
    }
  }

  e->ignore();
}
#endif
