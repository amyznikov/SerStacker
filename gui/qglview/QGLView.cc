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

void QGLView::setViewPoint(const QVector3D & eye)
{
  viewPoint_ = eye;
  dirty_ = true;
  update();
}

const QVector3D & QGLView::viewPoint() const
{
  return viewPoint_;
}

void QGLView::setViewTargetPoint(const QVector3D & v)
{
  viewTarget_ = v;
  dirty_ = true;
  update();
}

const QVector3D & QGLView::viewTargetPoint() const
{
  return viewTarget_;
}


void QGLView::setUpDirection(const QVector3D & v)
{
  viewUpDirection_ = v;
  dirty_ = true;
  update();
}

const QVector3D & QGLView::upDirection() const
{
  return viewUpDirection_;
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
  viewPoint_ = eye_pos;
  viewTarget_ = target_pos;
  viewUpDirection_ = up_direction;
  dirty_ = true;
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
      mtotal_.map(pos);

  if( ndc.z() < 1 && fabsf(ndc.x()) < 1 && fabsf(ndc.y()) < 1 ) {

    // apply view port to compute window coordinates
    screen_pos->setX(((ndc.x() + 1.0) / 2.0) * viewport.w + viewport.x);
    screen_pos->setY(((1.0 - ndc.y()) / 2.0) * viewport.h + viewport.y);
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
    dirty_ = true;
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

  if( dirty_ && viewport.w > 0 && viewport.h > 0 ) {

    mview_.setToIdentity();
    mview_.lookAt(viewPoint_, viewTarget_,
        viewUpDirection_);

    mperspective_.setToIdentity();
    mperspective_.perspective(fov_ * 180 / M_PI,
        GLfloat(viewport.w) / viewport.h,
        nearPlane_, farPlane_);

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
  const bool showViewTarget =
      hideViewTargetTimerId_ > 0;

  if ( showViewTarget ) {

    const qreal length = viewPoint_.length() / 8;

    const QVector3D arrow_start[3] = {
        QVector3D(viewTarget_.x() - length, viewTarget_.y(), viewTarget_.z()),
        QVector3D(viewTarget_.x(), viewTarget_.y() - length, viewTarget_.z()),
        QVector3D(viewTarget_.x(), viewTarget_.y(), viewTarget_.z() - length),
    };

    const QVector3D arrow_end[3] = {
        QVector3D(viewTarget_.x() + 2 * length, viewTarget_.y(), viewTarget_.z()),
        QVector3D(viewTarget_.x(), viewTarget_.y() + 2 * length, viewTarget_.z()),
        QVector3D(viewTarget_.x(), viewTarget_.y(), viewTarget_.z() + 2 * length),
    };

    glColor3ub(255, 255, 64);

    for ( int i = 0; i < 3; ++i ) {
      drawArrow(arrow_start[i], arrow_end[i], length / 32, 4);
    }
  }

  glFlush();

  Q_EMIT displayImageChanged();
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

  const qreal length = viewPoint_.length() / 3;

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

  if( (e->buttons() == Qt::LeftButton) && (e->modifiers() == (Qt::ShiftModifier | Qt::ControlModifier)) ) {
    setAutoShowViewTarget(!autoShowViewTarget());
    update();
  }

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
            viewPoint_ - viewTarget_;

        const QMatrix4x4 minv =
            mview_.inverted();

        const QVector3D T0 =
            minv.map(QVector3D(0, 0, forward.length()));

        if( delta.y() ) {

          const QVector3D TU =
              minv.map(QVector3D(0, delta.y() / viewport.h, forward.length()));

          const QVector3D Up =
              (TU - T0) * std::max( (float)nearPlane_, forward.length());

          viewTarget_ += Up;
          viewPoint_ += Up;
        }

        if( delta.x() ) {

          const QVector3D TR =
              minv.map(QVector3D(-delta.x() / viewport.w, 0, forward.length()));

          const QVector3D Right =
              (TR - T0) * std::max((float) nearPlane_, forward.length());

          viewTarget_ += Right;
          viewPoint_ += Right;
        }

        dirty_ = true;

        showViewTarget(true);
        update();
        Q_EMIT viewPointChanged();
      }

      prev_mouse_pos_ = newpos;
    }

    else if( e->buttons() == Qt::LeftButton ) { // Rotate camera

      const QPointF delta =
          newpos - prev_mouse_pos_;

      if( delta.x() || delta.y() ) {

        const QVector3D forward =
            viewPoint_ - viewTarget_;

        const QMatrix4x4 minv =
            mview_.inverted();

        if( e->modifiers() == Qt::ShiftModifier ) { // Rotate around forward looking axis

          const int signy =
              newpos.x() > viewport.w / 2 ? +1 : -1;

          const int signx =
              newpos.y() < viewport.h / 2 ? +1 : -1;

          viewUpDirection_ =
              QQuaternion::fromAxisAndAngle(forward,
                  0.2 * (signy * delta.y() + signx * delta.x()))
                  .rotatedVector(viewUpDirection_);

          dirty_ = true;

          showViewTarget(true);
          update();
          Q_EMIT viewPointChanged();
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

          viewPoint_ = newForward + viewTarget_;
          viewUpDirection_ = Up;
          dirty_ = true;

          showViewTarget(true);
          update();
          Q_EMIT viewPointChanged();
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
  const int delta =
      e->angleDelta().y();

  if( delta ) {

    if( !e->modifiers() ) {

      // Move both camera and viewTarget_ forward / backward

      const QVector3D forward =
          viewTarget_ - viewPoint_;

      const QVector3D neweye =
          viewPoint_ + 1e-4 * forward * delta;  // / forward.length();

      viewTarget_ += neweye - viewPoint_;
      viewPoint_ = neweye;
      dirty_ = true;

      showViewTarget(true);
      update();
      Q_EMIT viewPointChanged();

    }
    else if( e->modifiers() == Qt::ControlModifier ) {

      // Move viewTarget_ only (the effect is mouse sensitivity adjustment)

      const QVector3D forward =
          viewTarget_ - viewPoint_;

      const QVector3D newtarget =
          viewTarget_ + 1e-3 * forward * delta; // / forward.length();

      const QVector3D newforward =
          newtarget - viewPoint_;

      const double p =
          QVector3D::dotProduct(newforward,
              forward);

      if ( p > 0 &&  newforward.length() > nearPlane_ ) {
        viewTarget_ = newtarget;
        dirty_ = true;
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
  autoShowViewTarget_ = v;
}

bool QGLView::autoShowViewTarget() const
{
  return autoShowViewTarget_;
}

void QGLView::showViewTarget(bool show)
{
  if ( hideViewTargetTimerId_ > 0 ) {
    Base::killTimer(hideViewTargetTimerId_);
    hideViewTargetTimerId_ = 0;
  }

  if( show && autoShowViewTarget_ ) {
    hideViewTargetTimerId_ =
        Base::startTimer(500, Qt::CoarseTimer);
  }
}

void QGLView::timerEvent(QTimerEvent * e)
{
  if ( e->timerId() == hideViewTargetTimerId_)  {
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
