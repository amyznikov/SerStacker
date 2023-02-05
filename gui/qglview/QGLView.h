/*
 * QGLView.h
 *
 *  Created on: Feb 1, 2023
 *      Author: amyznikov
 *
 *  https://doc.qt.io/qt-6/qopenglwidget.html
 */

#pragma once
#ifndef __QGLView_h__
#define __QGLView_h__

#include <QtGui/QtGui>
#include <QtWidgets/QtWidgets>

#if QT_VERSION >= QT_VERSION_CHECK(6,0,0)
# include <QtOpenGLWidgets/QOpenGLWidget>
#endif

// GLU was removed from Qt in version 4.8
#ifdef Q_OS_MAC
# include <OpenGL/glu.h>
#else
# include <GL/glu.h>
#endif

class QGLView :
  public QOpenGLWidget,
  protected QOpenGLExtraFunctions
{
  Q_OBJECT;
public:
  typedef QGLView ThisClass;
  typedef QOpenGLWidget Base;

  explicit QGLView(QWidget * parent = nullptr);
  ~QGLView();

  void setBackgroundColor(const QColor &color);
  const QColor & backgroundColor() const;

  void setForegroundColor(const QColor &color);
  const QColor & foregroundColor() const;

  void setFOV(double radians);
  double fov() const;

  void setNearPlane(double v);
  double nearPlane() const;

  void setFarPlane(double v);
  double farPlane() const;

  void setTarget(const QVector3D & v);
  const QVector3D & target() const;

  void setUpDirection(const QVector3D & v);
  const QVector3D & upDirection() const;

  void setPerspecitive(double fov_radians, double nearPlane, double farPlane);

  void cameraTo(const QVector3D & eye_pos, const QVector3D & target_pos, const QVector3D & up_direction);
  void lookTo(const QVector3D &target);

  QPointF projectToScreen(const QVector3D & pos) const;

  void drawText(const QPointF & pos, const QFont &font, const QString &str);
  void drawText(double x, double y, const QFont &font, const QString &str);
  void drawText(const QVector3D & pos, const QFont &font, const QString &str);
  void drawText(double x, double y, double z, const QFont &font, const QString &str);

  void vaprintf(const QPointF & pos, const QFont &font, const char * format, va_list arglist) Q_ATTRIBUTE_FORMAT_PRINTF(4, 0);
  void vaprintf(double x, double y, const QFont &font, const char * format, va_list arglist) Q_ATTRIBUTE_FORMAT_PRINTF(5, 0);
  void vasprintf(const QVector3D & pos, const QFont &font, const char * format, va_list arglist) Q_ATTRIBUTE_FORMAT_PRINTF(4, 0);
  void vasprintf(double x, double y, double z, const QFont &font, const char * format, va_list arglist) Q_ATTRIBUTE_FORMAT_PRINTF(6, 0);

  void printf(const QPointF & pos, const QFont &font, const char * format, ...) Q_ATTRIBUTE_FORMAT_PRINTF(4, 5);
  void printf(double x, double y, const QFont &font, const char * format, ...) Q_ATTRIBUTE_FORMAT_PRINTF(5, 6);
  void printf(const QVector3D & pos, const QFont &font, const char * format, ...) Q_ATTRIBUTE_FORMAT_PRINTF(4, 5);
  void printf(double x, double y, double z, const QFont &font, const char * format, ...) Q_ATTRIBUTE_FORMAT_PRINTF(6, 7);

  void drawArrow(qreal length, qreal radius, int nbSubdivisions);
  void drawArrow(const QVector3D & start, const QVector3D & end, qreal radius, int nbSubdivisions );
  void drawMainAxes();

Q_SIGNALS:
  void eyeChanged();

protected:
  virtual void glInit();
  virtual void glPreDraw();
  virtual void glDraw();
  virtual void glPostDraw();
  virtual void glCleanup();

protected:
  void mousePressEvent(QMouseEvent *e) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void mouseDoubleClickEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
#if QT_CONFIG(wheelevent)
  void wheelEvent(QWheelEvent *event);
#endif

protected:
  void initializeGL() override;
  void resizeGL(int w, int h) override;
  void paintGL() override;
  virtual void cleanupGL();

protected:
  QColor backgroundColor_ = Qt::black;
  QColor foregroundColor_ = Qt::white;
  double fov_ = M_PI / 2;
  double nearPlane_ = 1.0;
  double farPlane_ = 100.0;

  QVector3D eye_ = QVector3D(40, 30, 30);
  QVector3D target_ = QVector3D(0, 0, 0);
  QVector3D updirection_ = QVector3D(0, 0, 1);

  QMatrix4x4 mview_;
  QMatrix4x4 mperspective_;
  QMatrix4x4 mtotal_;

  QPointF prev_mouse_pos_;
  bool dirty_ = true;

};


/**
 * Fills m[4][4] with the OpenGL (column-major) representation of the QQuaternion rotation
 * Return pointer to m[][]
 * */
GLfloat * getMatrix(const QQuaternion & q, float m[4][4]);


#endif /* __QGLView_h__ */
