/*
 * QGLView.h
 *
 *  Created on: Feb 1, 2023
 *      Author: amyznikov
 *
 *  https://doc.qt.io/qt-6/qopenglwidget.html
 *
 *  TODO: http://aco.ifmo.ru/el_books/computer_visualization/lectures/8.html
 */

#pragma once
#ifndef __QGLView_h__
#define __QGLView_h__

#include <QtGui/QtGui>
#include <QtWidgets/QtWidgets>
#include <opencv2/opencv.hpp>

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

  enum Projection {
    Perspective,
    Orthographic,
  };

  struct ViewParams
  {
    Projection projection = Projection::Perspective;
    double fov = 90; // degrees
    double nearPlane = 0.2;
    double farPlane = 1000.;
  };

  struct PlanarGridOptions
  {
    QString name;
    float maxDistance = 1000;
    float step = 100;
    QVector3D Rotation = QVector3D(0, 0, 0); // angles [Rx;Ry;Rz]
    QVector3D Translation = QVector3D(0, 0, 0);
    QColor gridColor = QColor(255, 255, 255);
    QColor fillColor = QColor(158, 172, 148);
    int gridOpaqueness = 128;
    int fillOpaqueness = 64;
    bool visible = false;

  };

  explicit QGLView(QWidget * parent = nullptr);
  ~QGLView();

  void loadParameters();
  void saveParameters();

  void setBackgroundColor(const QColor &color);
  const QColor & backgroundColor() const;

  void setForegroundColor(const QColor &color);
  const QColor & foregroundColor() const;

  const ViewParams & viewParams() const;

  const std::vector<PlanarGridOptions> & grids() const;
  std::vector<PlanarGridOptions> & grids();

  Projection projection() const;
  void setProjection(Projection v);

  void setFOV(double degrees);
  double fov() const;

  void setNearPlane(double v);
  double nearPlane() const;

  void setFarPlane(double v);
  double farPlane() const;

  void setShowMainAxes(bool v);
  bool showMainAxes() const;

  void setMainAxesLength(double v);
  double mainAxesLength() const;

  void setViewPoint(const QVector3D & eye);
  const QVector3D & viewPoint() const;

  void setViewTargetPoint(const QVector3D & v);
  const QVector3D & viewTargetPoint() const;

  void setUpDirection(const QVector3D & v);
  const QVector3D & upDirection() const;

  void setAutoShowViewTarget(bool v);
  bool autoShowViewTarget() const;

  void setEnableSelection(bool v);
  bool enableSelection() const;

  void setPerspecitive(double fov_degrees, double nearPlane, double farPlane);

  void cameraTo(const QVector3D & viewPoint, const QVector3D & viewTargetPoint, const QVector3D & viewUpDirection);
  void lookTo(const QVector3D &target);

  bool projectToScreen(const QVector3D & pos, QPointF * screen_pos) const;

  void drawText(const QPointF & pos, const QFont &font, const QString &str);
  void drawText(double x, double y, const QFont &font, const QString &str);
  void drawText(const QVector3D & pos, const QFont &font, const QString &str);
  void drawText(double x, double y, double z, const QFont &font, const QString &str);

  void glvaprintf(const QPointF & pos, const QFont &font, const char * format, va_list arglist) Q_ATTRIBUTE_FORMAT_PRINTF(4, 0);
  void glvaprintf(double x, double y, const QFont &font, const char * format, va_list arglist) Q_ATTRIBUTE_FORMAT_PRINTF(5, 0);
  void glvaprintf(const QVector3D & pos, const QFont &font, const char * format, va_list arglist) Q_ATTRIBUTE_FORMAT_PRINTF(4, 0);
  void glvaprintf(double x, double y, double z, const QFont &font, const char * format, va_list arglist) Q_ATTRIBUTE_FORMAT_PRINTF(6, 0);

  void glprintf(const QPointF & pos, const QFont &font, const char * format, ...) Q_ATTRIBUTE_FORMAT_PRINTF(4, 5);
  void glprintf(double x, double y, const QFont &font, const char * format, ...) Q_ATTRIBUTE_FORMAT_PRINTF(5, 6);
  void glprintf(const QVector3D & pos, const QFont &font, const char * format, ...) Q_ATTRIBUTE_FORMAT_PRINTF(4, 5);
  void glprintf(double x, double y, double z, const QFont &font, const char * format, ...) Q_ATTRIBUTE_FORMAT_PRINTF(6, 7);


  void drawArrow(qreal length, qreal radius, int nbSubdivisions);
  void drawArrow(const QVector3D & start, const QVector3D & end, qreal radius, int nbSubdivisions );
  void drawMainAxes();

  // void drawLine(const QVector3D & start, const QVector3D & end);

  bool copyViewportToClipboard();
  void showKeyBindings();

  static inline void glVertex(const QVector3D & v)
  {
    glVertex3f(v.x(), v.y(), v.z());
  }


Q_SIGNALS:
  void viewPointChanged();
  void displayImageChanged();

public Q_SLOTS:
  void redraw()
  {
    ThisClass::update();
  }

protected:
  virtual void glInit();
  virtual void glPreDraw();
  virtual void glDraw();
  virtual void glPostDraw();
  virtual void glCleanup();
  virtual void onLoadParameters(QSettings & settings);
  virtual void onSaveParameters(QSettings & settings);

protected:
  void timerEvent(QTimerEvent *event) override;
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
  void paintEvent(QPaintEvent *e) override;

  virtual void cleanupGL();
  virtual void glPointSelection(double objX, double objY, double objZ,
      const QPointF & mousePos, bool fMouseMove);

protected:
  void showViewTarget(bool v);
  void onGLPointSelection(const QPointF & mouse_pos, bool mouse_move);

protected:
  QColor _backgroundColor = QColor(80, 80, 80); // QColor(32, 32, 32);
  QColor _foregroundColor = QColor(232, 232, 232);

  ViewParams _viewParams;
  std::vector<PlanarGridOptions> _grids;

  QVector3D _viewPoint = QVector3D(40, 30, 30);
  QVector3D _viewTarget = QVector3D(0, 0, 0);
  QVector3D _viewUpDirection = QVector3D(0, 0, 1);
  double _mainAxesLength = 0; // auto
  bool _showMainAxes = true;

  QMatrix4x4 _mview;
  QMatrix4x4 _mprojection;
  QMatrix4x4 _mtotal;

  // current view port, update in resizeGL()
  struct {
    int x = 0;
    int y = 0;
    int w = 1;
    int h = 1;
  } viewport;

  QPointF _prev_mouse_pos;
  bool _dirty = true;
  bool _enableSelection = false;
  bool _autoShowViewTarget = false;
  int _hideViewTargetTimerId = 0;

  // selection support
  cv::Mat1f _depthBuffer;
  GLint _sviewport[4] = {0};
  GLdouble _smodelview[16] = {0};
  GLdouble _sprojection[16] = {0};


};


/**
 * Fills m[4][4] with the OpenGL (column-major) representation of the QQuaternion rotation
 * Return pointer to m[][]
 * */
GLfloat * getMatrix(const QQuaternion & q, float m[4][4]);


#endif /* __QGLView_h__ */
