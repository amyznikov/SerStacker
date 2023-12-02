/*
 * QCloudViewer.h
 *
 *  Created on: May 18, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCloudViewer_h__
#define __QCloudViewer_h__

#include <gui/qglview/QGLView.h>
#include <gui/qmtf/QMtfDisplay.h>
#include <core/proc/histogram.h>
#include "QPointCloud.h"

class QGLCloudViewer;

class QCloudViewMtfDisplay :
    public QMtfDisplay
{
  Q_OBJECT;
public:
  typedef QCloudViewMtfDisplay ThisClass;
  typedef QMtfDisplay Base;

  QCloudViewMtfDisplay(QGLCloudViewer * cloudView);

  QGLCloudViewer * cloudView() const;

  const c_enum_member * displayTypes() const override;
  void getInputDataRange(double * minval, double * maxval) const override;
  void getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax) override;
  void getOutputHistogramm(cv::OutputArray H, double * hmin, double * hmax) override;

protected:
  QGLCloudViewer * cloudView_ = nullptr;
};


class QGLCloudViewer :
    public QGLView
{
  Q_OBJECT;
public:
  typedef QGLCloudViewer ThisClass;
  typedef QGLView Base;
  friend class QCloudViewMtfDisplay;

  QGLCloudViewer(QWidget* parent = nullptr);

  QCloudViewMtfDisplay & mtfDisplay();
  const QCloudViewMtfDisplay & mtfDisplay() const;

  void setPointSize(double v);
  double pointSize() const;

  void setPointBrightness(double v);
  double pointBrightness() const;

  void setSceneOrigin(const QVector3D & v);
  QVector3D sceneOrigin() const;

  void rotateToShowCloud();

  const std::vector<QPointCloud::sptr> & clouds() const;
  const QPointCloud::sptr & cloud(int index) const;

  bool openPlyFile(const QString & pathFileName);
  void add(const QPointCloud::sptr & cloud);
  void clear();

  void updateDisplayPoints();
  void updateDisplayColors();

protected:
  void glInit() override;
  void glPreDraw() override;
  void glDraw() override;
  void glPostDraw() override;
  void computeDisplayPoints();

protected:
  std::vector<QPointCloud::sptr> clouds_;
  QCloudViewMtfDisplay mtfDisplay_;
  std::vector<cv::Vec3f> display_points_;
  std::vector<cv::Vec3b> display_colors_;

  QVector3D sceneOrigin_;
  double pointSize_ = 2;
  double pointBrightness_ = 0;

  bool update_display_points_ = false;
  bool update_display_colors_ = false;
  int display_color_channels_ = 0;
};

class QCloudViewer :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QCloudViewer ThisClass;
  typedef QWidget Base;

  QCloudViewer(QWidget* parent = nullptr);

  QToolBar * toolbar();
  QGLCloudViewer * cloudView() const;

  QCloudViewMtfDisplay & mtfDisplay();
  const QCloudViewMtfDisplay & mtfDisplay() const;

  void setAutoShowViewTarget(bool v);
  bool autoShowViewTarget() const;

  void setPointSize(double v);
  double pointSize() const;

  void setPointBrightness(double v);
  double pointBrightness() const;

  void setBackgroundColor(const QColor &color);
  const QColor & backgroundColor() const;

  void setSceneOrigin(const QVector3D & v);
  QVector3D sceneOrigin() const;

  void setCurrentFileName(const QString & v);
  const QString & currentFileName() const;


  bool openPlyFile(const QString & pathFileName);

  void add(const QPointCloud::sptr & cloud);
  void clear();

  void updateDisplayPoints();
  void updateDisplayColors();

  void redraw();

  void rotateToShowCloud();

  void showKeyBindings();

  bool copyViewportToClipboard();

  QPixmap grabViewportPixmap();

  const std::vector<QPointCloud::sptr> & clouds() const
  {
    return glViewer_->clouds();
  }

  const QPointCloud::sptr & cloud(int index) const
  {
    return glViewer_->cloud(index);
  }

Q_SIGNALS:
  void visibilityChanged(bool visible);
  void currentFileNameChanged();
  void displayImageChanged();

protected:
  void showEvent(QShowEvent *) override;
  void hideEvent(QHideEvent *) override;

protected:
  QVBoxLayout * layout_ = nullptr;
  mutable QToolBar * toolbar_ = nullptr;
  QGLCloudViewer * glViewer_ = nullptr;
  QString currentFileName_;
};


bool fromString(const QString & s, QVector3D * v);
QString toQString(const QVector3D & v);

bool load_parameter(const QSettings & settings, const QString & prefix, const char * name,  QVector3D * v);
void save_parameter(const QString & prefix, const char * name, const QVector3D & value );

#endif /* __QCloudViewer_h__ */
