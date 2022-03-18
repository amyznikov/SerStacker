/*
 * QCloudViewer.h
 *
 *  Created on: May 18, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCloudViewer_h__
#define __QCloudViewer_h__
#if HAVE_QGLViewer // Should come from CMakeLists.txt

#include <QtWidgets/QtWidgets>
#include <QtOpenGL/QGLWidget>
#include <QGLViewer/qglviewer.h>
#include "QPointCloud.h"


using QGLVector = qglviewer::Vec;


class QGLCloudViewer
  : public QGLViewer
{
  Q_OBJECT;
public:
  typedef QGLCloudViewer ThisClass;
  typedef QGLViewer Base;

  QGLCloudViewer(QWidget* parent = Q_NULLPTR);

  void setSceneRadius(qreal radius) override;

  void setPointSize(double v);
  double pointSize() const;

  void setPointBrightness(double v);
  double pointBrightness() const;

  void setSceneOrigin(const QGLVector & v);
  QGLVector sceneOrigin() const;

  std::vector<QPointCloud::ptr> & clouds();
  const std::vector<QPointCloud::ptr> & clouds() const;
  const QPointCloud::ptr & cloud(int index) const;
  void clear();
  bool openPlyFile(const QString & pathFileName);

protected:
  void init() override;
  void draw() override;

protected:
  std::vector<QPointCloud::ptr> clouds_;
  QGLVector sceneOrigin_;
  double pointSize_ = 2;
  double pointBrightness_ = 0;
};

class QCloudViewer
    : public QWidget
{
  Q_OBJECT;
public:
  typedef QCloudViewer ThisClass;
  typedef QWidget Base;

  QCloudViewer(QWidget* parent = Q_NULLPTR);

  QToolBar * toolbar() const;

  void setSceneRadius(qreal radius);
  qreal sceneRadius() const;

  void setSceneCenter(const QGLVector &center);
  QGLVector sceneCenter() const;

  void setPointSize(double v);
  double pointSize() const;

  void setPointBrightness(double v);
  double pointBrightness() const;

  void setSceneOrigin(const QGLVector & v);
  QGLVector sceneOrigin() const;

  bool openPlyFile(const QString & pathFileName);

  void setCurrentFileName(const QString & v);
  const QString & currentFileName() const;

  void clear();

  std::vector<QPointCloud::ptr> & clouds()
  {
    return glViewer_->clouds();
  }

  const std::vector<QPointCloud::ptr> & clouds() const
  {
    return glViewer_->clouds();
  }

  const QPointCloud::ptr & cloud(int index) const
  {
    return glViewer_->cloud(index);
  }

signals:
  void currentFileNameChanged();

protected:
  QVBoxLayout * layout_ = Q_NULLPTR;
  QToolBar * toolbar_ = Q_NULLPTR;
  QGLCloudViewer * glViewer_ = Q_NULLPTR;
  QString currentFileName_;
};


bool fromString(const QString & s, QGLVector * v);
QString toQString(const QGLVector & v);

bool load_parameter(const QSettings & settings, const QString & prefix, const char * name,  QGLVector * v);
void save_parameter(const QString & prefix, const char * name, const QGLVector & value );

#endif // HAVE_QGLViewer
#endif /* __QCloudViewer_h__ */
