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
#include "QPointCloud.h"

class QGLCloudViewer :
    public QGLView
{
  Q_OBJECT;
public:
  typedef QGLCloudViewer ThisClass;
  typedef QGLView Base;

  QGLCloudViewer(QWidget* parent = nullptr);

  void setPointSize(double v);
  double pointSize() const;

  void setPointBrightness(double v);
  double pointBrightness() const;

  void setSceneOrigin(const QVector3D & v);
  QVector3D sceneOrigin() const;

  std::vector<QPointCloud::ptr> & clouds();
  const std::vector<QPointCloud::ptr> & clouds() const;
  const QPointCloud::ptr & cloud(int index) const;
  void clear();
  bool openPlyFile(const QString & pathFileName);

protected:
  void glInit() override;
  void glPreDraw() override;
  void glDraw() override;
  void glPostDraw() override;

protected:
  std::vector<QPointCloud::ptr> clouds_;
  QVector3D sceneOrigin_;
  double pointSize_ = 2;
  double pointBrightness_ = 0;
};

class QCloudViewer :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QCloudViewer ThisClass;
  typedef QWidget Base;

  QCloudViewer(QWidget* parent = nullptr);

  QToolBar * toolbar() const;
  QGLCloudViewer * cloudView() const;


//  void setSceneRadius(qreal radius);
//  qreal sceneRadius() const;

//  void setSceneCenter(const QVector3D &center);
//  QVector3D sceneCenter() const;
//
//  void setPointSize(double v);
//  double pointSize() const;
//
//  void setPointBrightness(double v);
//  double pointBrightness() const;
//
//  void setSceneOrigin(const QVector3D & v);
//  QVector3D sceneOrigin() const;

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

Q_SIGNALS:
  void currentFileNameChanged();

protected:
  QVBoxLayout * layout_ = nullptr;
  QToolBar * toolbar_ = nullptr;
  QGLCloudViewer * glViewer_ = nullptr;
  QString currentFileName_;
};


bool fromString(const QString & s, QVector3D * v);
QString toQString(const QVector3D & v);

bool load_parameter(const QSettings & settings, const QString & prefix, const char * name,  QVector3D * v);
void save_parameter(const QString & prefix, const char * name, const QVector3D & value );

#endif /* __QCloudViewer_h__ */
