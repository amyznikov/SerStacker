/*
 * QPointCloud.h
 *
 *  Created on: May 18, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QPointCloud_h__
#define __QPointCloud_h__

#include <QtGui/QtGui>
#include <memory>

struct QPoint3D
{
  double x, y, z;

  QPoint3D() :
    x(0), y(0), z(0)
  {
  }

  QPoint3D(double _x, double _y, double _z) :
    x(_x), y(_y), z(_z)
  {
  }

};

class QPointCloud
{
public:

  typedef std::shared_ptr<QPointCloud> ptr;

  static ptr create()
  {
    return ptr(new QPointCloud());
  }


  void clear()
  {
    points.clear();
    colors.clear();
  }

public:
  QString filename;
  std::vector<QPoint3D> points;
  std::vector<QColor> colors;
  std::vector<QColor> display_colors;
  QPoint3D Rotation; // angles [Rx;Ry;Rz]
  QPoint3D Translation;
  QPoint3D Scale = QPoint3D (1.0,1.0,1.0);
  bool visible = true;
};


bool loadPlyFile(const QString & filename,
    QPointCloud * cloud);

#endif /* __QPointCloud_h__ */
