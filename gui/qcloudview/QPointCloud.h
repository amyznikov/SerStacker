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
#include <opencv2/opencv.hpp>
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

  static ptr create(const std::vector<cv::Vec3f> & points, const std::vector<cv::Vec3b> & colors)
  {
    ptr obj(new QPointCloud());

    obj->points.reserve(points.size());
    obj->colors.reserve(colors.size());

    for ( int i = 0, n = points.size(); i < n; ++i ) {
      obj->points.emplace_back(points[i][0], points[i][1], points[i][2]);
      obj->colors.emplace_back(colors[i][0], colors[i][1], colors[i][2]);
    }

    return obj;
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
