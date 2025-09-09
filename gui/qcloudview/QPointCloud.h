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

class QPointCloud
{
public:
  typedef std::shared_ptr<QPointCloud> sptr;

  static sptr create()
  {
    return sptr(new QPointCloud());
  }

  static sptr create(const std::vector<cv::Vec3f> & points, const std::vector<cv::Vec3b> & colors)
  {
    sptr obj(new QPointCloud());

    obj->points.create(points.size(), 1);
    obj->colors.create(colors.size(), 1, CV_8UC3);

    cv::Mat3f & pts =
        obj->points;

    cv::Mat3b clrs =
        obj->colors;

    for ( int i = 0, n = points.size(); i < n; ++i ) {
      pts[i][0] = points[i];
      clrs[i][0] = colors[i];
    }

    return obj;
  }

  static sptr create(const cv::Mat3f & points, const cv::Mat & colors, bool make_copy_of_data = true)
  {
    sptr obj(new QPointCloud());

    if( !make_copy_of_data ) {
      obj->points = points;
      obj->colors = colors;
    }
    else {
      points.copyTo(obj->points);
      colors.copyTo(obj->colors);
    }

    return obj;
  }

  void clear()
  {
    points.release();
    colors.release();
  }

public:
  QString filename;

  cv::Mat3f points;
  cv::Mat colors;
  cv::Mat1b display_mask;

  QVector3D Rotation; // angles [Rx;Ry;Rz]
  QVector3D Translation;
  QVector3D Scale = QVector3D(1.0,1.0,1.0);
  bool visible = true;
};


bool loadPlyFile(const QString & filename,
    QPointCloud * cloud);

#endif /* __QPointCloud_h__ */
