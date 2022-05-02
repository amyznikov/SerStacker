/*
 * QImageDisplayFunction.h
 *
 *  Created on: Apr 22, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QImageDisplayFunction_h__
#define __QImageDisplayFunction_h__

#include <QtCore/QtCore>
#include <opencv2/opencv.hpp>

class QImageDisplayFunction:
    public QObject
{
  Q_OBJECT;
public:
  typedef QImageDisplayFunction ThisClass;
  typedef QObject Base;

  QImageDisplayFunction(QObject * parent = Q_NULLPTR) :
      Base(parent)
  {
  }

  virtual void setCurrentImage(cv::InputArray image, cv::InputArray mask) = 0;
  virtual void getDisplayImage(cv::OutputArray image, int ddepth = -1) = 0;

signals:
  void update();
};

#endif /* __QImageDisplayFunction_h__ */
