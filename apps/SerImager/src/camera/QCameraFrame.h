/*
 * QCameraFrame.h
 *
 *  Created on: Dec 23, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCameraFrameh__
#define __QCameraFrameh__

#include <opencv2/opencv.hpp>
#include <core/io/debayer.h>
#include <memory>
#include <core/debug.h>

namespace serimager {

class QImagingCamera;

class QCameraFrame
{
public:
  typedef QCameraFrame ThisClass;
  typedef std::shared_ptr<ThisClass> sptr;

  virtual ~QCameraFrame() = default;

  static sptr create(const cv::Size & imageSize, int cvType, enum COLORID colorid, int bpp,
      void * data = nullptr, size_t step = cv::Mat::AUTO_STEP)
  {
    return sptr(new ThisClass(imageSize, cvType, colorid, bpp, data, step));
  }

  const cv::Mat & image() const
  {
    return image_;
  }

  cv::Mat & image()
  {
    return image_;
  }

  double ts() const
  {
    return ts_;
  }

  void set_ts(double ts)
  {
    ts_ = ts;
  }

  int index() const
  {
    return index_;
  }

  void set_index(int index)
  {
    index_ = index;
  }

  enum COLORID colorid() const
  {
    return colorid_;
  }

  int bpp() const
  {
    return bpp_;
  }

  void * data()
  {
    return data_;
  }

  int size() const
  {
    return size_;
  }

protected:
  QCameraFrame(const cv::Size & imageSize, int cvType, enum COLORID colorid, int bpp,
      void * data = nullptr, size_t step = cv::Mat::AUTO_STEP) :
      colorid_(colorid),
      bpp_(bpp)
  {
    if( !data ) {
      image_.create(imageSize, cvType);
    }
    else {
      image_ = cv::Mat(imageSize, cvType, data, step);
    }

    data_ = image_.data;
    size_ = image_.size().area() * image_.elemSize();
  }

protected:
  cv::Mat image_;
  void * data_ = nullptr;
  int size_ = 0;
  double ts_ = 0;
  int index_ = 0;
  enum COLORID colorid_ = COLORID_UNKNOWN;
  int bpp_ = 0;
};

} /* namespace qserimager */

#endif /* __QCameraFrameh__ */
