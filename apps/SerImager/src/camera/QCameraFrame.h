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
#include <atomic>

namespace serimager {

class QImagingCamera;

class QCameraFrame
{
public:
  typedef QCameraFrame ThisClass;
  typedef std::shared_ptr<ThisClass> sptr;

  static constexpr float QUALITY_UNKNOWN = -1.0f;
  static constexpr float QUALITY_ESTIMATING = -2.0f;

  virtual ~QCameraFrame() = default;

  static sptr create(const cv::Size & imageSize, int cvType, enum COLORID colorid, int bpp,
      void * data = nullptr, size_t step = cv::Mat::AUTO_STEP)
  {
    return sptr(new ThisClass(imageSize, cvType, colorid, bpp, data, step));
  }

  static sptr create(const cv::Mat & image, enum COLORID colorid = COLORID_UNKNOWN, int bpp = 0)
  {
    return sptr(new ThisClass(image, colorid, bpp));
  }

  const cv::Mat & image() const
  {
    return _image;
  }

  cv::Mat & image()
  {
    return _image;
  }

  // in real-time seconds
  double ts() const
  {
    return _ts;
  }

  // in real-time seconds
  void set_ts(double ts)
  {
    _ts = ts;
  }

  int index() const
  {
    return _index;
  }

  void set_index(int index)
  {
    _index = index;
  }

  enum COLORID colorid() const
  {
    return _colorid;
  }

  int bpp() const
  {
    return _bpp;
  }

  void * data()
  {
    return _data;
  }

  int size() const
  {
    return _size;
  }


  float quality() const
  {
    return _quality.load(std::memory_order_relaxed);
  }

  void set_quality(float q)
  {
    _quality.store(q, std::memory_order_release);
  }

  bool try_start_quality_etimate()
  {
    float expected = QUALITY_UNKNOWN;
    return _quality.compare_exchange_strong(expected, QUALITY_ESTIMATING,
        std::memory_order_acquire, std::memory_order_relaxed);
  }

protected:
  QCameraFrame(const cv::Size & imageSize, int cvType, enum COLORID colorid, int bpp,
      void * data = nullptr, size_t step = cv::Mat::AUTO_STEP) :
      _colorid(colorid), _bpp(bpp)
  {
    if( !data ) {
      _image.create(imageSize, cvType);
    }
    else {
      _image = cv::Mat(imageSize, cvType, data, step);
    }

    _data = _image.data;
    _size = _image.size().area() * _image.elemSize();
  }

  QCameraFrame(const cv::Mat & image, enum COLORID colorid, int bpp) :
    _image(image), _bpp(bpp), _colorid(colorid)
  {
    _data = _image.data;
    _size = _image.size().area() * _image.elemSize();
  }

protected:
  cv::Mat _image;
  void * _data = nullptr;
  int _size = 0;
  double _ts = 0; // in (real-time?) seconds
  int _index = 0;
  enum COLORID _colorid = COLORID_UNKNOWN;
  int _bpp = 0;

  //   -1.0f = Not Calculated (UNKNOWN),
  //   -2.0f = Calculating (ESTIMATING)
  // >= 0.0f = Valid quality
  std::atomic<float> _quality {QUALITY_UNKNOWN};
};

} /* namespace serimager */

#endif /* __QCameraFrameh__ */
