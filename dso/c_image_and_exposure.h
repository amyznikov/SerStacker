/**
* This file is derived from DSO source code.
*
*/

#pragma once
#ifndef __c_image_and_exposure_h__
#define __c_image_and_exposure_h__

#include <opencv2/opencv.hpp>
#include <memory>

namespace dso {

class c_image_and_exposure
{
public:
  typedef c_image_and_exposure this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_image_and_exposure() :
    timestamp_(0),
    exposure_(1)
  {
  }

  c_image_and_exposure(int rows, int cols, double timestamp = 0) :
      timestamp_(timestamp),
      exposure_(1)
  {
    if( rows > 0 && cols > 0 ) {
      image_.create(rows, cols);
    }
  }

  c_image_and_exposure(const cv::Size & size, double timestamp = 0) :
      timestamp_(timestamp),
      exposure_(1)
  {
    if( !size.empty() ) {
      image_.create(size);
    }
  }

  ~c_image_and_exposure()
  {
  }

  void create(int rows, int cols, double timestamp = 0)
  {
    if( rows <= 0 || cols <= 0 ) {
      image_.release();
    }
    else {
      image_.create(rows, cols);
    }

    timestamp_ = timestamp;
    exposure_ = 1;
  }

  void create(const cv::Size & size, double timestamp = 0)
  {
    if( size.empty() ) {
      image_.release();
    }
    else {
      image_.create(size);
    }

    timestamp_ = timestamp;
    exposure_ = 1;
  }

  void copyMetaTo(c_image_and_exposure &other)
  {
    other.exposure_ = this->exposure_;
  }

  cv::Mat1f & image()
  {
    return image_;
  }

  const cv::Mat1f & image() const
  {
    return image_;
  }

  const cv::Size size() const
  {
    return image_.size();
  }

  const int rows() const
  {
    return image_.rows;
  }

  const int cols() const
  {
    return image_.cols;
  }


  void set_timestamp(double v)
  {
    timestamp_ = v;
  }

  double timestamp() const
  {
    return timestamp_;
  }

  // exposure time in ms.
  void set_exposure(float v)
  {
    exposure_ = v;
  }

  // exposure time in ms.
  float exposure() const
  {
    return exposure_;
  }

  float * data()
  {
    return (float * )(image_.data);
  }

  const float * data() const
  {
    return (const float * )(image_.data);
  }

  void copy_from(const cv::Mat1f & rhs)
  {
    rhs.copyTo(image_);
  }

protected:
  cv::Mat1f image_; // irradiance. between 0 and 256
  double timestamp_;
  float exposure_;  // exposure time in ms.
};


}

#endif // __c_image_and_exposure_h__
