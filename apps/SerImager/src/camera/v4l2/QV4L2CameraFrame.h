/*
 * QV4L2CameraFrame.h
 *
 *  Created on: Dec 27, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QV4L2CameraFrame_h__
#define __QV4L2CameraFrame_h__

#include "QCameraFrame.h"
#include <libv4l2.h>
#include "v4l-utils/utils/common/cv4l-helpers.h"

namespace serimager {

class QV4L2CameraFrame:
    public QCameraFrame
{
public:
  typedef QV4L2CameraFrame ThisClass;
  typedef QCameraFrame Base;
  typedef std::shared_ptr<ThisClass> sptr;

  static sptr create(cv4l_queue & queue, uint index,  const cv::Size & imageSize,
      int cvType, enum COLORID colorid, int bpp,
      void * data = nullptr,  size_t step = cv::Mat::AUTO_STEP)
  {
    return sptr(new ThisClass(queue, index, imageSize,
        cvType, colorid, bpp, data, step));
  }

  cv4l_buffer & buf()
  {
    return buf_;
  }

  const cv4l_buffer & buf() const
  {
    return buf_;
  }

protected:
  QV4L2CameraFrame(cv4l_queue & queue, uint index, const cv::Size & imageSize,
      int cvType, enum COLORID colorid, int bpp,
      void * data = nullptr, size_t step = cv::Mat::AUTO_STEP) :
      Base(imageSize, cvType, colorid, bpp, data, step)
  {
    queue.buffer_init(buf_, index);
  }

  cv4l_buffer buf_;
};

} /* namespace qsercap */

#endif /* __QV4L2CameraFrame_h__ */
