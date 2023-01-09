/*
 * QCameraFocusMeasureThread.cc
 *
 *  Created on: Jan 2, 2023
 *      Author: amyznikov
 */

#include "QCameraFocusMeasureThread.h"
#include <core/proc/eccalign.h>
#include <core/proc/morphology.h>
#include <core/proc/smap.h>
#include <core/proc/reduce_channels.h>
#include <core/proc/focus.h>
#include <core/debug.h>

namespace serimager {

//static cv::Scalar compute_dogs_metric(cv::InputArray image)
//{
//  cv::Mat dogs;
//  compute_dogsmap(image, dogs, 3, 3, 3, 0);
//  return cv::mean(dogs);
//}


QCameraFocusMeasureThread::QCameraFocusMeasureThread(QObject * parent) :
  Base(parent)
{
}

QCameraFocusMeasureThread::~QCameraFocusMeasureThread()
{
  while (isRunning()) {
    isEnabled_ = false;
    QThread::msleep(20);
  }
}

const QImagingCamera::sptr QCameraFocusMeasureThread::camera() const
{
  return camera_;
}

void QCameraFocusMeasureThread::setCamera(const QImagingCamera::sptr & camera)
{
  QMutexLocker lock(&mutex_);

  if( camera_ ) {
    disconnect(camera_.get(), nullptr,
        this, nullptr);
  }

  if( (camera_ = camera) ) {
    connect(camera_.get(), &QImagingCamera::stateChanged,
        this, &ThisClass::onCameraStateChanged,
        Qt::QueuedConnection);
  }

  onCameraStateChanged();
}


bool QCameraFocusMeasureThread::enabled() const
{
  return isEnabled_;
}

void QCameraFocusMeasureThread::setEnabled(bool enabled)
{
  if( (isEnabled_ = enabled) && camera_ && camera_->state() == QImagingCamera::State_started ) {
    if( !isRunning() ) {
      Base::start();
    }
  }
}

void QCameraFocusMeasureThread::onCameraStateChanged()
{
  if( isEnabled_ && camera_ && camera_->state() == QImagingCamera::State_started ) {
    if( !isRunning() ) {
      Base::start();
    }
  }
}

void QCameraFocusMeasureThread::run()
{
  CF_DEBUG("ENTER");

  if( true ) {

    QMutexLocker lock(&mutex_);

    for( int i = 0; i < MAX_CHANNELS; ++i ) {
      data_[i].clear();
    }
  }

  Q_EMIT dataChanged();

  QMutexLocker lock(&mutex_);

  cv::Mat image;
  enum COLORID colorid = COLORID_UNKNOWN;
  int bpp = 0;
  int last_frame_index_ = -1;
  int data_size = 0;

  while (isEnabled_ && camera_ && camera_->state() == QImagingCamera::State_started) {

    bool haveUpdate =
        false;

    if( true ) {

      QImagingCamera::shared_lock lock(camera_->mutex());

      const std::deque<QCameraFrame::sptr> &deque =
          camera_->deque();

      if( !deque.empty() ) {

        const QCameraFrame::sptr &frame =
            deque.back();

        const int index =
            frame->index();

        if( index > last_frame_index_ ) {
          last_frame_index_ = index;
          bpp = frame->bpp();
          colorid = frame->colorid();
          frame->image().copyTo(image);
          haveUpdate = true;
        }
      }
    }

    lock.unlock();

    if( haveUpdate ) {
      if( image.empty() ) {
        haveUpdate = false;
      }
      else {

        if( false ) {
          double min, max;
          cv::minMaxLoc(image, &min, &max);
          CF_DEBUG("image: min=%g max=%g bpp=%d", min, max, bpp);
        }


        if( is_bayer_pattern(colorid) ) {
          if( !extract_bayer_planes(image, image, colorid) ) {
            CF_ERROR("extract_bayer_planes() fails");
          }
        }

        cv::Scalar v =
            c_local_contrast_measure::compute_contrast_map(image,
                cv::noArray(),
                eps_,
                cv::noArray());

//        cv::Scalar v =
//            compute_dogs_metric(image);



        lock.relock();

        while (data_size >= maxDataSize_) {

          for( int i = 0; i < MAX_CHANNELS; ++i ) {
            if( !data_[i].isEmpty() ) {
              data_[i].pop_front();
            }
          }

          --data_size;
        }

        for( int i = 0; i < image.channels(); ++i ) {
          data_[i].push_back(v[i]);
        }

        ++data_size;
        colorid_ = colorid;
        bpp_ = bpp;

        haveUpdate = true;

        lock.unlock();
      }
    }

    if( haveUpdate ) {
      Q_EMIT dataChanged();
    }

    Base::msleep(100);

    lock.relock();
  }

  CF_DEBUG("LEAVE");
}


} /* namespace serimager */
