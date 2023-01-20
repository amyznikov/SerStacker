/*
 * QCameraFocusMeasure.cc
 *
 *  Created on: Jan 2, 2023
 *      Author: amyznikov
 */

#include "QCameraFocusMeasure.h"
#include <gui/widgets/qsprintf.h>
#include <core/proc/eccalign.h>
#include <core/proc/morphology.h>
#include <core/proc/smap.h>
#include <core/proc/reduce_channels.h>
#include <core/debug.h>


namespace serimager {

QCameraFocusMeasure::CameraMonitorThread::CameraMonitorThread(QCameraFocusMeasure * parent) :
    Base(parent),
    p_(parent)
{
}

void QCameraFocusMeasure::CameraMonitorThread::run()
{
  if( true ) {

    QMutexLocker lock(&p_->mutex_);

    for( int i = 0; i < MAX_CHANNELS; ++i ) {
      p_->measurements_[i].clear();
    }
  }

  Q_EMIT p_->dataChanged();

  QMutexLocker lock(&p_->mutex_);

  cv::Mat image;
  enum COLORID colorid = COLORID_UNKNOWN;
  int bpp = 0;
  int last_frame_index_ = -1;
  int data_size = 0;

  while (p_->enabled() && p_->camera_ && p_->camera_->state() == QImagingCamera::State_started) {

    bool haveUpdate =
        false;

    if( true ) {

      QImagingCamera::shared_lock lock(p_->camera_->mutex());

      const std::deque<QCameraFrame::sptr> &deque =
          p_->camera_->deque();

      if( !deque.empty() ) {

        const QCameraFrame::sptr &frame =
            deque.back();

        const int index =
            frame->index();

        if( index > last_frame_index_ ) {

          last_frame_index_ = index;
          bpp = frame->bpp();
          colorid = frame->colorid();

          haveUpdate =
              p_->getImageROI(&image,
                  frame->image(),
                  p_->camera_->roi(),
                  true);
        }
      }
    }

    lock.unlock();

    if( haveUpdate ) {
      if( image.empty() ) {
        haveUpdate = false;
      }
      else {

        if( is_bayer_pattern(colorid) ) {
          if( !extract_bayer_planes(image, image, colorid) ) {
            CF_ERROR("extract_bayer_planes() fails");
          }
        }

        cv::Scalar v =
            p_->measure_.compute(image);

        const int cn = p_->measure_.avgchannel() ? 1 :
            image.channels();

        lock.relock();

        if( cn != p_->cn_ ) {

          p_->cn_ = cn;

          data_size = 0;

          for( int i = 0; i < MAX_CHANNELS; ++i ) {
            p_->measurements_[i].clear();
          }

        }
        else {

          while (data_size >= p_->max_measurements_) {

            for( int i = 0; i < MAX_CHANNELS; ++i ) {
              if( !p_->measurements_[i].isEmpty() ) {
                p_->measurements_[i].pop_front();
              }
            }

            --data_size;
          }
        }

        for( int i = 0; i < cn; ++i ) {
          p_->measurements_[i].push_back(v[i]);
        }

        ++data_size;
        p_->colorid_ = colorid;
        p_->bpp_ = bpp;

        haveUpdate = true;

        lock.unlock();
      }
    }

    if( haveUpdate ) {
      Q_EMIT p_->dataChanged();
    }

    Base::msleep(50);

    lock.relock();
  }
}


QCameraFocusMeasure::QCameraFocusMeasure(QObject * parent) :
  Base(parent),
  thread_(this)
{
  sprefix_ = "CameraFocusMeasure";
  load_parameters();
}

QCameraFocusMeasure::~QCameraFocusMeasure()
{
  while (thread_.isRunning()) {
    Base::enabled_ = false;
    QThread::msleep(20);
  }
}

const QImagingCamera::sptr QCameraFocusMeasure::camera() const
{
  return camera_;
}

void QCameraFocusMeasure::setCamera(const QImagingCamera::sptr & camera)
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


void QCameraFocusMeasure::setEnabled(bool enabled)
{
  if( (Base::enabled_ = enabled) && camera_ && camera_->state() == QImagingCamera::State_started ) {
    if( !thread_.isRunning() ) {
      thread_.start();
    }
  }
}

void QCameraFocusMeasure::onCameraStateChanged()
{
  if( Base::enabled_ && camera_ && camera_->state() == QImagingCamera::State_started ) {
    if( !thread_.isRunning() ) {
      thread_.start();
    }
  }
}



} /* namespace serimager */
