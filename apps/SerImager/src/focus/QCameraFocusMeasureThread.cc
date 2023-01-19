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
#include <core/debug.h>

namespace serimager {

namespace {

QString qsprintf(const char * format, ...)
  Q_ATTRIBUTE_FORMAT_PRINTF(1, 0);

QString qsprintf(const char * format, ...)
{
  va_list arglist;
  va_start(arglist, format);

#if QT_VERSION < QT_VERSION_CHECK(5, 14, 0)
  QString msg;
  msg.vsprintf(format, arglist);
#else
  QString msg = QString::vasprintf(format, arglist);
#endif

  va_end(arglist);

  return msg;
}

} // namespace


QCameraFocusMeasureThread::QCameraFocusMeasureThread(QObject * parent) :
  Base(parent)
{
  load_parameters();
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

c_local_contrast_measure & QCameraFocusMeasureThread::measure()
{
  return measure_;
}

const c_local_contrast_measure & QCameraFocusMeasureThread::measure() const
{
  return measure_;
}

int QCameraFocusMeasureThread::maxMeasurements() const
{
  return max_measurements_;
}

const QVector<double> & QCameraFocusMeasureThread::measurements(int channel) const
{
  return measurements_[channel];
}

enum COLORID QCameraFocusMeasureThread::colorid() const
{
  return measure_.avgchannel() ? COLORID_MONO : colorid_;
}

int QCameraFocusMeasureThread::bpp() const
{
  return bpp_;
}

QMutex & QCameraFocusMeasureThread::mutex()
{
  return mutex_;
}

void QCameraFocusMeasureThread::load_parameters()
{
  static constexpr const char * prefix =
      "FocusMeasure";

  QSettings settings;

  measure_.set_avgchannel(settings.value(qsprintf("%s/avgchannel", prefix), measure_.avgchannel()).value<bool>());
  measure_.set_dscale(settings.value(qsprintf("%s/dscale", prefix), measure_.dscale()).value<int>());
  measure_.set_eps(settings.value(qsprintf("%s/eps", prefix), measure_.eps()).value<double>());
}

void QCameraFocusMeasureThread::save_parameters()
{
  static constexpr const char * prefix =
      "FocusMeasure";

  QSettings settings;

  settings.setValue(qsprintf("%s/avgchannel", prefix), measure_.avgchannel());
  settings.setValue(qsprintf("%s/dscale", prefix), measure_.dscale());
  settings.setValue(qsprintf("%s/eps", prefix), measure_.eps());

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
  if( true ) {

    QMutexLocker lock(&mutex_);

    for( int i = 0; i < MAX_CHANNELS; ++i ) {
      measurements_[i].clear();
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

          const cv::Mat & frame_image =
              frame->image();

          const QRect rect =
              camera_->roi();

          cv::Rect roi(rect.x() & ~0x1, rect.y() & ~0x1,
              rect.width() & ~0x1, rect.height() & ~0x1);

          if ( roi.x + roi.width <= 0 ) {
            roi.width = 0;
          }
          else {
            if ( roi.x < 0 ) {
              roi.x = 0;
            }
            if ( roi.x + roi.width >= frame_image.cols ) {
              roi.width = (frame_image.cols - roi.x) & ~0x1;
            }
          }

          if ( roi.y + roi.height <= 0 ) {
            roi.height = 0;
          }
          else {
            if ( roi.y < 0 ) {
              roi.y = 0;
            }
            if ( roi.y + roi.height >= frame_image.rows ) {
              roi.height = (frame_image.rows - roi.y) & ~0x1;
            }
          }

          if ( roi.width > 0 && roi.height > 0 ) {
            frame_image(roi).copyTo(image);
            haveUpdate = true;
          }
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
            measure_.compute(image);

        lock.relock();

        while (data_size >= max_measurements_) {

          for( int i = 0; i < MAX_CHANNELS; ++i ) {
            if( !measurements_[i].isEmpty() ) {
              measurements_[i].pop_front();
            }
          }

          --data_size;
        }

        const int cn = measure_.avgchannel() ? 1 : image.channels();
        for( int i = 0; i < cn; ++i ) {
          measurements_[i].push_back(v[i]);
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
}


} /* namespace serimager */
