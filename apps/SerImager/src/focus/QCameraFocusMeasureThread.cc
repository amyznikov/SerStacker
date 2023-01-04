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
#include <core/debug.h>

namespace serimager {


namespace {

template<class T1, class T2>
cv::Scalar weighted_mean_(const cv::Mat & src, const cv::Mat & weights)
{
  const cv::Size src_size =
      src.size();

  const cv::Size weights_size =
      weights.size();

  if( src_size != weights_size ) {
    CF_ERROR("ERROR: src_size=%dx%d not equal to weights_size=%dx%d",
        src_size.width, src_size.height,
        weights_size.width, weights_size.height);
    return cv::Scalar::all(0);
  }

  const int src_channels =
      src.channels();

  const int weigts_channels =
      weights.channels();

  if( src_channels != weigts_channels && weigts_channels != 1 ) {
    CF_ERROR("ERROR: invalid number of channels: src_channels=%d weigts_channels=%d",
        src_channels, weigts_channels);
    return cv::Scalar::all(0);
  }

  cv::Scalar total =
      cv::Scalar::all(0);

  const cv::Scalar w =
      cv::sum(weights);

  if ( src_channels == weigts_channels ) {

    for( int y = 0; y < src_size.height; ++y ) {

      const T1 *srcp =
          src.ptr<const T1>(y);

      const T2 * wp =
          weights.ptr<const T2>(y);

      for( int x = 0; x < src_size.width; ++x ) {

        for( int c = 0; c < src_channels; ++c ) {
          total[c] +=
              srcp[x * src_channels + c] * wp[x * src_channels + c];
        }
      }
    }

    for( int c = 0; c < src_channels; ++c ) {
      total[c] /= w[c];
    }

  }
  else {

    for( int y = 0; y < src_size.height; ++y ) {

      const T1 *srcp =
          src.ptr<const T1>(y);

      const T2 * wp =
          weights.ptr<const T2>(y);

      for( int x = 0; x < src_size.width; ++x ) {

        for( int c = 0; c < src_channels; ++c ) {
          total[c] +=
              srcp[x * src_channels + c] * wp[x];
        }
      }
    }

    for( int c = 0; c < src_channels; ++c ) {
      total[c] /= w[0];
    }
  }


  return total;
}


cv::Scalar weighted_mean(cv::InputArray _src, cv::InputArray _weights)
{
  const cv::Mat src =
      _src.getMat();

  const cv::Mat weights =
      _weights.getMat();

  switch (src.depth()) {
    case CV_8U:
      switch (weights.depth()) {
        case CV_8U:
          return weighted_mean_<uint8_t, uint8_t>(src, weights);
        case CV_8S:
          return weighted_mean_<uint8_t, int8_t>(src, weights);
        case CV_16U:
          return weighted_mean_<uint8_t, uint16_t>(src, weights);
        case CV_16S:
          return weighted_mean_<uint8_t, int16_t>(src, weights);
        case CV_32S:
          return weighted_mean_<uint8_t, uint32_t>(src, weights);
        case CV_32F:
          return weighted_mean_<uint8_t, float>(src, weights);
        case CV_64F:
          return weighted_mean_<uint8_t, double>(src, weights);
      }
      break;
    case CV_8S:
      switch (weights.depth()) {
        case CV_8U:
          return weighted_mean_<int8_t, uint8_t>(src, weights);
        case CV_8S:
          return weighted_mean_<int8_t, int8_t>(src, weights);
        case CV_16U:
          return weighted_mean_<int8_t, uint16_t>(src, weights);
        case CV_16S:
          return weighted_mean_<int8_t, int16_t>(src, weights);
        case CV_32S:
          return weighted_mean_<int8_t, int32_t>(src, weights);
        case CV_32F:
          return weighted_mean_<int8_t, float>(src, weights);
        case CV_64F:
          return weighted_mean_<int8_t, double>(src, weights);
      }
      break;
    case CV_16U:
      switch (weights.depth()) {
        case CV_8U:
          return weighted_mean_<uint16_t, uint8_t>(src, weights);
        case CV_8S:
          return weighted_mean_<uint16_t, int8_t>(src, weights);
        case CV_16U:
          return weighted_mean_<uint16_t, uint16_t>(src, weights);
        case CV_16S:
          return weighted_mean_<uint16_t, int16_t>(src, weights);
        case CV_32S:
          return weighted_mean_<uint16_t, int32_t>(src, weights);
        case CV_32F:
          return weighted_mean_<uint16_t, float>(src, weights);
        case CV_64F:
          return weighted_mean_<uint16_t, double>(src, weights);
      }
      break;
    case CV_16S:
      switch (weights.depth()) {
        case CV_8U:
          return weighted_mean_<int16_t, uint8_t>(src, weights);
        case CV_8S:
          return weighted_mean_<int16_t, int8_t>(src, weights);
        case CV_16U:
          return weighted_mean_<int16_t, uint16_t>(src, weights);
        case CV_16S:
          return weighted_mean_<int16_t, int16_t>(src, weights);
        case CV_32S:
          return weighted_mean_<int16_t, int32_t>(src, weights);
        case CV_32F:
          return weighted_mean_<int16_t, float>(src, weights);
        case CV_64F:
          return weighted_mean_<int16_t, double>(src, weights);
      }
      break;
    case CV_32S:
      switch (weights.depth()) {
        case CV_8U:
          return weighted_mean_<int32_t, uint8_t>(src, weights);
        case CV_8S:
          return weighted_mean_<int32_t, int8_t>(src, weights);
        case CV_16U:
          return weighted_mean_<int32_t, uint16_t>(src, weights);
        case CV_16S:
          return weighted_mean_<int32_t, int16_t>(src, weights);
        case CV_32S:
          return weighted_mean_<int32_t, int32_t>(src, weights);
        case CV_32F:
          return weighted_mean_<int32_t, float>(src, weights);
        case CV_64F:
          return weighted_mean_<int32_t, double>(src, weights);
      }
      break;
    case CV_32F:
      switch (weights.depth()) {
        case CV_8U:
          return weighted_mean_<float, uint8_t>(src, weights);
        case CV_8S:
          return weighted_mean_<float, int8_t>(src, weights);
        case CV_16U:
          return weighted_mean_<float, uint16_t>(src, weights);
        case CV_16S:
          return weighted_mean_<float, int16_t>(src, weights);
        case CV_32S:
          return weighted_mean_<float, int32_t>(src, weights);
        case CV_32F:
          return weighted_mean_<float, float>(src, weights);
        case CV_64F:
          return weighted_mean_<float, double>(src, weights);
      }
      break;
    case CV_64F:
      switch (weights.depth()) {
        case CV_8U:
          return weighted_mean_<double, uint8_t>(src, weights);
        case CV_8S:
          return weighted_mean_<double, int8_t>(src, weights);
        case CV_16U:
          return weighted_mean_<double, uint16_t>(src, weights);
        case CV_16S:
          return weighted_mean_<double, int16_t>(src, weights);
        case CV_32S:
          return weighted_mean_<double, int32_t>(src, weights);
        case CV_32F:
          return weighted_mean_<double, float>(src, weights);
        case CV_64F:
          return weighted_mean_<double, double>(src, weights);
      }
      break;
  }

  return cv::Scalar::all(0);
}

cv::Scalar compute_local_contrast(const cv::Mat & src, int scale = 1)
{
  const cv::Mat1b SE(3, 3, 255);
  const cv::Mat1f K(3, 3, (float) (1.0 / 9));

  cv::Mat m, gx, gy, g;

  if( scale < 1 ) {
    cv::filter2D(src, m, CV_32F, K, cv::Point(1, 1), 1e-12);
    morphological_gradient(src, g, SE);
    //ecc_differentiate(src, gx, gy, CV_32F);
  }
  else {
    ecc_downscale(src, g, 1);
    cv::filter2D(g, m, CV_32F, K, cv::Point(1, 1), 1e-12);
    morphological_gradient(g, g, SE);
    //ecc_differentiate(g, gx, gy, CV_32F);
  }

  //cv::magnitude(gx, gy, g);

  if( g.depth() != CV_32F ) {
    g.convertTo(g, CV_32F);
  }


  cv::divide(g, m, m, 1.0, CV_32F);

  return weighted_mean(m, g);
  //return cv::mean(m);
}
}


QCameraFocusMeasureThread::QCameraFocusMeasureThread(QObject * parent) :
  Base(parent)
{
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

        if( is_bayer_pattern(colorid) ) {
          if( !extract_bayer_planes(image, image, colorid) ) {
            CF_ERROR("extract_bayer_planes() fails");
          }
        }

        cv::Scalar v =
            compute_local_contrast(image);

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
}


} /* namespace serimager */
