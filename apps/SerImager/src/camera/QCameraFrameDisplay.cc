/*
 * QCameraFrameDisplay.cc
 *
 *  Created on: Dec 21, 2022
 *      Author: amyznikov
 */

#include "QCameraFrameDisplay.h"
#include <gui/qimageview/cv2qt.h>
#include <core/mtf/mtf-histogram.h>
#include <core/proc/histogram.h>
#include <core/proc/minmax.h>
#include <core/ssprintf.h>

namespace {

typedef std::lock_guard<std::mutex>
  c_guard_lock;

typedef std::unique_lock<std::mutex>
  c_unique_lock;

} // namespace


namespace serimager {

QCameraFrameMtfDisplayFunction::QCameraFrameMtfDisplayFunction(QImageViewer * imageViewer) :
    Base(imageViewer, "QCameraFrameMtfDisplayFunction")
{
}

std::mutex & QCameraFrameMtfDisplayFunction::mutex()
{
  return mutex_;
}

//void QCameraFrameMtfDisplayFunction::setCurrentImage(cv::InputArray image, cv::InputArray mask)
//{
//  // must be locked by caller
//  //c_unique_lock lock(mutex_);
//  Base::setCurrentImage(image, mask);
//}

void QCameraFrameMtfDisplayFunction::getInputDataRange(double * minval, double * maxval) const
{
  c_unique_lock lock(mutex_);
  Base::getInputDataRange(minval, maxval);
}

void QCameraFrameMtfDisplayFunction::getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax)
{
  c_unique_lock lock(mutex_);
  Base::getInputHistogramm(H, hmin, hmax);
}

void QCameraFrameMtfDisplayFunction::getOutputHistogramm(cv::OutputArray H, double * hmin, double * hmax)
{
  INSTRUMENT_REGION("");

  if ( imageViewer_ ) {

    cv::Mat image, mask;

    mutex_.lock();
    isBusy_ = true;
    imageViewer_->displayImage().copyTo(image);
    imageViewer_->currentMask().copyTo(mask);
    mutex_.unlock();

    create_histogram(image, mask,
        H,
        hmin, hmax,
        256,
        false,
        false);


    mutex_.lock();
    isBusy_ = false;
    mutex_.unlock();
  }
}


void QCameraFrameMtfDisplayFunction::createDisplayImage(cv::InputArray currentImage, cv::InputArray currentMask,
    cv::OutputArray displayImage, int ddepth)
{
  //  c_unique_lock lock(mutex_);
  Base::createDisplayImage(currentImage, currentMask, displayImage, ddepth);
}


QCameraFrameDisplay::QCameraFrameDisplay(QWidget * parent) :
    Base(parent),
    mtfDisplayFunction_(this)
{
  setDisplayFunction(&mtfDisplayFunction_);

  connect(this, &ThisClass::pixmapChanged,
      this, &ThisClass::onPixmapChanged,
      Qt::QueuedConnection);


  connect(&mtfDisplayFunction_, &QMtfDisplay::parameterChanged,
      [this]() {
        if (workerState_ == Worker_Idle) {
          Base::updateDisplay();
        }
      });

  connect(this, &ThisClass::displayImageChanged,
      &mtfDisplayFunction_, &QMtfDisplay::displayImageChanged,
      Qt::QueuedConnection);
}

QCameraFrameDisplay::~QCameraFrameDisplay()
{
}

const QCameraFrameMtfDisplayFunction * QCameraFrameDisplay::mtfDisplayFunction() const
{
  return &mtfDisplayFunction_;
}

QCameraFrameMtfDisplayFunction * QCameraFrameDisplay::mtfDisplayFunction()
{
  return &mtfDisplayFunction_;
}

void QCameraFrameDisplay::setFrameProcessor(const c_image_processor::ptr & processor)
{
  mtfDisplayFunction_.mutex().lock();
  Base::current_processor_ = processor;
  if( this->workerState_ == Worker_Idle ) {
    Base::updateDisplay();
  }
  mtfDisplayFunction_.mutex().unlock();
}


void QCameraFrameDisplay::setCamera(const QImagingCamera::sptr & camera)
{
  stopWorkerThread();

  if( camera_ ) {
    (static_cast<QObject*>(camera_.get()))->disconnect(this);
  }

  if ( (camera_  = camera) ) {

    connect(camera_.get(), &QImagingCamera::stateChanged,
        this, &ThisClass::onCameraStateChanged,
        Qt::QueuedConnection);


    if ( camera_->state() == QImagingCamera::State_started ) {
      startWorkerThread();
    }
  }
}

const QImagingCamera::sptr & QCameraFrameDisplay::camera() const
{
  return camera_;
}


void QCameraFrameDisplay::onCameraStateChanged(QImagingCamera::State oldSate,
    QImagingCamera::State newState)
{
  if ( camera_ && camera_->state() == QImagingCamera::State_started ) {
    startWorkerThread();
  }
}


void QCameraFrameDisplay::showEvent(QShowEvent *event)
{
  Base::showEvent(event);

  if ( !isVisible() ) {
    stopWorkerThread();
  }
  else if ( camera_ && camera_->state() == QImagingCamera::State_started ) {
    startWorkerThread();
  }
}

void QCameraFrameDisplay::hideEvent(QHideEvent *event)
{
  Base::hideEvent(event);

  if ( !isVisible() ) {
    stopWorkerThread();
  }
  else if ( camera_ && camera_->state() == QImagingCamera::State_started ) {
    startWorkerThread();
  }
}

void QCameraFrameDisplay::startWorkerThread()
{
  c_unique_lock lock(mtfDisplayFunction_.mutex());

  while (workerState_ == Worker_Stopping) {
    lock.unlock();
    QThread::msleep(20);
    lock.lock();
  }

  if (workerState_ != Worker_Idle ) {
    return;
  }

  workerState_ = Worker_Starting;
  std::thread(&ThisClass::workerThread, this).detach();
}

void QCameraFrameDisplay::stopWorkerThread()
{
  c_unique_lock lock(mtfDisplayFunction_.mutex());
  while (workerState_ != Worker_Idle) {
    workerState_ = Worker_Stopping;
    lock.unlock();
    QThread::msleep(20);
    lock.lock();
  }
}

void QCameraFrameDisplay::onPixmapChanged()
{
  c_unique_lock lock(mtfDisplayFunction_.mutex());
  scene()->setSceneImage(pixmap_);
}


void QCameraFrameDisplay::workerThread()
{

  c_unique_lock lock(mtfDisplayFunction_.mutex());
  workerState_ = Worker_Running;


  bool haveImage = false;
  int last_index = -1;


  cv::Mat currentImage, displayImage, colormapImage, currentMask;
  QPixmap pixmap;


  while ( workerState_ == Worker_Running && camera_ && camera_->state() == QImagingCamera::State_started ) {

    INSTRUMENT_REGION("body");

    haveImage = false;

    if( true ) {
      INSTRUMENT_REGION("wait_frame");

      QImagingCamera::shared_lock lock(camera_->mutex());

      const std::deque<QCameraFrame::sptr> &deque =
          camera_->deque();

      if( !deque.empty() ) {

        const QCameraFrame::sptr &frame =
            deque.back();

        const int index =
            frame->index();

        if( index > last_index ) {

          last_index = index;
          bpp_ = frame->bpp();
          colorid_ = frame->colorid();
          frame->image().copyTo(currentImage);
          haveImage = true;
        }
      }
    }

    lock.unlock();

    if( haveImage && !currentImage.empty() ) {

      {
        INSTRUMENT_REGION("convert");

        //CF_DEBUG("colorid_=%s", toString(colorid_));

        if( currentImage.depth() != CV_8U && bpp_ > 0 ) {
          currentImage.convertTo(currentImage, CV_8U, 255. / (1 << bpp_));
        }

        if( is_bayer_pattern(colorid_) ) {
          debayer(currentImage, currentImage, colorid_, DEBAYER_NN);
        }
        else if( colorid_ == COLORID_RGB ) {
          cv::cvtColor(currentImage, currentImage, cv::COLOR_RGB2BGR);
        }

        //editImage(image, cv::noArray(), false);
        if( current_processor_ && !current_processor_->empty() ) {
          current_processor_->process(currentImage, currentMask);
        }

        const QMtfDisplay::DisplayParams &opts =
            mtfDisplayFunction_.displayParams();

        const bool needColormap =
            opts.colormap != COLORMAP_NONE &&
                currentImage.channels() == 1;

        mtfDisplayFunction_.applyMtf(currentImage,
            needColormap ? cv::noArray() : currentMask,
                displayImage, CV_8U);

        if ( needColormap ) {
          mtfDisplayFunction_.applyColorMap(displayImage, currentMask, colormapImage);
        }
        else {
          colormapImage = displayImage;
        }



        //mtfDisplayFunction_.createDisplayImage(currentImage, currentMask, displayImage, CV_8U);

        pixmap =
            createPixmap(colormapImage, true,
                Qt::NoFormatConversion |
                    Qt::ThresholdDither |
                    Qt::ThresholdAlphaDither |
                    Qt::NoOpaqueDetection);

      }


      {
        INSTRUMENT_REGION("setCurrentImage");
        lock.lock();
        cv::swap(currentImage, currentImage_);
        cv::swap(currentMask, currentMask_);
        cv::swap(displayImage, displayImage_);
        //cv::swap(colormapImage, colormapImage_);
        pixmap_ = pixmap;
        lock.unlock();
      }


      Q_EMIT pixmapChanged();

      if ( !mtfDisplayFunction_.isBusy() ) {
        //Q_EMIT mtfDisplayFunction_.displayImageChanged();
        Q_EMIT displayImageChanged();
      }

    }

    QThread::msleep(50);

    lock.lock();
  }


  workerState_ = Worker_Idle;
}

} /* namespace qserimager */
