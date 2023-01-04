/*
 * QCameraFrameDisplay.cc
 *
 *  Created on: Dec 21, 2022
 *      Author: amyznikov
 */

#include "QCameraFrameDisplay.h"
#include <gui/qimageview/cv2qt.h>

namespace {
  enum DISPLAY_TYPE {
    DISPLAY_PIXEL_VALUE,
  };
}

template<>
const c_enum_member* members_of<DISPLAY_TYPE>()
{
  static constexpr c_enum_member members[] = {
      { DISPLAY_PIXEL_VALUE, "VALUE" },
      { DISPLAY_PIXEL_VALUE }
  };

  return members;
}

namespace serimager {

QCameraFrameDisplaySettings::QCameraFrameDisplaySettings(QObject * parent) :
    Base(parent)
{
  Base::displayType_ = DISPLAY_PIXEL_VALUE;
  addDisplay(displayParams_, DISPLAY_PIXEL_VALUE, -1, -1);
}
const c_enum_member* QCameraFrameDisplaySettings::displayTypes() const
{
  return members_of<DISPLAY_TYPE>();
}

void QCameraFrameDisplaySettings::loadParameters()
{
  Base::loadParameters("QFrameDisplaySettings");
}

void QCameraFrameDisplaySettings::saveParameters() const
{
  Base::saveParameters("QFrameDisplaySettings");
}

QCameraFrameDisplay::QCameraFrameDisplay(QWidget * parent) :
    Base(parent),
    displaySettings_(this),
    displayFunction_(&displaySettings_, this)
{
  setDisplayFunction(&displayFunction_);

  connect(this, &ThisClass::pixmapChanged,
      this, &ThisClass::onPixmapChanged,
      Qt::QueuedConnection);

}

QCameraFrameDisplay::~QCameraFrameDisplay()
{
}

const QCameraFrameDisplaySettings * QCameraFrameDisplay::displaySettings() const
{
  return &displaySettings_;
}

QCameraFrameDisplaySettings * QCameraFrameDisplay::displaySettings()
{
  return &displaySettings_;
}


void QCameraFrameDisplay::setCamera(const QImagingCamera::sptr & camera)
{
  stopWorkerThread();

  if ( camera_ ) {
    disconnect(camera_.get());
  }

  if ( (camera_  = camera) ) {

    connect(camera_.get(), &QImagingCamera::stateChanged,
        this, &ThisClass::onCameraStateChanged,
        Qt::QueuedConnection);


    if ( camera_->state() == QImagingCamera::State_started ) {
      // timer_.start();
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

void QCameraFrameDisplay::showCurrentDisplayImage()
{
  INSTRUMENT_REGION("");
  if( displayImage_.empty() ) {
    scene()->setBackground(QPixmap());
  }
  else {

    QPixmap pixmap;

    if( displayImage_.type() == CV_8UC3 ) {

      QImage qimage ( displayImage_.data,
          displayImage_.cols, displayImage_.rows,
          (int) (size_t) (displayImage_.step),
          QImage::Format_BGR888 );

      pixmap =
          QPixmap::fromImage(qimage,
              Qt::NoFormatConversion |
                  Qt::ThresholdDither |
                  Qt::ThresholdAlphaDither |
                  Qt::NoOpaqueDetection);

    }
    else if( displayImage_.type() == CV_8UC1 ) {

      QImage qimage ( displayImage_.data,
          displayImage_.cols, displayImage_.rows,
          (int) (size_t) (displayImage_.step),
          QImage::Format_Grayscale8 );

      pixmap =
          QPixmap::fromImage(qimage,
              Qt::NoFormatConversion |
                  Qt::ThresholdDither |
                  Qt::ThresholdAlphaDither |
                  Qt::NoOpaqueDetection);

    }
    else if( displayImage_.type() == CV_16UC1 ) {

      QImage qimage ( displayImage_.data,
          displayImage_.cols, displayImage_.rows,
          (int) (size_t) (displayImage_.step),
          QImage::Format_Grayscale16 );

      pixmap =
          QPixmap::fromImage(qimage,
              Qt::NoFormatConversion |
                  Qt::ThresholdDither |
                  Qt::ThresholdAlphaDither |
                  Qt::NoOpaqueDetection);

    }

    else {

      cv2qt(displayImage_, &qimage_);

      pixmap =
          QPixmap::fromImage(qimage_,
              Qt::NoFormatConversion |
                  Qt::ThresholdDither |
                  Qt::ThresholdAlphaDither |
                  Qt::NoOpaqueDetection);
    }

    scene()->setBackground(pixmap);
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
  c_unique_lock lock(mutex_);

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
  c_unique_lock lock(mutex_);
  while (workerState_ != Worker_Idle) {
    workerState_ = Worker_Stopping;
    lock.unlock();
    QThread::msleep(20);
    lock.lock();
  }
}

void QCameraFrameDisplay::onPixmapChanged()
{
  c_unique_lock lock(mutex_);
  scene()->setBackground(pixmap_);
}


void QCameraFrameDisplay::workerThread()
{
  CF_DEBUG("ENTER");

  c_unique_lock lock(mutex_);
  workerState_ = Worker_Running;


  bool haveImage = false;
  int last_index = -1;


  while ( workerState_ == Worker_Running && camera_ && camera_->state() == QImagingCamera::State_started ) {

    haveImage = false;

    if( true ) {

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
          frame->image().copyTo(inputImage_);
          haveImage = true;
        }
      }
    }

    lock.unlock();

    if( haveImage && !inputImage_.empty() ) {

      if( inputImage_.depth() != CV_8U && bpp_ > 0 ) {
        inputImage_.convertTo(inputImage_, CV_8U, 255. / (1 << bpp_));
      }

      if( is_bayer_pattern(colorid_) ) {
        debayer(inputImage_, inputImage_, colorid_, DEBAYER_NN);
      }
      else if( colorid_ == COLORID_RGB ) {
        cv::cvtColor(inputImage_, inputImage_, cv::COLOR_RGB2BGR);
      }


      //editImage(image, cv::noArray(), false);
      if( current_processor_ && !current_processor_->empty() ) {
        current_processor_->process(inputImage_, inputMask_);
      }

      currentImage_ = inputImage_;
      currentMask_ = inputMask_;
      displayFunction_.setCurrentImage(currentImage_, currentMask_);
      //displayImage_.release();
      displayFunction_.getDisplayImage(displayImage_, CV_8U);

      QPixmap pixmap =
          createPixmap(displayImage_, true,
              Qt::NoFormatConversion |
                  Qt::ThresholdDither |
                  Qt::ThresholdAlphaDither |
                  Qt::NoOpaqueDetection);

      lock.lock();
      pixmap_ = pixmap;
      lock.unlock();
      Q_EMIT pixmapChanged();
    }

    QThread::msleep(50);

    lock.lock();
  }


  workerState_ = Worker_Idle;

  CF_DEBUG("LEAVE");
}

} /* namespace qserimager */
