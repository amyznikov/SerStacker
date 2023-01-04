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

  timer_.setSingleShot(true);
  timer_.setInterval(50);
  connect(&timer_, &QTimer::timeout,
      this, &ThisClass::onUpdateCameraFrameDisplay);

  // scene()->setBackgroundBrush(Qt::darkGray);
}

QCameraFrameDisplay::~QCameraFrameDisplay()
{
//  if ( camera_ ) {
//    disconnect(camera_.get(), nullptr, this, nullptr);
//  }
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
  if ( camera_ ) {
    disconnect(camera_.get(), nullptr, this, nullptr);
  }

  if ( (camera_  = camera) ) {

    connect(camera_.get(), &QImagingCamera::stateChanged,
        this, &ThisClass::onCameraStateChanged,
        Qt::QueuedConnection);


    if ( camera_->state() == QImagingCamera::State_started ) {
      timer_.start();
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
    last_index = -1;
    timer_.start();
  }
}

void QCameraFrameDisplay::onUpdateCameraFrameDisplay()
{
  INSTRUMENT_REGION("");

  if( camera_ ) {

    cv::Mat display_image;
    enum COLORID colorid = COLORID_UNKNOWN;
    int bpp = 0;

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
          bpp = frame->bpp();
          colorid = frame->colorid();
          frame->image().copyTo(display_image);
        }
      }
    }

    if( !display_image.empty() ) {

      if( display_image.depth() != CV_8U && bpp > 0 ) {
        display_image.convertTo(display_image, CV_8U, 255. / (1 << bpp));
      }

      if( is_bayer_pattern(colorid) ) {
        debayer(display_image, display_image, colorid, DEBAYER_NN);
      }
      else if ( colorid == COLORID_RGB ) {
        cv::cvtColor(display_image, display_image, cv::COLOR_RGB2BGR);
      }

      //  editImage(image, cv::noArray(), false);
      setCurrentImage(display_image, cv::noArray(), cv::noArray(), false);
      updateDisplay();
    }

    if( camera_->state() == QImagingCamera::State_started ) {
      timer_.start();
    }
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

} /* namespace qserimager */
