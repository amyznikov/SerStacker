/*
 * QVideoFrameDisplay.cc
 *
 *  Created on: Mar 19, 2023
 *      Author: amyznikov
 */

#include "QVideoFrameDisplay.h"
#include <gui/qimageview/cv2qt.h>
#include <core/mtf/mtf-histogram.h>
#include <core/proc/histogram.h>
#include <core/proc/minmax.h>
#include <core/proc/pixtype.h>
#include <core/ssprintf.h>

namespace serimager {

namespace {

typedef std::lock_guard<std::mutex>
  c_guard_lock;

typedef std::unique_lock<std::mutex>
  c_unique_lock;

inline int get_bpp(int ddepth)
{
  switch (ddepth) {

    case CV_8U:
    case CV_8S:
      return 8;

    case CV_16U:
    case CV_16S:
      return 16;

    case CV_32S:
      return 32;
  }

  return 0;
}

} // namespace

///////////////////////////////////////////////////////////////////////////////////////////////////////////

QVideoFrameMtfDisplayFunction::QVideoFrameMtfDisplayFunction(QImageViewer * imageViewer) :
    Base(imageViewer, "QVideoFrameMtfDisplayFunction")
{
}

void QVideoFrameMtfDisplayFunction::getInputDataRange(double * minval, double * maxval) const
{
  c_unique_lock lock(mutex_);
  Base::getInputDataRange(minval, maxval);
}

void QVideoFrameMtfDisplayFunction::getInputHistogramm(cv::OutputArray H, double * output_hmin, double * output_hmax)
{
  INSTRUMENT_REGION("");

  if ( imageViewer_ ) {

    cv::Mat image, mask;

    double scale = 1.0;
    double offset = 0.0;

    mutex_.lock();
    isBusy_ = true;

    const cv::Mat & currentImage =
        imageViewer_->currentImage();

    if ( currentImage.depth() == CV_8U ) {
      currentImage.copyTo(image);
    }
    else {
      get_scale_offset(currentImage.depth(), CV_8U, &scale, &offset);
      currentImage.convertTo(image, scale, offset);
    }

    imageViewer_->currentMask().copyTo(mask);
    mutex_.unlock();

    create_histogram(image, mask,
        H,
        output_hmin, output_hmax,
        256,
        false,
        false);

    mutex_.lock();
    isBusy_ = false;
    mutex_.unlock();

    (*output_hmin -= offset) /= scale;
    (*output_hmax -= offset) /= scale;
  }
}

void QVideoFrameMtfDisplayFunction::getOutputHistogramm(cv::OutputArray H, double * output_hmin, double * output_hmax)
{
  INSTRUMENT_REGION("");

  if ( imageViewer_ ) {

    cv::Mat image, mask;

    double scale = 1.0;
    double offset = 0.0;

    mutex_.lock();
    isBusy_ = true;

    const cv::Mat & currentImage =
        imageViewer_->displayImage();

    if ( currentImage.depth() == CV_8U ) {
      currentImage.copyTo(image);
    }
    else {
      get_scale_offset(currentImage.depth(), CV_8U, &scale, &offset);
      currentImage.convertTo(image, scale, offset);
    }

    imageViewer_->currentMask().copyTo(mask);
    mutex_.unlock();

    create_histogram(image, mask,
        H,
        output_hmin, output_hmax,
        256,
        false,
        false);

    (*output_hmin -= offset) /= scale;
    (*output_hmax -= offset) /= scale;

    mutex_.lock();
    isBusy_ = false;
    mutex_.unlock();

  }
}


void QVideoFrameMtfDisplayFunction::createDisplayImage(cv::InputArray currentImage, cv::InputArray currentMask,
    cv::OutputArray displayImage, int ddepth)
{
  //  c_unique_lock lock(mutex_);
  Base::createDisplayImage(currentImage, currentMask, displayImage, ddepth);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////

QVideoFrameDisplay::QVideoFrameDisplay(QWidget * parent) :
    Base(parent),
    mtfDisplayFunction_(this)
{
  setDisplayFunction(&mtfDisplayFunction_);

  connect(this, &ThisClass::pixmapChanged,
      this, &ThisClass::onPixmapChanged,
      Qt::QueuedConnection);

  connect(&mtfDisplayFunction_, &QMtfDisplay::parameterChanged,
      [this]() {
        if ( !mtfDisplayFunction_.isBusy() ) {
          Base::updateImage();
        }
      });

  connect(this, &ThisClass::displayImageChanged,
      &mtfDisplayFunction_, &QMtfDisplay::displayImageChanged,
      Qt::QueuedConnection);

  createShapes();
}

QVideoFrameDisplay::~QVideoFrameDisplay()
{

}

void QVideoFrameDisplay::createShapes()
{
  if( !rectShape_ ) {

    QRectF rect;

    if( currentImage_.empty() ) {
      rect.setRect(0, 0, 400, 400);
    }
    else {

      rect.setRect(0, 0, currentImage_.cols, currentImage_.rows);

      if( rect.width() > 400 ) {
        rect.setX((rect.left() + rect.right()) / 2 - 200);
        rect.setWidth(400);
      }

      if( rect.height() > 400 ) {
        rect.setY((rect.top() + rect.bottom()) / 2 - 200);
        rect.setHeight(400);
      }
    }

    rectShape_ = new QGraphicsRectShape(rect);
    rectShape_->setResizable(true);
    rectShape_->setFlag(QGraphicsItem::ItemIsMovable, true);
    rectShape_->setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
    rectShape_->setCosmeticPen(Qt::red);
    rectShape_->setVisible(false);
    scene_->addItem(rectShape_);
  }

  if( !lineShape_ ) {

    lineShape_ = new QGraphicsLineShape(-100, -100, 100, 100);
    lineShape_->setFlag(QGraphicsItem::ItemIsMovable, true);
    lineShape_->setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
    lineShape_->setCosmeticPen(Qt::green);
    lineShape_->setVisible(false);
    scene_->addItem(lineShape_);
  }

  if( !targetShape_ ) {
    targetShape_ = new QGraphicsTargetShape();
    targetShape_->setFlag(QGraphicsItem::ItemIsMovable, true);
    targetShape_->setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
    targetShape_->setCosmeticPen(Qt::red);
    targetShape_->setVisible(false);
    scene_->addItem(targetShape_);
  }

}

const QVideoFrameMtfDisplayFunction * QVideoFrameDisplay::mtfDisplayFunction() const
{
  return &mtfDisplayFunction_;
}

QVideoFrameMtfDisplayFunction * QVideoFrameDisplay::mtfDisplayFunction()
{
  return &mtfDisplayFunction_;
}


void QVideoFrameDisplay::setFrameProcessor(const c_image_processor::sptr & processor)
{
  mtfDisplayFunction_.mutex().lock();
  Base::current_processor_ = processor;

  //if( this->workerState_ == Worker_Idle )
  if ( !mtfDisplayFunction_.isBusy() ) {
    updateImage();
  }

  mtfDisplayFunction_.mutex().unlock();

}


QGraphicsRectShape * QVideoFrameDisplay::rectShape() const
{
  return rectShape_;
}

QGraphicsLineShape * QVideoFrameDisplay::lineShape() const
{
  return lineShape_;
}

QGraphicsTargetShape * QVideoFrameDisplay::targetShape() const
{
  return targetShape_;
}

void QVideoFrameDisplay::showEvent(QShowEvent *event)
{
  Base::showEvent(event);
}

void QVideoFrameDisplay::hideEvent(QHideEvent *event)
{
  Base::hideEvent(event);
}


void QVideoFrameDisplay::onPixmapChanged()
{
  c_unique_lock lock(mtfDisplayFunction_.mutex());
  scene()->setImage(pixmap_);
}

void QVideoFrameDisplay::showVideoFrame(const cv::Mat & image, COLORID colorid, int bpp)
{
  c_unique_lock lock(mtfDisplayFunction_.mutex());

  image.copyTo(inputImage_);
  inputMask_.release();
  currentMask_.release();

  if( !current_processor_ || current_processor_->empty() ) {
    currentImage_ = inputImage_;
    currentMask_ = inputMask_;
  }
  else {
    inputImage_.copyTo(currentImage_);
    inputMask_.copyTo(currentMask_);
    current_processor_->process(currentImage_, currentMask_);
  }

  mtfDisplayFunction_.createDisplayImage(
      currentImage_,
      currentMask_,
      displayImage_,
      CV_8U);

  pixmap_ =
      createPixmap(displayImage_, true,
          Qt::NoFormatConversion |
              Qt::ThresholdDither |
              Qt::ThresholdAlphaDither |
              Qt::NoOpaqueDetection);

  Q_EMIT pixmapChanged();

  if ( !mtfDisplayFunction_.isBusy() ) {
    Q_EMIT displayImageChanged();
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////



} /* namespace serimager */
