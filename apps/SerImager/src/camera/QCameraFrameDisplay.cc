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
#include <core/proc/pixtype.h>
#include <core/ssprintf.h>

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


namespace serimager {

QCameraFrameMtfDisplayFunction::QCameraFrameMtfDisplayFunction(QImageViewer * imageViewer) :
    Base(imageViewer, "QCameraFrameMtfDisplayFunction")
{
}

std::mutex & QCameraFrameMtfDisplayFunction::mutex()
{
  return mutex_;
}

void QCameraFrameMtfDisplayFunction::getInputDataRange(double * minval, double * maxval) const
{
  c_unique_lock lock(mutex_);
  Base::getInputDataRange(minval, maxval);
}

void QCameraFrameMtfDisplayFunction::getInputHistogramm(cv::OutputArray H, double * output_hmin, double * output_hmax)
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

void QCameraFrameMtfDisplayFunction::getOutputHistogramm(cv::OutputArray H, double * output_hmin, double * output_hmax)
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
        // if (workerState_ == Worker_Idle)
        {
          Base::updateImage();
        }
      });

  connect(this, &ThisClass::displayImageChanged,
      &mtfDisplayFunction_, &QMtfDisplay::displayImageChanged,
      Qt::QueuedConnection);

  createFocusRoiRectItem();
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

void QCameraFrameDisplay::set_debayer_algorithm(DEBAYER_ALGORITHM algo)
{
  debayer_algorithm_ = algo;
}

DEBAYER_ALGORITHM QCameraFrameDisplay::debayer_algorithm()
{
  return debayer_algorithm_;
}


void QCameraFrameDisplay::setFrameProcessor(const c_image_processor::sptr & processor)
{
  mtfDisplayFunction_.mutex().lock();
  Base::current_processor_ = processor;

  //if( this->workerState_ == Worker_Idle )
  {
    updateImage();
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
  scene()->setImage(pixmap_);
}

void QCameraFrameDisplay::createFocusRoiRectItem()
{
  if( !roiItem_ ) {

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

    roiItem_ = new QGraphicsRectShape(rect);
    roiItem_->setResizable(true);
    roiItem_->setFlag(QGraphicsItem::ItemIsMovable, true);
    roiItem_->setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
    roiItem_->setCosmeticPen(Qt::red);
    roiItem_->setVisible(false);
    scene_->addItem(roiItem_);

    connect(roiItem_, &QGraphicsRectShape::itemChanged,
        [this]() {
          const QRectF rc = roiItem_->sceneRect();
          Q_EMIT roiChanged(QRect(rc.x(), rc.y(), rc.width(), rc.height()));
        });
  }
}

void QCameraFrameDisplay::setShowROI(bool show)
{
  roiItem_->setVisible(show);
}

bool QCameraFrameDisplay::showROI() const
{
  return roiItem_->isVisible();
}

QRect QCameraFrameDisplay::roi() const
{
  const QRectF rc = roiItem_->sceneRect();
  return QRect(rc.x(), rc.y(), rc.width(), rc.height());
}


void QCameraFrameDisplay::workerThread()
{

  c_unique_lock lock(mtfDisplayFunction_.mutex());
  workerState_ = Worker_Running;


  bool haveInputImage = false;
  int last_index = -1;


  cv::Mat inputImage, currentImage, displayImage, currentMask;
  QPixmap pixmap;


  while ( workerState_ == Worker_Running && camera_ && camera_->state() == QImagingCamera::State_started ) {

    INSTRUMENT_REGION("body");

    haveInputImage = false;

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
          frame->image().copyTo(inputImage);
          haveInputImage = true;
        }
      }
    }

    lock.unlock();

    if( haveInputImage && !inputImage.empty() ) {

      {
        INSTRUMENT_REGION("convert");

        if( is_bayer_pattern(colorid_) ) {
          debayer(inputImage, inputImage, colorid_, debayer_algorithm_);
        }

        else if( colorid_ == COLORID_RGB ) {
          cv::cvtColor(inputImage, inputImage, cv::COLOR_RGB2BGR);
        }

        // editImage(image, cv::noArray(), false);
        inputImage.copyTo(currentImage);
        currentMask.release();
        if( current_processor_ && !current_processor_->empty() ) {
          current_processor_->process(currentImage, currentMask);
        }


        mtfDisplayFunction_.createDisplayImage(
            currentImage,
            currentMask,
            displayImage,
            CV_8U);

        pixmap =
            createPixmap(displayImage, true,
                Qt::NoFormatConversion |
                    Qt::ThresholdDither |
                    Qt::ThresholdAlphaDither |
                    Qt::NoOpaqueDetection);

      }


      {
        INSTRUMENT_REGION("setCurrentImage");
        lock.lock();
        cv::swap(inputImage, inputImage_);
        cv::swap(currentImage, currentImage_);
        cv::swap(currentMask, currentMask_);
        cv::swap(displayImage, displayImage_);
        pixmap_ = pixmap;
        lock.unlock();
      }


      Q_EMIT pixmapChanged();

      if ( !mtfDisplayFunction_.isBusy() ) {
        Q_EMIT displayImageChanged();
      }

    }

    QThread::msleep(50);

    lock.lock();
  }


  workerState_ = Worker_Idle;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////

QDisplayFrameProcessorSettingsWidget::QDisplayFrameProcessorSettingsWidget(QWidget * parent) :
    Base("QDisplayFrameProcessorSettings", parent)
{
  debayer_ctl =
      add_enum_combobox<DEBAYER_ALGORITHM>(
          "Debayer:",
          [this](DEBAYER_ALGORITHM v) {
            if ( display_ ) {
              display_->set_debayer_algorithm(v);
            }
          },
          [this](DEBAYER_ALGORITHM * v) {
            if ( display_ ) {
              *v = display_->debayer_algorithm();
              return true;
            }
            return false;
          });

  debayer_ctl->setWhatsThis("Selected debayer algorithm for bayer patterns:");


  updateControls();
}

void QDisplayFrameProcessorSettingsWidget::setDisplay(QCameraFrameDisplay * display)
{
  display_ = display;
  updateControls();
}

QCameraFrameDisplay* QDisplayFrameProcessorSettingsWidget::display() const
{
  return display_;
}

void QDisplayFrameProcessorSettingsWidget::onupdatecontrols()
{
  if ( !display_ ) {
    setEnabled(false);
  }
  else {
    Base::onupdatecontrols();
    setEnabled(true);
   }
}


QDisplayFrameProcessorSettingsDialogBox::QDisplayFrameProcessorSettingsDialogBox(QWidget * parent) :
    Base(parent)
{
  setWindowTitle("Frame Display Settings");

  lv_ = new QVBoxLayout(this);
  lv_->addWidget(setiingsWidget_ = new QDisplayFrameProcessorSettingsWidget(this));
}

void QDisplayFrameProcessorSettingsDialogBox::setDisplay(QCameraFrameDisplay * display)
{
  setiingsWidget_->setDisplay(display);
}

QCameraFrameDisplay* QDisplayFrameProcessorSettingsDialogBox::display() const
{
  return setiingsWidget_->display();
}

void QDisplayFrameProcessorSettingsDialogBox::closeEvent(QCloseEvent * e)
{
  hide();
}

void QDisplayFrameProcessorSettingsDialogBox::showEvent(QShowEvent *e)
{
  Base::showEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QDisplayFrameProcessorSettingsDialogBox::hideEvent(QHideEvent *e)
{
  Base::hideEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}


} /* namespace qserimager */
