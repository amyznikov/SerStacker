/*
 * QStackingProgressView.cc
 *
 *  Created on: Feb 19, 2021
 *      Author: amyznikov
 */

#include "QStackingProgressView.h"
#include "QStackingThread.h"
#include <gui/widgets/QWaitCursor.h>
#include <core/proc/inpaint.h>
#include <core/debug.h>

QStackingProgressView::QStackingProgressView(QWidget * parent)
  : Base(parent)
{
  setWindowTitle("Stacking progress view");

  layout_ = new QVBoxLayout(this);

  statusLabel_= new QLabel("INITIALIZATION...");
  layout_->addWidget(statusLabel_);

  progressLabel_ = new QLabel("INITIALIZATION...");
  layout_->addWidget(progressLabel_);

  //
  connect(QStackingThread::singleton(), &QStackingThread::started,
      this, &ThisClass::onStackingThreadStarted,
      Qt::QueuedConnection);

  connect(QStackingThread::singleton(), &QStackingThread::finished,
      this, &ThisClass::onStackingThreadFinished,
      Qt::QueuedConnection);

  connect(QStackingThread::singleton(), &QStackingThread::finishing,
      this, &ThisClass::onStackingThreadFinishing,
      Qt::QueuedConnection);

  connect(QStackingThread::singleton(), &QStackingThread::statusChanged,
      this, &ThisClass::onStatusChanged,
      Qt::QueuedConnection);

  connect(QStackingThread::singleton(), &QStackingThread::accumulatorChanged,
      this, &ThisClass::onAccumulatorChanged,
      Qt::QueuedConnection);

  if ( QStackingThread::isRunning() ) {
    onStackingThreadStarted();
  }
}

void QStackingProgressView::setImageViewer(QImageEditor * imageViewer)
{
  this->imageViewer_ = imageViewer;
  updateAccumulatedImageDisplay(true);
}

QImageEditor * QStackingProgressView::imageViewer() const
{
  return imageViewer_;
}

void QStackingProgressView::onStackingThreadStarted()
{
  hasCurrentImageUpdates_ = false;
  timerId = startTimer(1000, Qt::VeryCoarseTimer);

}

void QStackingProgressView::onStackingThreadFinishing()
{
  killTimer(timerId);
  timerId = 0;

  QApplication::setOverrideCursor(Qt::WaitCursor);
}


void QStackingProgressView::onStackingThreadFinished()
{
  if ( timerId > 0 ) {
    killTimer(timerId);
    timerId = 0;
  }

  updateAccumulatedImageDisplay();

  QApplication::restoreOverrideCursor();
}

void QStackingProgressView::onStatusChanged()
{
  hasCurrentStatisticsUpdates_ = true;
}

void QStackingProgressView::onAccumulatorChanged()
{
  hasCurrentImageUpdates_ = true;
}

void QStackingProgressView::showEvent(QShowEvent *e)
{
  Base::showEvent(e);
  updateAccumulatedImageDisplay();
}

void QStackingProgressView::timerEvent(QTimerEvent *event)
{
  if ( !updatingDisplay_ ) {
    updateAccumulatedImageDisplay();
  }
}

void QStackingProgressView::updateAccumulatedImageDisplay(bool force)
{
  if ( !force && !hasCurrentStatisticsUpdates_ && !hasCurrentImageUpdates_ ) {
    return;
  }

  c_image_stacking_pipeline * pipeline =
          QStackingThread::pipeline();

  if ( !pipeline ) {
    return;
  }

  //pipeline->lock();

  if ( force || hasCurrentStatisticsUpdates_ ) {

    statusLabel_->setText(pipeline->status_message().c_str());
    progressLabel_->setText(QString("F %1 / %2 / %3").
        arg(pipeline->accumulated_frames()).
        arg(pipeline->processed_frames()).
            arg(pipeline->total_frames()));
  }

  if ( (force || hasCurrentImageUpdates_) && imageViewer_ && imageViewer_->isVisible() ) {

    updatingDisplay_ = true;

    QWaitCursor wait(this);

    cv::Mat currentImage;
    cv::Mat currentMask;

    bool computed = pipeline->compute_accumulated_image(currentImage, currentMask);
    if ( computed ) {

      if ( pipeline->anscombe().method() != anscombe_none ) {
        pipeline->anscombe().inverse(currentImage, currentImage);
      }

      imageViewer_->editImage(currentImage, currentMask);
    }
  }

  //pipeline->unlock();

  hasCurrentStatisticsUpdates_ = false;
  hasCurrentImageUpdates_ = false;
  updatingDisplay_ = false;
}

//void QStackingProgressView::updateCurrentImage()
//{
//  if ( imageViewer_->isVisible() ) {
//    hasCurrentImageUpdates_ = true;
//    updateAccumulatedImageDisplay();
//  }
//}
