/*
 * QStackProgressView.cc
 *
 *  Created on: Jan 31, 2023
 *      Author: amyznikov
 */

#include <gui/widgets/style.h>
#include <gui/widgets/QWaitCursor.h>
#include <gui/widgets/qsprintf.h>
#include <gui/qstackingthread/QStackingThread.h>
#include "QStackProgressView.h"

namespace qserstacker {

#define ICON_menu      ":/icons/menu.png"

///////////////////////////////////////////////////////////////////////////////

QStackProgressView::QStackProgressView(QWidget * parent) :
    Base(parent)
{
  layout_ = new QHBoxLayout(this);
  layout_->setContentsMargins(2, 2, 2, 2);

  progressStrip_ctl = new QProgressStrip(this);
  progressStrip_ctl->setNumStrips(2);
  progressStrip_ctl->setBrush(0, Qt::yellow);
  progressStrip_ctl->setBrush(1, Qt::green);
  layout_->addWidget(progressStrip_ctl, 100);

  menuButton_ctl = new QToolButton(this);
  menuButton_ctl->setContentsMargins(0, 0, 0, 0);
  menuButton_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
  menuButton_ctl->setIconSize(QSize(12, 12));
  menuButton_ctl->setIcon(getIcon(ICON_menu));
  layout_->addWidget(menuButton_ctl, 1);


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

  if( QStackingThread::isRunning() ) {
    onStackingThreadStarted();
  }
}

void QStackProgressView::setImageViewer(QImageEditor * imageViewer)
{
  this->imageViewer_ = imageViewer;
  updateAccumulatedImageDisplay(true);
}

QImageEditor * QStackProgressView::imageViewer() const
{
  return imageViewer_;
}

void QStackProgressView::showEvent(QShowEvent *e)
{
  Base::showEvent(e);
  updateAccumulatedImageDisplay();
}

void QStackProgressView::timerEvent(QTimerEvent *event)
{
  if ( !updatingDisplay_ ) {
    updateAccumulatedImageDisplay();
  }
}

void QStackProgressView::onStackingThreadStarted()
{
  hasCurrentImageUpdates_ = false;
  timerId = startTimer(1000, Qt::VeryCoarseTimer);

}

void QStackProgressView::onStackingThreadFinishing()
{
  killTimer(timerId);
  timerId = 0;

  QApplication::setOverrideCursor(Qt::WaitCursor);
}


void QStackProgressView::onStackingThreadFinished()
{
  if ( timerId > 0 ) {
    killTimer(timerId);
    timerId = 0;
  }

  updateAccumulatedImageDisplay();

  QApplication::restoreOverrideCursor();
}

void QStackProgressView::onStatusChanged()
{
  hasCurrentStatisticsUpdates_ = true;
}

void QStackProgressView::onAccumulatorChanged()
{
  hasCurrentImageUpdates_ = true;
}


void QStackProgressView::updateAccumulatedImageDisplay(bool force)
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

//    statusLabel_->setText(pipeline->status_message().c_str());
//
//    progressLabel_->setText(QString("F %1 / %2 / %3").
//        arg(pipeline->accumulated_frames()).
//        arg(pipeline->processed_frames()).
//            arg(pipeline->total_frames()));

    progressStrip_ctl->setRange(0, pipeline->total_frames());
    progressStrip_ctl->setValue(0, pipeline->processed_frames());
    progressStrip_ctl->setValue(1, pipeline->accumulated_frames());

    progressStrip_ctl->setText(qsprintf("%d/%d/%d",
        pipeline->accumulated_frames(),
        pipeline->processed_frames(),
        pipeline->total_frames()));

    Q_EMIT progressTextChanged();
  }

  if ( (force || hasCurrentImageUpdates_) && imageViewer_ && imageViewer_->isVisible() ) {

    updatingDisplay_ = true;

    QWaitCursor wait(this);

    cv::Mat currentImage;
    cv::Mat currentMask;

    if( pipeline->compute_accumulated_image(currentImage, currentMask) ) {

      if( pipeline->anscombe().method() != anscombe_none ) {
        pipeline->anscombe().inverse(currentImage, currentImage);
      }

      imageViewer_->setCurrentFileName(pipeline->options()->cname());
      imageViewer_->editImage(currentImage, currentMask);
    }
  }

  //pipeline->unlock();

  hasCurrentStatisticsUpdates_ = false;
  hasCurrentImageUpdates_ = false;
  updatingDisplay_ = false;
}

} // namespace qserstacker
