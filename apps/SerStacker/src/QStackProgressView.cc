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

  connect(QStackingThread::singleton(), &QStackingThread::selectedMasterFrameChanged,
      this, &ThisClass::onSelectedMasterFrameChanged,
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
  accumuatorImageChanged_ = false;
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
  accumuatorImageChanged_ = true;
}

void QStackProgressView::onSelectedMasterFrameChanged()
{
  selectedMasterFrameChanged_ = true;
}

void QStackProgressView::updateAccumulatedImageDisplay(bool force)
{
  c_image_stacking_pipeline * pipeline =
          QStackingThread::pipeline();

  if ( !pipeline ) {
    return;
  }

  if ( !force && !hasCurrentStatisticsUpdates_ && !accumuatorImageChanged_ && !selectedMasterFrameChanged_ ) {
    return;
  }

  if ( force || hasCurrentStatisticsUpdates_ ) {

    progressStrip_ctl->setRange(0, pipeline->total_frames());
    progressStrip_ctl->setValue(0, pipeline->processed_frames());
    progressStrip_ctl->setValue(1, pipeline->accumulated_frames());

    progressStrip_ctl->setText(qsprintf("%d/%d/%d",
        pipeline->accumulated_frames(),
        pipeline->processed_frames(),
        pipeline->total_frames()));

    Q_EMIT progressTextChanged();
  }


  if( (force || accumuatorImageChanged_ || selectedMasterFrameChanged_) && imageViewer_ && imageViewer_->isVisible() ) {

    updatingDisplay_ = true;

    QWaitCursor wait(this);


    if( pipeline->stacking_stage() == stacking_stage_idle ) {

      std::string output_file_name =
          pipeline->output_file_name();

      if ( !output_file_name.empty() ) {
        imageViewer_->openImage(output_file_name);
      }

    }
    else {

      cv::Mat currentImage;
      cv::Mat currentMask;

      if( selectedMasterFrameChanged_ ) {
        pipeline->get_selected_master_frame(currentImage, currentMask);
      }
      else {
        pipeline->compute_accumulated_image(currentImage, currentMask);
      }

      if( !currentImage.empty() && pipeline->anscombe().method() != anscombe_none ) {
        pipeline->anscombe().inverse(currentImage, currentImage);
      }

      imageViewer_->setCurrentFileName(pipeline->options()->cname());
      imageViewer_->editImage(currentImage, currentMask);
    }
  }

  hasCurrentStatisticsUpdates_ = false;
  accumuatorImageChanged_ = false;
  selectedMasterFrameChanged_ = false;
  updatingDisplay_ = false;
}

} // namespace qserstacker
