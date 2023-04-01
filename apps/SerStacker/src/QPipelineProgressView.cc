/*
 * QPipelineProgressView.cc
 *
 *  Created on: Jan 31, 2023
 *      Author: amyznikov
 */

#include "QPipelineProgressView.h"
#include <gui/qpipelinethread/QImageProcessingPipeline.h>
#include <gui/widgets/style.h>
#include <gui/widgets/QWaitCursor.h>
#include <gui/widgets/qsprintf.h>
#include <core/pipeline/c_image_stacking_pipeline.h>
#include <core/pipeline/c_camera_calibration_pipeline.h>
#include <core/pipeline/c_stereo_calibration_pipeline.h>
#include <core/pipeline/c_regular_stereo_pipeline.h>

namespace serstacker {

#define ICON_menu      ":/serstacker/icons/menu.png"

///////////////////////////////////////////////////////////////////////////////

QPipelineProgressView::QPipelineProgressView(QWidget * parent) :
    Base(parent)
{
  layout_ = new QHBoxLayout(this);
  layout_->setContentsMargins(2, 2, 2, 2);

  progressStrip_ = new QProgressStrip(this);
  progressStrip_->setNumStrips(2);
  progressStrip_->setBrush(0, Qt::yellow);
  progressStrip_->setBrush(1, Qt::green);
  layout_->addWidget(progressStrip_, 100);

  menuButton_ = new QToolButton(this);
  menuButton_->setContentsMargins(0, 0, 0, 0);
  menuButton_->setToolButtonStyle(Qt::ToolButtonIconOnly);
  menuButton_->setIconSize(QSize(12, 12));
  menuButton_->setIcon(getIcon(ICON_menu));
  layout_->addWidget(menuButton_, 1);


  //
  connect(QImageProcessingPipeline::singleton(), &QImageProcessingPipeline::started,
      this, &ThisClass::onStackingThreadStarted,
      Qt::QueuedConnection);

  connect(QImageProcessingPipeline::singleton(), &QImageProcessingPipeline::finished,
      this, &ThisClass::onStackingThreadFinished,
      Qt::QueuedConnection);

  connect(QImageProcessingPipeline::singleton(), &QImageProcessingPipeline::finishing,
      this, &ThisClass::onStackingThreadFinishing,
      Qt::QueuedConnection);

//  connect(QImageProcessingPipeline::singleton(), &QImageProcessingPipeline::statusChanged,
//      this, &ThisClass::onStatusChanged,
//      Qt::QueuedConnection);
//
//  connect(QImageProcessingPipeline::singleton(), &QImageProcessingPipeline::accumulatorChanged,
//      this, &ThisClass::onAccumulatorChanged,
//      Qt::QueuedConnection);
//
//  connect(QImageProcessingPipeline::singleton(), &QImageProcessingPipeline::selectedMasterFrameChanged,
//      this, &ThisClass::onSelectedMasterFrameChanged,
//      Qt::QueuedConnection);

  if( QImageProcessingPipeline::isRunning() ) {
    onStackingThreadStarted();
  }
}

void QPipelineProgressView::setImageViewer(QImageEditor * imageViewer)
{
  this->imageViewer_ = imageViewer;
  updateAccumulatedImageDisplay(true);
}

QImageEditor * QPipelineProgressView::imageViewer() const
{
  return imageViewer_;
}

void QPipelineProgressView::showEvent(QShowEvent *e)
{
  Base::showEvent(e);
  updateAccumulatedImageDisplay();
}

void QPipelineProgressView::timerEvent(QTimerEvent *event)
{
  if ( !updatingDisplay_ ) {
    updateAccumulatedImageDisplay();
  }
}

void QPipelineProgressView::onStackingThreadStarted()
{
  c_image_processing_pipeline::sptr pipeline =
      QImageProcessingPipeline::current_pipeline();

  if( pipeline ) {

    on_status_update =
        pipeline->on_status_update.add([this]() {
          hasStatusUpdates_ = true;
        });

  }

  timerId = startTimer(1000, Qt::VeryCoarseTimer);
}

void QPipelineProgressView::onStackingThreadFinishing()
{
  c_image_processing_pipeline::sptr pipeline =
      QImageProcessingPipeline::current_pipeline();

  if ( pipeline ) {
    pipeline->on_status_update.remove(on_status_update);
  }


  killTimer(timerId);
  timerId = 0;

  QApplication::setOverrideCursor(Qt::WaitCursor);
}


void QPipelineProgressView::onStackingThreadFinished()
{
  if ( timerId > 0 ) {
    killTimer(timerId);
    timerId = 0;
  }

  updateAccumulatedImageDisplay(true);

  QApplication::restoreOverrideCursor();
}

void QPipelineProgressView::updateAccumulatedImageDisplay(bool force)
{

  const c_image_processing_pipeline::sptr & pipeline =
      QImageProcessingPipeline::current_pipeline();

  if ( !pipeline ) {
    return;
  }

  if ( !force && !hasStatusUpdates_ ) {
    return;
  }

  progressStrip_->setRange(0, pipeline->total_frames());
  progressStrip_->setValue(0, pipeline->processed_frames());
  progressStrip_->setValue(1, pipeline->accumulated_frames());

  progressStrip_->setText(qsprintf("%d/%d/%d",
      pipeline->accumulated_frames(),
      pipeline->processed_frames(),
      pipeline->total_frames()));

  Q_EMIT progressTextChanged();

  if( imageViewer_ && imageViewer_->isVisible() ) {

    updatingDisplay_ = true;

    if( const c_image_stacking_pipeline::sptr image_stacking =
        std::dynamic_pointer_cast<c_image_stacking_pipeline>(pipeline) ) {

      QWaitCursor wait(this);

      if( image_stacking->pipeline_stage() == stacking_stage_idle ) {

        std::string output_file_name =
            image_stacking->output_file_name();

        CF_DEBUG("output_file_name: %s", output_file_name.c_str());

        if( !output_file_name.empty() ) {
          imageViewer_->openImage(output_file_name);
        }

      }
      else {

        cv::Mat currentImage, currentMask;

        if( image_stacking->get_display_image(currentImage, currentMask) ) {
          if( !currentImage.empty() && image_stacking->anscombe().method() != anscombe_none ) {
            image_stacking->anscombe().inverse(currentImage, currentImage);
          }
        }

        imageViewer_->setCurrentFileName(pipeline->cname());
        imageViewer_->editImage(currentImage, currentMask);
      }
    }

    else {
      QWaitCursor wait(this);

      cv::Mat currentImage, currentMask;

      pipeline->get_display_image(currentImage, currentMask);
      imageViewer_->setCurrentFileName(pipeline->cname());
      imageViewer_->editImage(currentImage, currentMask);
    }


    hasStatusUpdates_ = false;
    updatingDisplay_ = false;
  }
}


} // namespace serstacker
