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

namespace qserstacker {

#define ICON_menu      ":/icons/menu.png"

///////////////////////////////////////////////////////////////////////////////

QPipelineProgressView::QPipelineProgressView(QWidget * parent) :
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
  CF_DEBUG("H");

  c_image_processing_pipeline::sptr pipeline =
      QImageProcessingPipeline::current_pipeline();

  if ( pipeline ) {

    on_status_changed =
        pipeline->on_status_changed.add([this]() {
          hasCurrentStatisticsUpdates_ = true;
        });


    if( const c_image_stacking_pipeline::sptr image_stacking =
        std::dynamic_pointer_cast<c_image_stacking_pipeline>(pipeline) ) {

      on_stacking_stage_changed =
          image_stacking->on_stacking_stage_changed.add(
              [this](STACKING_STAGE oldstage, STACKING_STAGE newstage) {
                accumuatorImageChanged_ = true;
              });

      on_accumulator_changed =
          image_stacking->on_accumulator_changed.add([this]() {
            accumuatorImageChanged_ = true;
          });

      on_selected_master_frame_changed =
          image_stacking->on_selected_master_frame_changed.add([this]() {
            selectedMasterFrameChanged_ = true;
          });


    }
    else if ( const c_camera_calibration_pipeline::sptr camera_calibration =
        std::dynamic_pointer_cast<c_camera_calibration_pipeline>(pipeline) ) {

//      on_pipeline_stage_changed =
//          camera_calibration->on_pipeline_stage_changed.add(
//              [this](CHESSBOARD_CAMERA_CALIBRATION_STAGE, CHESSBOARD_CAMERA_CALIBRATION_STAGE) {
//              });

      on_current_frame_changed =
          camera_calibration->on_current_frame_changed.add([this]() {
            accumuatorImageChanged_ = true;
          });

      on_accumulator_changed =
          camera_calibration->on_accumulator_changed.add([this]() {
            accumuatorImageChanged_ = true;
          });

    }

    else if ( const c_stereo_calibration_pipeline::sptr stereo_calibration =
        std::dynamic_pointer_cast<c_stereo_calibration_pipeline>(pipeline) ) {

//      on_pipeline_stage_changed =
//          stereo_calibration->on_pipeline_stage_changed.add(
//              [this](CHESSBOARD_CAMERA_CALIBRATION_STAGE, CHESSBOARD_CAMERA_CALIBRATION_STAGE) {
//              });

      on_current_frame_changed =
          stereo_calibration->on_current_frame_changed.add([this]() {
            accumuatorImageChanged_ = true;
          });

      on_accumulator_changed =
          stereo_calibration->on_accumulator_changed.add([this]() {
            accumuatorImageChanged_ = true;
          });

    }



  }

  CF_DEBUG("H");

  timerId = startTimer(1000, Qt::VeryCoarseTimer);
}

void QPipelineProgressView::onStackingThreadFinishing()
{
  c_image_processing_pipeline::sptr pipeline =
      QImageProcessingPipeline::current_pipeline();

  if ( pipeline ) {

    pipeline->on_status_changed.remove(on_status_changed);

    if( const c_image_stacking_pipeline::sptr image_stacking =
        std::dynamic_pointer_cast<c_image_stacking_pipeline>(pipeline) ) {

      image_stacking->on_stacking_stage_changed.remove(on_stacking_stage_changed);
      image_stacking->on_accumulator_changed.remove(on_accumulator_changed);
      image_stacking->on_selected_master_frame_changed.remove(on_selected_master_frame_changed);
    }

    else if ( const c_camera_calibration_pipeline::sptr camera_calibration =
        std::dynamic_pointer_cast<c_camera_calibration_pipeline>(pipeline) ) {

      camera_calibration->on_accumulator_changed.remove(on_accumulator_changed);
      camera_calibration->on_current_frame_changed.remove(on_current_frame_changed);

    }

    else if ( const c_stereo_calibration_pipeline::sptr stereo_calibration =
        std::dynamic_pointer_cast<c_stereo_calibration_pipeline>(pipeline) ) {

      stereo_calibration->on_accumulator_changed.remove(on_accumulator_changed);
      stereo_calibration->on_current_frame_changed.remove(on_current_frame_changed);

    }



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

  if ( !force && !hasCurrentStatisticsUpdates_ && !selectedMasterFrameChanged_ ) {
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

  if( imageViewer_ && imageViewer_->isVisible() ) {

    updatingDisplay_ = true;

    if( const c_image_stacking_pipeline::sptr image_stacking =
        std::dynamic_pointer_cast<c_image_stacking_pipeline>(pipeline) ) {


      QWaitCursor wait(this);

      if( image_stacking->stacking_stage() == stacking_stage_idle ) {

        std::string output_file_name =
            image_stacking->output_file_name();

        CF_DEBUG("output_file_name: %s", output_file_name.c_str());

        if( !output_file_name.empty() ) {
          imageViewer_->openImage(output_file_name);
        }

      }
      else {

        cv::Mat currentImage;
        cv::Mat currentMask;

        if( selectedMasterFrameChanged_ ) {
          image_stacking->get_selected_master_frame(currentImage, currentMask);
        }
        else {
          image_stacking->compute_accumulated_image(currentImage, currentMask);
        }

        if( !currentImage.empty() && image_stacking->anscombe().method() != anscombe_none ) {
          image_stacking->anscombe().inverse(currentImage, currentImage);
        }

        imageViewer_->setCurrentFileName(pipeline->cname());
        imageViewer_->editImage(currentImage, currentMask);
      }
    }

    else if( const c_camera_calibration_pipeline::sptr camera_calibration =
        std::dynamic_pointer_cast<c_camera_calibration_pipeline>(pipeline) ) {

      QWaitCursor wait(this);

      if ( accumuatorImageChanged_ ) {

        cv::Mat currentImage;
        cv::Mat currentMask;

        camera_calibration->get_display_image(currentImage, currentMask);
        imageViewer_->setCurrentFileName(pipeline->cname());
        imageViewer_->editImage(currentImage, currentMask);

      }
    }

    else if( const c_stereo_calibration_pipeline::sptr stereo_calibration =
        std::dynamic_pointer_cast<c_stereo_calibration_pipeline>(pipeline) ) {


      QWaitCursor wait(this);

      if ( accumuatorImageChanged_ ) {

        cv::Mat currentImage;
        cv::Mat currentMask;

        stereo_calibration->get_display_image(currentImage, currentMask);
        imageViewer_->setCurrentFileName(pipeline->cname());
        imageViewer_->editImage(currentImage, currentMask);

      }
    }


    hasCurrentStatisticsUpdates_ = false;
    accumuatorImageChanged_ = false;
    selectedMasterFrameChanged_ = false;
    updatingDisplay_ = false;
  }
}


} // namespace qserstacker
