/*
 * QPipelineProgressView.cc
 *
 *  Created on: Jan 31, 2023
 *      Author: amyznikov
 */

#include "QPipelineProgressView.h"
#include <gui/qpipeline/QImageProcessingPipeline.h>
#include <gui/qpipeline/QPipelineThread.h>
#include <gui/widgets/style.h>
#include <gui/widgets/QWaitCursor.h>
#include <gui/widgets/qsprintf.h>
#include <core/pipeline/c_image_stacking_pipeline/c_image_stacking_pipeline.h>
#include <core/pipeline/c_camera_calibration_pipeline/c_camera_calibration_pipeline.h>
#include <core/pipeline/c_stereo_calibration_pipeline/c_stereo_calibration_pipeline.h>
#include <core/pipeline/c_regular_stereo_pipeline/c_regular_stereo_pipeline.h>

namespace serstacker {

#define ICON_menu      ":/serstacker/icons/menu.png"

///////////////////////////////////////////////////////////////////////////////

QPipelineProgressView::QPipelineProgressView(QWidget * parent) :
    Base(parent)
{
  _layout = new QHBoxLayout(this);
  _layout->setContentsMargins(2, 2, 2, 2);

  _progressStrip = new QProgressStrip(this);
  _progressStrip->setNumStrips(2);
  _progressStrip->setBrush(0, Qt::yellow);
  _progressStrip->setBrush(1, Qt::green);
  _layout->addWidget(_progressStrip, 100);

  _menuButton = new QToolButton(this);
  _menuButton->setContentsMargins(0, 0, 0, 0);
  _menuButton->setToolButtonStyle(Qt::ToolButtonIconOnly);
  _menuButton->setIconSize(QSize(12, 12));
  _menuButton->setIcon(getIcon(ICON_menu));
  _layout->addWidget(_menuButton, 1);


  //
  connect(QPipelineThread::instance(), &QPipelineThread::started,
      this, &ThisClass::onPipelineThreadStarted,
      Qt::QueuedConnection);

  connect(QPipelineThread::instance(), &QPipelineThread::finished,
      this, &ThisClass::onPipelineThreadFinished,
      Qt::QueuedConnection);

  connect(QPipelineThread::instance(), &QPipelineThread::finishing,
      this, &ThisClass::onPipelineThreadFinishing,
      Qt::QueuedConnection);

  connect(_menuButton, &QToolButton::clicked,
      this, &ThisClass::onMenuButtonClicked);
//
//  connect(QPipelineThread::instance(), &QPipelineThread::accumulatorChanged,
//      this, &ThisClass::onAccumulatorChanged,
//      Qt::QueuedConnection);
//
//  connect(QPipelineThread::instance(), &QPipelineThread::selectedMasterFrameChanged,
//      this, &ThisClass::onSelectedMasterFrameChanged,
//      Qt::QueuedConnection);

  if( QPipelineThread::isRunning() ) {
    onPipelineThreadStarted();
  }
}

void QPipelineProgressView::setImageViewer(QImageEditor * imageViewer)
{
  this->_imageViewer = imageViewer;
  updateAccumulatedImageDisplay(true);
}

QImageEditor * QPipelineProgressView::imageViewer() const
{
  return _imageViewer;
}

void QPipelineProgressView::showEvent(QShowEvent *e)
{
  Base::showEvent(e);
  updateAccumulatedImageDisplay();
}

void QPipelineProgressView::timerEvent(QTimerEvent *event)
{
  if ( !_updatingDisplay ) {
    updateAccumulatedImageDisplay();
  }
}

void QPipelineProgressView::onPipelineThreadStarted()
{
  c_image_processing_pipeline::sptr pipeline =
      QPipelineThread::currentPipeline();

  if( pipeline ) {

    QImageProcessingPipeline *p =
        dynamic_cast<QImageProcessingPipeline*>(pipeline.get());

    if( p ) {
      connect(p, &QImageProcessingPipeline::frameProcessed,
          [this]() {
            _hasStatusUpdates = true;
          });
    }
  }

  timerId = startTimer(1000, Qt::VeryCoarseTimer);
}

void QPipelineProgressView::onPipelineThreadFinishing()
{
  c_image_processing_pipeline::sptr pipeline =
      QPipelineThread::currentPipeline();

  if( pipeline ) {

    QImageProcessingPipeline *p =
        dynamic_cast<QImageProcessingPipeline*>(pipeline.get());

    if( p ) {
      p->disconnect(this);
    }
  }

  killTimer(timerId);
  timerId = 0;

  QApplication::setOverrideCursor(Qt::WaitCursor);
}


void QPipelineProgressView::onPipelineThreadFinished()
{
  if ( timerId > 0 ) {
    killTimer(timerId);
    timerId = 0;
  }

  updateAccumulatedImageDisplay(true);

  QApplication::restoreOverrideCursor();
}


void QPipelineProgressView::onMenuButtonClicked()
{
  c_image_processing_pipeline::sptr pipeline =
      QPipelineThread::currentPipeline();

  if( pipeline ) {

    QMenu menu;

    const c_enum_member *display_types =
        pipeline->get_display_types();

    if( display_types ) {

      const int display_type =
          pipeline->display_type();

      int items_count = 0;

      for( ; !display_types->name.empty(); ++display_types ) {

        QAction *action =
            menu.addAction(display_types->name.c_str());

        action->setData(QVariant::fromValue(display_types->value));
        action->setCheckable(true);
        action->setChecked(display_type == display_types->value);

        ++items_count;
      }

      if( items_count > 1 ) {

        QAction *action =
            menu.exec(_menuButton->mapToGlobal(QPoint(_menuButton->width() / 2, _menuButton->height() / 2)));

        if( action ) {

          const int selected_display_type =
              action->data().value<int>();

          if( display_type != selected_display_type ) {
            pipeline->set_display_type(selected_display_type);
          }
        }
      }
    }
  }
}


void QPipelineProgressView::updateAccumulatedImageDisplay(bool force)
{
  const c_image_processing_pipeline::sptr & pipeline =
      QPipelineThread::currentPipeline();

  if ( !pipeline ) {
    return;
  }

  if ( !force && !_hasStatusUpdates ) {
    return;
  }

  _progressStrip->setRange(0, pipeline->total_frames());
  _progressStrip->setValue(0, pipeline->processed_frames());
  _progressStrip->setValue(1, pipeline->accumulated_frames());

  _progressStrip->setText(qsprintf("%d/%d/%d",
      pipeline->accumulated_frames(),
      pipeline->processed_frames(),
      pipeline->total_frames()));

  Q_EMIT progressTextChanged();

  if( _imageViewer && _imageViewer->isVisible() ) {

    QWaitCursor wait(this);
    cv::Mat currentImage, currentMask;

    _updatingDisplay = true;

    if( pipeline->get_display(currentImage, currentMask) ) {
      _imageViewer->setCurrentFileName(pipeline->cname());
      _imageViewer->editImage(currentImage, currentMask);
    }

    _hasStatusUpdates = false;
    _updatingDisplay = false;
  }
}


} // namespace serstacker
