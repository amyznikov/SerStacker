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

  _updateTimer.setSingleShot(true);
  _updateTimer.setTimerType(Qt::VeryCoarseTimer);

  QObject::connect(&_updateTimer, &QTimer::timeout,
      this, &ThisClass::onUpdateTimerTimeout);

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

  if( QPipelineThread::isRunning() ) {
    onPipelineThreadStarted();
  }
}

void QPipelineProgressView::setImageView(QInputSourceView * imageView)
{
  if ( _imageView ) {
    _imageView->toolbar()->setEnabled(true);
  }

  if ( (_imageView = imageView) ) {
    _imageView->toolbar()->setEnabled(false);
  }

  updateAccumulatedImageDisplay(true);
}

QInputSourceView * QPipelineProgressView::imageView() const
{
  return _imageView;
}

void QPipelineProgressView::showEvent(QShowEvent *e)
{
  Base::showEvent(e);
  updateAccumulatedImageDisplay();
}

void QPipelineProgressView::onUpdateTimerTimeout()
{
  updateAccumulatedImageDisplay(false);
  if ( QPipelineThread::isRunning() ) {
    _updateTimer.start(1000);
  }
}

void QPipelineProgressView::onPipelineThreadStarted()
{
  c_image_processing_pipeline::sptr pipeline = QPipelineThread::currentPipeline();
  if ( pipeline ) {
    QImageProcessingPipeline *p = dynamic_cast<QImageProcessingPipeline*>(pipeline.get());
    if ( p ) {
      QObject::connect(p, &QImageProcessingPipeline::frameProcessed,
          this, [this]() {
            _hasStatusUpdates = true;
          },
          Qt::DirectConnection);
    }
  }

  _updateTimer.start(1000);
}

void QPipelineProgressView::onPipelineThreadFinishing()
{
  _updateTimer.stop();

  c_image_processing_pipeline::sptr pipeline = QPipelineThread::currentPipeline();
  if ( pipeline ) {
    QImageProcessingPipeline *p = dynamic_cast<QImageProcessingPipeline*>(pipeline.get());
    if ( p ) {
      p->disconnect(this);
    }
  }
}

void QPipelineProgressView::onPipelineThreadFinished()
{
  _updateTimer.stop();
  updateAccumulatedImageDisplay(true);
}

void QPipelineProgressView::onMenuButtonClicked()
{
  c_image_processing_pipeline::sptr pipeline =
      QPipelineThread::currentPipeline();

  if( pipeline ) {

    QMenu menu;

    const c_enum_member *display_types = pipeline->get_preview_displays();

    if( display_types ) {

      const int display_type = pipeline->preview_display();

      int items_count = 0;

      for( ; !display_types->name.empty(); ++display_types ) {
        QAction *action = menu.addAction(display_types->name.c_str());
        action->setData(QVariant::fromValue(display_types->value));
        action->setCheckable(true);
        action->setChecked(display_type == display_types->value);
        ++items_count;
      }

      if( items_count > 1 ) {

        QAction * action =
            menu.exec(_menuButton->mapToGlobal(QPoint(_menuButton->width() / 2,
                _menuButton->height() / 2)));

        if( action ) {
          const int selected_display_type = action->data().value<int>();
          if( display_type != selected_display_type ) {
            pipeline->set_preview_display(selected_display_type);
          }
        }
      }
    }
  }
}


void QPipelineProgressView::updateAccumulatedImageDisplay(bool force)
{
  const c_image_processing_pipeline::sptr & pipeline = QPipelineThread::currentPipeline();
  if ( !pipeline || (!force && !_hasStatusUpdates)) {
    return;
  }

  _hasStatusUpdates = false;

  _progressStrip->setRange(0, pipeline->total_frames());
  _progressStrip->setValue(0, pipeline->processed_frames());
  _progressStrip->setValue(1, pipeline->accumulated_frames());

  _progressStrip->setText(qsprintf("%d/%d/%d",
      pipeline->accumulated_frames(),
      pipeline->processed_frames(),
      pipeline->total_frames()));

  Q_EMIT progressTextChanged();

  if( _imageView && _imageView->isVisible() ) {

    QWaitCursor wait(this);

    if( pipeline->get_display(_currentImage, _currentMask) ) {

      if ( !_dataframe ) {
        _dataframe.reset(new c_data_frame());
      }

      _dataframe->add_image("PROGRESS", _currentImage, _currentMask, cv::noArray(), false);
      _imageView->showFrame(_dataframe);
    }

  }
}


} // namespace serstacker
