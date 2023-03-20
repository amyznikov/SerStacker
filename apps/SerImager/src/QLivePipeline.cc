/*
 * QLivePipeline.cc
 *
 *  Created on: Mar 20, 2023
 *      Author: amyznikov
 */

#include "QLivePipeline.h"
#include <core/debug.h>

namespace serimager {

namespace {

typedef std::lock_guard<std::mutex>
  c_guard_lock;

typedef std::unique_lock<std::mutex>
  c_unique_lock;

} // namespace


///////////////////////////////////////////////////////////////////////////////////////////////////
QLivePipeline::QLivePipeline()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QLivePipelineThread::QLivePipelineThread(QObject * parent) :
    Base(parent)
{

}

QLivePipelineThread::~QLivePipelineThread()
{
  finish(true);
}


const QImagingCamera::sptr & QLivePipelineThread::camera() const
{
  return camera_;
}

void QLivePipelineThread::setCamera(const QImagingCamera::sptr & camera)
{
  if ( isRunning() ) {
    finish(true);
  }

  if( camera_ ) {
    disconnect(camera_.get(), nullptr,
        this, nullptr);
  }

  camera_ = camera;
  if( (camera_ = camera) ) {

    connect(camera_.get(), &QImagingCamera::stateChanged,
        this, &ThisClass::onCameraStateChanged,
        Qt::QueuedConnection);

    startPipeline(pipeline_);
  }

}

void QLivePipelineThread::onCameraStateChanged(QImagingCamera::State oldState, QImagingCamera::State newState)
{
  switch (newState) {
    case QImagingCamera::State_started:
      startPipeline(pipeline_);
      break;
    default:
      finish(true);
      break;
  }
}

void QLivePipelineThread::setDisplay(QVideoFrameDisplay * display)
{
  display_ = display;
}

QVideoFrameDisplay* QLivePipelineThread::display() const
{
  return display_;
}

QLivePipeline* QLivePipelineThread::currentPipeline() const
{
  return pipeline_;
}

bool QLivePipelineThread::startPipeline(QLivePipeline * pipeline)
{
  if( isRunning() ) {
    finish(true);
  }

  this->pipeline_ = pipeline;
  this->finish_ = false;

  if( !display_ ) {
    CF_ERROR("ERROR: no display was specified, can not start");
    return false;
  }

  if( !camera_ ) {
    CF_ERROR("ERROR: no camera specified, can not start");
    return false;
  }

  if( camera_->state() != QImagingCamera::State_started ) {
    // CF_ERROR("ERROR: camera is not started");
    return false;
  }

  Base::start();

  return true;
}

void QLivePipelineThread::finish(bool wait)
{
  finish_ = true;

  if( wait ) {
    while (isRunning()) {
      Base::msleep(100);
    }
  }
}

void QLivePipelineThread::run()
{
  cv::Mat inputImage;
  bool haveInputImage;

  int last_frame_index = -1;
  int bpp;
  COLORID colorid;

  while (!finish_) {

    if( 42 ) {

      QImagingCamera::shared_lock lock(camera_->mutex());

      const std::deque<QCameraFrame::sptr> &deque =
          camera_->deque();

      if( !deque.empty() ) {

        const QCameraFrame::sptr &frame =
            deque.back();

        const int index =
            frame->index();

        if( index > last_frame_index ) {

          last_frame_index = index;
          bpp = frame->bpp();
          colorid = frame->colorid();
          frame->image().copyTo(inputImage);

          haveInputImage = true;
        }
      }
    }

    if( haveInputImage ) {

      display_->showVideoFrame(inputImage,
          colorid,
          bpp);
    }

    QThread::msleep(30);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////

} /* namespace serimager */
