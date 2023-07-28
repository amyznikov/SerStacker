/*
 * QPipelineThread.cc
 *
 *  Created on: Jul 2, 2023
 *      Author: amyznikov
 */

#include "QPipelineThread.h"
#include <core/debug.h>

std::mutex QPipelineThread::lock_;

QPipelineThread::QPipelineThread(QObject * parent) :
    Base(parent)
{
}

QPipelineThread::~QPipelineThread()
{
  cancel();
}

QPipelineThread* QPipelineThread::instance()
{
  static ThisClass *const instance_ =
      new ThisClass(nullptr);

  return instance_;
}

void QPipelineThread::lock()
{
  lock_.lock();
}

void QPipelineThread::unlock()
{
  lock_.unlock();
}

const c_image_processing_pipeline::sptr & QPipelineThread::currentPipeline()
{
  return instance()->currentPipeline_;
}

bool QPipelineThread::isRunning()
{
  return ((QThread*) instance())->isRunning();
}

bool QPipelineThread::start(const c_image_processing_pipeline::sptr & pipeline)
{
  auto_lock lock;

  if( isRunning() ) {
    CF_ERROR("QPipelineThread already running");
    return false;
  }

  instance()->currentPipeline_ = pipeline;
  Q_EMIT instance()->starting();
  ((QThread*) instance())->start(QThread::LowPriority);

  return true;
}

void QPipelineThread::cancel()
{
  auto_lock lock;

  if( instance()->currentPipeline_ ) {
    CF_DEBUG("currentPipeline_->cancel(true)");
    instance()->currentPipeline_->cancel(true);
  }
}

void QPipelineThread::run()
{
  CF_DEBUG("enter");

  if( currentPipeline_ && !currentPipeline_->run() ) {
    CF_ERROR("currentPipeline_->run() fails");
  }

  Q_EMIT finishing();
  CF_DEBUG("leave");
}
