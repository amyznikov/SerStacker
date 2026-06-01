/*
 * QPipelineThread.cc
 *
 *  Created on: Jul 2, 2023
 *      Author: amyznikov
 */

#include "QPipelineThread.h"
#include <core/debug.h>

std::mutex QPipelineThread::_lock;

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
  _lock.lock();
}

void QPipelineThread::unlock()
{
  _lock.unlock();
}

const c_image_processing_pipeline::sptr & QPipelineThread::currentPipeline()
{
  return instance()->_currentPipeline;
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

  instance()->_currentPipeline = pipeline;
  Q_EMIT instance()->starting();
  ((QThread*) instance())->start(QThread::LowPriority);

  return true;
}

void QPipelineThread::cancel()
{
  auto_lock lock;

  if( instance()->_currentPipeline ) {
    CF_DEBUG("currentPipeline_->cancel(true)");
    instance()->_currentPipeline->cancel(true);
  }
}

void QPipelineThread::run()
{
  CF_DEBUG("enter");

  QImageProcessingPipeline * qppline =
      dynamic_cast<QImageProcessingPipeline*>(_currentPipeline.get());
  if ( qppline ) {
    QObject::connect(qppline, &QImageProcessingPipeline::parametersUpdate,
        this, &ThisClass::currentPipelineParametersUpdate,
        Qt::QueuedConnection);
  }

  if( _currentPipeline && !_currentPipeline->run() ) {
    CF_ERROR("currentPipeline_->run() fails");
  }

  if ( qppline ) {
    qppline->disconnect(this);
  }

  Q_EMIT finishing();
  CF_DEBUG("leave");
}
