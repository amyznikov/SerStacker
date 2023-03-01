/*
 * QImageProcessingPipeline.cc
 *
 *  Created on: Feb 8, 2021
 *      Author: amyznikov
 */

#include "QImageProcessingPipeline.h"
#include <core/debug.h>

QImageProcessingPipeline::QImageProcessingPipeline()
{
  connect(this, &Base::finished,
      [this]() {
        current_pipeline_.reset();
      });
}

QImageProcessingPipeline * QImageProcessingPipeline::singleton()
{
  static QImageProcessingPipeline * const instance =
      new QImageProcessingPipeline();

  return instance;
}

void QImageProcessingPipeline::lock()
{
  //singleton()->pipeline_.lock();
}

void QImageProcessingPipeline::unlock()
{
  //singleton()->pipeline_.unlock();
}


c_image_sequence::sptr QImageProcessingPipeline::current_sequence()
{
  return singleton()->current_sequence_;
}

c_image_processing_pipeline::sptr QImageProcessingPipeline::current_pipeline()
{
  QImageProcessingPipeline * _this =
      ThisClass::singleton();

  if ( _this->current_pipeline_ ) {
    return _this->current_pipeline_;
  }

  if ( _this->current_sequence_ ) {
    return _this->current_sequence_->current_pipeline();
  }

  return nullptr;
}


bool QImageProcessingPipeline::isRunning()
{
  return ((Base*) singleton())->isRunning();
}

bool QImageProcessingPipeline::start(const c_image_sequence::sptr & sequence)
{
  auto_lock lock;

  if ( isRunning() ) {
    return false;
  }

  singleton()->current_sequence_ = sequence;
  singleton()->current_pipeline_ = sequence->current_pipeline();

  Q_EMIT singleton()->starting();
  ((Base*) singleton())->start(QThread::LowPriority);

  return true;
}

void QImageProcessingPipeline::cancel()
{
  QImageProcessingPipeline * _this =
      ThisClass::singleton();

  if( _this->current_pipeline_ ) {
    Q_EMIT _this->finishing();
    _this->current_pipeline_->cancel(true);
  }
}

void QImageProcessingPipeline::run()
{
  if( current_pipeline_ ) {

    current_pipeline_->run();
  }

  // current_pipeline_.reset();
}
