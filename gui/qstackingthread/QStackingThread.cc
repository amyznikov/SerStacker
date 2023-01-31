/*
 * QStackingThread.cc
 *
 *  Created on: Feb 8, 2021
 *      Author: amyznikov
 */

#include "QStackingThread.h"
#include <core/debug.h>

QStackingThread::QStackingThread() :
    pipeline_(this)
{
}

QStackingThread * QStackingThread::singleton()
{
  static QStackingThread * const instance =
      new QStackingThread();

  return instance;
}

void QStackingThread::lock()
{
  //singleton()->pipeline_.lock();
}

void QStackingThread::unlock()
{
  //singleton()->pipeline_.unlock();
}

c_image_stacking_pipeline * QStackingThread::pipeline()
{
  return &singleton()->pipeline_;
}

const c_image_stacking_options::ptr & QStackingThread::currentStack()
{
  return singleton()->options_;
}

bool QStackingThread::isRunning()
{
  return ((Base*) singleton())->isRunning();
}

bool QStackingThread::start(const c_image_stacking_options::ptr pipeline)
{
  auto_lock lock;

  if ( isRunning() ) {
    return false;
  }

  singleton()->options_ = pipeline;
  ((Base*) singleton())->start(QThread::LowPriority);

  return true;
}

void QStackingThread::cancel()
{
  emit singleton()->finishing();
  singleton()->pipeline_.set_canceled(true);
}


void QStackingThread::run()
{
  pipeline_.run(options_);
}
