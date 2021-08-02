/*
 * QStackingThread.cc
 *
 *  Created on: Feb 8, 2021
 *      Author: amyznikov
 */

#include "QStackingThread.h"
#include <core/debug.h>

QStackingThread::c_stacking_thread_impl::c_stacking_thread_impl(QStackingThread * _qobj)
  : qobj(_qobj)
{
}


QStackingThread::QStackingThread()
  : pipeline_(this)
{

}


QStackingThread * QStackingThread::singleton()
{
  static QStackingThread * const instance = new QStackingThread();
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
  CF_DEBUG("H");
  emit singleton()->finishing();
  CF_DEBUG("H");
  singleton()->pipeline_.set_canceled(true);
  CF_DEBUG("H");

  CF_DEBUG("H");
//  while ( ((Base*) singleton())->isRunning() ) {
//    CF_DEBUG("H");
//    qApp->processEvents();
//    qApp->sendPostedEvents();
//    msleep(1);
//  }
  CF_DEBUG("H");
}


void QStackingThread::run()
{
  pipeline_.run(options_);
}
