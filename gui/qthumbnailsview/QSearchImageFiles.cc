/*
 * QSearchImageFiles.cc
 *
 *  Created on: Jul 12, 2021
 *      Author: amyznikov
 */

#include "QSearchImageFiles.h"
#include "QThumbnails.h"
#include <core/debug.h>

QSearchImageFiles::QSearchImageFiles()
{
  supported_suffixes_ = getSupportedThumbnailsExtensions();
  for ( QString & s : supported_suffixes_ ) {
    s = QString("*.%1").arg(s);
  }
}

QSearchImageFiles::~QSearchImageFiles()
{
  cancel();
}

int QSearchImageFiles::start(const QString & directory)
{
  int rid;

  cancel();

  ++req_id_;
  cancel_requested_ = false;
  current_path_ = directory;

  QThread::start();

  return req_id_;
}

void QSearchImageFiles::cancel()
{
  while ( isRunning() ) {
    mtx_.lock();
    cancel_requested_ = true;
    mtx_.unlock();
    wait();
  }
}

bool QSearchImageFiles::canceled() const
{
  return cancel_requested_;
}

void QSearchImageFiles::run()
{
  const int rid = req_id_;

  QDirIterator iterator(current_path_,
      supported_suffixes_,
      QDir::Files | QDir::Readable,
      QDirIterator::FollowSymlinks);

  while ( !cancel_requested_ && iterator.hasNext() ) {

    emit imageFound(rid, iterator.next());

    QThread::yieldCurrentThread();
  }
}


