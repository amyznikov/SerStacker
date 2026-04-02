/*
 * QSearchImageFiles.cc
 *
 *  Created on: Jul 12, 2021
 *      Author: amyznikov
 */

#include "QSearchImageFiles.h"
#include "QThumbnails.h"
// #include <core/debug.h>

QSearchImageFiles::QSearchImageFiles()
{
  _supportedSuffixes = getSupportedThumbnailsExtensions();
  for ( QString & s : _supportedSuffixes ) {
    s = QString("*%1").arg(s);
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

  ++_reqId;
  _cancelRequested = false;
  _currentPath = directory;

  QThread::start();

  return _reqId;
}

void QSearchImageFiles::cancel()
{
  while ( isRunning() )
  {
    _mtx.lock();
    _cancelRequested = true;
    _mtx.unlock();
    QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
  }
}

bool QSearchImageFiles::canceled() const
{
  return _cancelRequested;
}

void QSearchImageFiles::run()
{
  const int rid = _reqId;

  QDirIterator iterator(_currentPath,
      _supportedSuffixes,
      QDir::Files | QDir::Readable,
      QDirIterator::FollowSymlinks);

  while ( !_cancelRequested && iterator.hasNext() ) {
    Q_EMIT imageFound(rid, iterator.next());
    QThread::yieldCurrentThread();
  }
}


