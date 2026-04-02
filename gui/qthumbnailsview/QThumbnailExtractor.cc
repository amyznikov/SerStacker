/*
 * QThumbnailExtractor.cc
 *
 *  Created on: Mar 7, 2020
 *      Author: amyznikov
 */

#include "QThumbnailExtractor.h"
#include "QThumbnails.h"
#include <core/debug.h>

QThumbnailExtractor::QThumbnailExtractor()
{
}

QThumbnailExtractor::~QThumbnailExtractor()
{
  cancel();
}

void QThumbnailExtractor::setThumbnailSize(int thumb_size)
{
  _thumbSize = thumb_size;
}

int QThumbnailExtractor::thumbnailSize() const
{
  return _thumbSize;
}

QSize QThumbnailExtractor::compute_thumbnail_size(const QSize & imageSize) const
{
  return ::compute_thumbnail_size(imageSize, _thumbSize);
}

int QThumbnailExtractor::start(const QString & imagePathFileName)
{
  int rid;

  cancel();

  ++_reqId;
  _cancelRequested = false;
  _currentImagePathFileName = imagePathFileName;

  QThread::start();

  return _reqId;
}

void QThumbnailExtractor::cancel()
{
  while ( isRunning() ) {
    _mtx.lock();
    _cancelRequested = true;
    _mtx.unlock();
    wait();
  }
}

bool QThumbnailExtractor::canceled() const
{
  return _cancelRequested;
}


/* worker thread */
void QThumbnailExtractor::run()
{
  const int rid = _reqId;

  if( !_currentImagePathFileName.isEmpty() ) {

    Q_EMIT extracted(rid,
        loadThumbnailIcon(_currentImagePathFileName, _thumbSize),
        _currentImagePathFileName);

  }

}
