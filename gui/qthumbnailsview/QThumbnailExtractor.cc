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
  thumbSize_ = thumb_size;
}

int QThumbnailExtractor::thumbnailSize() const
{
  return thumbSize_;
}

QSize QThumbnailExtractor::compute_thumbnail_size(const QSize & imageSize) const
{
  return ::compute_thumbnail_size(imageSize, thumbSize_);
}

int QThumbnailExtractor::start(const QString & imagePathFileName)
{
  int rid;

  cancel();

  ++reqId_;
  cancelRequested_ = false;
  currentImagePathFileName_ = imagePathFileName;

  QThread::start();

  return reqId_;
}

void QThumbnailExtractor::cancel()
{
  while ( isRunning() ) {
    mtx_.lock();
    cancelRequested_ = true;
    mtx_.unlock();
    wait();
  }
}

bool QThumbnailExtractor::canceled() const
{
  return cancelRequested_;
}


/* worker thread */
void QThumbnailExtractor::run()
{
  const int rid = reqId_;

  if( !currentImagePathFileName_.isEmpty() ) {

    Q_EMIT extracted(rid,
        loadThumbnailIcon(currentImagePathFileName_, thumbSize_),
        currentImagePathFileName_);

  }

}
