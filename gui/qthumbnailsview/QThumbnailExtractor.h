/*
 * QThumbnailExtractor.h
 *
 *  Created on: Mar 7, 2020
 *      Author: amyznikov
 */

#ifndef __QThumbnailExtractor_h__
#define __QThumbnailExtractor_h__

#include <QtGui/QtGui>

class QThumbnailExtractor :
    public QThread
{
  Q_OBJECT;
public:
  typedef QThumbnailExtractor ThisClass;
  typedef QThread Base;

  QThumbnailExtractor();
  ~QThumbnailExtractor();

public:
  void setThumbnailSize(int thumb_size);
  int thumbnailSize() const;

  QSize compute_thumbnail_size(const QSize & imageSize) const;

  int start(const QString & imagePathFileName);
  void cancel();
  bool canceled() const;

Q_SIGNALS:
  void extracted(int reqId, const QIcon & icon, const QString & iconPathFileName);

private:
  void run() override;

private:
  QMutex _mtx;
  QString _currentImagePathFileName;
  int _thumbSize = 256;
  int _reqId = -1;
  bool _cancelRequested = false;
};

#endif /* __QThumbnailExtractor_h__ */
