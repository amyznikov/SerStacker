/*
 * QThumbnailExtractor.h
 *
 *  Created on: Mar 7, 2020
 *      Author: amyznikov
 */

#ifndef __QThumbnailExtractor_h__
#define __QThumbnailExtractor_h__

#include <QtGui/QtGui>

class QThumbnailExtractor
    : public QThread
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

  int start(const QString & imagePathFileName);
  void cancel();
  bool canceled() const;

signals:
  void extracted(int reqId, const QIcon & icon, const QString & iconPathFileName);

private:
  void run() override;

private:
  QMutex mtx_;
  QString currentImagePathFileName_;
  int thumbSize_ = 256;
  int reqId_ = -1;
  bool cancelRequested_ = false;
};

#endif /* __QThumbnailExtractor_h__ */
