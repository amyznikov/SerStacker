/*
 * QSearchImageFiles.h
 *
 *  Created on: Jul 12, 2021
 *      Author: amyznikov
 */

#ifndef __QSearchImageFiles_h__
#define __QSearchImageFiles_h__

#include <QtCore/QtCore>

class QSearchImageFiles :
    public QThread
{
  Q_OBJECT;
public:
  typedef QSearchImageFiles ThisClass;
  typedef QThread Base;

  QSearchImageFiles();
  ~QSearchImageFiles();

public:
  int start(const QString & directory);
  void cancel();
  bool canceled() const;

Q_SIGNALS:
  void imageFound(int reqId, const QString & imagePathFileName);

private:
  void run() override;

private:
  QMutex mtx_;
  QString current_path_;
  QStringList supported_suffixes_;
  int req_id_ = -1;
  volatile bool cancel_requested_ = false;
};

#endif /* __QSearchImageFiles_h__ */
