/*
 * QDSOThread.h
 *
 *  Created on: Jun 13, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __QDSOThread_h__
#define __QDSOThread_h__

#include <QtCore/QtCore>
#include <dso/FullSystem.h>
#include <dso/c_dso_dataset_reader.h>

namespace qdso {

class QDSOThread:
    public QThread
{
  Q_OBJECT;
public:
  typedef QDSOThread ThisClass;
  typedef QThread Base;
  using c_dso_display = dso::c_dso_display ;
  using FullSystem = dso::FullSystem;

  QDSOThread(QObject * parent = nullptr);

  bool start(c_dso_dataset_reader * dataset,
      c_dso_display * display);

  void stop();

protected:
  void run() override;

protected:
  c_dso_dataset_reader * dataset = nullptr;
  c_dso_display * display = nullptr;
  volatile bool stop_ = false;
};

} /* namespace qdso */

#endif /* __QDSOThread_h__ */
