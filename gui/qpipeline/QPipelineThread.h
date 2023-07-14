/*
 * QPipelineThread.h
 *
 *  Created on: Jul 2, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QPipelineThread_h__
#define __QPipelineThread_h__

#include "QImageProcessingPipeline.h"

class QPipelineThread:
    public QThread
{
Q_OBJECT;

public:

  typedef QPipelineThread ThisClass;
  typedef QThread Base;

  QPipelineThread(QObject * parent = nullptr);
  ~QPipelineThread();

  struct auto_lock
  {
    auto_lock()
    {
      ThisClass::lock();
    }
    ~auto_lock()
    {
      ThisClass::unlock();
    }
  };

  static ThisClass* instance();

  static bool start(const c_image_processing_pipeline::sptr & pipeline);
  static void cancel();
  static bool isRunning();
  static void lock();
  static void unlock();

  static const c_image_processing_pipeline::sptr & currentPipeline();

Q_SIGNALS:
  void starting();
  void finishing();

protected:
  QPipelineThread();
  void run() override;

protected:
  c_image_processing_pipeline::sptr currentPipeline_;
  static std::mutex lock_;
};

#endif /* __QPipelineThread_h__ */
