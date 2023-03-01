/*
 * QImageProcessingPipeline.h
 *
 *  Created on: Feb 8, 2021
 *      Author: amyznikov
 */

#ifndef __QImageProcessingPipeline_h__
#define __QImageProcessingPipeline_h__

#include <QtCore/QtCore>
#include <core/pipeline/c_image_processing_pipeline.h>


class QImageProcessingPipeline :
    public QThread
{
  Q_OBJECT;
public:
  typedef QImageProcessingPipeline ThisClass;
  typedef QThread Base;

  struct auto_lock {
    auto_lock() {
      QImageProcessingPipeline::lock();
    }
    ~auto_lock() {
      QImageProcessingPipeline::unlock();
    }
  };

  static QImageProcessingPipeline * singleton();

  static bool start(const c_image_sequence::sptr & image_sequence);
  static void cancel();
  static bool isRunning();
  static void lock();
  static void unlock();

  static c_image_sequence::sptr current_sequence();
  static c_image_processing_pipeline::sptr current_pipeline();

Q_SIGNALS:
  void starting();
  void finishing();

protected:
  QImageProcessingPipeline();
  void run() override;

protected:
  c_image_sequence::sptr current_sequence_;
  c_image_processing_pipeline::sptr current_pipeline_;
};

#endif /* __QImageProcessingPipeline_h__ */
