/*
 * QStackingThread.h
 *
 *  Created on: Feb 8, 2021
 *      Author: amyznikov
 */

#ifndef __QStackingThread_h__
#define __QStackingThread_h__

#include <QtCore/QtCore>
#include <core/pipeline/c_image_stacking_pipeline.h>

class QStackingThread
    : public QThread
{
  Q_OBJECT;
public:
  typedef QStackingThread ThisClass;
  typedef QThread Base;

  struct auto_lock {
    auto_lock() {
      QStackingThread::lock();
    }
    ~auto_lock() {
      QStackingThread::unlock();
    }
  };

  static QStackingThread * singleton();

  static bool start(const c_image_stacking_options::ptr pipeline);
  static void cancel();
  static bool isRunning();

  static void lock();
  static void unlock();

  static c_image_stacking_pipeline * pipeline();
  static const c_image_stacking_options::ptr & currentStack();


signals:
  void accumulatorChanged();
  void statusChanged();
  void finishing();

protected:

  class c_stacking_thread_impl: public c_image_stacking_pipeline {
    QStackingThread * qobj;
  public:
    typedef c_stacking_thread_impl this_class;
    typedef std::shared_ptr<this_class> ptr;

    c_stacking_thread_impl(QStackingThread * qobj);

protected:
    void emit_status_changed() const override {
      emit qobj->statusChanged();
    }
    void emit_accumulator_changed() const override {
      emit qobj->accumulatorChanged();
    }
  };

protected:
  QStackingThread();
  void run() override;

protected:
  c_image_stacking_options::ptr options_;
  c_stacking_thread_impl pipeline_;
};


#endif /* __QStackingThread_h__ */
