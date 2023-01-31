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

class QStackingThread :
    public QThread
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


Q_SIGNALS:
  void stackingStageChanged(STACKING_STAGE oldstage, STACKING_STAGE newstage);
  void selectedMasterFrameChanged();
  void statusChanged();
  void accumulatorChanged();
  void finishing();

protected:

  class c_stacking_thread_impl:
      public c_image_stacking_pipeline
  {
    QStackingThread * qobj;

  public:
    typedef c_stacking_thread_impl this_class;
    typedef std::shared_ptr<this_class> ptr;

    c_stacking_thread_impl(QStackingThread * _qobj) :
        qobj(_qobj)
    {
    }

  protected:
    void emit_status_changed() const override
    {
      Q_EMIT qobj->statusChanged();
    }

    void emit_accumulator_changed() const override
    {
      Q_EMIT qobj->accumulatorChanged();
    }

    void emit_stacking_stage_changed(STACKING_STAGE oldstage, STACKING_STAGE newstage) const override
    {
      CF_DEBUG("%s -> %s", toString(oldstage), toString(newstage));
      Q_EMIT qobj->stackingStageChanged(oldstage, newstage);
    }

    void emit_selected_master_frame_changed() const override
    {
      Q_EMIT qobj->selectedMasterFrameChanged();
    }
  };

protected:
  QStackingThread();
  void run() override;

protected:
  c_image_stacking_options::ptr options_;
  c_stacking_thread_impl pipeline_;
};

Q_DECLARE_METATYPE(STACKING_STAGE);

#endif /* __QStackingThread_h__ */
