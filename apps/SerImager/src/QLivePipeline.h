/*
 * QLivePipeline.h
 *
 *  Created on: Mar 20, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QLivePipeline_h__
#define __QLivePipeline_h__

#include <QtCore/QtCore>
#include "QVideoFrameDisplay.h"
#include "camera/QImagingCamera.h"

namespace serimager {

class QLivePipeline:
    public QObject
{
  Q_OBJECT;
public:
  QLivePipeline();
};


class QLivePipelineThread :
    public QThread
{
  Q_OBJECT;
public:
  typedef QLivePipelineThread ThisClass;
  typedef QThread Base;

  QLivePipelineThread(QObject * parent = nullptr);
  ~QLivePipelineThread();

  void finish(bool wait = true);

  void setCamera(const QImagingCamera::sptr & camera);
  const QImagingCamera::sptr& camera() const;

  void setDisplay(QVideoFrameDisplay * display);
  QVideoFrameDisplay* display() const;

  bool startPipeline(QLivePipeline *pipeline);
  QLivePipeline* currentPipeline() const;

Q_SIGNALS:
  void pipelineStarted(QLivePipeline *pipeline);
  void pipelineFinished(QLivePipeline *pipeline);

protected Q_SLOTS:
  void onCameraStateChanged(QImagingCamera::State oldState, QImagingCamera::State newState);

protected:
  void run() override;

protected:
  QImagingCamera::sptr camera_;
  QVideoFrameDisplay *display_ = nullptr;
  QLivePipeline * pipeline_ = nullptr;

  std::atomic_bool finish_ = false;
};



} /* namespace serimager */

#endif /* __QLivePipeline_h__ */
