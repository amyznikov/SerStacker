/*
 * QPipelineProgressView.h
 *
 *  Created on: Jan 31, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QPipelineProgressView_h__
#define __QPipelineProgressView_h__

#include <QtWidgets/QtWidgets>
#include <gui/widgets/QProgressStrip.h>
#include <core/notification.h>
#include "QSerStackerImageEditor.h"

namespace serstacker {
///////////////////////////////////////////////////////////////////////////////

class QPipelineProgressView:
    public QFrame
{
  Q_OBJECT;
public:
  typedef QPipelineProgressView ThisClass;
  typedef QFrame Base;

  QPipelineProgressView(QWidget * parent = nullptr);

  void setImageViewer(QSerStackerImageEditor * imageViewer);
  QSerStackerImageEditor * imageViewer() const;

Q_SIGNALS:
  void progressTextChanged();

protected:
  void showEvent(QShowEvent *event) override;
  void timerEvent(QTimerEvent *event) override;
  void updateAccumulatedImageDisplay(bool force = false);

protected Q_SLOTS:
  void onPipelineThreadStarted();
  void onPipelineThreadFinishing();
  void onPipelineThreadFinished();
  void onMenuButtonClicked();
//  void onStatusChanged();
//  void onAccumulatorChanged();
//  void onSelectedMasterFrameChanged();

protected:
  QHBoxLayout * layout_ = nullptr;
  QProgressStrip * progressStrip_ = nullptr;
  QToolButton * menuButton_ = nullptr;
  QSerStackerImageEditor * imageViewer_ = nullptr;

  int timerId = 0;
  std::atomic<bool> hasStatusUpdates_ = false;
  std::atomic<bool> updatingDisplay_ = false;
  // c_slotptr on_status_update;
};


} // namespace serstacker

#endif /* __QPipelineProgressView_h__ */
