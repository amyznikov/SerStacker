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
#include "QImageEditor.h"
#include <core/notification.h>

namespace qserstacker {
///////////////////////////////////////////////////////////////////////////////

class QPipelineProgressView:
    public QFrame
{
  Q_OBJECT;
public:
  typedef QPipelineProgressView ThisClass;
  typedef QFrame Base;

  QPipelineProgressView(QWidget * parent = nullptr);

  void setImageViewer(QImageEditor * imageViewer);
  QImageEditor * imageViewer() const;

Q_SIGNALS:
  void progressTextChanged();

protected:
  void showEvent(QShowEvent *event) override;
  void timerEvent(QTimerEvent *event) override;
  void updateAccumulatedImageDisplay(bool force = false);

protected Q_SLOTS:
  void onStackingThreadStarted();
  void onStackingThreadFinishing();
  void onStackingThreadFinished();
//  void onStatusChanged();
//  void onAccumulatorChanged();
//  void onSelectedMasterFrameChanged();

protected:
  QHBoxLayout * layout_ = nullptr;
  QProgressStrip * progressStrip_ctl = nullptr;
  QToolButton * menuButton_ctl = nullptr;
  QImageEditor * imageViewer_ = nullptr;

  int timerId = 0;
  std::atomic<bool> hasCurrentStatisticsUpdates_ = false;
  std::atomic<bool> accumuatorImageChanged_ = false;
  std::atomic<bool> selectedMasterFrameChanged_ = false;
  std::atomic<bool> updatingDisplay_ = false;

  c_slotptr on_status_changed;
  c_slotptr on_accumulator_changed;

  c_slotptr on_stacking_stage_changed;
  c_slotptr on_selected_master_frame_changed;

  c_slotptr on_pipeline_stage_changed;
  c_slotptr on_current_frame_changed;
};


} // namespace qserstacker

#endif /* __QPipelineProgressView_h__ */
