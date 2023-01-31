/*
 * QStackProgressView.h
 *
 *  Created on: Jan 31, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStackProgressView_h__
#define __QStackProgressView_h__

#include <QtWidgets/QtWidgets>
#include <gui/widgets/QProgressStrip.h>
#include "QImageEditor.h"
//#include <core/improc/c_image_processor.h>

namespace qserstacker {
///////////////////////////////////////////////////////////////////////////////

class QStackProgressView:
    public QFrame
{
  Q_OBJECT;
public:
  typedef QStackProgressView ThisClass;
  typedef QFrame Base;

  QStackProgressView(QWidget * parent = nullptr);

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
  void onStatusChanged();
  void onAccumulatorChanged();
  void onSelectedMasterFrameChanged();

protected:
  QHBoxLayout * layout_ = nullptr;
  QProgressStrip * progressStrip_ctl = nullptr;
  QToolButton * menuButton_ctl = nullptr;
  QImageEditor * imageViewer_ = nullptr;

  int timerId = 0;
  bool hasCurrentStatisticsUpdates_ = false;
  bool accumuatorImageChanged_ = false;
  bool selectedMasterFrameChanged_ = false;
  bool updatingDisplay_ = false;
};


} // namespace qserstacker

#endif /* __QStackProgressView_h__ */
