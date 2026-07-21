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
#include <gui/qimageview/QImageEditor.h>
#include "QInputSourceView.h"

namespace serstacker {
///////////////////////////////////////////////////////////////////////////////

// QInputSourceView::showFrame(const c_data_frame::sptr & frame)
class QPipelineProgressView:
    public QFrame
{
  Q_OBJECT;
public:
  typedef QPipelineProgressView ThisClass;
  typedef QFrame Base;

  QPipelineProgressView(QWidget * parent = nullptr);

  void setImageView(QInputSourceView * view);
  QInputSourceView * imageView() const;

Q_SIGNALS:
  void progressTextChanged();

protected:
  void showEvent(QShowEvent *event) final;
  void timerEvent(QTimerEvent *event) final;
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
  QHBoxLayout * _layout = nullptr;
  QProgressStrip * _progressStrip = nullptr;
  QToolButton * _menuButton = nullptr;
  QInputSourceView * _imageView = nullptr;
  c_data_frame::sptr _dataframe;

  int timerId = 0;
  std::atomic<bool> _hasStatusUpdates = false;
  std::atomic<bool> _updatingDisplay = false;
};


} // namespace serstacker

#endif /* __QPipelineProgressView_h__ */
