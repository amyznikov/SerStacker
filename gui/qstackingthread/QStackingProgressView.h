/*
 * QStackingProgressView.h
 *
 *  Created on: Feb 19, 2021
 *      Author: amyznikov
 */

#ifndef __QStackingProgressView_h__
#define __QStackingProgressView_h__

#include <QtWidgets/QtWidgets>
#include <gui/qimageview/QImageEditor.h>
#include <core/improc/c_image_processor.h>

class QStackingProgressView
    : public QDialog
{
  Q_OBJECT;
public:
  typedef QStackingProgressView ThisClass;
  typedef QDialog Base;

  QStackingProgressView(QWidget * parent = Q_NULLPTR);

  void setImageViewer(QImageEditor * imageViewer);
  QImageEditor * imageViewer() const;

//public slots:
//  void updateCurrentImage();

protected slots:
  void onStackingThreadStarted();
  void onStackingThreadFinishing();
  void onStackingThreadFinished();
  void onFrameProcessed();
  void onArameAccumulated();

protected:
  void showEvent(QShowEvent *event) override;
  //void hideEvent(QHideEvent *event) override;
  void timerEvent(QTimerEvent *event) override;
  void updateAccumulatedImageDisplay(bool force = false);

protected:
  QVBoxLayout * layout_ = Q_NULLPTR;
  QLabel * progressLabel_ = Q_NULLPTR;
  QImageEditor * imageViewer_ = Q_NULLPTR;
  //  cv::Mat currentImage_;
  //  cv::Mat currentMask_;
  int timerId = 0;
  bool hasCurrentStatisticsUpdates_ = false;
  bool hasCurrentImageUpdates_ = false;
  bool updatingDisplay_ = false;

  //c_image_processor_chain::ptr accumulatedImageProcessor_;

};

#endif /* __QStackingProgressView_h__ */
