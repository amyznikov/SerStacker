/*
 * QMtfControl.h
 *
 *  Created on: Dec 11, 2020
 *      Author: amyznikov
 */

#ifndef __QMtfControl_h__
#define __QMtfControl_h__

#include "QHistogramView.h"
#include "QMtfSlider.h"
#include <core/mtf/c_pixinsight_midtones_transfer_function.h>

class QMtfControl
    : public QWidget
{
  Q_OBJECT;
public:
  typedef QMtfControl ThisClass;
  typedef QWidget Base;

  QMtfControl(QWidget * parent = Q_NULLPTR);

  void setMtf(const c_pixinsight_midtones_transfer_function::ptr & mtf);
  const c_pixinsight_midtones_transfer_function::ptr & mtf() const;

  void setInputImage(cv::InputArray image, cv::InputArray mask);
  void setOutputImage(cv::InputArray image, cv::InputArray mask);

signals:
  void mtfChanged();

protected slots:
  void updateControls();
  void onResetMtfClicked();
  void onAutoClipClicked();
  void onFindAutoMidtonesBalanceClicked();
  void onChartTypeSelectorClicked();
  void onDisplayChannelComboCurrentIndexChanged(int);



protected:
  c_pixinsight_midtones_transfer_function::ptr mtf_;

  QVBoxLayout * vbox_ = Q_NULLPTR;
  QToolBar * topToolbar_ = Q_NULLPTR;
  QComboBox * displayChannel_ctl = Q_NULLPTR;

  QAction * resetMtfAction_ = Q_NULLPTR;
  QAction * autoClipAction_ = Q_NULLPTR;
  QAction * autoMtfAction_ = Q_NULLPTR;
  QAction * logScaleSelectionAction_ = Q_NULLPTR;
  QToolButton * chartTypeSelectorButton_ = Q_NULLPTR;

  QHistogramView * levelsView_ = Q_NULLPTR;
  QMtfSlider * mtfSlider_ = Q_NULLPTR;

  QToolBar * bottomToolbar_ = Q_NULLPTR;

  enum SPINID {
    SPIN_SHADOWS = 0,
    SPIN_MIDTONES = 1,
    SPIN_HIGHLIGHTS = 2,
  };

  QDoubleSpinBox * spins[3] = {
      Q_NULLPTR };

  cv::Mat inputImage_;
  cv::Mat inputMask_;

  bool updatingControls_ = false;

};

#endif /* __QMtfControl_h__ */
