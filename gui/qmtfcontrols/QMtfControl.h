/*
 * QMtfControl.h
 *
 *  Created on: Dec 11, 2020
 *      Author: amyznikov
 */

#ifndef __QMtfControl_h__
#define __QMtfControl_h__

#include <core/mtf/c_pixinsight_mtf.h>
#include "QHistogramView.h"
#include "QMtfSlider.h"

class QMtfControl
    : public QWidget
{
  Q_OBJECT;
public:
  typedef QMtfControl ThisClass;
  typedef QWidget Base;

  QMtfControl(QWidget * parent = Q_NULLPTR);

  void setMtf(const c_pixinsight_mtf::sptr & mtf);
  const c_pixinsight_mtf::sptr & mtf() const;

  void setInputImage(cv::InputArray image, cv::InputArray mask);
  void setOutputImage(cv::InputArray image, cv::InputArray mask);

  bool isAutoMtfActionEnabled() const;

  bool updatingControls() const;
  void setUpdatingControls(bool v) ;

signals:
  void mtfChanged();

protected slots:
  void updateControls();
  void onResetMtfClicked();
  void onAutoMtfCtrlClicked();
  void onChartTypeSelectorClicked();
  void onDisplayChannelComboCurrentIndexChanged(int);

protected:
  void findAutoHistogramClips();
  void findAutoMidtonesBalance();


protected:
  c_pixinsight_mtf::sptr mtf_;

  enum AutoMtfAction {
    AutoMtfAction_AutoClip = 0,
    AutoMtfAction_AutoMtf = 1,
  } selectedAutoMtfAction_ = AutoMtfAction_AutoClip;


  QVBoxLayout * vbox_ = Q_NULLPTR;
  QToolBar * topToolbar_ = Q_NULLPTR;
  QComboBox * displayChannel_ctl = Q_NULLPTR;


  QAction * resetMtfAction_ = Q_NULLPTR;
  QToolButton * autoMtf_ctl = Q_NULLPTR;
  QMenu autoMtfMenu;

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
