/*
 * QMtfControl.h
 *
 *  Created on: Dec 11, 2020
 *      Author: amyznikov
 */

#ifndef __QMtfControl_h__
#define __QMtfControl_h__

#include "QMtfDisplayFunction.h"
#include "QHistogramView.h"
#include "QMtfSlider.h"
#include <gui/widgets/QLineEditBox.h>
#include <gui/widgets/QEnumComboBox.h>

class QMtfControl
    : public QWidget
{
  Q_OBJECT;
public:
  typedef QMtfControl ThisClass;
  typedef QWidget Base;

  QMtfControl(QWidget * parent = Q_NULLPTR);

  void setDisplayFunction(QMtfDisplayFunction * displayFunction);
  QMtfDisplayFunction * displayFunction() const;

  void setInputImage(cv::InputArray image, cv::InputArray mask, bool make_copy = false);

  bool isAutoMtfActionEnabled() const;

  bool updatingControls() const;
  void setUpdatingControls(bool v) ;

  void setOutputHistogram(const cv::Mat1f & H, double hmin, double hmax);

public slots:
  void updateControls();
  void updateOutputHistogram();

protected slots:
  void onInputDataRangeChanged();
  void onColormapCtlClicked();
  void onResetMtfClicked();
  void onAutoMtfCtrlClicked();
  void onChartTypeSelectorClicked();
  void onDisplayChannelComboCurrentIndexChanged(int);

protected:
  void resizeEvent(QResizeEvent *event) override;
  void findAutoHistogramClips();
  void findAutoMidtonesBalance();
  void updateColormapPixmap();
  void updateColormapStrip();


protected:
  QMtfDisplayFunction * displayFunction_ = Q_NULLPTR;

  enum AutoMtfAction {
    AutoMtfAction_AutoClip = 0,
    AutoMtfAction_AutoMtf = 1,
  } selectedAutoMtfAction_ = AutoMtfAction_AutoClip;


  QVBoxLayout * vbox_ = Q_NULLPTR;
  QToolBar * topToolbar_ = Q_NULLPTR;
  QComboBox * displayChannel_ctl = Q_NULLPTR;
  QNumberEditBox * inputDataRange_ctl = Q_NULLPTR;
  QToolButton * colormap_ctl = Q_NULLPTR;


  QAction * resetMtfAction_ = Q_NULLPTR;
  QToolButton * autoMtf_ctl = Q_NULLPTR;
  QMenu autoMtfMenu;

  QAction * logScaleSelectionAction_ = Q_NULLPTR;
  QToolButton * chartTypeSelectorButton_ = Q_NULLPTR;

  QHistogramView * levelsView_ = Q_NULLPTR;
  QMtfSlider * mtfSlider_ = Q_NULLPTR;
  QLabel * colormap_strip_ctl = Q_NULLPTR;
  QPixmap colormap_pixmap_;

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
