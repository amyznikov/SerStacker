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
#include <core/histogram/c_pixinsight_midtones_transfer_function.h>

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

  void setImage(cv::InputArray image);

signals:
  void mtfChanged();

protected:
  void updateControls();
  void updateHistogramLevelsView();
  void findAutoMidtonesBalance();
  void onChartTypeSelectorClicked();



protected:
  c_pixinsight_midtones_transfer_function::ptr mtf_;

  QVBoxLayout * vbox_ = Q_NULLPTR;
  QToolBar * topToolbar_ = Q_NULLPTR;
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

  cv::Mat1f imageHistogram;

  bool updatingControls_ = false;

};

#endif /* __QMtfControl_h__ */
