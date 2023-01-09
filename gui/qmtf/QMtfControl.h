/*
 * QMtfControl.h
 *
 *  Created on: Apr 19, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMtfControl_h__
#define __QMtfControl_h__

#include <QtWidgets/QtWidgets>
#include <gui/widgets/UpdateControls.h>
#include <gui/widgets/QLineEditBox.h>
#include "QHistogramView.h"
#include "QMtfDisplay.h"
#include "QMtfSlider.h"

class QMtfControl:
    public QWidget,
    public HasUpdateControls
{
  Q_OBJECT;
public:
  typedef QMtfControl ThisClass;
  typedef QWidget Base;

  QMtfControl(QWidget * parent = nullptr);

  void setDisplaySettings(QMtfDisplay* display);
  QMtfDisplay * displaySettings() const;

  bool isAutoMtfActionEnabled() const;

  void setHistoramViewSizeHint(const QSize & s);
  QSize historamViewSizeHint() const;

  //  bool updatingControls() const;
  //  void setUpdatingControls(bool v) ;

protected Q_SLOTS:
  //void updateControls();
  void updateHistogramLevels();
  void onChartTypeSelectorClicked();
  void onResetMtfClicked();
  void onAutoMtfCtrlClicked();
  void onColormapCtlClicked();
  void onDisplayTypeCurrentItemChanged();
  void onInputDataRangeChanged();

protected:
  void onupdatecontrols() override;
  void resizeEvent(QResizeEvent *event) override;
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
  void findAutoHistogramClips();
  void findAutoMidtonesBalance();
  void updateColormapStrip();
  void updateColormapPixmap();

protected:
  QMtfDisplay * displaySettings_ = nullptr;

  QVBoxLayout * vbox_ = nullptr;
  QToolBar * topToolbar_ctl = nullptr;
  QComboBox * displayType_ctl = nullptr;
  QNumberEditBox * inputDataRange_ctl = nullptr;
  QToolButton * colormap_ctl = nullptr;

  QAction * resetMtfAction_ = nullptr;
  QToolButton * autoMtf_ctl = nullptr;
  QMenu autoMtfMenu;

  QAction * logScaleSelectionAction_ = nullptr;
  QToolButton * chartTypeSelectorButton_ = nullptr;

  QHistogramView * levelsView_ctl = nullptr;
  QMtfSlider * mtfSlider_ctl = nullptr;
  QLabel * colormap_strip_ctl = nullptr;
  QPixmap colormap_pixmap_;

  QToolBar * bottomToolbar_ctl = nullptr;

  enum AutoMtfAction {
    AutoMtfAction_AutoClip = 0,
    AutoMtfAction_AutoMtf = 1,
  } selectedAutoMtfAction_ = AutoMtfAction_AutoClip;

  enum SPINID {
    SPIN_SHADOWS = 0,
    SPIN_MIDTONES = 1,
    SPIN_HIGHLIGHTS = 2,
  };

  QDoubleSpinBox * spins[3] = {
      nullptr };

  // bool updatingControls_ = false;
};



class QMtfControlDialogBox:
    public QDialog
{
  Q_OBJECT;
public:
  typedef QMtfControlDialogBox ThisClass;
  typedef QDialog Base;

  QMtfControlDialogBox(QWidget * parent = nullptr);

  QMtfControl * mtfControl() const;

  void setMtfDisplaySettings(QMtfDisplay * display);
  QMtfDisplay * mtfDisplaySettings() const;

signals:
  void visibilityChanged(bool visible);

protected:
  void showEvent(QShowEvent * event) override;
  void hideEvent(QHideEvent * event) override;

protected:
  QVBoxLayout *vbox_ = nullptr;
  QMtfControl * mtfControl_ = nullptr;
  QSize lastWidnowSize_;
  QPoint lastWidnowPos_;
};
#endif /* __QMtfControl_h__ */
