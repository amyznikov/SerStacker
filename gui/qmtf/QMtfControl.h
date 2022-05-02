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
#include <gui/widgets/QLineEditBox.h>
#include "QHistogramView.h"
#include "QMtfSlider.h"
#include "QMtfDisplaySettings.h"

class QMtfControl:
    public QWidget
{
  Q_OBJECT;
public:
  typedef QMtfControl ThisClass;
  typedef QWidget Base;

  QMtfControl(QWidget * parent = Q_NULLPTR);

  void setDisplaySettings(QMtfDisplaySettingsBase * displaySettings);
  QMtfDisplaySettingsBase * displaySettings() const;

  bool isAutoMtfActionEnabled() const;
  bool updatingControls() const;
  void setUpdatingControls(bool v) ;

protected slots:
  void updateControls();
  void updateHistogramLevels();
  void onChartTypeSelectorClicked();
  void onResetMtfClicked();
  void onAutoMtfCtrlClicked();
  void onColormapCtlClicked();
  void onDisplayTypeCurrentItemChanged();
  void onInputDataRangeChanged();

protected:
  void resizeEvent(QResizeEvent *event) override;
  void findAutoHistogramClips();
  void findAutoMidtonesBalance();
  void updateColormapStrip();
  void updateColormapPixmap();

protected:
  QMtfDisplaySettingsBase * displaySettings_ = Q_NULLPTR;

  QVBoxLayout * vbox_ = Q_NULLPTR;
  QToolBar * topToolbar_ = Q_NULLPTR;
  QComboBox * displayType_ctl = Q_NULLPTR;
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
      Q_NULLPTR };

  bool updatingControls_ = false;
};



class QMtfControlDialogBox:
    public QDialog
{
  Q_OBJECT;
public:
  typedef QMtfControlDialogBox ThisClass;
  typedef QDialog Base;

  QMtfControlDialogBox(QWidget * parent = Q_NULLPTR);

  void setMtfDisplaySettings(QMtfDisplaySettingsBase * display);
  QMtfDisplaySettingsBase * mtfDisplaySettings() const;

  signals:
  void visibilityChanged(bool visible);

protected:
  void showEvent(QShowEvent * event) override;
  void hideEvent(QHideEvent * event) override;

protected:
  QVBoxLayout *vbox_ = Q_NULLPTR;
  QMtfControl * widget = Q_NULLPTR;
  QSize lastWidnowSize_;
  QPoint lastWidnowPos_;
};
#endif /* __QMtfControl_h__ */
