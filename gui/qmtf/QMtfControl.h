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
#include "QMtfSliderBand.h"
#include "QMtfDisplay.h"

class QMtfControl:
    public QWidget,
    public HasUpdateControls
{
  Q_OBJECT;
public:
  typedef QMtfControl ThisClass;
  typedef QWidget Base;

  QMtfControl(QWidget * parent = nullptr);

  void setDisplaySettings(IMtfDisplay* display);
  IMtfDisplay * displaySettings() const;

  //bool isAutoMtfActionEnabled() const;

  void setHistoramViewSizeHint(const QSize & s);
  QSize historamViewSizeHint() const;

protected Q_SLOTS:
  //void updateControls();
  void updateHistogramLevels();
  void onChartTypeSelectorClicked();
  void onResetMtfClicked();
  void onAutoMtfCtrlClicked();
  void onColormapCtlClicked();
  void onDisplayChannelCurrentItemChanged();
  void onInputDataRangeChanged();
  void onDisplayChannelCustomContextMenuRequested(const QPoint &pos);
  void onDisplayChannelsChanged();
  void onDisplayImageChanged();

protected:
  void onupdatecontrols() override;
  void resizeEvent(QResizeEvent *event) override;
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
  void findAutoHistogramClips();
  void findAutoMidtonesBalance();
  void updateColormapStrip();
  void updateColormapPixmap();
  bool getInputDataRangeCtl(double range[2]) const;
  void setInputDataRangeCtl(const double range[2]);

protected:
  IMtfDisplay * _displaySettings = nullptr;

  QVBoxLayout * _vbox = nullptr;
  QToolBar * topToolbar_ctl = nullptr;
  QComboBox * displayChannel_ctl = nullptr;
  QNumericBox * inputDataRange_ctl = nullptr;
  QToolButton * colormap_ctl = nullptr;

  QAction * _resetMtfAction = nullptr;
  QAction * _autoMtffAction = nullptr;
  //QToolButton * autoMtf_ctl = nullptr;
  //QMenu autoMtfMenu;

  QAction * logScaleSelectionAction_ = nullptr;
  QToolButton * chartTypeSelectorButton_ = nullptr;

  QHistogramView * levelsView_ctl = nullptr;
  QMtfSliderBand * mtfSliderBand_ctl = nullptr;
  QLabel * colormap_strip_ctl = nullptr;
  QPixmap _colormap_pixmap;

  QToolBar * bottomToolbar_ctl = nullptr;

//  enum AutoMtfAction {
//    AutoMtfAction_AutoClip = 0,
//    AutoMtfAction_AutoMtf = 1,
//  } selectedAutoMtfAction_ = AutoMtfAction_AutoClip;

  QDoubleSpinBox * lclip_ctl = nullptr;
  QDoubleSpinBox * hclip_ctl = nullptr;
  QDoubleSpinBox * midtones_ctl = nullptr;
  QDoubleSpinBox * shadows_ctl = nullptr;
  QDoubleSpinBox * highlights_ctl = nullptr;

  QStatusBar * statusBar_ctl = nullptr;
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

  void setMtfDisplaySettings(IMtfDisplay * display);
  IMtfDisplay * mtfDisplaySettings() const;

signals:
  void visibilityChanged(bool visible);

protected:
  void showEvent(QShowEvent * event) override;
  void hideEvent(QHideEvent * event) override;

protected:
  QVBoxLayout *_vbox = nullptr;
  QMtfControl * _mtfControl = nullptr;
  QSize _lastWidnowSize;
  QPoint _lastWidnowPos;
};
#endif /* __QMtfControl_h__ */
