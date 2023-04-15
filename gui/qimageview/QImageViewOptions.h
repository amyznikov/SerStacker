/*
 * QImageViewOptions.h
 *
 *  Created on: Jul 1, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QImageViewModeOptions_h__
#define __QImageViewModeOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include "QImageViewer.h"

class QPenOptionsControl;

class QImageViewOptions:
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QImageViewOptions ThisClass;
  typedef QSettingsWidget Base;
  typedef QEnumComboBox<QImageViewer::DisplayType> DisplayTypeCombo;

  QImageViewOptions(QWidget * parent = nullptr);

  void setImageViewer(QImageViewer * imageViewer);
  QImageViewer * imageViewer() const;

protected:
  void onupdatecontrols() override;
  void hideEvent(QHideEvent *event) override;

protected:
  QImageViewer * imageViewer_ = nullptr;
  DisplayTypeCombo * displayType_ctl = nullptr;
  QNumericBox * blendAlpha_ctl = nullptr;
  QCheckBox * transparentMask_ctl = nullptr;
  QPenOptionsControl * penOptions_ctl = nullptr;
};


class QImageViewOptionsDlgBox :
    public QDialog
{
  Q_OBJECT;
public:
  typedef QImageViewOptionsDlgBox ThisClass;
  typedef QDialog Base;

  QImageViewOptionsDlgBox(QWidget * parent = nullptr);

  QImageViewOptions * viewOptions() const;

  void setImageViewer(QImageViewer * imageViewer);
  QImageViewer * imageViewer() const;

Q_SIGNALS:
  void visibilityChanged(bool visible);

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

protected:
  QImageViewOptions * viewOptions_ctl = nullptr;
};

class QPenOptionsControl :
    public QToolBar
{
  Q_OBJECT;
public:
  typedef QPenOptionsControl ThisClass;
  typedef QToolBar Base;

  QPenOptionsControl(QWidget * parent = nullptr);

  void setEnableEditMask(bool enable);
  bool enableEditMask() const;

  void setEditMaskPenRadius(int v);
  int editMaskPenRadius() const;

  void setEditMaskPenShape(QImageViewer::PenShape v);
  QImageViewer::PenShape editMaskPenShape() const;

signals:
  void enableEditMaskChanged();
  void editMaskPenRadiusChanged();
  void editMaskPenShapeChanged();

protected:
  QToolButton * enableEdit_ctl = nullptr;
  QComboBox * penShape_ctl = nullptr;
  QComboBox * penSize_ctl = nullptr;
  bool updatingControls_ = false;
};

#endif /* __QImageViewModeOptions_h__ */
