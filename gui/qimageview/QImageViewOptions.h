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
    public QSettingsWidgetTemplate<QImageViewer>
{
  Q_OBJECT;
public:
  typedef QImageViewOptions ThisClass;
  typedef QSettingsWidgetTemplate<QImageViewer> Base;
  typedef QEnumComboBox<QImageViewer::DisplayType> DisplayTypeCombo;

  QImageViewOptions(QWidget * parent = nullptr);

  void setImageViewer(QImageViewer * imageViewer);
  QImageViewer * imageViewer() const;

protected:
  void hideEvent(QHideEvent *event) final;

protected:
  DisplayTypeCombo * displayType_ctl = nullptr;
  QNumericBox * blendAlpha_ctl = nullptr;
  QCheckBox * transparentMask_ctl = nullptr;
  QCheckBox * keepMaskOnMaskEditMode_ctl = nullptr;
  QPenOptionsControl * penOptions_ctl = nullptr;
};


class QImageViewOptionsDlgBox :
    public QSettingsDialogBoxTemplate<QImageViewOptions>
{
  Q_OBJECT;
public:
  typedef QImageViewOptionsDlgBox ThisClass;
  typedef QSettingsDialogBoxTemplate<QImageViewOptions> Base;

  QImageViewOptionsDlgBox(QWidget * parent = nullptr);

  QImageViewOptions * viewOptions() const;

  void setImageViewer(QImageViewer * imageViewer);
  QImageViewer * imageViewer() const;
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
