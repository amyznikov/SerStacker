/*
 * QInputOptions.h
 *
 *  Created on: Dec 15, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QInputOptions_h__
#define __QInputOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/io/c_input_options.h>

class QVideoInputOptions :
    public QSettingsWidgetTemplate<c_video_input_options>
{
  Q_OBJECT;
public:
  typedef QVideoInputOptions ThisClass;
  typedef QSettingsWidgetTemplate<c_video_input_options> Base;

  QVideoInputOptions(QWidget * parent = nullptr);

protected:
  QEnumComboBox<DEBAYER_ALGORITHM> * debayer_ctl = nullptr;
  QCheckBox * enable_color_maxtrix_ctl = nullptr;
  QCheckBox * filter_bad_pixels_ctl = nullptr;
  QNumericBox * bad_pixels_variation_threshold_ctl = nullptr;

  //  QBrowsePathCombo * darkframe_ctl = nullptr;
  //  QNumericBox * darkFrameScale_ctl  = nullptr;
};

class QInputOptions :
    public QSettingsWidgetTemplate<c_input_options>
{
  Q_OBJECT;
public:
  typedef QInputOptions ThisClass;
  typedef QSettingsWidgetTemplate<c_input_options> Base;

  QInputOptions(QWidget * parent = nullptr);

  void set_options(c_input_options * options) override;

protected:
  QTabWidget * tab_ctl = nullptr;
  QVideoInputOptions * videoOptions_ctl = nullptr;
};


class QInputOptionsDialogBox :
    public QDialog
{
  Q_OBJECT;
public:
  typedef QInputOptionsDialogBox ThisClass;
  typedef QDialog Base;

  QInputOptionsDialogBox(QWidget * parent = nullptr);

  void setInputOptions(c_input_options * options);
  c_input_options * inputOptions() const;

Q_SIGNALS:
  void visibilityChanged(bool visible);
  void parameterChanged();

protected:
  void showEvent(QShowEvent * e) override;
  void hideEvent(QHideEvent * e) override;

protected:
  QInputOptions * inputOptions_ctl = nullptr;
};


#endif /* __QInputOptions_h__ */
