/*
 * QImageStackingInputOptions.h
 *
 *  Created on: Jul 20, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QImageStackingInputOptions_h__
#define __QImageStackingInputOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/QEnumComboBox.h>
#include <gui/widgets/QLineEditBox.h>
#include <core/pipeline/c_image_stacking_pipeline.h>

class QImageStackingInputOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QImageStackingInputOptions ThisClass;
  typedef QSettingsWidget Base;

  QImageStackingInputOptions(QWidget * parent = Q_NULLPTR);

  void set_input_options(c_input_options * options);
  c_input_options * input_options() const;

  class QAnscombeMethodCombo :
      public QEnumComboBox<anscombe_method>
  {
  public:
    typedef QEnumComboBox<anscombe_method> Base;
    QAnscombeMethodCombo(QWidget * parent = Q_NULLPTR)
        : Base(parent, anscombe_methods)
      {}
  };

  static QString toString(enum anscombe_method v) {
    return QString::fromStdString(toStdString(v));
  }

  static enum anscombe_method fromString(const QString  & s, enum anscombe_method defval ) {
    return fromStdString(s.toStdString(), defval);
  }

signals:
  void applyInputOptionsToAllRequested(const c_input_options & options);

protected:
  void onupdatecontrols() override;

protected:
  c_input_options * options_ = Q_NULLPTR;

  QCheckBox * enable_remove_bad_pixels_ctl = Q_NULLPTR;
  QCheckBox * enable_color_maxtrix_ctl = Q_NULLPTR;
  QAnscombeMethodCombo * anscombe_ctl  = Q_NULLPTR;
  QToolButton * applyToAll_ctl = Q_NULLPTR;
};

#endif /* __QImageStackingInputOptions_h__ */
