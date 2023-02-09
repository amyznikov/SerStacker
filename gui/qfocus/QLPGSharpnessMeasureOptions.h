/*
 * QLPGSharpnessMeasureOptions.h
 *
 *  Created on: Feb 9, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QLPGSharpnessMeasureOptions_h__
#define __QLPGSharpnessMeasureOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/proc/focus.h>

class QLPGSharpnessMeasureOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QLPGSharpnessMeasureOptions ThisClass;
  typedef QSettingsWidget Base;

  QLPGSharpnessMeasureOptions(QWidget * parent = nullptr);
  QLPGSharpnessMeasureOptions(const QString & prefix, QWidget * parent = nullptr);

  void set_measure_options(c_lpg_sharpness_measure * options);
  const c_lpg_sharpness_measure * measure_options() const;

protected:
  void onupdatecontrols() override;
  void onload(QSettings & settings) override;

protected:
  c_lpg_sharpness_measure * options_ = nullptr;

  QCheckBox * avgc_ctl = nullptr;
  QNumberEditBox * lw_ctl = nullptr;
  QNumberEditBox * k_ctl = nullptr;
  QNumberEditBox * dscale_ctl = nullptr;
  QNumberEditBox * uscale_ctl = nullptr;
  QCheckBox * square_ctl = nullptr;

};



#endif /* __QLPGSharpnessMeasureOptions_h__ */
