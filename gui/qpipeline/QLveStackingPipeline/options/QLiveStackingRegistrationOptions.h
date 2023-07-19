/*
 * QLiveStackingRegistrationOptions.h
 *
 *  Created on: Jul 16, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QLiveStackingRegistrationOptions_h__
#define __QLiveStackingRegistrationOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/pipeline/c_live_stacking_pipeline/c_live_stacking_pipeline.h>

class QLiveStackingRegistrationOptions :
    public QSettingsWidget
{
public:
  typedef QLiveStackingRegistrationOptions ThisClass;
  typedef QSettingsWidget Base;

  QLiveStackingRegistrationOptions(QWidget * parent = nullptr);

  void set_options(c_live_stacking_registration_options * options);
  c_live_stacking_registration_options * options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_live_stacking_registration_options * options_ = nullptr;
  QCheckBox * enable_image_registration_ctl = nullptr;
  QNumericBox * minumum_image_size_ctl = nullptr;
  QNumericBox * min_rho_ctl = nullptr;
};

#endif /* __QLiveStackingRegistrationOptions_h__ */
