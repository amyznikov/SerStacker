/*
 * QUnsharpMaskSettings.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "QUnsharpMaskSettings.h"


const QUnsharpMaskSettings::ClassFactory QUnsharpMaskSettings::classFactory;


QUnsharpMaskSettings::QUnsharpMaskSettings(const c_unsharp_mask_routine::ptr & routine, QWidget * parent) :
    Base(&classFactory, routine, parent)
{

  sigma_ctl = add_numeric_box("sigma",
      &c_unsharp_mask_routine::sigma,
      &c_unsharp_mask_routine::set_sigma);

  alpha_ctl = add_numeric_box("alpha",
      &c_unsharp_mask_routine::alpha,
      &c_unsharp_mask_routine::set_alpha);

  updateControls();
}

void QUnsharpMaskSettings::onupdatecontrols()
{
  if ( !routine_ ) {
    setEnabled(false);
  }
  else {
    sigma_ctl->setValue(routine_->sigma());
    alpha_ctl->setValue(routine_->alpha());

    setEnabled(true);
  }
  Base::onupdatecontrols();
}

