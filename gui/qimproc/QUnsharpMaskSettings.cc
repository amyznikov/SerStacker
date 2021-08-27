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


  outmin_ctl = add_numeric_box("outmin",
      &c_unsharp_mask_routine::outmin,
      &c_unsharp_mask_routine::set_outmin);

  outmax_ctl = add_numeric_box("outmax",
      &c_unsharp_mask_routine::outmax,
      &c_unsharp_mask_routine::set_outmax);

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
    outmin_ctl->setValue(routine_->outmin());
    outmax_ctl->setValue(routine_->outmax());

    setEnabled(true);
  }
  Base::onupdatecontrols();
}

