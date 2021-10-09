/*
 * QGaussianFilterSettings.cc
 *
 *  Created on: Oct 9, 2021
 *      Author: amyznikov
 */

#include "QGaussianFilterSettings.h"

const QGaussianFilterSettings::ClassFactory QGaussianFilterSettings::classFactory;

QGaussianFilterSettings::QGaussianFilterSettings(const c_gaussian_filter_routine::ptr & routine, QWidget * parent)
  : Base(&classFactory, routine, parent)
{
  sigma_ctl = add_numeric_box("sigma [px]:",
      &c_gaussian_filter_routine::sigma,
      &c_gaussian_filter_routine::set_sigma);

  updateControls();
}

void QGaussianFilterSettings::onupdatecontrols()
{
  if ( !routine_ ) {
    setEnabled(false);
  }
  else {
    sigma_ctl->setValue(routine_->sigma());
    setEnabled(true);
  }
  Base::onupdatecontrols();
}


