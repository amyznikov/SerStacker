/*
 * QStereoBMOptions.cc
 *
 *  Created on: Mar 27, 2023
 *      Author: amyznikov
 */

#include "QStereoBMOptions.h"

QStereoBMOptions::QStereoBMOptions(QWidget * parent) :
  Base("", parent)
{
}

void QStereoBMOptions::set_options(c_cvStereoBM_options * options)
{
  options_ = options;
  updateControls();
}

c_cvStereoBM_options * QStereoBMOptions::options() const
{
  return options_;
}

void QStereoBMOptions::onupdatecontrols()
{
}
