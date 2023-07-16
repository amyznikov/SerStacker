/*
 * QLiveStackingRegistrationOptions.cc
 *
 *  Created on: Jul 16, 2023
 *      Author: amyznikov
 */

#include "QLiveStackingRegistrationOptions.h"


QLiveStackingRegistrationOptions::QLiveStackingRegistrationOptions(QWidget * parent) :
  Base("", parent)
{

  enable_image_registration_ctl =
      add_checkbox("Enable registration:",
          "Set checked to enable image registration",
          [this](bool checked) {
            if ( options_ && options_->enabled != checked ) {
              options_->enabled = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * v) {
            if ( options_ ) {
              * v = options_->enabled;
              return true;
            }
            return false;
          });


  updateControls();
}

void QLiveStackingRegistrationOptions::set_options(c_live_stacking_registration_options * options)
{
  options_ = options;
  updateControls();
}

c_live_stacking_registration_options * QLiveStackingRegistrationOptions::options() const
{
  return options_;
}

void QLiveStackingRegistrationOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {
    Base::populatecontrols();
    setEnabled(true);
  }
}
