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

  minumum_image_size_ctl =
      add_numeric_box<int>("minumum_image_size",
          "minumum_image_size for ecch",
          [this](int v) {
            if ( options_ && options_->minimum_image_size != v ) {
              options_->minimum_image_size = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( options_ ) {
              * v = options_->minimum_image_size;
              return true;
            }
            return false;
          });

  min_rho_ctl =
      add_numeric_box<double>("min rho",
          "minumum acceptable correlation",
          [this](double v) {
            if ( options_ && options_->min_rho != v ) {
              options_->min_rho = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( options_ ) {
              * v = options_->min_rho;
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
