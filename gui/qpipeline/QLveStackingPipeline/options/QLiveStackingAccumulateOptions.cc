/*
 * QLiveStackingAccumulateOptions.cc
 *
 *  Created on: Jul 16, 2023
 *      Author: amyznikov
 */

#include "QLiveStackingAccumulateOptions.h"

QLiveStackingAccumulateOptions::QLiveStackingAccumulateOptions(QWidget * parent) :
  Base("", parent)
{

  accumulation_type_ctl =
      add_enum_combobox<live_stacking_accumulation_type>("accumulation_type:",
          "Specify accumulation method",
          [this](live_stacking_accumulation_type v) {
            if ( options_ && options_->accumulation_type != v ) {
              options_->accumulation_type = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](live_stacking_accumulation_type * v) {
            if ( options_ ) {
              * v = options_->accumulation_type;
              return true;
            }
            return false;
          });



  ignore_input_mask_ctl =
      add_checkbox("ignore_input_mask",
          "ignore input mask",
          [this](bool checked) {
            if ( options_ && options_->ignore_input_mask != checked ) {
              options_->ignore_input_mask = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * v) {
            if ( options_ ) {
              * v = options_->ignore_input_mask;
              return true;
            }
            return false;
          });


  updateControls();
}

void QLiveStackingAccumulateOptions::set_options(c_live_stacking_accumulation_options * options)
{
  options_ = options;
  updateControls();
}

c_live_stacking_accumulation_options * QLiveStackingAccumulateOptions::options() const
{
  return options_;
}

void QLiveStackingAccumulateOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {
    Base::populatecontrols();
    setEnabled(true);
  }
}

