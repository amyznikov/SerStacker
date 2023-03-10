/*
 * QRStereoMatchingOptions.cc
 *
 *  Created on: Mar 11, 2023
 *      Author: amyznikov
 */

#include "QRStereoMatchingOptions.h"

QRStereoMatchingOptions::QRStereoMatchingOptions(QWidget * parent) :
  Base("QRStereoMatchingOptions", parent)
{
  enable_stereo_matching_ctl =
      add_checkbox("enable stereo matching",
          [this](bool checked) {
            if ( pipeline_ ) {
              pipeline_->stereo_matching_options().enable_stereo_matchning = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( pipeline_ ) {
              * checked = pipeline_->stereo_matching_options().enable_stereo_matchning;
              return true;
            }
            return false;
          });


  max_disparity_ctl =
      add_numeric_box<int>("max_disparity:",
          [this](int value) {
            if ( pipeline_ ) {
              pipeline_->stereo_matching_options().max_disparity = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( pipeline_ ) {
              * value = pipeline_->stereo_matching_options().max_disparity;
              return true;
            }
            return false;
          });

  max_scale_ctl =
      add_numeric_box<int>("max_scale:",
          [this](int value) {
            if ( pipeline_ ) {
              pipeline_->stereo_matching_options().max_scale = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( pipeline_ ) {
              * value = pipeline_->stereo_matching_options().max_scale;
              return true;
            }
            return false;
          });

  updateControls();
}


void QRStereoMatchingOptions::set_current_pipeline(const c_regular_stereo_pipeline::sptr & pipeline)
{
  pipeline_ = pipeline;
  updateControls();
}

const c_regular_stereo_pipeline::sptr& QRStereoMatchingOptions::current_pipeline() const
{
  return pipeline_;
}

void QRStereoMatchingOptions::onupdatecontrols()
{
  if( !pipeline_ ) {
    setEnabled(false);
  }
  else {

    Base::onupdatecontrols();
    setEnabled(true);
  }
}
