/*
 * QStereoMatcherProcessingOptions.cc
 *
 *  Created on: Jun 21, 2023
 *      Author: amyznikov
 */

#include "QStereoMatcherProcessingOptions.h"

QStereoMatcherProcessingOptions::QStereoMatcherProcessingOptions(QWidget * parent) :
    Base("", parent)
{
  camera_focus_ctl =
      add_numeric_box<double>("Camera focus [px]",
          "Camera focus in pixels",
          [this](double v) {
            if ( options_ && options_->camera_focus != v ) {
              options_->camera_focus = v;
              Q_EMIT parameterChanged();

            }
          },
          [this](double * v) {
            if ( options_ ) {
              * v = options_->camera_focus;
              return true;
            }
            return false;
          });

  stereo_baseline_ctl =
      add_numeric_box<double>("stereo baseline",
          "Stereo base line",
          [this](double v) {
            if ( options_ && options_->stereo_baseline != v ) {
              options_->stereo_baseline = v;
              Q_EMIT parameterChanged();

            }
          },
          [this](double * v) {
            if ( options_ ) {
              * v = options_->stereo_baseline;
              return true;
            }
            return false;
          });

}

void QStereoMatcherProcessingOptions::set_processing_options(c_stereo_matcher_processing_options * options)
{
  options_ = options;
  updateControls();
}

c_stereo_matcher_processing_options* QStereoMatcherProcessingOptions::processing_options() const
{
  return options_;
}

void QStereoMatcherProcessingOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {
    Base::onupdatecontrols();
    setEnabled(true);
  }

}

