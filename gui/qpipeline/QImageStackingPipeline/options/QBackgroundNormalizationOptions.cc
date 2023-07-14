/*
 * QBackgroundNormalizationOptions.cc
 *
 *  Created on: Jun 4, 2023
 *      Author: amyznikov
 */

#include "QBackgroundNormalizationOptions.h"

QBackgroundNormalizationOptions::QBackgroundNormalizationOptions(QWidget * parent) :
    Base("parent")
{
  norm_type_ctl =
      add_enum_combobox<histogram_normalization_type>(
          "Norm type",
          "Specify background normalization type",
          [this](histogram_normalization_type v) {
            if ( options_ && options_->norm_type != v) {
              options_->norm_type = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](histogram_normalization_type * v) {
            if ( options_ ) {
              *v = options_->norm_type;
              return true;
            }
            return false;
          });

  stretch_ctl =
      add_numeric_box<cv::Scalar>("Stretch:",
          "Specify channel stretch (contrast)",
          [this](const cv::Scalar & v) {
            if ( options_ && options_->stretch != v) {
              options_->stretch = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Scalar * v) {
            if ( options_ ) {
              *v = options_->stretch;
              return true;
            }
            return false;
          });

  offset_ctl =
      add_numeric_box<cv::Scalar>("Offset:",
          "Specify channel offsets (brigtness)",
          [this](const cv::Scalar & v) {
            if ( options_ && options_->offset != v) {
              options_->offset = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Scalar * v) {
            if ( options_ ) {
              *v = options_->offset;
              return true;
            }
            return false;
          });

  updateControls();
}

void QBackgroundNormalizationOptions::set_options(c_histogram_normalization_options * options)
{
  options_ = options;
  updateControls();
}

c_histogram_normalization_options* QBackgroundNormalizationOptions::options() const
{
  return options_;
}

void QBackgroundNormalizationOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {
    Base::onupdatecontrols();
    setEnabled(true);
  }
}
