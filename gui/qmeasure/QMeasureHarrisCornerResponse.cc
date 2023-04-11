/*
 * QMeasureHarrisCornerResponse.cc
 *
 *  Created on: Apr 11, 2023
 *      Author: amyznikov
 */

#include "QMeasureHarrisCornerResponse.h"

QMeasureHarrisCornerResponse::QMeasureHarrisCornerResponse() :
  Base("HarrisCorner", "Compute Harris Corner Response over image ROI")
{
}

int QMeasureHarrisCornerResponse::compute(cv::InputArray image, cv::InputArray mask, const cv::Rect & roi, cv::Scalar * value) const
{
  const cv::Mat src =
      image.getMat();

  const cv::Mat1b msk =
      mask.getMat();

  const int cn =
      src.channels();

  cv::Rect rc;

  if( !adjust_roi(roi, src.size(), &rc) ) {
    return 0;
  }

  *value = c_harris_sharpness_measure::compute(src(rc));

  return avgchannel_ ? 1 : src.channels();
}

bool QMeasureHarrisCornerResponse::hasOptions() const
{
  return true;
}

QMeasureSettingsWidget* QMeasureHarrisCornerResponse::createSettingsWidget(QWidget * parent) const
{
  return new QHarrisCornerResponseSettingsWidget(parent);
}



QHarrisCornerResponseSettingsWidget::QHarrisCornerResponseSettingsWidget(QWidget * parent) :
  Base(parent)
{
  avgc_ctl =
      add_checkbox("Average color channels:",
          "",
          [this](bool checked) {
            if ( measure_ && measure_->avgchannel() != checked ) {
              measure_->set_avgchannel(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( measure_ ) {
              * checked = measure_->avgchannel();
              return true;
            }
            return false;
          });

  dscale_ctl =
      add_numeric_box<int>("dscale:",
          "",
          [this](int v) {
            if ( measure_ && v != measure_->dscale() ) {
              measure_->set_dscale(v);
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( measure_ ) {
              *v = measure_->dscale();
              return true;
            }
            return false;
          });

  uscale_ctl =
      add_numeric_box<int>("uscale:",
          "",
          [this](int v) {
            if ( measure_ && v != measure_->uscale() ) {
              measure_->set_uscale(v);
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( measure_ ) {
              *v = measure_->uscale();
              return true;
            }
            return false;
          });

  k_ctl =
      add_numeric_box<double>("K:",
          "",
          [this](double v) {
            if ( measure_ && v != measure_->k() ) {
              measure_->set_k(v);
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( measure_ ) {
              *v = measure_->k();
              return true;
            }
            return false;
          });

  updateControls();
}
