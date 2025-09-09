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

QMeasureSettingsWidget* QMeasureHarrisCornerResponse::createSettingsWidget(QWidget * parent) const
{
  return new QHarrisCornerResponseSettingsWidget(parent);
}

//void QMeasureHarrisCornerResponse::setAverageColorChannels(bool v)
//{
//  c_harris_sharpness_measure::set_avgchannel(_average_color_channels = v);
//}
//
//bool QMeasureHarrisCornerResponse::averageColorChannels() const
//{
//  return c_harris_sharpness_measure::avgchannel();
//}

int QMeasureHarrisCornerResponse::compute(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const
{
  *output_value = c_harris_sharpness_measure::compute(image);
  return avgchannel_ ? 1 : image.channels();
}

QHarrisCornerResponseSettingsWidget::QHarrisCornerResponseSettingsWidget(QWidget * parent) :
  Base(parent)
{
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
