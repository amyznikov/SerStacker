/*
 * QMeasureLC.cc
 *
 *  Created on: Apr 10, 2023
 *      Author: amyznikov
 */

#include "QMeasureLC.h"

QMeasureLC::QMeasureLC() :
    Base("LC", "LC")
{
}

QMeasureSettingsWidget* QMeasureLC::createSettingsWidget(QWidget * parent) const
{
  return new QLCMeasureSettingsWidget(parent);
}

//void QMeasureLC::setAverageColorChannels(bool v)
//{
//  c_local_contrast_measure::set_avgchannel(_average_color_channels = v);
//}
//
//bool QMeasureLC::averageColorChannels() const
//{
//  return c_local_contrast_measure::avgchannel();
//}

int QMeasureLC::compute(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const
{
  *output_value = c_local_contrast_measure::compute(image);
  return _opts.avgchannel ? 1 : image.channels();
}

QLCMeasureSettingsWidget::QLCMeasureSettingsWidget(QWidget * parent) :
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

  eps_ctl =
      add_numeric_box<double>("eps:",
          "",
          [this](double v) {
            if ( measure_ && v != measure_->eps() ) {
              measure_->set_eps(v);
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( measure_ ) {
              *v = measure_->eps();
              return true;
            }
            return false;
          });

  updateControls();
}
