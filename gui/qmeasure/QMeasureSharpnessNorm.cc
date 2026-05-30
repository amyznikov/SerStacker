/*
 * QMeasureSharpnessNorm.cc
 *
 *  Created on: Apr 12, 2023
 *      Author: amyznikov
 */

#include "QMeasureSharpnessNorm.h"

QMeasureSharpnessNorm::QMeasureSharpnessNorm() :
  Base("SharpnessNorm", "SharpnessNorm measure over image ROI")
{
}

int QMeasureSharpnessNorm::compute(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const
{
  (*output_value)[0] = c_sharpness_norm_measure::measure(image, mask);
  return 1;
}

QMeasureSettingsWidget* QMeasureSharpnessNorm::createSettingsWidget(QWidget * parent) const
{
  return new QSharpnessNormMeasureSettingsWidget(parent);
}

QSharpnessNormMeasureSettingsWidget::QSharpnessNormMeasureSettingsWidget(QWidget * parent) :
    Base(parent)
{
  norm_type_ctl =
      add_enum_combobox<cv::NormTypes>("Norm type:",
          "",
          [this](cv::NormTypes v) {
            if ( _measure && _measure->norm_type() != v ) {
              _measure->set_norm_type(v);
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::NormTypes * v) {
            if ( _measure ) {
              *v = _measure->norm_type();
              return true;
            }
            return false;
          });

  sigma_ctl =
      add_numeric_box<double>("Low-pass sigma [pix]:",
          "",
          [this](double v) {
            if ( _measure && v > 0 ) {
              _measure->set_sigma(v);
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _measure ) {
              * v = _measure->sigma();
              return true;
            }
            return false;
          });

  updateControls();
}
