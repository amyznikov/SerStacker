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


int QMeasureSharpnessNorm::compute(cv::InputArray image, cv::InputArray mask, const cv::Rect & roi, cv::Scalar * value) const
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

  (*value)[0] = c_sharpness_norm_measure::measure(src(rc), msk.empty() ? cv::noArray() : msk(rc));

  return 1;
}

bool QMeasureSharpnessNorm::hasOptions() const
{
  return true;
}

QMeasureSettingsWidget* QMeasureSharpnessNorm::createSettingsWidget(QWidget * parent) const
{
  return new QSharpnessNormMeasureSettingsWidget(parent);
}



QSharpnessNormMeasureSettingsWidget::QSharpnessNormMeasureSettingsWidget(QWidget * parent) :
    Base(parent)
{
  norm_type_ctl =
      add_enum_combobox<NormType>("Norm type:",
          "",
          [this](NormType v) {
            if ( measure_ && measure_->norm_type() != v ) {
              measure_->set_norm_type(v);
              Q_EMIT parameterChanged();
            }
          },
          [this](NormType * v) {
            if ( measure_ ) {
              *v = measure_->norm_type();
              return true;
            }
            return false;
          });

  sigma_ctl =
      add_numeric_box<double>("Low-pass sigma [pix]:",
          "",
          [this](double v) {
            if ( measure_ && v > 0 && measure_->sigma() != v ) {
              measure_->set_sigma(v);
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( measure_ ) {
              * v = measure_->sigma();
              return true;
            }
            return false;
          });

  updateControls();
}
