/*
 * QRStereoFeature2dOptions.cc
 *
 *  Created on: Mar 4, 2023
 *      Author: amyznikov
 */

#include "QRStereoFeature2dOptions.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QRStereoFeature2dOptions::QRStereoFeature2dOptions(QWidget * parent) :
    Base("QRStereoFeature2dOptions", parent)
{
  controls.append(scale_ctl =
      add_numeric_box<double>("Image scale",
          [this](double value) {
            if ( options_ && options_->scale != value ) {
              options_->scale = value;
              Q_EMIT parameterChanged();
            }
          }));

  add_expandable_groupbox("Sparse Feature Detector Options",
      sparseFeatureDetectorOptions_ctl = new QSparseFeatureDetectorOptions());
  controls.append(sparseFeatureDetectorOptions_ctl);

  connect(sparseFeatureDetectorOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Sparse Feature Descriptor Options",
      sparseFeatureDescriptorOptions_ctl = new QSparseFeatureDescriptorOptions());
  controls.append(sparseFeatureDescriptorOptions_ctl);

  connect(sparseFeatureDescriptorOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Sparse Feature Matcher Options",
      sparseFeatureMatcherOptions_ctl = new QSparseFeatureMatcherOptions());
  controls.append(sparseFeatureMatcherOptions_ctl);

  connect(sparseFeatureMatcherOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  connect(sparseFeatureDetectorOptions_ctl, &QSparseFeatureDetectorOptions::detectorTypeChanged,
      this, &ThisClass::onDetectorTypeChanged);

}

void QRStereoFeature2dOptions::set_feature2d_options(c_rstereo_feature2d_options * options)
{
  options_ = options;
  updateControls();
}

c_rstereo_feature2d_options * QRStereoFeature2dOptions::feature2d_options() const
{
  return options_;
}

void QRStereoFeature2dOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {
    scale_ctl->setValue(options_->scale);
    sparseFeatureDetectorOptions_ctl->set_feature_detector_options(&options_->sparse_feature_extractor.detector);
    sparseFeatureDescriptorOptions_ctl->set_feature_descriptor_options(&options_->sparse_feature_extractor.descriptor);
    sparseFeatureMatcherOptions_ctl->set_feature_matcher_options(&options_->sparse_feature_matcher);
    //update_controls_state();
    onDetectorTypeChanged();
    setEnabled(true);
  }
}

//void QRStereoFeature2dOptions::update_controls_state()
//{
//  const bool enable_controls =
//      options_ && options_->enabled;
//
//  for( QWidget *ctrl : controls ) {
//    ctrl->setEnabled(enable_controls);
//  }
//}

void QRStereoFeature2dOptions::onDetectorTypeChanged()
{
  if ( options_ ) {

    if ( options_->sparse_feature_extractor.detector.type == SPARSE_FEATURE_DETECTOR_PLANETARY_DISK ) {
      sparseFeatureDescriptorOptions_ctl->setEnabled(false);
      sparseFeatureMatcherOptions_ctl->setEnabled(false);
    }
    else {
      sparseFeatureDescriptorOptions_ctl->setEnabled(true);
      sparseFeatureMatcherOptions_ctl->setEnabled(true);
    }
  }
}

