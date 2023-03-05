/*
 * QFrameRegistrationSettings.cc
 *
 *  Created on: Feb 9, 2021
 *      Author: amyznikov
 */

#include "QFrameRegistrationOptions.h"
#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QFeatureBasedRegistrationOptions::QFeatureBasedRegistrationOptions(QWidget * parent) :
    Base("QFeatureBasedRegistrationOptions", parent)
{
  enableFeatureBasedRegistration_ctl =
      add_checkbox("Enable feature based registration",
          [this](bool checked) {
            if ( options_ && options_->enabled != checked ) {
              options_->enabled = checked;
              update_controls_state();
              Q_EMIT parameterChanged();
            }
          });

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

void QFeatureBasedRegistrationOptions::set_registration_options(c_feature_based_registration_options * options)
{
  options_ = options;
  updateControls();
}

c_feature_based_registration_options * QFeatureBasedRegistrationOptions::registration_options() const
{
  return options_;
}

void QFeatureBasedRegistrationOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {
    enableFeatureBasedRegistration_ctl->setChecked(options_->enabled);
    scale_ctl->setValue(options_->scale);
    sparseFeatureDetectorOptions_ctl->set_feature_detector_options(&options_->sparse_feature_extractor.detector);
    sparseFeatureDescriptorOptions_ctl->set_feature_descriptor_options(&options_->sparse_feature_extractor.descriptor);
    sparseFeatureMatcherOptions_ctl->set_feature_matcher_options(&options_->sparse_feature_matcher);
    update_controls_state();
    onDetectorTypeChanged();
    setEnabled(true);
  }
}

void QFeatureBasedRegistrationOptions::update_controls_state()
{
  const bool enable_controls =
      options_ && options_->enabled;

  for( QWidget *ctrl : controls ) {
    ctrl->setEnabled(enable_controls);
  }
}

void QFeatureBasedRegistrationOptions::onDetectorTypeChanged()
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QEccRegistrationOptions::QEccRegistrationOptions(QWidget * parent) :
    Base("QEccRegistrationOptions", parent)
{
  enableEcc_ctl =
      add_checkbox("Enable ECC",
          [this](bool checked) {
            if ( options_ && options_->enabled != checked ) {
              options_->enabled = checked;
              update_controls_state();
              Q_EMIT parameterChanged();
            }
          });

  controls.append(scale_ctl =
      add_numeric_box<double>("image scale",
          [this](double value) {
            if ( options_ && options_->scale != value ) {
              options_->scale = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(eps_ctl =
      add_numeric_box<double>("eps",
          [this](double value) {
            if ( options_ && options_->eps != value ) {
              options_->eps = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(min_rho_ctl =
      add_numeric_box<double>("min_rho",
          [this](double value) {
            if ( options_ && options_->min_rho != value ) {
              options_->min_rho = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(input_smooth_sigma_ctl =
      add_numeric_box<double>("input_smooth_sigma",
          [this](double value) {
            if ( options_ && options_->input_smooth_sigma != value ) {
              options_->input_smooth_sigma = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(reference_smooth_sigma_ctl =
      add_numeric_box<double>("reference_smooth_sigma",
          [this](double value) {
            if ( options_ && options_->reference_smooth_sigma != value ) {
              options_->reference_smooth_sigma = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(update_step_scale_ctl =
      add_numeric_box<double>("update_step_scale",
          [this](double value) {
            if ( options_ && options_->update_step_scale != value ) {
              options_->update_step_scale = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(normalization_noise_ctl =
      add_numeric_box<double>("normalization_noise",
          [this](double value) {
            if ( options_ && options_->normalization_noise != value ) {
              options_->normalization_noise = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(normalization_scale_ctl =
      add_numeric_box<int>("normalization_scale",
          [this](int value) {
            if ( options_ && options_->normalization_scale != value ) {
              options_->normalization_scale = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(max_iterations_ctl =
      add_numeric_box<int>("max_iterations",
          [this](int value) {
            if ( options_ && options_->max_iterations != value ) {
              options_->max_iterations = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(enable_ecch_ctl =
      add_checkbox("enable_ecch",
          [this](bool checked) {
            if ( options_ && options_->enable_ecch != checked ) {
              options_->enable_ecch = checked;
              ecch_minimum_image_size_ctl->setEnabled(options_->enable_ecch);
              ecch_estimate_translation_first_ctl->setEnabled(options_->enable_ecch);
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(ecch_estimate_translation_first_ctl =
      add_checkbox("Estimate translation first",
          [this](bool checked) {
            if ( options_ && options_->ecch_estimate_translation_first != checked ) {
              options_->ecch_estimate_translation_first = checked;
              Q_EMIT parameterChanged();
            }
          }));


  controls.append(ecch_minimum_image_size_ctl =
      add_numeric_box<int>("ecch_minimum_image_size",
          [this](int value) {
            if ( options_ && options_->ecch_minimum_image_size != value ) {
              options_->ecch_minimum_image_size = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(replace_planetary_disk_with_mask_ctl =
      add_checkbox("replace_planetary_disk_with_mask",
          [this](bool checked) {
            if ( options_ && options_->replace_planetary_disk_with_mask != checked ) {
              options_->replace_planetary_disk_with_mask = checked;
              planetary_disk_mask_stdev_factor_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(planetary_disk_mask_stdev_factor_ctl =
      add_numeric_box<double>("stdev_factor",
          [this](double value) {
            if ( options_ && options_->planetary_disk_mask_stdev_factor != value ) {
              options_->planetary_disk_mask_stdev_factor = value;
              Q_EMIT parameterChanged();
            }
          }));


  updateControls();
}

void QEccRegistrationOptions::set_registration_options(c_ecc_registration_options * options)
{
  options_ = options;
  updateControls();
}

c_ecc_registration_options* QEccRegistrationOptions::registration_options() const
{
  return options_;
}

void QEccRegistrationOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {
    enableEcc_ctl->setChecked(options_->enabled);
    scale_ctl->setValue(options_->scale);
    eps_ctl->setValue(options_->eps);
    min_rho_ctl->setValue(options_->min_rho);
    input_smooth_sigma_ctl->setValue(options_->input_smooth_sigma);
    reference_smooth_sigma_ctl->setValue(options_->reference_smooth_sigma);
    update_step_scale_ctl->setValue(options_->update_step_scale);
    normalization_noise_ctl->setValue(options_->normalization_noise);
    normalization_scale_ctl->setValue(options_->normalization_scale);
    max_iterations_ctl->setValue(options_->max_iterations);

    enable_ecch_ctl->setChecked(options_->enable_ecch);
    ecch_estimate_translation_first_ctl->setChecked(options_->ecch_estimate_translation_first);
    ecch_minimum_image_size_ctl->setValue(options_->ecch_minimum_image_size);

    ecch_minimum_image_size_ctl->setEnabled(options_->enable_ecch);
    ecch_estimate_translation_first_ctl->setEnabled(options_->enable_ecch);

    replace_planetary_disk_with_mask_ctl->setChecked(options_->replace_planetary_disk_with_mask);
    planetary_disk_mask_stdev_factor_ctl->setEnabled(options_->replace_planetary_disk_with_mask);
    planetary_disk_mask_stdev_factor_ctl->setValue(options_->planetary_disk_mask_stdev_factor);

    update_controls_state();

    setEnabled(true);
  }
}

void QEccRegistrationOptions::update_controls_state()
{
  const bool enable_controls =
      options_ && options_->enabled;

  for( QWidget *ctrl : controls ) {
    ctrl->setEnabled(enable_controls);
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QEccFlowRegistrationOptions::QEccFlowRegistrationOptions(QWidget * parent) :
    Base("QEccFlowRegistrationOptions", parent)
{
  enableEccFlow_ctl =
      add_checkbox("Enable ECC flow",
          [this](bool checked) {
            if ( options_ && options_->enabled != checked ) {
              options_->enabled = checked;
              update_controls_state();
              Q_EMIT parameterChanged();
            }
          });



  controls.append(support_scale_ctl =
      add_numeric_box<int>("support_scale",
          [this](int value) {
            if ( options_ && options_->support_scale != value ) {
              options_->support_scale = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(input_smooth_sigma_ctl =
      add_numeric_box<double>("input_smooth_sigma",
          [this](double value) {
            if ( options_ && options_->input_smooth_sigma != value ) {
              options_->input_smooth_sigma = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(reference_smooth_sigma_ctl =
      add_numeric_box<double>("reference_smooth_sigma",
          [this](double value) {
            if ( options_ && options_->reference_smooth_sigma != value ) {
              options_->reference_smooth_sigma = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(update_multiplier_ctl =
      add_numeric_box<double>("update_multiplier",
          [this](double value) {
            if ( options_ && options_->update_multiplier != value ) {
              options_->update_multiplier = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(max_iterations_ctl =
      add_numeric_box<int>("max_iterations",
          [this](int value) {
            if ( options_ && options_->max_iterations != value ) {
              options_->max_iterations = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(normalization_scale_ctl =
      add_numeric_box<int>("normalization_scale",
          [this](int value) {
            if ( options_ && options_->normalization_scale != value ) {
              options_->normalization_scale = value;
              Q_EMIT parameterChanged();
            }
          }));
}

void QEccFlowRegistrationOptions::set_registration_options(c_eccflow_registration_options * options)
{
  options_ = options;
  updateControls();
}

c_eccflow_registration_options* QEccFlowRegistrationOptions::registration_options() const
{
  return options_;
}

void QEccFlowRegistrationOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {
    enableEccFlow_ctl->setChecked(options_->enabled);
    update_multiplier_ctl->setValue(options_->update_multiplier);
    input_smooth_sigma_ctl->setValue(options_->input_smooth_sigma);
    reference_smooth_sigma_ctl->setValue(options_->reference_smooth_sigma);
    max_iterations_ctl->setValue(options_->max_iterations);
    support_scale_ctl->setValue(options_->support_scale);
    normalization_scale_ctl->setValue(options_->normalization_scale);
    update_controls_state();
    setEnabled(true);
  }
}

void QEccFlowRegistrationOptions::update_controls_state()
{
  const bool enable_controls =
      options_ && options_->enabled;

  for( QWidget *ctrl : controls ) {
    ctrl->setEnabled(enable_controls);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


QJovianDerotationOptions::QJovianDerotationOptions(QWidget * parent) :
    Base("QJovianDerotationOptions", parent)
{
  enableJovianDerotation_ctl =
      add_checkbox("Enable Jovian Derotation",
          [this](bool checked) {
            if ( options_ && options_->enabled != checked ) {
              options_->enabled = checked;
              update_controls_state();
              Q_EMIT parameterChanged();
            }
          });

  add_expandable_groupbox("Jovian detector options",
      detector_setting_ctl = new QJovianEllipseDetectorSettings(this));
  controls.append(detector_setting_ctl);

  connect(detector_setting_ctl, &QJovianEllipseDetectorSettings::parameterChanged,
      this, &ThisClass::parameterChanged);

  controls.append(max_pyramid_level_ctl =
      add_numeric_box<int>("max pyramid level:",
          [this](int value) {
            if ( options_ && options_->max_pyramid_level != value ) {
              options_->max_pyramid_level = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(min_rotation_ctl =
      add_numeric_box<double>("min_rotation [deg]:",
          [this](double value) {
            if ( options_ && options_->min_rotation != (value *= M_PI / 180) ) {
              options_->min_rotation = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(max_rotation_ctl =
      add_numeric_box<double>("max_rotation [deg]:",
          [this](double value) {
            if ( options_ && options_->max_rotation != (value *= M_PI / 180) ) {
              options_->max_rotation = value;
              Q_EMIT parameterChanged();
            }
          }));



  controls.append(num_orientations_ctl =
      add_numeric_box<int>("num_orientations:",
          [this](int value) {
            if ( options_ && options_->num_orientations != value ) {
              options_->num_orientations = value;
              Q_EMIT parameterChanged();
            }
          }));



//  controls.append(align_planetary_disk_masks_ctl =
//      add_checkbox("align planetary disk masks",
//          [this](bool checked) {
//            if ( options_ && options_->align_planetary_disk_masks != checked ) {
//              options_->align_planetary_disk_masks = checked;
//              Q_EMIT parameterChanged();
//            }
//          }));

  controls.append(eccflow_support_scale_ctl =
      add_numeric_box<int>("eccflow_support_scale:",
          [this](double value) {
            if ( options_ && options_->eccflow_support_scale != value ) {
              options_->eccflow_support_scale = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(eccflow_normalization_scale_ctl =
      add_numeric_box<int>("eccflow_normalization_scale:",
          [this](double value) {
            if ( options_ && options_->eccflow_normalization_scale != value ) {
              options_->eccflow_normalization_scale = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(eccflow_max_pyramid_level_ctl =
      add_numeric_box<int>("eccflow_max_pyramid_level:",
          [this](double value) {
            if ( options_ && options_->eccflow_max_pyramid_level != value ) {
              options_->eccflow_max_pyramid_level = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(derotate_all_frames_ctl =
      add_checkbox("process all frames",
          [this](bool checked) {
            if ( options_ && options_->derotate_all_frames != checked ) {
              options_->derotate_all_frames = checked;
              derotate_all_frames_max_context_size_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(derotate_all_frames_max_context_size_ctl =
      add_numeric_box<int>("max_context_size:",
          [this](double value) {
            if ( options_ && options_->derotate_all_frames_max_context_size != value ) {
              options_->derotate_all_frames_max_context_size = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(align_jovian_disk_horizontally_ctl =
      add_checkbox("align_jovian_disk_horizontally",
          [this](bool checked) {
            if ( options_ && options_->rotate_jovian_disk_horizontally != checked ) {
              options_->rotate_jovian_disk_horizontally = checked;
              Q_EMIT parameterChanged();
            }
          }));

  updateControls();
}

void QJovianDerotationOptions::set_derotation_options(c_jovian_derotation_options * options)
{
  options_ = options;
  updateControls();
}

c_jovian_derotation_options* QJovianDerotationOptions::derotation_options() const
{
  return options_;
}

void QJovianDerotationOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {
    enableJovianDerotation_ctl->setChecked(options_->enabled);
    max_pyramid_level_ctl->setValue(options_->max_pyramid_level);
    min_rotation_ctl->setValue(options_->min_rotation * 180 / M_PI);
    max_rotation_ctl->setValue(options_->max_rotation * 180 / M_PI);
    num_orientations_ctl->setValue(options_->num_orientations);
    //align_planetary_disk_masks_ctl->setChecked(options_->align_planetary_disk_masks);
    eccflow_support_scale_ctl->setValue(options_->eccflow_support_scale);
    eccflow_normalization_scale_ctl->setValue(options_->eccflow_normalization_scale);
    eccflow_max_pyramid_level_ctl->setValue(options_->eccflow_max_pyramid_level);
    derotate_all_frames_ctl->setChecked(options_->derotate_all_frames);
    derotate_all_frames_max_context_size_ctl->setValue(options_->derotate_all_frames_max_context_size);
    align_jovian_disk_horizontally_ctl->setChecked(options_->rotate_jovian_disk_horizontally);
    detector_setting_ctl->set_jovian_ellipse_detector_options(&options_->ellipse);
    update_controls_state();

    derotate_all_frames_max_context_size_ctl->setEnabled(options_->derotate_all_frames);

    setEnabled(true);
  }
}

void QJovianDerotationOptions::update_controls_state()
{
  const bool enable_controls =
      options_ && options_->enabled;

  for( QWidget *ctrl : controls ) {
    ctrl->setEnabled(enable_controls);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QImageRegistrationOptions::QImageRegistrationOptions(QWidget * parent) :
    Base("QImageRegistrationOptions", parent)
{
  motion_type_ctl =
      add_enum_combobox<IMAGE_MOTION_TYPE>("Motion type:",
          [this](IMAGE_MOTION_TYPE value) {
            if ( options_ && options_->image_registration_options.motion_type != value ) {
              options_->image_registration_options.motion_type = value;
              Q_EMIT parameterChanged();
            }
          });

  registration_channel_ctl =
      add_enum_combobox<color_channel_type>(
          "Registration channel:",
          [this](color_channel_type value) {
            if ( options_ && options_->image_registration_options.registration_channel != value ) {
              options_->image_registration_options.registration_channel = value;
              Q_EMIT parameterChanged();
            }
          });

  interpolation_method_ctl =
      add_enum_combobox<ECC_INTERPOLATION_METHOD>(
          "Interpolation method:",
          [this](ECC_INTERPOLATION_METHOD value) {
            if ( options_ && options_->image_registration_options.interpolation != value) {
              options_->image_registration_options.interpolation = value;
              Q_EMIT parameterChanged();
            }
          });

  border_mode_ctl =
      add_enum_combobox<ECC_BORDER_MODE>(
          "Border mode:",
          [this](ECC_BORDER_MODE value) {
            if ( options_ && options_->image_registration_options.border_mode != value ) {
              options_->image_registration_options.border_mode = value;
              Q_EMIT parameterChanged();
            }
          });

  border_value_ctl =
      add_numeric_box<cv::Scalar>(
          "Border Value",
          [this](const cv::Scalar & value) {
            if ( options_ && options_->image_registration_options.border_value != value ) {
              options_->image_registration_options.border_value = value;
              Q_EMIT parameterChanged();
            }
          });

  ///

  add_expandable_groupbox("Master Frame Options",
      masterFrameOptions_ctl = new QMasterFrameOptions(this));

  connect(masterFrameOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Feature Based Registration Options",
      featureRegistrationOptions_ctl = new QFeatureBasedRegistrationOptions(this));
  connect(featureRegistrationOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("ECC Registration Options",
      eccOptions_ctl = new QEccRegistrationOptions(this));
  connect(eccOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("ECC Flow Registration Options",
      eccFlowOptions_ctl = new QEccFlowRegistrationOptions(this));
  connect(eccFlowOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Jovian Derotation Options",
      jovianDerotationOptions_ctl = new QJovianDerotationOptions(this));
  connect(jovianDerotationOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);
}

void QImageRegistrationOptions::set_current_pipeline(const c_image_stacking_pipeline::sptr & current_pipeline)
{
  current_pipeline_ = current_pipeline;
  options_ = current_pipeline_ ? &current_pipeline_->frame_registration_options() : nullptr;
  updateControls();
}

const c_image_stacking_pipeline::sptr & QImageRegistrationOptions::current_pipeline() const
{
  return current_pipeline_;
}

void QImageRegistrationOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {

    motion_type_ctl->setValue(options_->image_registration_options.motion_type);
    registration_channel_ctl->setValue(options_->image_registration_options.registration_channel);
    interpolation_method_ctl->setValue(options_->image_registration_options.interpolation);
    border_mode_ctl->setValue(options_->image_registration_options.border_mode);
    border_value_ctl->setValue(options_->image_registration_options.border_value);

    masterFrameOptions_ctl->set_current_pipeline(current_pipeline_);
    featureRegistrationOptions_ctl->set_registration_options(&options_->image_registration_options.feature_registration);
    eccOptions_ctl->set_registration_options(&options_->image_registration_options.ecc);
    eccFlowOptions_ctl->set_registration_options(&options_->image_registration_options.eccflow);
    jovianDerotationOptions_ctl->set_derotation_options(&options_->image_registration_options.jovian_derotation);

    setEnabled(true);
  }
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QFrameRegistrationOptions::QFrameRegistrationOptions(QWidget * parent) :
    Base("QFrameRegistrationOptions", parent)
{
  enable_frame_registration_ctl =
      add_checkbox("Enable image registration",
          [this](bool checked) {
            if ( options_ && options_->image_registration_options.enable_frame_registration != checked ) {
              options_->image_registration_options.enable_frame_registration = checked;
              update_controls_visibility();
              Q_EMIT parameterChanged();
            }
          });

  imageRegistrationOptions_ctl =
      add_widget<QImageRegistrationOptions>();

  connect(imageRegistrationOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  accumulateAndCompensateTurbulentFlow_ctl =
      add_checkbox("accumulate and compensate turbulent flow",
          [this](bool checked) {
            if ( options_ && options_->accumulate_and_compensate_turbulent_flow != checked ) {
              options_->accumulate_and_compensate_turbulent_flow = checked;
              Q_EMIT parameterChanged();
            }
          });

  update_controls_visibility();
}

void QFrameRegistrationOptions::set_current_pipeline(const c_image_stacking_pipeline::sptr & current_pipeline)
{
  current_pipeline_ = current_pipeline;
  options_ = current_pipeline_ ? &current_pipeline_->frame_registration_options() : nullptr;
  updateControls();
}

const c_image_stacking_pipeline::sptr & QFrameRegistrationOptions::current_pipeline() const
{
  return current_pipeline_;
}

void QFrameRegistrationOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {

    enable_frame_registration_ctl->setChecked(options_->image_registration_options.
        enable_frame_registration);

    imageRegistrationOptions_ctl->set_current_pipeline(current_pipeline_);

    accumulateAndCompensateTurbulentFlow_ctl->setChecked(
        options_->accumulate_and_compensate_turbulent_flow);

    update_controls_visibility();

    setEnabled(true);
  }
}

void QFrameRegistrationOptions::update_controls_visibility()
{
  if( !options_ ) {
    imageRegistrationOptions_ctl->setVisible(false);
    accumulateAndCompensateTurbulentFlow_ctl->setEnabled(false);
  }
  else {
    imageRegistrationOptions_ctl->setVisible(options_->image_registration_options.enable_frame_registration);
    accumulateAndCompensateTurbulentFlow_ctl->setEnabled(options_->image_registration_options.enable_frame_registration);
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

