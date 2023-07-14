/*
 * QFrameRegistrationSettings.h
 *
 *  Created on: Feb 9, 2021
 *      Author: amyznikov
 */

#ifndef __QFrameRegistrationSettings_h__
#define __QFrameRegistrationSettings_h__

#include <gui/qfeature2d/QFeature2dOptions.h>
#include <gui/qjovian/QJovianEllipseDetectorSettings.h>
#include <core/pipeline/c_image_stacking_pipeline/c_image_stacking_pipeline.h>
#include "QMasterFrameOptions.h"


class QFeatureBasedRegistrationOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QFeatureBasedRegistrationOptions ThisClass;
  typedef QSettingsWidget Base;

  QFeatureBasedRegistrationOptions(QWidget * parent = nullptr);

  void set_registration_options(c_feature_based_registration_options * options);
  c_feature_based_registration_options * registration_options() const;

protected:
  void onupdatecontrols() override;
  void update_controls_state();

protected slots:
  void onDetectorTypeChanged();

protected:
  c_feature_based_registration_options * options_ = nullptr;
  QCheckBox * enableFeatureBasedRegistration_ctl = nullptr;
  QNumericBox * scale_ctl = nullptr;
  QSparseFeatureDetectorOptions * sparseFeatureDetectorOptions_ctl = nullptr;
  QSparseFeatureDescriptorOptions * sparseFeatureDescriptorOptions_ctl = nullptr;
  QSparseFeatureMatcherOptions * sparseFeatureMatcherOptions_ctl = nullptr;
  QWidgetList controls;
};


class QEccRegistrationOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QEccRegistrationOptions ThisClass;
  typedef QSettingsWidget Base;

  QEccRegistrationOptions(QWidget * parent = nullptr);

  void set_registration_options(c_ecc_registration_options * options);
  c_ecc_registration_options* registration_options() const;

protected:
  void onupdatecontrols() override;
  void update_controls_state();

protected:
  c_ecc_registration_options * options_ = nullptr;
  QCheckBox * enableEcc_ctl = nullptr;
  QNumericBox * scale_ctl = nullptr;
  QNumericBox * eps_ctl = nullptr;
  QNumericBox * min_rho_ctl = nullptr;
  QNumericBox * input_smooth_sigma_ctl = nullptr;
  QNumericBox * reference_smooth_sigma_ctl = nullptr;
  QNumericBox * update_step_scale_ctl = nullptr;
  QNumericBox * normalization_noise_ctl = nullptr;
  QNumericBox * normalization_scale_ctl = nullptr;
  QNumericBox * max_iterations_ctl = nullptr;
  QCheckBox * enable_ecch_ctl = nullptr;
  QCheckBox * ecch_estimate_translation_first_ctl = nullptr;
  QNumericBox * ecch_minimum_image_size_ctl = nullptr;
  QCheckBox * replace_planetary_disk_with_mask_ctl = nullptr;
  QNumericBox * planetary_disk_mask_stdev_factor_ctl = nullptr;
  QWidgetList controls;
};


class QEccFlowRegistrationOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QEccFlowRegistrationOptions ThisClass;
  typedef QSettingsWidget Base;

  QEccFlowRegistrationOptions(QWidget * parent = nullptr);

  void set_registration_options(c_eccflow_registration_options * options);
  c_eccflow_registration_options* registration_options() const;

protected:
  void onupdatecontrols() override;
  void update_controls_state();

protected:
  c_eccflow_registration_options * options_ = nullptr;
  QCheckBox * enableEccFlow_ctl = nullptr;
  QNumericBox * update_multiplier_ctl = nullptr;
  QNumericBox * input_smooth_sigma_ctl = nullptr;
  QNumericBox * reference_smooth_sigma_ctl = nullptr;
  QNumericBox * max_iterations_ctl = nullptr;
  QNumericBox * support_scale_ctl = nullptr;
  QNumericBox * normalization_scale_ctl = nullptr;
  QWidgetList controls;
};


class QJovianDerotationOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QJovianDerotationOptions ThisClass;
  typedef QSettingsWidget Base;

  QJovianDerotationOptions(QWidget * parent = nullptr);

  void set_derotation_options(c_jovian_derotation_options * options);
  c_jovian_derotation_options* derotation_options() const;

protected:
  void onupdatecontrols() override;
  void update_controls_state();

protected:
  c_jovian_derotation_options *options_ = nullptr;
  QCheckBox * enableJovianDerotation_ctl = nullptr;
  QNumericBox * min_rotation_ctl = nullptr;
  QNumericBox * max_rotation_ctl = nullptr;
  QNumericBox * max_pyramid_level_ctl = nullptr;
  QNumericBox * num_orientations_ctl = nullptr;

//  QNumberEditBox * normalization_scale_ctl = nullptr;
//  QNumberEditBox * normalization_blur_ctl = nullptr;
  QNumericBox * eccflow_support_scale_ctl = nullptr;
  QNumericBox * eccflow_normalization_scale_ctl = nullptr;
  QNumericBox * eccflow_max_pyramid_level_ctl = nullptr;
  //QNumberEditBox * hlines_ctl = nullptr;
  //QCheckBox * align_planetary_disk_masks_ctl = nullptr;
  QCheckBox * derotate_all_frames_ctl = nullptr;
  QNumericBox * derotate_all_frames_max_context_size_ctl = nullptr;
  QCheckBox * align_jovian_disk_horizontally_ctl = nullptr;

  QJovianEllipseDetectorSettings * detector_setting_ctl = nullptr;

  QWidgetList controls;
};

class QImageRegistrationOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QImageRegistrationOptions ThisClass;
  typedef QSettingsWidget Base;
  typedef QEnumComboBox<IMAGE_MOTION_TYPE> QMotionTypeCombo;
  typedef QEnumComboBox<color_channel_type> QRegistrationColorChannelCombo;
  typedef QEnumComboBox<ECC_INTERPOLATION_METHOD> QEccInterpolatioMethodCombo;
  typedef QEnumComboBox<ECC_BORDER_MODE> QEccBorderModeCombo;

  QImageRegistrationOptions(QWidget * parent = nullptr);

  void set_current_pipeline(c_image_stacking_pipeline * pipeline);
  c_image_stacking_pipeline * current_pipeline() const;

protected:
  void onupdatecontrols() override;

protected:
  c_image_stacking_pipeline * current_pipeline_ = nullptr;
  c_frame_registration_options * options_ = nullptr;
  QMotionTypeCombo * motion_type_ctl = nullptr;
  QRegistrationColorChannelCombo * registration_channel_ctl = nullptr;
  QEccInterpolatioMethodCombo * interpolation_method_ctl = nullptr;
  QEccBorderModeCombo * border_mode_ctl = nullptr;
  QNumericBox * border_value_ctl = nullptr;

  QMasterFrameOptions * masterFrameOptions_ctl = nullptr;
  QFeatureBasedRegistrationOptions * featureRegistrationOptions_ctl = nullptr;
  QEccRegistrationOptions * eccOptions_ctl = nullptr;
  QEccFlowRegistrationOptions * eccFlowOptions_ctl = nullptr;
  QJovianDerotationOptions * jovianDerotationOptions_ctl = nullptr;
};


class QFrameRegistrationOptions:
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QFrameRegistrationOptions ThisClass;
  typedef QSettingsWidget Base;

  QFrameRegistrationOptions(QWidget * parent = nullptr);

  void set_current_pipeline(c_image_stacking_pipeline * pipeline);
  c_image_stacking_pipeline * current_pipeline() const;

protected:
  void onupdatecontrols() override;
  void update_controls_visibility();

protected:
  c_image_stacking_pipeline * current_pipeline_ = nullptr;
  c_frame_registration_options * options_ = nullptr;
  QCheckBox * enable_frame_registration_ctl = nullptr;
  QImageRegistrationOptions * imageRegistrationOptions_ctl = nullptr;
  QCheckBox * accumulateAndCompensateTurbulentFlow_ctl = nullptr;
};

#endif /* __QFrameRegistrationSettings_h__ */
