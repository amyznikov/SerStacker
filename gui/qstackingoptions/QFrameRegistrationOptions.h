/*
 * QFrameRegistrationSettings.h
 *
 *  Created on: Feb 9, 2021
 *      Author: amyznikov
 */

#ifndef __QFrameRegistrationSettings_h__
#define __QFrameRegistrationSettings_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/qimproc/QImageProcessorsCollection.h>
#include <core/pipeline/c_image_stacking_pipeline.h>
#include "QMasterFrameOptions.h"


class QSparseFeatureDetectorOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QSparseFeatureDetectorOptions ThisClass;
  typedef QSettingsWidget Base;
  typedef QEnumComboBox<SPARSE_FEATURE_DETECTOR_TYPE> QSparseFeatureDetectorTypeCombo;

  QSparseFeatureDetectorOptions(QWidget * parent = Q_NULLPTR);

  void set_feature_detector_options(c_sparse_feature_detector_options * options);
  c_sparse_feature_detector_options* feature_detector_options() const;

signals:
 void detectorTypeChanged();

protected:
  void onupdatecontrols() override;
  void update_detector_specific_controls();

protected:
  c_sparse_feature_detector_options *options_ = Q_NULLPTR;
  QSparseFeatureDetectorTypeCombo * detectorType_ctl = Q_NULLPTR;
  QNumberEditBox * max_keypoints_ctl = Q_NULLPTR;
  QWidgetList controls;
};

class QSparseFeatureDescriptorOptions:
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QSparseFeatureDescriptorOptions ThisClass;
  typedef QSettingsWidget Base;
  typedef QEnumComboBox<SPARSE_FEATURE_DESCRIPTOR_TYPE> QSparseFeatureDecriptorTypeCombo;

  QSparseFeatureDescriptorOptions(QWidget * parent = Q_NULLPTR);

  void set_feature_descriptor_options(c_sparse_feature_descriptor_options * options);
  c_sparse_feature_descriptor_options* feature_descriptor_options() const;

protected:
  void onupdatecontrols() override;
  void update_descriptor_specific_controls();

protected:
  c_sparse_feature_descriptor_options *options_ = Q_NULLPTR;
  QSparseFeatureDecriptorTypeCombo * descriptorType_ctl = Q_NULLPTR;
  QCheckBox * useDetectorSettings_ctl = Q_NULLPTR;
  QWidgetList controls;
};

class QHammingDistanceFeature2dMatcherOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QHammingDistanceFeature2dMatcherOptions ThisClass;
  typedef QSettingsWidget Base;

  QHammingDistanceFeature2dMatcherOptions(QWidget * parent = Q_NULLPTR);

  void set_feature_matcher_options(c_hamming_distance_feature2d_matcher_options * options);
  c_hamming_distance_feature2d_matcher_options * feature_matcher_options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_hamming_distance_feature2d_matcher_options * options_ = Q_NULLPTR;
  QNumberEditBox * max_acceptable_distance_ctl = Q_NULLPTR;
};

class QFlannBasedFeature2dMatcherOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QFlannBasedFeature2dMatcherOptions ThisClass;
  typedef QSettingsWidget Base;
  typedef QEnumComboBox<FlannIndexType > QFlannIndexTypeCombo;
  typedef QEnumComboBox<cvflann::flann_distance_t> QFlannDistanceTypeCombo;

  QFlannBasedFeature2dMatcherOptions(QWidget * parent = Q_NULLPTR);

  void set_feature_matcher_options( c_flann_based_feature2d_matcher_options * options);
  c_flann_based_feature2d_matcher_options * feature_matcher_options() const;

protected:
  void onupdatecontrols() override;
  void update_matcher_specific_controls();

protected:
  c_flann_based_feature2d_matcher_options * options_ = Q_NULLPTR;
  QFlannIndexTypeCombo * flannIndexType_ctl = Q_NULLPTR;
  QFlannDistanceTypeCombo * flannDistanceType_ctl = Q_NULLPTR;
  QNumberEditBox * lowe_ratio_ctl = Q_NULLPTR;
  QWidgetList controls;
};

class QTriangleMatcherOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QTriangleMatcherOptions ThisClass;
  typedef QSettingsWidget Base;

  QTriangleMatcherOptions(QWidget * parent = Q_NULLPTR);

  void set_feature_matcher_options( c_triangle_matcher_options * options);
  c_triangle_matcher_options * feature_matcher_options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_triangle_matcher_options * options_ = Q_NULLPTR;
  QNumberEditBox * eps_ctl = Q_NULLPTR;
};

class QSnormBasedFeature2dMatcherOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QSnormBasedFeature2dMatcherOptions ThisClass;
  typedef QSettingsWidget Base;

  QSnormBasedFeature2dMatcherOptions(QWidget * parent = Q_NULLPTR);

  void set_feature_matcher_options( c_snorm_based_feature2d_matcher_options * options);
  c_snorm_based_feature2d_matcher_options * feature_matcher_options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_snorm_based_feature2d_matcher_options * options_ = Q_NULLPTR;
  QNumberEditBox * max_acceptable_distance_ctl = Q_NULLPTR;
  QNumberEditBox * lowe_ratio_ctl = Q_NULLPTR;
};


class QSparseFeatureMatcherOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QSparseFeatureMatcherOptions ThisClass;
  typedef QSettingsWidget Base;
  typedef QEnumComboBox<FEATURE2D_MATCHER_TYPE> QSparseFeatureMatcherTypeCombo;

  QSparseFeatureMatcherOptions(QWidget * parent = Q_NULLPTR);

  void set_feature_matcher_options(c_feature2d_matcher_options * options);
  c_feature2d_matcher_options * feature_matcher_options() const;

protected:
  void onupdatecontrols() override;
  void show_current_matcher_controls();

protected:
  c_feature2d_matcher_options * options_ = Q_NULLPTR;
  QSparseFeatureMatcherTypeCombo * sparseFeatureMatcherType_ctl = Q_NULLPTR;
  QHammingDistanceFeature2dMatcherOptions * hammingDistanceFeature2dMatcherOptions_ctl = Q_NULLPTR;
  QFlannBasedFeature2dMatcherOptions * flannBasedFeature2dMatcherOptions_ctl = Q_NULLPTR;
  QTriangleMatcherOptions * triangleMatcherOptions_ctl = Q_NULLPTR;
  QSnormBasedFeature2dMatcherOptions * snormBasedFeature2dMatcherOptions_ctl = Q_NULLPTR;
};


class QFeatureBasedRegistrationOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QFeatureBasedRegistrationOptions ThisClass;
  typedef QSettingsWidget Base;

  QFeatureBasedRegistrationOptions(QWidget * parent = Q_NULLPTR);

  void set_registration_options(c_feature_based_registration_options * options);
  c_feature_based_registration_options * registration_options() const;

protected:
  void onupdatecontrols() override;
  void update_controls_state();

protected slots:
  void onDetectorTypeChanged();

protected:
  c_feature_based_registration_options * options_ = Q_NULLPTR;
  QCheckBox * enableFeatureBasedRegistration_ctl = Q_NULLPTR;
  QNumberEditBox * scale_ctl = Q_NULLPTR;
  QSparseFeatureDetectorOptions * sparseFeatureDetectorOptions_ctl = Q_NULLPTR;
  QSparseFeatureDescriptorOptions * sparseFeatureDescriptorOptions_ctl = Q_NULLPTR;
  QSparseFeatureMatcherOptions * sparseFeatureMatcherOptions_ctl = Q_NULLPTR;
  QWidgetList controls;

};


class QEccRegistrationOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QEccRegistrationOptions ThisClass;
  typedef QSettingsWidget Base;

  QEccRegistrationOptions(QWidget * parent = Q_NULLPTR);

  void set_registration_options(c_ecc_registration_options * options);
  c_ecc_registration_options* registration_options() const;

protected:
  void onupdatecontrols() override;
  void update_controls_state();

protected:
  c_ecc_registration_options * options_ = Q_NULLPTR;
  QCheckBox * enableEcc_ctl = Q_NULLPTR;
  QNumberEditBox * scale_ctl = Q_NULLPTR;
  QNumberEditBox * eps_ctl = Q_NULLPTR;
  QNumberEditBox * min_rho_ctl = Q_NULLPTR;
  QNumberEditBox * input_smooth_sigma_ctl = Q_NULLPTR;
  QNumberEditBox * reference_smooth_sigma_ctl = Q_NULLPTR;
  QNumberEditBox * update_step_scale_ctl = Q_NULLPTR;
  QNumberEditBox * normalization_noise_ctl = Q_NULLPTR;
  QNumberEditBox * normalization_scale_ctl = Q_NULLPTR;
  QNumberEditBox * max_iterations_ctl = Q_NULLPTR;
  QCheckBox * enable_ecch_ctl = Q_NULLPTR;
  QNumberEditBox * ecch_minimum_image_size_ctl = Q_NULLPTR;
  QWidgetList controls;
};


class QEccFlowRegistrationOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QEccFlowRegistrationOptions ThisClass;
  typedef QSettingsWidget Base;

  QEccFlowRegistrationOptions(QWidget * parent = Q_NULLPTR);

  void set_registration_options(c_eccflow_registration_options * options);
  c_eccflow_registration_options* registration_options() const;

protected:
  void onupdatecontrols() override;
  void update_controls_state();

protected:
  c_eccflow_registration_options * options_ = Q_NULLPTR;
  QCheckBox * enableEccFlow_ctl = Q_NULLPTR;
  QNumberEditBox * update_multiplier_ctl = Q_NULLPTR;
  QNumberEditBox * input_smooth_sigma_ctl = Q_NULLPTR;
  QNumberEditBox * reference_smooth_sigma_ctl = Q_NULLPTR;
  QNumberEditBox * max_iterations_ctl = Q_NULLPTR;
  QNumberEditBox * support_scale_ctl = Q_NULLPTR;
  QNumberEditBox * normalization_scale_ctl = Q_NULLPTR;
  QWidgetList controls;
};


class QJovianDerotationOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QJovianDerotationOptions ThisClass;
  typedef QSettingsWidget Base;

  QJovianDerotationOptions(QWidget * parent = Q_NULLPTR);

  void set_derotation_options(c_jovian_derotation_options * options);
  c_jovian_derotation_options* derotation_options() const;

protected:
  void onupdatecontrols() override;
  void update_controls_state();

protected:
  c_jovian_derotation_options *options_ = Q_NULLPTR;
  QCheckBox * enableJovianDerotation_ctl = Q_NULLPTR;
  QNumberEditBox * min_rotation_ctl = Q_NULLPTR;
  QNumberEditBox * max_rotation_ctl = Q_NULLPTR;
  QNumberEditBox * eccflow_support_scale_ctl = Q_NULLPTR;
  QNumberEditBox * eccflow_normalization_scale_ctl = Q_NULLPTR;
  QNumberEditBox * eccflow_max_pyramid_level_ctl = Q_NULLPTR;
  QCheckBox * align_jovian_disk_horizontally_ctl = Q_NULLPTR;
  QWidgetList controls;
};

class QImageRegistrationOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QImageRegistrationOptions ThisClass;
  typedef QSettingsWidget Base;
  typedef QEnumComboBox<ECC_MOTION_TYPE> QEccMotionTypeCombo;
  typedef QEnumComboBox<color_channel_type> QRegistrationColorChannelCombo;
  typedef QEnumComboBox<ECC_INTERPOLATION_METHOD> QEccInterpolatioMethodCombo;
  typedef QEnumComboBox<ECC_BORDER_MODE> QEccBorderModeCombo;

  QImageRegistrationOptions(QWidget * parent = Q_NULLPTR);

  void set_stack_options(const c_image_stacking_options::ptr & stack_options);
  const c_image_stacking_options::ptr & stack_options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_image_stacking_options::ptr stack_options_;
  c_frame_registration_options * options_ = nullptr;
  QEccMotionTypeCombo * motion_type_ctl = Q_NULLPTR;
  QRegistrationColorChannelCombo * registration_channel_ctl = Q_NULLPTR;
  QEccInterpolatioMethodCombo * interpolation_method_ctl = Q_NULLPTR;
  QEccBorderModeCombo * border_mode_ctl = Q_NULLPTR;
  QNumberEditBox * border_value_ctl = Q_NULLPTR;

  QMasterFrameOptions * masterFrameOptions_ctl = Q_NULLPTR;
  QFeatureBasedRegistrationOptions * featureRegistrationOptions_ctl = Q_NULLPTR;
  QEccRegistrationOptions * eccOptions_ctl = Q_NULLPTR;
  QEccFlowRegistrationOptions * eccFlowOptions_ctl = Q_NULLPTR;
  QJovianDerotationOptions * jovianDerotationOptions_ctl = Q_NULLPTR;
};


class QFrameRegistrationOptions:
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QFrameRegistrationOptions ThisClass;
  typedef QSettingsWidget Base;

  QFrameRegistrationOptions(QWidget * parent = Q_NULLPTR);

  void set_stack_options(const c_image_stacking_options::ptr & stack_options);
  const c_image_stacking_options::ptr & stack_options() const;

protected:
  void onupdatecontrols() override;
  void update_controls_visibility();

protected:
  c_image_stacking_options::ptr stack_options_;
  c_frame_registration_options * options_ = Q_NULLPTR;
  QCheckBox * enable_frame_registration_ctl = Q_NULLPTR;
  QImageRegistrationOptions * imageRegistrationOptions_ctl = Q_NULLPTR;
  QCheckBox * accumulateAndCompensateTurbulentFlow_ctl = Q_NULLPTR;
  QImageProcessorSelectionCombo * alignedFramesPostProcessor_ctl = Q_NULLPTR;
};

#endif /* __QFrameRegistrationSettings_h__ */
