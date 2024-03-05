/*
 * QFrameRegistrationSettings.h
 *
 *  Created on: Feb 9, 2021
 *      Author: amyznikov
 */

#ifndef __QFrameRegistrationSettings_h__
#define __QFrameRegistrationSettings_h__

#include <gui/widgets/UpdateControls.h>
#include <gui/qfeature2d/QFeature2dOptions.h>
#include <gui/qpipeline/QInputSourceSelectionControl.h>
#include <gui/widgets/QMatrixEdit.h>
#include <core/pipeline/c_image_processing_pipeline.h>
#include <core/proc/image_registration/c_frame_registration.h>

class QEstimateImageTransformOptionsBase :
    public QSettingsWidgetTemplate<c_estimate_image_transform_options>
{
public:
  typedef QEstimateImageTransformOptionsBase ThisClass;
  typedef QSettingsWidgetTemplate<c_estimate_image_transform_options> Base;

  QEstimateImageTransformOptionsBase(QWidget * parent = nullptr) :
    Base(parent)
  {
  }
};

class QEstimateTranslationImageTransformOptions :
    public QEstimateImageTransformOptionsBase
{
  Q_OBJECT;
public:
  typedef QEstimateTranslationImageTransformOptions ThisClass;
  typedef QEstimateImageTransformOptionsBase Base;

  QEstimateTranslationImageTransformOptions(QWidget * parent = nullptr);

protected:
  QNumericBox * rmse_factor_ctl = nullptr;
  QNumericBox * max_iterations_ctl = nullptr;
};

class QEstimateEuclideanImageTransformOptions :
    public QEstimateImageTransformOptionsBase
{
  Q_OBJECT;
public:
  typedef QEstimateEuclideanImageTransformOptions ThisClass;
  typedef QEstimateImageTransformOptionsBase Base;

  QEstimateEuclideanImageTransformOptions(QWidget * parent = nullptr);

protected:
  QNumericBox * rmse_threshold_ctl = nullptr;
  QNumericBox * max_iterations_ctl = nullptr;
};

class QEstimateScaledEuclideanImageTransformOptions :
    public QEstimateImageTransformOptionsBase
{
  Q_OBJECT;
public:
  typedef QEstimateScaledEuclideanImageTransformOptions ThisClass;
  typedef QEstimateImageTransformOptionsBase Base;

  QEstimateScaledEuclideanImageTransformOptions(QWidget * parent = nullptr);

protected:
  QEnumComboBox<ROBUST_METHOD> * method_ctl = nullptr;
  QNumericBox * ransacReprojThreshold_ctl = nullptr;
  QNumericBox * confidence_ctl = nullptr;
  QNumericBox * maxIters_ctl = nullptr;
  QNumericBox * refineIters_ctl = nullptr;
};

class QEstimateAffineImageTransformOptions :
    public QEstimateImageTransformOptionsBase
{
  Q_OBJECT;
public:
  typedef QEstimateAffineImageTransformOptions ThisClass;
  typedef QEstimateImageTransformOptionsBase Base;

  QEstimateAffineImageTransformOptions(QWidget * parent = nullptr);

protected:
  QEnumComboBox<ROBUST_METHOD> * method_ctl = nullptr;
  QNumericBox * ransacReprojThreshold_ctl = nullptr;
  QNumericBox * maxIters_ctl = nullptr;
  QNumericBox * confidence_ctl = nullptr;
  QNumericBox * refineIters_ctl = nullptr;
};

class QEstimateHomographyImageTransformOptions :
    public QEstimateImageTransformOptionsBase
{
  Q_OBJECT;
public:
  typedef QEstimateHomographyImageTransformOptions ThisClass;
  typedef QEstimateImageTransformOptionsBase Base;

  QEstimateHomographyImageTransformOptions(QWidget * parent = nullptr);

protected:
  QEnumComboBox<ROBUST_METHOD> * method_ctl = nullptr;
  QNumericBox * ransacReprojThreshold_ctl = nullptr;
  QNumericBox * maxIters_ctl = nullptr;
  QNumericBox * confidence_ctl = nullptr;
};

class QEstimateSemiQuadraticImageTransformOptions :
    public QEstimateImageTransformOptionsBase
{
  Q_OBJECT;
public:
  typedef QEstimateSemiQuadraticImageTransformOptions ThisClass;
  typedef QEstimateImageTransformOptionsBase Base;

  QEstimateSemiQuadraticImageTransformOptions(QWidget * parent = nullptr);

protected:
  QNumericBox * rmse_factor_ctl = nullptr;
};

class QEstimateQuadraticImageTransformOptions :
    public QEstimateImageTransformOptionsBase
{
  Q_OBJECT;
public:
  typedef QEstimateQuadraticImageTransformOptions ThisClass;
  typedef QEstimateImageTransformOptionsBase Base;

  QEstimateQuadraticImageTransformOptions(QWidget * parent = nullptr);

protected:
  QNumericBox * rmse_factor_ctl = nullptr;
};

class QEstimateEpipolarDerotationImageTransformOptions :
    public QEstimateImageTransformOptionsBase
{
  Q_OBJECT;
public:
  typedef QEstimateEpipolarDerotationImageTransformOptions ThisClass;
  typedef QEstimateImageTransformOptionsBase Base;

  QEstimateEpipolarDerotationImageTransformOptions(QWidget * parent = nullptr);

protected:
  QEnumComboBox<EPIPOLAR_MOTION_DIRECTION> * direction_ctl = nullptr;
  QNumericBox * max_iterations_ctl = nullptr;
  QNumericBox * max_levmar_iterations_ctl = nullptr;
  QNumericBox * robust_threshold_ctl = nullptr;
  QNumericBox * epsf_ctl = nullptr;
  QNumericBox * epsx_ctl = nullptr;
};



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

protected Q_SLOTS:
  void onDetectorTypeChanged();

protected:
  c_feature_based_registration_options * options_ = nullptr;
  QCheckBox * enableFeatureBasedRegistration_ctl = nullptr;

  QNumericBox * scale_ctl = nullptr;
  QSparseFeatureDetectorOptions * sparseFeatureDetectorOptions_ctl = nullptr;
  QSparseFeatureDescriptorOptions * sparseFeatureDescriptorOptions_ctl = nullptr;
  QSparseFeatureMatcherOptions * sparseFeatureMatcherOptions_ctl = nullptr;

  QExpandableGroupBox * transformOptionsGroup_ctl = nullptr;
  QSettingsWidget * transformOptionsSettings_ctl = nullptr;

  QEstimateTranslationImageTransformOptions *estimateTranslation_ctl = nullptr;
  QEstimateEuclideanImageTransformOptions *estimateEuclidean_ctl = nullptr;
  QEstimateScaledEuclideanImageTransformOptions *estimateScaledEuclidean_ctl = nullptr;
  QEstimateAffineImageTransformOptions *estimateAffine_ctl = nullptr;
  QEstimateHomographyImageTransformOptions *estimateHomography_ctl = nullptr;
  QEstimateSemiQuadraticImageTransformOptions *estimateSemiQuadratic_ctl = nullptr;
  QEstimateQuadraticImageTransformOptions *estimateQuadratic_ctl = nullptr;
  QEstimateEpipolarDerotationImageTransformOptions *estimateEpipolarDerotation_ctl = nullptr;

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
  QNumericBox * max_pyramid_level_ctl = nullptr;
  QNumericBox * normalization_scale_ctl = nullptr;
  QNumericBox * noise_level_ctl = nullptr;
  QCheckBox * enable_debug_ctl = nullptr;

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

  QNumericBox * jovian_detector_stdev_factor_ctl = nullptr;
  QNumericBox * jovian_detector_pca_blur_ctl = nullptr;
  QNumericBox * jovian_detector_ellipse_offset_ctl = nullptr;

  QNumericBox * min_rotation_ctl = nullptr;
  QNumericBox * max_rotation_ctl = nullptr;
  QNumericBox * max_pyramid_level_ctl = nullptr;
  QNumericBox * num_orientations_ctl = nullptr;

  QCheckBox * derotate_all_frames_ctl = nullptr;
  QNumericBox * max_context_size_ctl = nullptr;
  QCheckBox * align_jovian_disk_horizontally_ctl = nullptr;

  QWidgetList controls;
};


class QMasterSourceSelectionCombo :
    public QWidget,
    public HasUpdateControls
{
  Q_OBJECT;
public:
  typedef QMasterSourceSelectionCombo ThisClass;
  typedef QWidget Base;

  struct InputSourceData {
    std::string source_pathfilename;
    int source_size = 0;
  };

  QMasterSourceSelectionCombo(QWidget * parent = nullptr);

  void refreshInputSources(c_image_processing_pipeline * pipeline);
  void setEnableExternalFile(bool v);
  bool enableExternalFile() const;

  void setCurrentInputSource(const std::string & pathfilename);
  InputSourceData currentInputSource() const;

Q_SIGNALS:
  void currentSourceChanged();

protected Q_SLOTS:
  void onBrowseButtonClicked();
  void onComboboxCurrentIndexChanged(int index);

protected:
  QComboBox * combo_ = nullptr;
  QToolButton * browse_ctl = nullptr;
};

// must be declared outside of any namespace
Q_DECLARE_METATYPE(QMasterSourceSelectionCombo::InputSourceData);


class QMasterFrameOptions :
    public QSettingsWidget,
    public QInputSourceSelectionControl
{
  Q_OBJECT;
public:
  typedef QMasterFrameOptions ThisClass;
  typedef QSettingsWidget Base;

  QMasterFrameOptions(QWidget * parent = nullptr);

//  void set_current_pipeline(c_image_stacking_pipeline * current_pipeline);
//  c_image_stacking_pipeline * current_pipeline() const;

  void set_master_frame_options(c_master_frame_options * options);
  c_master_frame_options * master_frame_options() const;

  void refreshInputSources(c_image_processing_pipeline * pipeline) override;
  void setEnableExternalFile(bool v) override;
  bool enableExternalFile() const override;

protected:
  void onupdatecontrols() override;
  //QString browseForMasterFrame();
  //QString browseForMasterFFTSPath();
  //void updateMasterSourceBasingOnComboboxItemIndex(int comboboxItemIndex);
  //void updateMasterFrameIndex();

protected Q_SLOTS:
  //void onMasterSourceComboCurrentIndexChanged(int);
  void updateMasterSourceControlStates();
  //void onSpinBoxValueChanged(int value);
  void updateGenerateMasterFrameControlStates();
  //void onEccFlowScaleChanged();
  //void onMasterSharpenFactorChanged();
  //void onAccumulatedSharpenFactorChanged();
  //void onSaveMasterFrameCheckboxStateChanged(int);
  //void onApplyInputFramePprocessorCheckboxStateChanged(int);

protected:
  c_master_frame_options * options_ = nullptr;

  QEnumComboBox<master_frame_selection_method> * masterFrameSelectionMethod_ctl = nullptr;
  // QInputSourceSelectionCombo * masterSource_ctl = nullptr;
  QMasterSourceSelectionCombo * masterSource_ctl = nullptr;
  QSpinBox * masterFrameIndex_ctl = nullptr;
  QCheckBox * apply_input_frame_processors_ctl = nullptr;
  QCheckBox * generateMasterFrame_ctl = nullptr;
  QNumericBox * maxFramesForMasterFrameGeneration_ctl = nullptr;

//  QNumericBox * featureScale_ctl = nullptr;
//  QNumericBox * eccScale_ctl = nullptr;
  QNumericBox * eccFlowScale_ctl = nullptr;
  QNumericBox * eccflowMaxPyramidLevel_ctl = nullptr;

  QNumericBox * master_sharpen_factor_ctl = nullptr;
  QNumericBox * accumulated_sharpen_factor_ctl = nullptr;

  //QCheckBox * compensateMasterFlow_ctl = nullptr;
  QCheckBox * saveMasterFrame_ctl = nullptr;

  //QToolButton * applyToAll_ctl = nullptr;
  //int previousComboboxItemIndex = -1;
};

class QImageRegistrationOptions :
    public QSettingsWidget,
    public QInputSourceSelectionControl
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

  void set_registration_options(c_image_registration_options * options);
  c_image_registration_options* registration_options() const;

  void refreshInputSources(c_image_processing_pipeline * pipeline) override;
  void setEnableExternalFile(bool v) override;
  bool enableExternalFile() const override;

protected:
  void onupdatecontrols() override;

protected:
  c_image_registration_options * options_ = nullptr;
  QMotionTypeCombo * motion_type_ctl = nullptr;
  QRegistrationColorChannelCombo * registration_channel_ctl = nullptr;
  QEccInterpolatioMethodCombo * interpolation_method_ctl = nullptr;
  QEccBorderModeCombo * border_mode_ctl = nullptr;
  QNumericBox * border_value_ctl = nullptr;
  QCheckBox * accumulateAndCompensateTurbulentFlow_ctl = nullptr;
  QMatrixEdit * camera_matrix_ctl = nullptr;
  QExpandableGroupBox * camera_matrix_groupbox_ctl = nullptr;

  QMasterFrameOptions * masterFrameOptions_ctl = nullptr;
  QFeatureBasedRegistrationOptions * featureRegistrationOptions_ctl = nullptr;
  QEccRegistrationOptions * eccOptions_ctl = nullptr;
  QJovianDerotationOptions * jovianDerotationOptions_ctl = nullptr;
  QEccFlowRegistrationOptions * eccFlowOptions_ctl = nullptr;
};


class QFrameRegistrationOptions:
    public QSettingsWidget,
    public QInputSourceSelectionControl
{
  Q_OBJECT;
public:
  typedef QFrameRegistrationOptions ThisClass;
  typedef QSettingsWidget Base;

  QFrameRegistrationOptions(QWidget * parent = nullptr);

  void set_registration_options(c_image_registration_options * options);
  c_image_registration_options* registration_options() const;

  void refreshInputSources(c_image_processing_pipeline * pipeline) override;
  void setEnableExternalFile(bool v) override;
  bool enableExternalFile() const override;

protected:
  void onupdatecontrols() override;
  void update_controls_visibility();

protected:
  c_image_registration_options * options_ = nullptr;
  QCheckBox * enable_frame_registration_ctl = nullptr;
  QImageRegistrationOptions * imageRegistrationOptions_ctl = nullptr;
};

#endif /* __QFrameRegistrationSettings_h__ */
