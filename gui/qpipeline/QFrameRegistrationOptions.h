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
#include <gui/widgets/QCameraIntrinsicsEditBox.h>
#include <core/pipeline/c_image_processing_pipeline.h>
#include <core/proc/image_registration/c_frame_registration.h>



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


class QMasterFrameSelectionControl :
    public QSettingsWidgetTemplate<c_master_frame_selection_options>,
    public QInputSourceSelectionControl
{
  Q_OBJECT;
public:
  typedef QMasterFrameSelectionControl ThisClass;
  typedef QSettingsWidgetTemplate<c_master_frame_selection_options> Base;

  QMasterFrameSelectionControl(QWidget * parent = nullptr);

  void refreshInputSources(c_image_processing_pipeline * pipeline) override;
  void setEnableExternalFile(bool v) override;
  bool enableExternalFile() const override;

protected Q_SLOTS:
  void updateMasterSourceControlStates();


protected:
  QEnumComboBox<master_frame_selection_method> * masterFrameSelectionMethod_ctl = nullptr;
  QMasterSourceSelectionCombo * masterSource_ctl = nullptr;
  QSpinBox * masterFrameIndex_ctl = nullptr;
};


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
  QNumericBox * initial_translation_ctl = nullptr;
  QNumericBox * initial_rotation_ctl = nullptr;
};



class QFeatureBasedRegistrationOptions :
    public QSettingsWidgetTemplate<c_feature_registration_options>
{
  Q_OBJECT;
public:
  typedef QFeatureBasedRegistrationOptions ThisClass;
  typedef QSettingsWidgetTemplate<c_feature_registration_options> Base;

  QFeatureBasedRegistrationOptions(QWidget * parent = nullptr);

protected:
  void onupdatecontrols() override;

protected Q_SLOTS:
  void onDetectorTypeChanged();

protected:
  QEnumComboBox<color_channel_type> * registrationChannel_ctl = nullptr;
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
};


class QEccRegistrationOptions :
    public QSettingsWidgetTemplate<c_ecc_registration_options>
{
  Q_OBJECT;
public:
  typedef QEccRegistrationOptions ThisClass;
  typedef QSettingsWidgetTemplate<c_ecc_registration_options> Base;

  QEccRegistrationOptions(QWidget * parent = nullptr);

protected:
  void onupdatecontrols() override;

protected:
  QNumericBox * scale_ctl = nullptr;
  QEnumComboBox<ECC_ALIGN_METHOD> * ecc_method_ctl = nullptr;
  QNumericBox * eps_ctl = nullptr;
  QNumericBox * min_rho_ctl = nullptr;
  QNumericBox * input_smooth_sigma_ctl = nullptr;
  QNumericBox * reference_smooth_sigma_ctl = nullptr;
  QNumericBox * normalization_scale_ctl = nullptr;
  QNumericBox * normalization_noise_ctl = nullptr;
  QNumericBox * update_step_scale_ctl = nullptr;
  QNumericBox * max_iterations_ctl = nullptr;
  QNumericBox * ecch_max_level_ctl = nullptr;
  QNumericBox * ecch_minimum_image_size_ctl = nullptr;
  QCheckBox * ecch_estimate_translation_first_ctl = nullptr;
  QCheckBox * replace_planetary_disk_with_mask_ctl = nullptr;
  QNumericBox * planetary_disk_mask_stdev_factor_ctl = nullptr;
  QNumericBox * se_close_size_ctl = nullptr;
};


class QEccFlowRegistrationOptions :
    public QSettingsWidgetTemplate<c_eccflow_registration_options>
{
  Q_OBJECT;
public:
  typedef QEccFlowRegistrationOptions ThisClass;
  typedef QSettingsWidgetTemplate<c_eccflow_registration_options> Base;

  QEccFlowRegistrationOptions(QWidget * parent = nullptr);

protected:
  void onupdatecontrols() override;

protected:
  QEnumComboBox<c_eccflow::DownscaleMethod> * downscale_method_ctl = nullptr;
  QNumericBox * update_multiplier_ctl = nullptr;
  QNumericBox * input_smooth_sigma_ctl = nullptr;
  QNumericBox * reference_smooth_sigma_ctl = nullptr;
  QNumericBox * max_iterations_ctl = nullptr;
  QNumericBox * support_scale_ctl = nullptr;
  QNumericBox * min_image_size_ctl = nullptr;
  QNumericBox * max_pyramid_level_ctl = nullptr;
  QNumericBox * noise_level_ctl = nullptr;
  QNumericBox * scale_factor_ctl = nullptr;
};


class QJovianDerotationOptions :
    public QSettingsWidgetTemplate<c_jovian_derotation_options>
{
  Q_OBJECT;
public:
  typedef QJovianDerotationOptions ThisClass;
  typedef QSettingsWidgetTemplate<c_jovian_derotation_options> Base;

  QJovianDerotationOptions(QWidget * parent = nullptr);

protected:
  void onupdatecontrols() override;

protected:
  QNumericBox * jovian_detector_stdev_factor_ctl = nullptr;
  QNumericBox * jovian_detector_pca_blur_ctl = nullptr;
  QNumericBox * jovian_detector_ellipse_offset_ctl = nullptr;

//  QNumericBox * min_rotation_ctl = nullptr;
//  QNumericBox * max_rotation_ctl = nullptr;
//  QNumericBox * max_pyramid_level_ctl = nullptr;
//  QNumericBox * num_orientations_ctl = nullptr;

  QCheckBox * derotate_all_frames_ctl = nullptr;
  QNumericBox * max_context_size_ctl = nullptr;
//  QCheckBox * align_jovian_disk_horizontally_ctl = nullptr;
};


class QSaturnDerotationOptions :
    public QSettingsWidgetTemplate<c_saturn_derotation_options>
{
  Q_OBJECT;
public:
  typedef QSaturnDerotationOptions ThisClass;
  typedef QSettingsWidgetTemplate<c_saturn_derotation_options> Base;

  QSaturnDerotationOptions(QWidget * parent = nullptr);


protected:
//  QNumericBox * jovian_detector_stdev_factor_ctl = nullptr;
//  QNumericBox * jovian_detector_pca_blur_ctl = nullptr;
//  QNumericBox * jovian_detector_ellipse_offset_ctl = nullptr;
//
//  QNumericBox * min_rotation_ctl = nullptr;
//  QNumericBox * max_rotation_ctl = nullptr;
//  QNumericBox * max_pyramid_level_ctl = nullptr;
//  QNumericBox * num_orientations_ctl = nullptr;
//
//  QCheckBox * derotate_all_frames_ctl = nullptr;
//  QNumericBox * max_context_size_ctl = nullptr;
//  QCheckBox * align_jovian_disk_horizontally_ctl = nullptr;
};





#endif /* __QFrameRegistrationSettings_h__ */
