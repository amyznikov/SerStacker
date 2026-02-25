/*
 * QFrameRegistrationSettings.cc
 *
 *  Created on: Feb 9, 2021
 *      Author: amyznikov
 */

#include "QFrameRegistrationOptions.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QEstimateTranslationImageTransformOptions::QEstimateTranslationImageTransformOptions(QWidget * parent) :
    Base(parent)
{
  max_iterations_ctl =
      add_numeric_box<int>("max_iterations",
          "max number of iterations for outliers removal",
          [this](int v) {
            if ( _opts && _opts->translation.max_iterations != v ) {
              _opts->translation.max_iterations = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _opts ) {
              *v = _opts->translation.max_iterations;
              return true;
            }
            return false;
          });

  rmse_factor_ctl =
      add_numeric_box<double>("rmse_factor",
          "rmse factor for outliers removal",
          [this](double v) {
            if ( _opts && _opts->translation.rmse_factor != v ) {
              _opts->translation.rmse_factor = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _opts ) {
              *v = _opts->translation.rmse_factor;
              return true;
            }
            return false;
          });

  updateControls();
}

QEstimateEuclideanImageTransformOptions::QEstimateEuclideanImageTransformOptions(QWidget * parent) :
    Base(parent)
{
  max_iterations_ctl =
      add_numeric_box<int>("max_iterations",
          "max number of iterations for outliers removal",
          [this](int v) {
            if ( _opts && _opts->euclidean.max_iterations != v ) {
              _opts->euclidean.max_iterations = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _opts ) {
              *v = _opts->euclidean.max_iterations;
              return true;
            }
            return false;
          });

  rmse_threshold_ctl =
      add_numeric_box<double>("rmse_threshold",
          "",
          [this](double v) {
            if ( _opts && _opts->euclidean.rmse_threshold != v ) {
              _opts->euclidean.rmse_threshold = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _opts ) {
              *v = _opts->euclidean.rmse_threshold;
              return true;
            }
            return false;
          });

  updateControls();
}

QEstimateScaledEuclideanImageTransformOptions::QEstimateScaledEuclideanImageTransformOptions(QWidget * parent) :
    Base(parent)
{
  method_ctl =
      add_enum_combobox<ROBUST_METHOD>("ROBUST METHOD",
          "",
          [this](ROBUST_METHOD v) {
            if ( _opts && _opts->scaled_euclidean.method != v ) {
              _opts->scaled_euclidean.method = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](ROBUST_METHOD * v) {
            if ( _opts ) {
              *v = _opts->scaled_euclidean.method;
              return true;
            }
            return false;
          });

  ransacReprojThreshold_ctl =
      add_numeric_box<double>("ransacReprojThreshold",
          "",
          [this](double v) {
            if ( _opts && _opts->scaled_euclidean.ransacReprojThreshold != v ) {
              _opts->scaled_euclidean.ransacReprojThreshold = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _opts ) {
              *v = _opts->scaled_euclidean.ransacReprojThreshold;
              return true;
            }
            return false;
          });

  confidence_ctl =
      add_numeric_box<double>("confidence",
          "",
          [this](double v) {
            if ( _opts && _opts->scaled_euclidean.confidence != v ) {
              _opts->scaled_euclidean.confidence = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _opts ) {
              *v = _opts->scaled_euclidean.confidence;
              return true;
            }
            return false;
          });

  maxIters_ctl =
      add_numeric_box<int>("maxIters",
          "",
          [this](int v) {
            if ( _opts && _opts->scaled_euclidean.maxIters != v ) {
              _opts->scaled_euclidean.maxIters = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _opts ) {
              *v = _opts->scaled_euclidean.maxIters;
              return true;
            }
            return false;
          });

  refineIters_ctl =
      add_numeric_box<int>("refineIters",
          "",
          [this](int v) {
            if ( _opts && _opts->scaled_euclidean.refineIters != v ) {
              _opts->scaled_euclidean.refineIters = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _opts ) {
              *v = _opts->scaled_euclidean.refineIters;
              return true;
            }
            return false;
          });

  updateControls();
}

QEstimateAffineImageTransformOptions::QEstimateAffineImageTransformOptions(QWidget * parent) :
    Base(parent)
{
  method_ctl =
      add_enum_combobox<ROBUST_METHOD>("ROBUST METHOD",
          "",
          [this](ROBUST_METHOD v) {
            if ( _opts && _opts->affine.method != v ) {
              _opts->affine.method = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](ROBUST_METHOD * v) {
            if ( _opts ) {
              *v = _opts->affine.method;
              return true;
            }
            return false;
          });

  ransacReprojThreshold_ctl =
      add_numeric_box<double>("ransacReprojThreshold",
          "",
          [this](double v) {
            if ( _opts && _opts->affine.ransacReprojThreshold != v ) {
              _opts->affine.ransacReprojThreshold = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _opts ) {
              *v = _opts->affine.ransacReprojThreshold;
              return true;
            }
            return false;
          });

  confidence_ctl =
      add_numeric_box<double>("confidence",
          "",
          [this](double v) {
            if ( _opts && _opts->affine.confidence != v ) {
              _opts->affine.confidence = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _opts ) {
              *v = _opts->affine.confidence;
              return true;
            }
            return false;
          });

  maxIters_ctl =
      add_numeric_box<int>("maxIters",
          "",
          [this](int v) {
            if ( _opts && _opts->affine.maxIters != v ) {
              _opts->affine.maxIters = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _opts ) {
              *v = _opts->affine.maxIters;
              return true;
            }
            return false;
          });

  refineIters_ctl =
      add_numeric_box<int>("refineIters",
          "",
          [this](int v) {
            if ( _opts && _opts->affine.refineIters != v ) {
              _opts->affine.refineIters = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _opts ) {
              *v = _opts->affine.refineIters;
              return true;
            }
            return false;
          });

  updateControls();
}

QEstimateHomographyImageTransformOptions::QEstimateHomographyImageTransformOptions(QWidget * parent) :
    Base(parent)
{
  method_ctl =
      add_enum_combobox<ROBUST_METHOD>("ROBUST METHOD",
          "",
          [this](ROBUST_METHOD v) {
            if ( _opts && _opts->homography.method != v ) {
              _opts->homography.method = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](ROBUST_METHOD * v) {
            if ( _opts ) {
              *v = _opts->homography.method;
              return true;
            }
            return false;
          });

  ransacReprojThreshold_ctl =
      add_numeric_box<double>("ransacReprojThreshold",
          "",
          [this](double v) {
            if ( _opts && _opts->homography.ransacReprojThreshold != v ) {
              _opts->homography.ransacReprojThreshold = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _opts ) {
              *v = _opts->homography.ransacReprojThreshold;
              return true;
            }
            return false;
          });

  confidence_ctl =
      add_numeric_box<double>("confidence",
          "",
          [this](double v) {
            if ( _opts && _opts->homography.confidence != v ) {
              _opts->homography.confidence = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _opts ) {
              *v = _opts->homography.confidence;
              return true;
            }
            return false;
          });

  maxIters_ctl =
      add_numeric_box<int>("maxIters",
          "",
          [this](int v) {
            if ( _opts && _opts->homography.maxIters != v ) {
              _opts->homography.maxIters = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _opts ) {
              *v = _opts->homography.maxIters;
              return true;
            }
            return false;
          });

  updateControls();
}

QEstimateSemiQuadraticImageTransformOptions::QEstimateSemiQuadraticImageTransformOptions(QWidget * parent) :
    Base(parent)
{
  rmse_factor_ctl =
      add_numeric_box<double>("rmse_factor",
          "",
          [this](double v) {
            if ( _opts && _opts->semi_quadratic.rmse_factor != v ) {
              _opts->semi_quadratic.rmse_factor = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _opts ) {
              *v = _opts->semi_quadratic.rmse_factor;
              return true;
            }
            return false;
          });

  updateControls();
}

QEstimateQuadraticImageTransformOptions::QEstimateQuadraticImageTransformOptions(QWidget * parent) :
    Base(parent)
{
  rmse_factor_ctl =
      add_numeric_box<double>("rmse_factor",
          "",
          [this](double v) {
            if ( _opts && _opts->quadratic.rmse_factor != v ) {
              _opts->quadratic.rmse_factor = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _opts ) {
              *v = _opts->quadratic.rmse_factor;
              return true;
            }
            return false;
          });

  updateControls();
}

QEstimateEpipolarDerotationImageTransformOptions::QEstimateEpipolarDerotationImageTransformOptions(QWidget * parent) :
    Base(parent)
{
  direction_ctl =
      add_enum_combobox<EPIPOLAR_MOTION_DIRECTION>("Motion direction",
          "Optimize assuming a priori known motion direction",
          [this](EPIPOLAR_MOTION_DIRECTION v) {
            if ( _opts && _opts->epipolar_derotation.camera_pose.direction != v ) {
              _opts->epipolar_derotation.camera_pose.direction = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](EPIPOLAR_MOTION_DIRECTION * v) {
            if ( _opts ) {
              *v = _opts->epipolar_derotation.camera_pose.direction;
              return true;
            }
            return false;
          });

  max_iterations_ctl =
      add_numeric_box<int>("max_iterations",
          "",
          [this](int v) {
            if ( _opts && _opts->epipolar_derotation.camera_pose.max_iterations != v ) {
              _opts->epipolar_derotation.camera_pose.max_iterations = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _opts ) {
              *v = _opts->epipolar_derotation.camera_pose.max_iterations;
              return true;
            }
            return false;
          });

  max_levmar_iterations_ctl =
      add_numeric_box<int>("max_levmar_iterations",
          "",
          [this](int v) {
            if ( _opts && _opts->epipolar_derotation.camera_pose.max_levmar_iterations != v ) {
              _opts->epipolar_derotation.camera_pose.max_levmar_iterations = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _opts ) {
              *v = _opts->epipolar_derotation.camera_pose.max_levmar_iterations;
              return true;
            }
            return false;
          });

  robust_threshold_ctl =
      add_numeric_box<double>("robust_threshold",
          "",
          [this](double v) {
            if ( _opts && _opts->epipolar_derotation.camera_pose.robust_threshold != v ) {
              _opts->epipolar_derotation.camera_pose.robust_threshold = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _opts ) {
              *v = _opts->epipolar_derotation.camera_pose.robust_threshold;
              return true;
            }
            return false;
          });

  epsf_ctl =
      add_numeric_box<double>("epsf",
          "",
          [this](double v) {
            if ( _opts && _opts->epipolar_derotation.camera_pose.epsf != v ) {
              _opts->epipolar_derotation.camera_pose.epsf = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _opts ) {
              *v = _opts->epipolar_derotation.camera_pose.epsf;
              return true;
            }
            return false;
          });

  epsx_ctl =
      add_numeric_box<double>("epsx",
          "",
          [this](double v) {
            if ( _opts && _opts->epipolar_derotation.camera_pose.epsx != v ) {
              _opts->epipolar_derotation.camera_pose.epsx = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _opts ) {
              *v = _opts->epipolar_derotation.camera_pose.epsx;
              return true;
            }
            return false;
          });

  initial_translation_ctl =
      add_numeric_box<cv::Vec3f>("initial_translation",
          "",
          [this](const cv::Vec3f & v) {
            if ( _opts && _opts->epipolar_derotation.initial_translation != v ) {
              _opts->epipolar_derotation.initial_translation = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Vec3f * v) {
            if ( _opts ) {
              *v = _opts->epipolar_derotation.initial_translation;
              return true;
            }
            return false;
          });

  initial_rotation_ctl =
      add_numeric_box<cv::Vec3f>("initial_rotation [deg]",
          "",
          [this](const cv::Vec3f & v) {
            if ( _opts && _opts->epipolar_derotation.initial_rotation != v ) {
              _opts->epipolar_derotation.initial_rotation = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Vec3f * v) {
            if ( _opts ) {
              *v = _opts->epipolar_derotation.initial_rotation;
              return true;
            }
            return false;
          });


  updateControls();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QFeatureBasedRegistrationOptions::QFeatureBasedRegistrationOptions(QWidget * parent) :
    Base(parent)
{
  registrationChannel_ctl =
      add_enum_combobox<color_channel_type>("Channel:",
          "color channel for feature2d extraction",
          [this](color_channel_type value) {
            if ( _opts && _opts->registration_channel != value ) {
              _opts->registration_channel = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](color_channel_type * value) {
            if ( _opts ) {
              * value = _opts->registration_channel;
              return true;
            }
            return false;
          });

  scale_ctl =
      add_numeric_box<double>("Image scale",
          "Scale input image before feature extraction",
          [this](double value) {
            if ( _opts && _opts->image_scale != value ) {
              _opts->image_scale = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * value) {
            if ( _opts ) {
              * value = _opts->image_scale;
              return true;
            }
            return false;
          });

  triangle_eps_ctl =
      add_numeric_box<double>("Triangle_eps [px]",
          "Max distance in pixels between current and remapped reference keypoints for triangle-based transfrom reestimation",
          [this](double value) {
            if ( _opts && _opts->triangle_eps != value ) {
              _opts->triangle_eps = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * value) {
            if ( _opts ) {
              * value = _opts->triangle_eps;
              return true;
            }
            return false;
          });

  add_expandable_groupbox("Sparse Feature Detector Options",
      sparseFeatureDetectorOptions_ctl = new QSparseFeatureDetectorOptions());
  connect(sparseFeatureDetectorOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);
  connect(sparseFeatureDetectorOptions_ctl, &QSparseFeatureDetectorOptions::detectorTypeChanged,
      this, &ThisClass::onDetectorTypeChanged);

  add_expandable_groupbox("Sparse Feature Descriptor Options",
      sparseFeatureDescriptorOptions_ctl = new QSparseFeatureDescriptorOptions());
  connect(sparseFeatureDescriptorOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Sparse Feature Matcher Options",
      sparseFeatureMatcherOptions_ctl = new QSparseFeatureMatcherOptions());
  connect(sparseFeatureMatcherOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  transformOptionsGroup_ctl =
      add_expandable_groupbox("Transform Estimation ...",
          transformOptionsSettings_ctl = new QSettingsWidget(this));

  ///
  transformOptionsSettings_ctl->add_expandable_groupbox("Estimate Translation",
      estimateTranslation_ctl = new QEstimateTranslationImageTransformOptions());
  connect(estimateTranslation_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  transformOptionsSettings_ctl->add_expandable_groupbox("Estimate Euclidean ",
      estimateEuclidean_ctl = new QEstimateEuclideanImageTransformOptions());
  connect(estimateEuclidean_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  transformOptionsSettings_ctl->add_expandable_groupbox("Estimate ScaledEuclidean",
      estimateScaledEuclidean_ctl = new QEstimateScaledEuclideanImageTransformOptions());
  connect(estimateScaledEuclidean_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  transformOptionsSettings_ctl->add_expandable_groupbox("Estimate Affine",
      estimateAffine_ctl = new QEstimateAffineImageTransformOptions());
  connect(estimateAffine_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  transformOptionsSettings_ctl->add_expandable_groupbox("Estimate Homography",
      estimateHomography_ctl = new QEstimateHomographyImageTransformOptions());
  connect(estimateHomography_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  transformOptionsSettings_ctl->add_expandable_groupbox("Estimate SemiQuadratic",
      estimateSemiQuadratic_ctl = new QEstimateSemiQuadraticImageTransformOptions());
  connect(estimateSemiQuadratic_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  transformOptionsSettings_ctl->add_expandable_groupbox("Estimate Quadratic",
      estimateQuadratic_ctl = new QEstimateQuadraticImageTransformOptions());
  connect(estimateQuadratic_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  transformOptionsSettings_ctl->add_expandable_groupbox("Estimate EpipolarDerotation",
      estimateEpipolarDerotation_ctl = new QEstimateEpipolarDerotationImageTransformOptions());
  connect(estimateEpipolarDerotation_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  ///

  QObject::connect(this, &ThisClass::populatecontrols,
      [this]() {
        sparseFeatureDetectorOptions_ctl->setOpts(_opts ? &_opts->sparse_feature_extractor_and_matcher.detector : nullptr);
        sparseFeatureDescriptorOptions_ctl->setOpts(_opts ? &_opts->sparse_feature_extractor_and_matcher.descriptor : nullptr);
        sparseFeatureMatcherOptions_ctl->setOpts(_opts ? &_opts->sparse_feature_extractor_and_matcher.matcher : nullptr);
        estimateTranslation_ctl->setOpts(_opts ? &_opts->estimate_options : nullptr);
        estimateEuclidean_ctl->setOpts(_opts ? &_opts->estimate_options : nullptr);
        estimateScaledEuclidean_ctl->setOpts(_opts ? &_opts->estimate_options : nullptr);
        estimateAffine_ctl->setOpts(_opts ? &_opts->estimate_options : nullptr);
        estimateHomography_ctl->setOpts(_opts ? &_opts->estimate_options : nullptr);
        estimateSemiQuadratic_ctl->setOpts(_opts ? &_opts->estimate_options : nullptr);
        estimateQuadratic_ctl->setOpts(_opts ? &_opts->estimate_options : nullptr);
        estimateEpipolarDerotation_ctl->setOpts(_opts ? &_opts->estimate_options : nullptr);
  });




  updateControls();
}

void QFeatureBasedRegistrationOptions::onDetectorTypeChanged()
{
  if( _opts ) {

    if( _opts->sparse_feature_extractor_and_matcher.detector.type == SPARSE_FEATURE_DETECTOR_PLANETARY_DISK ) {
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
    Base(parent)
{
  scale_ctl =
      add_numeric_box<double>("image scale",
          "",
          [this](double value) {
            if ( _opts && _opts->scale != value ) {
              _opts->scale = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * value) {
            if ( _opts ) {
              * value = _opts->scale;
              return true;
            }
            return false;
          });

  ecc_method_ctl =
      add_enum_combobox<ECC_ALIGN_METHOD>("ecc_method",
          "",
          [this](ECC_ALIGN_METHOD value) {
            if ( _opts && _opts->ecc_method != value ) {
              _opts->ecc_method = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](ECC_ALIGN_METHOD * value) {
            if ( _opts ) {
              * value = _opts->ecc_method;
              return true;
            }
            return false;
          });

  eps_ctl =
      add_numeric_box<double>("eps",
          "",
          [this](double value) {
            if ( _opts && _opts->eps != value ) {
              _opts->eps = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * value) {
            if ( _opts ) {
              * value = _opts->eps;
              return true;
            }
            return false;
          });

  min_rho_ctl =
      add_numeric_box<double>("min_rho",
          "",
          [this](double value) {
            if ( _opts && _opts->min_rho != value ) {
              _opts->min_rho = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * value) {
            if ( _opts ) {
              * value = _opts->min_rho;
              return true;
            }
            return false;
          });

  input_smooth_sigma_ctl =
      add_numeric_box<double>("input_smooth_sigma",
          "",
          [this](double value) {
            if ( _opts && _opts->input_smooth_sigma != value ) {
              _opts->input_smooth_sigma = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * value) {
            if ( _opts ) {
              * value = _opts->input_smooth_sigma ;
              return true;
            }
            return false;
          });

  reference_smooth_sigma_ctl =
      add_numeric_box<double>("reference_smooth_sigma",
          "",
          [this](double value) {
            if ( _opts && _opts->reference_smooth_sigma != value ) {
              _opts->reference_smooth_sigma = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * value) {
            if ( _opts ) {
              * value = _opts->reference_smooth_sigma;
              return true;
            }
            return false;
          });


    normalization_scale_ctl =
      add_numeric_box<int>("normalization_scale",
          "",
          [this](int value) {
            if ( _opts && _opts->normalization_scale != value ) {
              _opts->normalization_scale = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->normalization_scale;
              return true;
            }
            return false;
          });

  normalization_noise_ctl =
      add_numeric_box<double>("normalization_noise",
          "",
          [this](double value) {
            if ( _opts && _opts->normalization_noise != value ) {
              _opts->normalization_noise = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * value) {
            if ( _opts ) {
              * value = _opts->normalization_noise;
              return true;
            }
            return false;
          });



  update_step_scale_ctl =
      add_numeric_box<double>("update_step_scale",
          "",
          [this](double value) {
            if ( _opts && _opts->update_step_scale != value ) {
              _opts->update_step_scale = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * value) {
            if ( _opts  ) {
              * value = _opts->update_step_scale;
              return true;
            }
            return false;
          });


  max_iterations_ctl =
      add_numeric_box<int>("max_iterations",
          "",
          [this](int value) {
            if ( _opts && _opts->max_iterations != value ) {
              _opts->max_iterations = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->max_iterations;
              return true;
            }
            return false;
          });

  ecch_max_level_ctl =
      add_numeric_box<int>("ecch_max_level",
          "",
          [this](int value) {
            if ( _opts && _opts->ecch_max_level != value ) {
              _opts->ecch_max_level = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->ecch_max_level;
              return true;
            }
            return false;
          });

  ecch_minimum_image_size_ctl =
      add_numeric_box<int>("ecch_minimum_image_size",
          "",
          [this](int value) {
            if ( _opts && _opts->ecch_minimum_image_size != value ) {
              _opts->ecch_minimum_image_size = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              *value = _opts->ecch_minimum_image_size;
              return true;
            }
            return false;
          });

  ecch_estimate_translation_first_ctl =
      add_checkbox("Estimate translation first",
          "",
          [this](bool checked) {
            if ( _opts && _opts->ecch_estimate_translation_first != checked ) {
              _opts->ecch_estimate_translation_first = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( _opts ) {
              * checked = _opts->ecch_estimate_translation_first;
              return true;
            }
            return false;
          });


  replace_planetary_disk_with_mask_ctl =
      add_checkbox("replace_planetary_disk_with_mask",
          "",
          [this](bool checked) {
            if ( _opts && _opts->replace_planetary_disk_with_mask != checked ) {
              _opts->replace_planetary_disk_with_mask = checked;
              planetary_disk_mask_stdev_factor_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( _opts ) {
              * checked = _opts->replace_planetary_disk_with_mask;
              return true;
            }
            return false;
          });

  planetary_disk_mask_stdev_factor_ctl =
      add_numeric_box<double>("stdev_factor",
          "",
          [this](double value) {
            if ( _opts && _opts->planetary_disk_mask_stdev_factor != value ) {
              _opts->planetary_disk_mask_stdev_factor = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * value) {
            if ( _opts ) {
              * value = _opts->planetary_disk_mask_stdev_factor;
              return true;
            }
            return false;
          });

  se_close_size_ctl =
      add_numeric_box<int>("se_close_size",
      "",
      [this](int value) {
        if ( _opts && _opts->se_close_size != value ) {
          _opts->se_close_size = value;
          Q_EMIT parameterChanged();
        }
      },
      [this](int * value) {
        if ( _opts ) {
          * value = _opts->se_close_size;
          return true;
        }
        return false;
      });


  updateControls();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QEccFlowRegistrationOptions::QEccFlowRegistrationOptions(QWidget * parent) :
    Base(parent)
{
  support_scale_ctl =
      add_numeric_box<int>("support_scale",
          "",
          [this](int value) {
            if ( _opts && _opts->support_scale != value ) {
              _opts->support_scale = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->support_scale;
              return true;
            }
            return false;
          });

  downscale_method_ctl =
      add_enum_combobox<ECCFlowDownscaleMethod>("downscale_method",
          "",
          [this](ECCFlowDownscaleMethod value) {
            if ( _opts && _opts->downscale_method != value ) {
              _opts->downscale_method = value;
              Q_EMIT parameterChanged();
            }
          },
        [this](ECCFlowDownscaleMethod * value) {
          if ( _opts ) {
            * value = _opts->downscale_method;
            return true;
          }
          return false;
        });

  min_image_size_ctl =
      add_numeric_box<int>("min_image_size",
          "",
          [this](int value) {
            if ( _opts && _opts->min_image_size != value ) {
              _opts->min_image_size = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->min_image_size;
              return true;
            }
            return false;
          });

  max_pyramid_level_ctl =
      add_numeric_box<int>("max_pyramid_level",
          "",
          [this](int value) {
            if ( _opts && _opts->max_pyramid_level != value ) {
              _opts->max_pyramid_level = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->max_pyramid_level;
              return true;
            }
            return false;
          });

  scale_factor_ctl =
      add_numeric_box<double>("scale_factor",
          "",
          [this](double value) {
            if ( _opts && _opts->scale_factor != value ) {
              _opts->scale_factor = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * value) {
            if ( _opts ) {
              * value = _opts->scale_factor;
              return true;
            }
            return false;
          });

  input_smooth_sigma_ctl =
      add_numeric_box<double>("input_smooth_sigma",
          "",
          [this](double value) {
            if ( _opts && _opts->input_smooth_sigma != value ) {
              _opts->input_smooth_sigma = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * value) {
            if ( _opts ) {
              * value = _opts->input_smooth_sigma;
              return true;
            }
            return false;
          });

  reference_smooth_sigma_ctl =
      add_numeric_box<double>("reference_smooth_sigma",
          "",
          [this](double value) {
            if ( _opts && _opts->reference_smooth_sigma != value ) {
              _opts->reference_smooth_sigma = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * value) {
            if ( _opts ) {
              * value = _opts->reference_smooth_sigma;
              return true;
            }
            return false;
          });

  update_multiplier_ctl =
      add_numeric_box<double>("update_multiplier",
          "",
          [this](double value) {
            if ( _opts && _opts->update_multiplier != value ) {
              _opts->update_multiplier = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * value) {
            if ( _opts ) {
              * value = _opts->update_multiplier;
              return true;
            }
            return false;
          });

  max_iterations_ctl =
      add_numeric_box<int>("max_iterations",
          "",
          [this](int value) {
            if ( _opts && _opts->max_iterations != value ) {
              _opts->max_iterations = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->max_iterations;
              return true;
            }
            return false;
          });

  noise_level_ctl =
      add_numeric_box<double>("noise_level",
          "Set > 0 to manually force noise level",
          [this](double value) {
            if ( _opts && _opts->noise_level != value ) {
              _opts->noise_level = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * value) {
            if ( _opts ) {
              * value = _opts->noise_level;
              return true;
            }
            return false;
          });

  updateControls();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QJovianDerotationOptions::QJovianDerotationOptions(QWidget * parent) :
    Base(parent)
{
  jovian_detector_stdev_factor_ctl =
      add_numeric_box<double>("stdev_factor:",
          "",
          [this](double value) {
            if ( _opts && _opts->detector_options.stdev_factor != value ) {
              _opts->detector_options.stdev_factor = value;
              Q_EMIT parameterChanged();
            }
          });

  jovian_detector_pca_blur_ctl =
      add_numeric_box<double>("pca_blur:",
          "",
          [this](double value) {
            if ( _opts && _opts->detector_options.pca_blur != value ) {
              _opts->detector_options.pca_blur = value;
              Q_EMIT parameterChanged();
            }
          });

  jovian_detector_ellipse_offset_ctl =
      add_numeric_box<cv::Point2f>("ellipse offset:",
          "",
          [this](const cv::Point2f & value) {
            if ( _opts && _opts->detector_options.offset != value ) {
              _opts->detector_options.offset = value;
              Q_EMIT parameterChanged();
            }
          });

//  max_pyramid_level_ctl =
//      add_numeric_box<int>("max pyramid level:",
//          "",
//          [this](int value) {
//            if ( _opts && _opts->max_pyramid_level != value ) {
//              _opts->max_pyramid_level = value;
//              Q_EMIT parameterChanged();
//            }
//          });
//
//  min_rotation_ctl =
//      add_numeric_box<double>("min_rotation [deg]:",
//          "",
//          [this](double value) {
//            if ( _opts && _opts->min_rotation != (value *= M_PI / 180) ) {
//              _opts->min_rotation = value;
//              Q_EMIT parameterChanged();
//            }
//          });
//
//  max_rotation_ctl =
//      add_numeric_box<double>("max_rotation [deg]:",
//          "",
//          [this](double value) {
//            if ( _opts && _opts->max_rotation != (value *= M_PI / 180) ) {
//              _opts->max_rotation = value;
//              Q_EMIT parameterChanged();
//            }
//          });
//
//  num_orientations_ctl =
//      add_numeric_box<int>("num_orientations:",
//          "",
//          [this](int value) {
//            if ( _opts && _opts->num_orientations != value ) {
//              _opts->num_orientations = value;
//              Q_EMIT parameterChanged();
//            }
//          });

  max_context_size_ctl =
      add_numeric_box<int>("max_context_size:",
          "",
          [this](double value) {
            if ( _opts && _opts->max_context_size != value ) {
              _opts->max_context_size = value;
              Q_EMIT parameterChanged();
            }
          });

  derotate_all_frames_ctl =
      add_checkbox("process all frames",
          "",
          [this](bool checked) {
            if ( _opts && _opts->derotate_all_frames != checked ) {
              _opts->derotate_all_frames = checked;
              Q_EMIT parameterChanged();
            }
          });

  updateControls();
}
//void QJovianDerotationOptions::onupdatecontrols()
//{
//  if( !_opts ) {
//    setEnabled(false);
//  }
//  else {
//    jovian_detector_stdev_factor_ctl->setValue(_opts->detector_options.stdev_factor);
//    jovian_detector_pca_blur_ctl->setValue(_opts->detector_options.pca_blur);
//    jovian_detector_ellipse_offset_ctl->setValue(_opts->detector_options.offset);
////    max_pyramid_level_ctl->setValue(_opts->max_pyramid_level);
////    min_rotation_ctl->setValue(_opts->min_rotation * 180 / M_PI);
////    max_rotation_ctl->setValue(_opts->max_rotation * 180 / M_PI);
////    num_orientations_ctl->setValue(_opts->num_orientations);
//    max_context_size_ctl->setValue(_opts->max_context_size);
//    derotate_all_frames_ctl->setChecked(_opts->derotate_all_frames);
//
//    setEnabled(true);
//  }
//}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QSaturnDerotationOptions::QSaturnDerotationOptions(QWidget * parent) :
    Base(parent)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

