/*
 * QFrameRegistrationSettings.cc
 *
 *  Created on: Feb 9, 2021
 *      Author: amyznikov
 */

#include "QFrameRegistrationOptions.h"

#include <core/io/image/c_fits_input_source.h>
#include <core/io/image/c_fits_input_source.h>
#include <core/io/image/c_raw_image_input_source.h>
#include <core/io/image/c_regular_image_input_source.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


QMasterSourceSelectionCombo::QMasterSourceSelectionCombo(QWidget * parent) :
    Base(parent)
{
  QHBoxLayout * layout = new QHBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);

  combo_ = new QComboBox(this);
  combo_->setEditable(false);
  combo_->setSizeAdjustPolicy(QComboBox::AdjustToContents);
  combo_->setFocusPolicy(Qt::FocusPolicy::StrongFocus);
  combo_->setDuplicatesEnabled(true);
  layout->addWidget(combo_);

  browse_ctl = new QToolButton(this);
  browse_ctl->setText("Browse...");
  layout->addWidget(browse_ctl);

  QObject::connect(combo_, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
      this, &ThisClass::onComboboxCurrentIndexChanged);

  QObject::connect(browse_ctl, &QToolButton::clicked,
      this, &ThisClass::onBrowseButtonClicked);
}

void QMasterSourceSelectionCombo::setEnableExternalFile(bool v)
{
  browse_ctl->setEnabled(v);
}

bool QMasterSourceSelectionCombo::enableExternalFile() const
{
  return browse_ctl->isEnabled();
}

void QMasterSourceSelectionCombo::refreshInputSources(c_image_processing_pipeline * pipeline)
{
  c_update_controls_lock lock(this);

  combo_->clear();

  if( !pipeline ) {
    setEnabled(false);
    return;
  }

  const c_input_sequence::sptr & input_sequence =
      pipeline->input_sequence();

  if( input_sequence ) {

    setEnabled(false);

    for( const c_input_source::sptr & source : input_sequence->sources() ) {

      InputSourceData data = {
          .source_pathfilename = source->cfilename(),
          .source_size = source->size()
      };

      combo_->addItem(QFileInfo(source->cfilename()).fileName(),
          QVariant::fromValue(data));
    }
  }

  setEnabled(true);
}

void QMasterSourceSelectionCombo::onComboboxCurrentIndexChanged(int index)
{
  if( !updatingControls() ) {
    Q_EMIT currentSourceChanged();
  }
}

void QMasterSourceSelectionCombo::onBrowseButtonClicked()
{
  static QString filter;

  if( filter.isEmpty() ) {

#if have_regular_image_input_source
    filter.append("Regular images (");
    for( const std::string &s : c_regular_image_input_source::suffixes() ) {
      filter.append(QString("*%1 ").arg(QString(s.c_str())));
    }
    filter.append(");;");
#endif

#if have_raw_image_input_source
      filter.append("RAW/DSLR images (");
      for ( const std::string & s : c_raw_image_input_source::suffixes() ) {
        filter.append(QString("*%1 ").arg(s.c_str()));
      }
      filter.append(");;");
  #endif

#if have_fits_input_source
      filter.append("FITS files (");
      for ( const std::string & s : c_fits_input_source::suffixes() ) {
        filter.append(QString("*%1 ").arg(s.c_str()));
      }
      filter.append(");;");
#endif

    filter.append("All Files (*.*);;");
  }

  static const QString lastSourcesDirectoryKeyName =
      "lastSourcesDirectory";

  static const QString lastMasterFrameSelectionFilter =
      "lastMasterFrameSelectionFilter";

  QSettings settings;

  QString selectedFilter =
      settings.value(lastMasterFrameSelectionFilter).toString();

  QString proposedMasterSourcePath =
      settings.value(lastSourcesDirectoryKeyName).toString();

  QString selectedFile =
      QFileDialog::getOpenFileName(this,
          "Select master frame",
          proposedMasterSourcePath,
          filter,
          &selectedFilter);

  if( selectedFile.isEmpty() ) {
    return;
  }

  settings.setValue(lastSourcesDirectoryKeyName,
      QFileInfo(selectedFile).absolutePath());

  settings.setValue(lastMasterFrameSelectionFilter,
      selectedFilter);

  c_input_source::sptr source =
      c_input_source::create(selectedFile.toStdString());

  if( !source ) {
    QMessageBox::critical(this, "ERROR",
        qsprintf("Can not opent input source '%s'\n"
            "Check debug log for details",
            selectedFile.toUtf8().constData()));
    return;
  }

  InputSourceData data = {
      .source_pathfilename = source->cfilename(),
      .source_size = source->size()
  };

  combo_->addItem(QFileInfo(source->cfilename()).fileName(),
      QVariant::fromValue(data));

  combo_->setCurrentIndex(combo_->count() - 1);
}

void QMasterSourceSelectionCombo::setCurrentInputSource(const std::string & pathfilename)
{
  if ( !pathfilename.empty() ) {

    c_update_controls_lock lock(this);

    for( int i = 0, n = combo_->count(); i < n; ++i ) {

      const InputSourceData data =
          combo_->itemData(i).value<InputSourceData>();

      if( data.source_pathfilename == pathfilename ) {
        combo_->setCurrentIndex(i);
        return;
      }
    }

    c_input_source::sptr source =
        c_input_source::create(pathfilename);

    if( !source ) {
      CF_ERROR("c_input_source::create(pathfilename='%s') fails",
          pathfilename.c_str());
      return;
    }

    InputSourceData data = {
        .source_pathfilename = source->cfilename(),
        .source_size = source->size()
    };

    combo_->addItem(QFileInfo(source->cfilename()).fileName(),
        QVariant::fromValue(data));

    combo_->setCurrentIndex(combo_->count() - 1);
  }
}

QMasterSourceSelectionCombo::InputSourceData QMasterSourceSelectionCombo::currentInputSource() const
{
  return combo_->itemData(combo_->currentIndex()).value<InputSourceData>();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QMasterFrameSelectionControl::QMasterFrameSelectionControl(QWidget * parent) :
    Base(parent)
{
  masterFrameSelectionMethod_ctl =
      add_enum_combobox<master_frame_selection_method>("Master frame selection:",
          "",
          [this](master_frame_selection_method v) {
            if ( _options && _options->master_selection_method != v ) {
              _options->master_selection_method = v;
              updateMasterSourceControlStates();
              Q_EMIT parameterChanged();
            }
          },
          [this](master_frame_selection_method * v) {
            if ( _options ) {
              *v = _options->master_selection_method;
              return true;
            }
            return false;
          });

  masterSource_ctl = add_widget<QMasterSourceSelectionCombo>("Master file:");
  masterSource_ctl->setToolTip("Specify input source for master frame");
  connect(masterSource_ctl, &QMasterSourceSelectionCombo::currentSourceChanged,
      [this]() {
        if( _options && !updatingControls() ) {

          const QMasterSourceSelectionCombo::InputSourceData data = masterSource_ctl->currentInputSource();

          _options->master_fiename = data.source_pathfilename;

          if ( true ) {
            c_update_controls_lock lock(this);
            masterFrameIndex_ctl->setRange(0, data.source_size-1);
            _options->master_frame_index = masterFrameIndex_ctl->value();
          }

          Q_EMIT parameterChanged();
        }
      });

  connect(this, &ThisClass::populatecontrols,
      [this]() {
        if( _options ) {

          masterSource_ctl->setCurrentInputSource(_options->master_fiename);

          c_update_controls_lock lock(this);
          masterFrameIndex_ctl->setRange(0, masterSource_ctl->currentInputSource().source_size - 1);
          masterFrameIndex_ctl->setValue(_options->master_frame_index);
          _options->master_frame_index = masterFrameIndex_ctl->value();
        }
      });

  masterFrameIndex_ctl =
      add_spinbox("Master frame Index:",
          "",
          [this](int v) {
            if ( _options && _options->master_frame_index != v ) {
              _options->master_frame_index = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _options ) {
              *v = _options->master_frame_index;
              return true;
            }
            return false;
          });

  updateControls();
}

void QMasterFrameSelectionControl::updateMasterSourceControlStates()
{
  if( _options ) {

    switch (_options->master_selection_method) {
      case master_frame_specific_index:
        masterFrameIndex_ctl->setEnabled(true);
        break;
      case master_frame_middle_index:
        masterFrameIndex_ctl->setEnabled(false);
        break;
      case master_frame_best_of_100_in_middle:
        masterFrameIndex_ctl->setEnabled(false);
        break;
      default:
        break;
    }
  }
}

void QMasterFrameSelectionControl::refreshInputSources(c_image_processing_pipeline * pipeline)
{
  c_update_controls_lock lock(this);
  masterSource_ctl->refreshInputSources(pipeline);
}

void QMasterFrameSelectionControl::setEnableExternalFile(bool v)
{
  masterSource_ctl->setEnableExternalFile(v);
}

bool QMasterFrameSelectionControl::enableExternalFile() const
{
  return masterSource_ctl->enableExternalFile();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QEstimateTranslationImageTransformOptions::QEstimateTranslationImageTransformOptions(QWidget * parent) :
    Base(parent)
{
  max_iterations_ctl =
      add_numeric_box<int>("max_iterations",
          "max number of iterations for outliers removal",
          [this](int v) {
            if ( _options && _options->translation.max_iterations != v ) {
              _options->translation.max_iterations = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _options ) {
              *v = _options->translation.max_iterations;
              return true;
            }
            return false;
          });

  rmse_factor_ctl =
      add_numeric_box<double>("rmse_factor",
          "rmse factor for outliers removal",
          [this](double v) {
            if ( _options && _options->translation.rmse_factor != v ) {
              _options->translation.rmse_factor = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _options ) {
              *v = _options->translation.rmse_factor;
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
            if ( _options && _options->euclidean.max_iterations != v ) {
              _options->euclidean.max_iterations = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _options ) {
              *v = _options->euclidean.max_iterations;
              return true;
            }
            return false;
          });

  rmse_threshold_ctl =
      add_numeric_box<double>("rmse_threshold",
          "",
          [this](double v) {
            if ( _options && _options->euclidean.rmse_threshold != v ) {
              _options->euclidean.rmse_threshold = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _options ) {
              *v = _options->euclidean.rmse_threshold;
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
            if ( _options && _options->scaled_euclidean.method != v ) {
              _options->scaled_euclidean.method = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](ROBUST_METHOD * v) {
            if ( _options ) {
              *v = _options->scaled_euclidean.method;
              return true;
            }
            return false;
          });

  ransacReprojThreshold_ctl =
      add_numeric_box<double>("ransacReprojThreshold",
          "",
          [this](double v) {
            if ( _options && _options->scaled_euclidean.ransacReprojThreshold != v ) {
              _options->scaled_euclidean.ransacReprojThreshold = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _options ) {
              *v = _options->scaled_euclidean.ransacReprojThreshold;
              return true;
            }
            return false;
          });

  confidence_ctl =
      add_numeric_box<double>("confidence",
          "",
          [this](double v) {
            if ( _options && _options->scaled_euclidean.confidence != v ) {
              _options->scaled_euclidean.confidence = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _options ) {
              *v = _options->scaled_euclidean.confidence;
              return true;
            }
            return false;
          });

  maxIters_ctl =
      add_numeric_box<int>("maxIters",
          "",
          [this](int v) {
            if ( _options && _options->scaled_euclidean.maxIters != v ) {
              _options->scaled_euclidean.maxIters = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _options ) {
              *v = _options->scaled_euclidean.maxIters;
              return true;
            }
            return false;
          });

  refineIters_ctl =
      add_numeric_box<int>("refineIters",
          "",
          [this](int v) {
            if ( _options && _options->scaled_euclidean.refineIters != v ) {
              _options->scaled_euclidean.refineIters = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _options ) {
              *v = _options->scaled_euclidean.refineIters;
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
            if ( _options && _options->affine.method != v ) {
              _options->affine.method = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](ROBUST_METHOD * v) {
            if ( _options ) {
              *v = _options->affine.method;
              return true;
            }
            return false;
          });

  ransacReprojThreshold_ctl =
      add_numeric_box<double>("ransacReprojThreshold",
          "",
          [this](double v) {
            if ( _options && _options->affine.ransacReprojThreshold != v ) {
              _options->affine.ransacReprojThreshold = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _options ) {
              *v = _options->affine.ransacReprojThreshold;
              return true;
            }
            return false;
          });

  confidence_ctl =
      add_numeric_box<double>("confidence",
          "",
          [this](double v) {
            if ( _options && _options->affine.confidence != v ) {
              _options->affine.confidence = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _options ) {
              *v = _options->affine.confidence;
              return true;
            }
            return false;
          });

  maxIters_ctl =
      add_numeric_box<int>("maxIters",
          "",
          [this](int v) {
            if ( _options && _options->affine.maxIters != v ) {
              _options->affine.maxIters = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _options ) {
              *v = _options->affine.maxIters;
              return true;
            }
            return false;
          });

  refineIters_ctl =
      add_numeric_box<int>("refineIters",
          "",
          [this](int v) {
            if ( _options && _options->affine.refineIters != v ) {
              _options->affine.refineIters = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _options ) {
              *v = _options->affine.refineIters;
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
            if ( _options && _options->homography.method != v ) {
              _options->homography.method = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](ROBUST_METHOD * v) {
            if ( _options ) {
              *v = _options->homography.method;
              return true;
            }
            return false;
          });

  ransacReprojThreshold_ctl =
      add_numeric_box<double>("ransacReprojThreshold",
          "",
          [this](double v) {
            if ( _options && _options->homography.ransacReprojThreshold != v ) {
              _options->homography.ransacReprojThreshold = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _options ) {
              *v = _options->homography.ransacReprojThreshold;
              return true;
            }
            return false;
          });

  confidence_ctl =
      add_numeric_box<double>("confidence",
          "",
          [this](double v) {
            if ( _options && _options->homography.confidence != v ) {
              _options->homography.confidence = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _options ) {
              *v = _options->homography.confidence;
              return true;
            }
            return false;
          });

  maxIters_ctl =
      add_numeric_box<int>("maxIters",
          "",
          [this](int v) {
            if ( _options && _options->homography.maxIters != v ) {
              _options->homography.maxIters = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _options ) {
              *v = _options->homography.maxIters;
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
            if ( _options && _options->semi_quadratic.rmse_factor != v ) {
              _options->semi_quadratic.rmse_factor = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _options ) {
              *v = _options->semi_quadratic.rmse_factor;
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
            if ( _options && _options->quadratic.rmse_factor != v ) {
              _options->quadratic.rmse_factor = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _options ) {
              *v = _options->quadratic.rmse_factor;
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
            if ( _options && _options->epipolar_derotation.camera_pose.direction != v ) {
              _options->epipolar_derotation.camera_pose.direction = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](EPIPOLAR_MOTION_DIRECTION * v) {
            if ( _options ) {
              *v = _options->epipolar_derotation.camera_pose.direction;
              return true;
            }
            return false;
          });

  max_iterations_ctl =
      add_numeric_box<int>("max_iterations",
          "",
          [this](int v) {
            if ( _options && _options->epipolar_derotation.camera_pose.max_iterations != v ) {
              _options->epipolar_derotation.camera_pose.max_iterations = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _options ) {
              *v = _options->epipolar_derotation.camera_pose.max_iterations;
              return true;
            }
            return false;
          });

  max_levmar_iterations_ctl =
      add_numeric_box<int>("max_levmar_iterations",
          "",
          [this](int v) {
            if ( _options && _options->epipolar_derotation.camera_pose.max_levmar_iterations != v ) {
              _options->epipolar_derotation.camera_pose.max_levmar_iterations = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _options ) {
              *v = _options->epipolar_derotation.camera_pose.max_levmar_iterations;
              return true;
            }
            return false;
          });

  robust_threshold_ctl =
      add_numeric_box<double>("robust_threshold",
          "",
          [this](double v) {
            if ( _options && _options->epipolar_derotation.camera_pose.robust_threshold != v ) {
              _options->epipolar_derotation.camera_pose.robust_threshold = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _options ) {
              *v = _options->epipolar_derotation.camera_pose.robust_threshold;
              return true;
            }
            return false;
          });

  epsf_ctl =
      add_numeric_box<double>("epsf",
          "",
          [this](double v) {
            if ( _options && _options->epipolar_derotation.camera_pose.epsf != v ) {
              _options->epipolar_derotation.camera_pose.epsf = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _options ) {
              *v = _options->epipolar_derotation.camera_pose.epsf;
              return true;
            }
            return false;
          });

  epsx_ctl =
      add_numeric_box<double>("epsx",
          "",
          [this](double v) {
            if ( _options && _options->epipolar_derotation.camera_pose.epsx != v ) {
              _options->epipolar_derotation.camera_pose.epsx = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _options ) {
              *v = _options->epipolar_derotation.camera_pose.epsx;
              return true;
            }
            return false;
          });

  initial_translation_ctl =
      add_numeric_box<cv::Vec3f>("initial_translation",
          "",
          [this](const cv::Vec3f & v) {
            if ( _options && _options->epipolar_derotation.initial_translation != v ) {
              _options->epipolar_derotation.initial_translation = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Vec3f * v) {
            if ( _options ) {
              *v = _options->epipolar_derotation.initial_translation;
              return true;
            }
            return false;
          });

  initial_rotation_ctl =
      add_numeric_box<cv::Vec3f>("initial_rotation [deg]",
          "",
          [this](const cv::Vec3f & v) {
            if ( _options && _options->epipolar_derotation.initial_rotation != v ) {
              _options->epipolar_derotation.initial_rotation = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Vec3f * v) {
            if ( _options ) {
              *v = _options->epipolar_derotation.initial_rotation;
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
            if ( _options && _options->registration_channel != value ) {
              _options->registration_channel = value;
              Q_EMIT parameterChanged();
            }
          });


  scale_ctl =
      add_numeric_box<double>("Image scale",
          "Scale input image before feature extraction",
          [this](double value) {
            if ( _options && _options->image_scale != value ) {
              _options->image_scale = value;
              Q_EMIT parameterChanged();
            }
          });

  triangle_eps_ctl =
      add_numeric_box<double>("Triangle_eps [px]",
          "Max distance in pixels between current and remapped reference keypoints for triangle-based transfrom reestimation",
          [this](double value) {
            if ( _options && _options->triangle_eps != value ) {
              _options->triangle_eps = value;
              Q_EMIT parameterChanged();
            }
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
          transformOptionsSettings_ctl = new QSettingsWidget("", this));

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

  updateControls();
}

void QFeatureBasedRegistrationOptions::onupdatecontrols()
{
  if( !_options ) {
    setEnabled(false);
  }
  else {

    registrationChannel_ctl->setValue(_options->registration_channel);
    scale_ctl->setValue(_options->image_scale);
    triangle_eps_ctl->setValue(_options->triangle_eps);
    sparseFeatureDetectorOptions_ctl->set_feature_detector_options(
        &_options->sparse_feature_extractor_and_matcher.detector);
    sparseFeatureDescriptorOptions_ctl->set_feature_descriptor_options(
        &_options->sparse_feature_extractor_and_matcher.descriptor);
    sparseFeatureMatcherOptions_ctl->set_feature_matcher_options(
        &_options->sparse_feature_extractor_and_matcher.matcher);

    estimateTranslation_ctl->set_options(&_options->estimate_options);
    estimateEuclidean_ctl->set_options(&_options->estimate_options);
    estimateScaledEuclidean_ctl->set_options(&_options->estimate_options);
    estimateAffine_ctl->set_options(&_options->estimate_options);
    estimateHomography_ctl->set_options(&_options->estimate_options);
    estimateSemiQuadratic_ctl->set_options(&_options->estimate_options);
    estimateQuadratic_ctl->set_options(&_options->estimate_options);
    estimateEpipolarDerotation_ctl->set_options(&_options->estimate_options);

    onDetectorTypeChanged();
    setEnabled(true);
  }
}

void QFeatureBasedRegistrationOptions::onDetectorTypeChanged()
{
  if( _options ) {

    if( _options->sparse_feature_extractor_and_matcher.detector.type == SPARSE_FEATURE_DETECTOR_PLANETARY_DISK ) {
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
            if ( _options && _options->scale != value ) {
              _options->scale = value;
              Q_EMIT parameterChanged();
            }
          });

  ecc_method_ctl =
      add_enum_combobox<ECC_ALIGN_METHOD>("ecc_method",
          "",
          [this](ECC_ALIGN_METHOD value) {
            if ( _options && _options->ecc_method != value ) {
              _options->ecc_method = value;
              Q_EMIT parameterChanged();
            }
          });

  eps_ctl =
      add_numeric_box<double>("eps",
          "",
          [this](double value) {
            if ( _options && _options->eps != value ) {
              _options->eps = value;
              Q_EMIT parameterChanged();
            }
          });

  min_rho_ctl =
      add_numeric_box<double>("min_rho",
          "",
          [this](double value) {
            if ( _options && _options->min_rho != value ) {
              _options->min_rho = value;
              Q_EMIT parameterChanged();
            }
          });

  input_smooth_sigma_ctl =
      add_numeric_box<double>("input_smooth_sigma",
          "",
          [this](double value) {
            if ( _options && _options->input_smooth_sigma != value ) {
              _options->input_smooth_sigma = value;
              Q_EMIT parameterChanged();
            }
          });

  reference_smooth_sigma_ctl =
      add_numeric_box<double>("reference_smooth_sigma",
          "",
          [this](double value) {
            if ( _options && _options->reference_smooth_sigma != value ) {
              _options->reference_smooth_sigma = value;
              Q_EMIT parameterChanged();
            }
          });


    normalization_scale_ctl =
      add_numeric_box<int>("normalization_scale",
          "",
          [this](int value) {
            if ( _options && _options->normalization_scale != value ) {
              _options->normalization_scale = value;
              Q_EMIT parameterChanged();
            }
          });

  normalization_noise_ctl =
      add_numeric_box<double>("normalization_noise",
          "",
          [this](double value) {
            if ( _options && _options->normalization_noise != value ) {
              _options->normalization_noise = value;
              Q_EMIT parameterChanged();
            }
          });



  update_step_scale_ctl =
      add_numeric_box<double>("update_step_scale",
          "",
          [this](double value) {
            if ( _options && _options->update_step_scale != value ) {
              _options->update_step_scale = value;
              Q_EMIT parameterChanged();
            }
          });


  max_iterations_ctl =
      add_numeric_box<int>("max_iterations",
          "",
          [this](int value) {
            if ( _options && _options->max_iterations != value ) {
              _options->max_iterations = value;
              Q_EMIT parameterChanged();
            }
          });

  ecch_max_level_ctl =
      add_numeric_box<int>("ecch_max_level",
          "",
          [this](int value) {
            if ( _options && _options->ecch_max_level != value ) {
              _options->ecch_max_level = value;
              Q_EMIT parameterChanged();
            }
          });

  ecch_minimum_image_size_ctl =
      add_numeric_box<int>("ecch_minimum_image_size",
          "",
          [this](int value) {
            if ( _options && _options->ecch_minimum_image_size != value ) {
              _options->ecch_minimum_image_size = value;
              Q_EMIT parameterChanged();
            }
          });

  ecch_estimate_translation_first_ctl =
      add_checkbox("Estimate translation first",
          "",
          [this](bool checked) {
            if ( _options && _options->ecch_estimate_translation_first != checked ) {
              _options->ecch_estimate_translation_first = checked;
              Q_EMIT parameterChanged();
            }
          });


  replace_planetary_disk_with_mask_ctl =
      add_checkbox("replace_planetary_disk_with_mask",
          "",
          [this](bool checked) {
            if ( _options && _options->replace_planetary_disk_with_mask != checked ) {
              _options->replace_planetary_disk_with_mask = checked;
              planetary_disk_mask_stdev_factor_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          });

  planetary_disk_mask_stdev_factor_ctl =
      add_numeric_box<double>("stdev_factor",
          "",
          [this](double value) {
            if ( _options && _options->planetary_disk_mask_stdev_factor != value ) {
              _options->planetary_disk_mask_stdev_factor = value;
              Q_EMIT parameterChanged();
            }
          });

  se_close_size_ctl =
      add_numeric_box<int>("se_close_size",
      "",
      [this](int value) {
        if ( _options && _options->se_close_size != value ) {
          _options->se_close_size = value;
          Q_EMIT parameterChanged();
        }
      });


  updateControls();
}

void QEccRegistrationOptions::onupdatecontrols()
{
  if( !_options ) {
    setEnabled(false);
  }
  else {
    scale_ctl->setValue(_options->scale);
    ecc_method_ctl->setValue(_options->ecc_method);
    eps_ctl->setValue(_options->eps);
    min_rho_ctl->setValue(_options->min_rho);
    input_smooth_sigma_ctl->setValue(_options->input_smooth_sigma);
    reference_smooth_sigma_ctl->setValue(_options->reference_smooth_sigma);
    update_step_scale_ctl->setValue(_options->update_step_scale);
    normalization_noise_ctl->setValue(_options->normalization_noise);
    normalization_scale_ctl->setValue(_options->normalization_scale);
    max_iterations_ctl->setValue(_options->max_iterations);

    ecch_max_level_ctl->setValue(_options->ecch_max_level);
    ecch_estimate_translation_first_ctl->setChecked(_options->ecch_estimate_translation_first);
    ecch_minimum_image_size_ctl->setValue(_options->ecch_minimum_image_size);

//    ecch_minimum_image_size_ctl->setEnabled(_options->enable_ecch);
//    ecch_estimate_translation_first_ctl->setEnabled(_options->enable_ecch);

    replace_planetary_disk_with_mask_ctl->setChecked(_options->replace_planetary_disk_with_mask);
    planetary_disk_mask_stdev_factor_ctl->setEnabled(_options->replace_planetary_disk_with_mask);
    planetary_disk_mask_stdev_factor_ctl->setValue(_options->planetary_disk_mask_stdev_factor);
    se_close_size_ctl->setEnabled(_options->replace_planetary_disk_with_mask);
    se_close_size_ctl->setValue(_options->se_close_size);

    setEnabled(true);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QEccFlowRegistrationOptions::QEccFlowRegistrationOptions(QWidget * parent) :
    Base(parent)
{
  support_scale_ctl =
      add_numeric_box<int>("support_scale",
          "",
          [this](int value) {
            if ( _options && _options->support_scale != value ) {
              _options->support_scale = value;
              Q_EMIT parameterChanged();
            }
          });

  downscale_method_ctl =
      add_enum_combobox<c_eccflow::DownscaleMethod>("downscale_method",
          "",
          [this](c_eccflow::DownscaleMethod value) {
            if ( _options && _options->downscale_method != value ) {
              _options->downscale_method = value;
              Q_EMIT parameterChanged();
            }
          });

  min_image_size_ctl =
      add_numeric_box<int>("min_image_size",
          "",
          [this](int value) {
            if ( _options && _options->min_image_size != value ) {
              _options->min_image_size = value;
              Q_EMIT parameterChanged();
            }
          });

  max_pyramid_level_ctl =
      add_numeric_box<int>("max_pyramid_level",
          "",
          [this](int value) {
            if ( _options && _options->max_pyramid_level != value ) {
              _options->max_pyramid_level = value;
              Q_EMIT parameterChanged();
            }
          });

  scale_factor_ctl =
      add_numeric_box<double>("scale_factor",
          "",
          [this](double value) {
            if ( _options && _options->scale_factor != value ) {
              _options->scale_factor = value;
              Q_EMIT parameterChanged();
            }
          });

  input_smooth_sigma_ctl =
      add_numeric_box<double>("input_smooth_sigma",
          "",
          [this](double value) {
            if ( _options && _options->input_smooth_sigma != value ) {
              _options->input_smooth_sigma = value;
              Q_EMIT parameterChanged();
            }
          });

  reference_smooth_sigma_ctl =
      add_numeric_box<double>("reference_smooth_sigma",
          "",
          [this](double value) {
            if ( _options && _options->reference_smooth_sigma != value ) {
              _options->reference_smooth_sigma = value;
              Q_EMIT parameterChanged();
            }
          });

  update_multiplier_ctl =
      add_numeric_box<double>("update_multiplier",
          "",
          [this](double value) {
            if ( _options && _options->update_multiplier != value ) {
              _options->update_multiplier = value;
              Q_EMIT parameterChanged();
            }
          });

  max_iterations_ctl =
      add_numeric_box<int>("max_iterations",
          "",
          [this](int value) {
            if ( _options && _options->max_iterations != value ) {
              _options->max_iterations = value;
              Q_EMIT parameterChanged();
            }
          });

  noise_level_ctl =
      add_numeric_box<double>("noise_level",
          "Set > 0 to manually force noise level",
          [this](double value) {
            if ( _options && _options->noise_level != value ) {
              _options->noise_level = value;
              Q_EMIT parameterChanged();
            }
          });

  updateControls();
}

void QEccFlowRegistrationOptions::onupdatecontrols()
{
  if( !_options ) {
    setEnabled(false);
  }
  else {
    update_multiplier_ctl->setValue(_options->update_multiplier);
    input_smooth_sigma_ctl->setValue(_options->input_smooth_sigma);
    reference_smooth_sigma_ctl->setValue(_options->reference_smooth_sigma);
    max_iterations_ctl->setValue(_options->max_iterations);
    downscale_method_ctl->setValue(_options->downscale_method);
    support_scale_ctl->setValue(_options->support_scale);
    min_image_size_ctl->setValue(_options->min_image_size);
    max_pyramid_level_ctl->setValue(_options->max_pyramid_level);
    scale_factor_ctl->setValue(_options->scale_factor);
    noise_level_ctl->setValue(_options->noise_level);
    setEnabled(true);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QJovianDerotationOptions::QJovianDerotationOptions(QWidget * parent) :
    Base(parent)
{
  jovian_detector_stdev_factor_ctl =
      add_numeric_box<double>("stdev_factor:",
          "",
          [this](double value) {
            if ( _options && _options->detector_options.stdev_factor != value ) {
              _options->detector_options.stdev_factor = value;
              Q_EMIT parameterChanged();
            }
          });

  jovian_detector_pca_blur_ctl =
      add_numeric_box<double>("pca_blur:",
          "",
          [this](double value) {
            if ( _options && _options->detector_options.pca_blur != value ) {
              _options->detector_options.pca_blur = value;
              Q_EMIT parameterChanged();
            }
          });

  jovian_detector_ellipse_offset_ctl =
      add_numeric_box<cv::Point2f>("ellipse offset:",
          "",
          [this](const cv::Point2f & value) {
            if ( _options && _options->detector_options.offset != value ) {
              _options->detector_options.offset = value;
              Q_EMIT parameterChanged();
            }
          });

//  max_pyramid_level_ctl =
//      add_numeric_box<int>("max pyramid level:",
//          "",
//          [this](int value) {
//            if ( _options && _options->max_pyramid_level != value ) {
//              _options->max_pyramid_level = value;
//              Q_EMIT parameterChanged();
//            }
//          });
//
//  min_rotation_ctl =
//      add_numeric_box<double>("min_rotation [deg]:",
//          "",
//          [this](double value) {
//            if ( _options && _options->min_rotation != (value *= M_PI / 180) ) {
//              _options->min_rotation = value;
//              Q_EMIT parameterChanged();
//            }
//          });
//
//  max_rotation_ctl =
//      add_numeric_box<double>("max_rotation [deg]:",
//          "",
//          [this](double value) {
//            if ( _options && _options->max_rotation != (value *= M_PI / 180) ) {
//              _options->max_rotation = value;
//              Q_EMIT parameterChanged();
//            }
//          });
//
//  num_orientations_ctl =
//      add_numeric_box<int>("num_orientations:",
//          "",
//          [this](int value) {
//            if ( _options && _options->num_orientations != value ) {
//              _options->num_orientations = value;
//              Q_EMIT parameterChanged();
//            }
//          });

  max_context_size_ctl =
      add_numeric_box<int>("max_context_size:",
          "",
          [this](double value) {
            if ( _options && _options->max_context_size != value ) {
              _options->max_context_size = value;
              Q_EMIT parameterChanged();
            }
          });

  derotate_all_frames_ctl =
      add_checkbox("process all frames",
          "",
          [this](bool checked) {
            if ( _options && _options->derotate_all_frames != checked ) {
              _options->derotate_all_frames = checked;
              Q_EMIT parameterChanged();
            }
          });

  updateControls();
}

void QJovianDerotationOptions::onupdatecontrols()
{
  if( !_options ) {
    setEnabled(false);
  }
  else {
    jovian_detector_stdev_factor_ctl->setValue(_options->detector_options.stdev_factor);
    jovian_detector_pca_blur_ctl->setValue(_options->detector_options.pca_blur);
    jovian_detector_ellipse_offset_ctl->setValue(_options->detector_options.offset);
//    max_pyramid_level_ctl->setValue(_options->max_pyramid_level);
//    min_rotation_ctl->setValue(_options->min_rotation * 180 / M_PI);
//    max_rotation_ctl->setValue(_options->max_rotation * 180 / M_PI);
//    num_orientations_ctl->setValue(_options->num_orientations);
    max_context_size_ctl->setValue(_options->max_context_size);
    derotate_all_frames_ctl->setChecked(_options->derotate_all_frames);

    setEnabled(true);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QSaturnDerotationOptions::QSaturnDerotationOptions(QWidget * parent) :
    Base(parent)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

