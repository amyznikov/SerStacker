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
            if ( options_ && options_->master_selection_method != v ) {
              options_->master_selection_method = v;
              updateMasterSourceControlStates();
              Q_EMIT parameterChanged();
            }
          },
          [this](master_frame_selection_method * v) {
            if ( options_ ) {
              *v = options_->master_selection_method;
              return true;
            }
            return false;
          });

  masterSource_ctl = add_widget<QMasterSourceSelectionCombo>("Master file:");
  masterSource_ctl->setToolTip("Specify input source for master frame");
  connect(masterSource_ctl, &QMasterSourceSelectionCombo::currentSourceChanged,
      [this]() {
        if( options_ && !updatingControls() ) {

          const QMasterSourceSelectionCombo::InputSourceData data = masterSource_ctl->currentInputSource();

          options_->master_fiename = data.source_pathfilename;

          if ( true ) {
            c_update_controls_lock lock(this);
            masterFrameIndex_ctl->setRange(0, data.source_size-1);
            options_->master_frame_index = masterFrameIndex_ctl->value();
          }

          Q_EMIT parameterChanged();
        }
      });

  connect(this, &ThisClass::populatecontrols,
      [this]() {
        if( options_ ) {

          masterSource_ctl->setCurrentInputSource(options_->master_fiename);

          c_update_controls_lock lock(this);
          masterFrameIndex_ctl->setRange(0, masterSource_ctl->currentInputSource().source_size - 1);
          masterFrameIndex_ctl->setValue(options_->master_frame_index);
          options_->master_frame_index = masterFrameIndex_ctl->value();
        }
      });

  masterFrameIndex_ctl =
      add_spinbox("Master frame Index:",
          "",
          [this](int v) {
            if ( options_ && options_->master_frame_index != v ) {
              options_->master_frame_index = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( options_ ) {
              *v = options_->master_frame_index;
              return true;
            }
            return false;
          });

  updateControls();
}

void QMasterFrameSelectionControl::updateMasterSourceControlStates()
{
  if( options_ ) {

    switch (options_->master_selection_method) {
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
            if ( options_ && options_->translation.max_iterations != v ) {
              options_->translation.max_iterations = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( options_ ) {
              *v = options_->translation.max_iterations;
              return true;
            }
            return false;
          });

  rmse_factor_ctl =
      add_numeric_box<double>("rmse_factor",
          "rmse factor for outliers removal",
          [this](double v) {
            if ( options_ && options_->translation.rmse_factor != v ) {
              options_->translation.rmse_factor = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( options_ ) {
              *v = options_->translation.rmse_factor;
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
            if ( options_ && options_->euclidean.max_iterations != v ) {
              options_->euclidean.max_iterations = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( options_ ) {
              *v = options_->euclidean.max_iterations;
              return true;
            }
            return false;
          });

  rmse_threshold_ctl =
      add_numeric_box<double>("rmse_threshold",
          "",
          [this](double v) {
            if ( options_ && options_->euclidean.rmse_threshold != v ) {
              options_->euclidean.rmse_threshold = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( options_ ) {
              *v = options_->euclidean.rmse_threshold;
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
            if ( options_ && options_->scaled_euclidean.method != v ) {
              options_->scaled_euclidean.method = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](ROBUST_METHOD * v) {
            if ( options_ ) {
              *v = options_->scaled_euclidean.method;
              return true;
            }
            return false;
          });

  ransacReprojThreshold_ctl =
      add_numeric_box<double>("ransacReprojThreshold",
          "",
          [this](double v) {
            if ( options_ && options_->scaled_euclidean.ransacReprojThreshold != v ) {
              options_->scaled_euclidean.ransacReprojThreshold = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( options_ ) {
              *v = options_->scaled_euclidean.ransacReprojThreshold;
              return true;
            }
            return false;
          });

  confidence_ctl =
      add_numeric_box<double>("confidence",
          "",
          [this](double v) {
            if ( options_ && options_->scaled_euclidean.confidence != v ) {
              options_->scaled_euclidean.confidence = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( options_ ) {
              *v = options_->scaled_euclidean.confidence;
              return true;
            }
            return false;
          });

  maxIters_ctl =
      add_numeric_box<int>("maxIters",
          "",
          [this](int v) {
            if ( options_ && options_->scaled_euclidean.maxIters != v ) {
              options_->scaled_euclidean.maxIters = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( options_ ) {
              *v = options_->scaled_euclidean.maxIters;
              return true;
            }
            return false;
          });

  refineIters_ctl =
      add_numeric_box<int>("refineIters",
          "",
          [this](int v) {
            if ( options_ && options_->scaled_euclidean.refineIters != v ) {
              options_->scaled_euclidean.refineIters = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( options_ ) {
              *v = options_->scaled_euclidean.refineIters;
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
            if ( options_ && options_->affine.method != v ) {
              options_->affine.method = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](ROBUST_METHOD * v) {
            if ( options_ ) {
              *v = options_->affine.method;
              return true;
            }
            return false;
          });

  ransacReprojThreshold_ctl =
      add_numeric_box<double>("ransacReprojThreshold",
          "",
          [this](double v) {
            if ( options_ && options_->affine.ransacReprojThreshold != v ) {
              options_->affine.ransacReprojThreshold = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( options_ ) {
              *v = options_->affine.ransacReprojThreshold;
              return true;
            }
            return false;
          });

  confidence_ctl =
      add_numeric_box<double>("confidence",
          "",
          [this](double v) {
            if ( options_ && options_->affine.confidence != v ) {
              options_->affine.confidence = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( options_ ) {
              *v = options_->affine.confidence;
              return true;
            }
            return false;
          });

  maxIters_ctl =
      add_numeric_box<int>("maxIters",
          "",
          [this](int v) {
            if ( options_ && options_->affine.maxIters != v ) {
              options_->affine.maxIters = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( options_ ) {
              *v = options_->affine.maxIters;
              return true;
            }
            return false;
          });

  refineIters_ctl =
      add_numeric_box<int>("refineIters",
          "",
          [this](int v) {
            if ( options_ && options_->affine.refineIters != v ) {
              options_->affine.refineIters = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( options_ ) {
              *v = options_->affine.refineIters;
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
            if ( options_ && options_->homography.method != v ) {
              options_->homography.method = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](ROBUST_METHOD * v) {
            if ( options_ ) {
              *v = options_->homography.method;
              return true;
            }
            return false;
          });

  ransacReprojThreshold_ctl =
      add_numeric_box<double>("ransacReprojThreshold",
          "",
          [this](double v) {
            if ( options_ && options_->homography.ransacReprojThreshold != v ) {
              options_->homography.ransacReprojThreshold = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( options_ ) {
              *v = options_->homography.ransacReprojThreshold;
              return true;
            }
            return false;
          });

  confidence_ctl =
      add_numeric_box<double>("confidence",
          "",
          [this](double v) {
            if ( options_ && options_->homography.confidence != v ) {
              options_->homography.confidence = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( options_ ) {
              *v = options_->homography.confidence;
              return true;
            }
            return false;
          });

  maxIters_ctl =
      add_numeric_box<int>("maxIters",
          "",
          [this](int v) {
            if ( options_ && options_->homography.maxIters != v ) {
              options_->homography.maxIters = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( options_ ) {
              *v = options_->homography.maxIters;
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
            if ( options_ && options_->semi_quadratic.rmse_factor != v ) {
              options_->semi_quadratic.rmse_factor = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( options_ ) {
              *v = options_->semi_quadratic.rmse_factor;
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
            if ( options_ && options_->quadratic.rmse_factor != v ) {
              options_->quadratic.rmse_factor = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( options_ ) {
              *v = options_->quadratic.rmse_factor;
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
            if ( options_ && options_->epipolar_derotation.camera_pose.direction != v ) {
              options_->epipolar_derotation.camera_pose.direction = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](EPIPOLAR_MOTION_DIRECTION * v) {
            if ( options_ ) {
              *v = options_->epipolar_derotation.camera_pose.direction;
              return true;
            }
            return false;
          });

  max_iterations_ctl =
      add_numeric_box<int>("max_iterations",
          "",
          [this](int v) {
            if ( options_ && options_->epipolar_derotation.camera_pose.max_iterations != v ) {
              options_->epipolar_derotation.camera_pose.max_iterations = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( options_ ) {
              *v = options_->epipolar_derotation.camera_pose.max_iterations;
              return true;
            }
            return false;
          });

  max_levmar_iterations_ctl =
      add_numeric_box<int>("max_levmar_iterations",
          "",
          [this](int v) {
            if ( options_ && options_->epipolar_derotation.camera_pose.max_levmar_iterations != v ) {
              options_->epipolar_derotation.camera_pose.max_levmar_iterations = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( options_ ) {
              *v = options_->epipolar_derotation.camera_pose.max_levmar_iterations;
              return true;
            }
            return false;
          });

  robust_threshold_ctl =
      add_numeric_box<double>("robust_threshold",
          "",
          [this](double v) {
            if ( options_ && options_->epipolar_derotation.camera_pose.robust_threshold != v ) {
              options_->epipolar_derotation.camera_pose.robust_threshold = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( options_ ) {
              *v = options_->epipolar_derotation.camera_pose.robust_threshold;
              return true;
            }
            return false;
          });

  epsf_ctl =
      add_numeric_box<double>("epsf",
          "",
          [this](double v) {
            if ( options_ && options_->epipolar_derotation.camera_pose.epsf != v ) {
              options_->epipolar_derotation.camera_pose.epsf = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( options_ ) {
              *v = options_->epipolar_derotation.camera_pose.epsf;
              return true;
            }
            return false;
          });

  epsx_ctl =
      add_numeric_box<double>("epsx",
          "",
          [this](double v) {
            if ( options_ && options_->epipolar_derotation.camera_pose.epsx != v ) {
              options_->epipolar_derotation.camera_pose.epsx = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( options_ ) {
              *v = options_->epipolar_derotation.camera_pose.epsx;
              return true;
            }
            return false;
          });

  initial_translation_ctl =
      add_numeric_box<cv::Vec3f>("initial_translation",
          "",
          [this](const cv::Vec3f & v) {
            if ( options_ && options_->epipolar_derotation.initial_translation != v ) {
              options_->epipolar_derotation.initial_translation = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Vec3f * v) {
            if ( options_ ) {
              *v = options_->epipolar_derotation.initial_translation;
              return true;
            }
            return false;
          });

  initial_rotation_ctl =
      add_numeric_box<cv::Vec3f>("initial_rotation [deg]",
          "",
          [this](const cv::Vec3f & v) {
            if ( options_ && options_->epipolar_derotation.initial_rotation != v ) {
              options_->epipolar_derotation.initial_rotation = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Vec3f * v) {
            if ( options_ ) {
              *v = options_->epipolar_derotation.initial_rotation;
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
  scale_ctl =
      add_numeric_box<double>("Image scale",
          "",
          [this](double value) {
            if ( options_ && options_->scale != value ) {
              options_->scale = value;
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
  if( !options_ ) {
    setEnabled(false);
  }
  else {

    scale_ctl->setValue(options_->scale);
    sparseFeatureDetectorOptions_ctl->set_feature_detector_options(
        &options_->sparse_feature_extractor_and_matcher.detector);
    sparseFeatureDescriptorOptions_ctl->set_feature_descriptor_options(
        &options_->sparse_feature_extractor_and_matcher.descriptor);
    sparseFeatureMatcherOptions_ctl->set_feature_matcher_options(
        &options_->sparse_feature_extractor_and_matcher.matcher);

    estimateTranslation_ctl->set_options(&options_->estimate_options);
    estimateEuclidean_ctl->set_options(&options_->estimate_options);
    estimateScaledEuclidean_ctl->set_options(&options_->estimate_options);
    estimateAffine_ctl->set_options(&options_->estimate_options);
    estimateHomography_ctl->set_options(&options_->estimate_options);
    estimateSemiQuadratic_ctl->set_options(&options_->estimate_options);
    estimateQuadratic_ctl->set_options(&options_->estimate_options);
    estimateEpipolarDerotation_ctl->set_options(&options_->estimate_options);

    onDetectorTypeChanged();
    setEnabled(true);
  }
}

void QFeatureBasedRegistrationOptions::onDetectorTypeChanged()
{
  if( options_ ) {

    if( options_->sparse_feature_extractor_and_matcher.detector.type == SPARSE_FEATURE_DETECTOR_PLANETARY_DISK ) {
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
            if ( options_ && options_->scale != value ) {
              options_->scale = value;
              Q_EMIT parameterChanged();
            }
          });

  eps_ctl =
      add_numeric_box<double>("eps",
          "",
          [this](double value) {
            if ( options_ && options_->eps != value ) {
              options_->eps = value;
              Q_EMIT parameterChanged();
            }
          });

  min_rho_ctl =
      add_numeric_box<double>("min_rho",
          "",
          [this](double value) {
            if ( options_ && options_->min_rho != value ) {
              options_->min_rho = value;
              Q_EMIT parameterChanged();
            }
          });

  input_smooth_sigma_ctl =
      add_numeric_box<double>("input_smooth_sigma",
          "",
          [this](double value) {
            if ( options_ && options_->input_smooth_sigma != value ) {
              options_->input_smooth_sigma = value;
              Q_EMIT parameterChanged();
            }
          });

  reference_smooth_sigma_ctl =
      add_numeric_box<double>("reference_smooth_sigma",
          "",
          [this](double value) {
            if ( options_ && options_->reference_smooth_sigma != value ) {
              options_->reference_smooth_sigma = value;
              Q_EMIT parameterChanged();
            }
          });

  update_step_scale_ctl =
      add_numeric_box<double>("update_step_scale",
          "",
          [this](double value) {
            if ( options_ && options_->update_step_scale != value ) {
              options_->update_step_scale = value;
              Q_EMIT parameterChanged();
            }
          });

//  normalization_noise_ctl =
//      add_numeric_box<double>("normalization_noise",
//          "",
//          [this](double value) {
//            if ( options_ && options_->normalization_noise != value ) {
//              options_->normalization_noise = value;
//              Q_EMIT parameterChanged();
//            }
//          });
//
//  normalization_scale_ctl =
//      add_numeric_box<int>("normalization_scale",
//          "",
//          [this](int value) {
//            if ( options_ && options_->normalization_scale != value ) {
//              options_->normalization_scale = value;
//              Q_EMIT parameterChanged();
//            }
//          });

  max_iterations_ctl =
      add_numeric_box<int>("max_iterations",
          "",
          [this](int value) {
            if ( options_ && options_->max_iterations != value ) {
              options_->max_iterations = value;
              Q_EMIT parameterChanged();
            }
          });

  enable_ecch_ctl =
      add_checkbox("enable_ecch",
          "",
          [this](bool checked) {
            if ( options_ && options_->enable_ecch != checked ) {
              options_->enable_ecch = checked;
              ecch_minimum_image_size_ctl->setEnabled(options_->enable_ecch);
              ecch_estimate_translation_first_ctl->setEnabled(options_->enable_ecch);
              Q_EMIT parameterChanged();
            }
          });

  ecch_estimate_translation_first_ctl =
      add_checkbox("Estimate translation first",
          "",
          [this](bool checked) {
            if ( options_ && options_->ecch_estimate_translation_first != checked ) {
              options_->ecch_estimate_translation_first = checked;
              Q_EMIT parameterChanged();
            }
          });

  ecch_minimum_image_size_ctl =
      add_numeric_box<int>("ecch_minimum_image_size",
          "",
          [this](int value) {
            if ( options_ && options_->ecch_minimum_image_size != value ) {
              options_->ecch_minimum_image_size = value;
              Q_EMIT parameterChanged();
            }
          });

  replace_planetary_disk_with_mask_ctl =
      add_checkbox("replace_planetary_disk_with_mask",
          "",
          [this](bool checked) {
            if ( options_ && options_->replace_planetary_disk_with_mask != checked ) {
              options_->replace_planetary_disk_with_mask = checked;
              planetary_disk_mask_stdev_factor_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          });

  planetary_disk_mask_stdev_factor_ctl =
      add_numeric_box<double>("stdev_factor",
          "",
          [this](double value) {
            if ( options_ && options_->planetary_disk_mask_stdev_factor != value ) {
              options_->planetary_disk_mask_stdev_factor = value;
              Q_EMIT parameterChanged();
            }
          });

  updateControls();
}

void QEccRegistrationOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {
    scale_ctl->setValue(options_->scale);
    eps_ctl->setValue(options_->eps);
    min_rho_ctl->setValue(options_->min_rho);
    input_smooth_sigma_ctl->setValue(options_->input_smooth_sigma);
    reference_smooth_sigma_ctl->setValue(options_->reference_smooth_sigma);
    update_step_scale_ctl->setValue(options_->update_step_scale);
//    normalization_noise_ctl->setValue(options_->normalization_noise);
//    normalization_scale_ctl->setValue(options_->normalization_scale);
    max_iterations_ctl->setValue(options_->max_iterations);

    enable_ecch_ctl->setChecked(options_->enable_ecch);
    ecch_estimate_translation_first_ctl->setChecked(options_->ecch_estimate_translation_first);
    ecch_minimum_image_size_ctl->setValue(options_->ecch_minimum_image_size);

    ecch_minimum_image_size_ctl->setEnabled(options_->enable_ecch);
    ecch_estimate_translation_first_ctl->setEnabled(options_->enable_ecch);

    replace_planetary_disk_with_mask_ctl->setChecked(options_->replace_planetary_disk_with_mask);
    planetary_disk_mask_stdev_factor_ctl->setEnabled(options_->replace_planetary_disk_with_mask);
    planetary_disk_mask_stdev_factor_ctl->setValue(options_->planetary_disk_mask_stdev_factor);

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
            if ( options_ && options_->support_scale != value ) {
              options_->support_scale = value;
              Q_EMIT parameterChanged();
            }
          });

  downscale_method_ctl =
      add_enum_combobox<c_eccflow::DownscaleMethod>("downscale_method",
          "",
          [this](c_eccflow::DownscaleMethod value) {
            if ( options_ && options_->downscale_method != value ) {
              options_->downscale_method = value;
              Q_EMIT parameterChanged();
            }
          });

  min_image_size_ctl =
      add_numeric_box<int>("min_image_size",
          "",
          [this](int value) {
            if ( options_ && options_->min_image_size != value ) {
              options_->min_image_size = value;
              Q_EMIT parameterChanged();
            }
          });

  max_pyramid_level_ctl =
      add_numeric_box<int>("max_pyramid_level",
          "",
          [this](int value) {
            if ( options_ && options_->max_pyramid_level != value ) {
              options_->max_pyramid_level = value;
              Q_EMIT parameterChanged();
            }
          });

  scale_factor_ctl =
      add_numeric_box<double>("scale_factor",
          "",
          [this](double value) {
            if ( options_ && options_->scale_factor != value ) {
              options_->scale_factor = value;
              Q_EMIT parameterChanged();
            }
          });

  input_smooth_sigma_ctl =
      add_numeric_box<double>("input_smooth_sigma",
          "",
          [this](double value) {
            if ( options_ && options_->input_smooth_sigma != value ) {
              options_->input_smooth_sigma = value;
              Q_EMIT parameterChanged();
            }
          });

  reference_smooth_sigma_ctl =
      add_numeric_box<double>("reference_smooth_sigma",
          "",
          [this](double value) {
            if ( options_ && options_->reference_smooth_sigma != value ) {
              options_->reference_smooth_sigma = value;
              Q_EMIT parameterChanged();
            }
          });

  update_multiplier_ctl =
      add_numeric_box<double>("update_multiplier",
          "",
          [this](double value) {
            if ( options_ && options_->update_multiplier != value ) {
              options_->update_multiplier = value;
              Q_EMIT parameterChanged();
            }
          });

  max_iterations_ctl =
      add_numeric_box<int>("max_iterations",
          "",
          [this](int value) {
            if ( options_ && options_->max_iterations != value ) {
              options_->max_iterations = value;
              Q_EMIT parameterChanged();
            }
          });

  noise_level_ctl =
      add_numeric_box<double>("noise_level",
          "Set > 0 to manually force noise level",
          [this](double value) {
            if ( options_ && options_->noise_level != value ) {
              options_->noise_level = value;
              Q_EMIT parameterChanged();
            }
          });

  updateControls();
}

void QEccFlowRegistrationOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {
    update_multiplier_ctl->setValue(options_->update_multiplier);
    input_smooth_sigma_ctl->setValue(options_->input_smooth_sigma);
    reference_smooth_sigma_ctl->setValue(options_->reference_smooth_sigma);
    max_iterations_ctl->setValue(options_->max_iterations);
    downscale_method_ctl->setValue(options_->downscale_method);
    support_scale_ctl->setValue(options_->support_scale);
    min_image_size_ctl->setValue(options_->min_image_size);
    max_pyramid_level_ctl->setValue(options_->max_pyramid_level);
    scale_factor_ctl->setValue(options_->scale_factor);
    noise_level_ctl->setValue(options_->noise_level);
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
            if ( options_ && options_->ellipse.stdev_factor != value ) {
              options_->ellipse.stdev_factor = value;
              Q_EMIT parameterChanged();
            }
          });

  jovian_detector_pca_blur_ctl =
      add_numeric_box<double>("pca_blur:",
          "",
          [this](double value) {
            if ( options_ && options_->ellipse.pca_blur != value ) {
              options_->ellipse.pca_blur = value;
              Q_EMIT parameterChanged();
            }
          });

  jovian_detector_ellipse_offset_ctl =
      add_numeric_box<cv::Point2f>("ellipse offset:",
          "",
          [this](const cv::Point2f & value) {
            if ( options_ && options_->ellipse.offset != value ) {
              options_->ellipse.offset = value;
              Q_EMIT parameterChanged();
            }
          });

  max_pyramid_level_ctl =
      add_numeric_box<int>("max pyramid level:",
          "",
          [this](int value) {
            if ( options_ && options_->max_pyramid_level != value ) {
              options_->max_pyramid_level = value;
              Q_EMIT parameterChanged();
            }
          });

  min_rotation_ctl =
      add_numeric_box<double>("min_rotation [deg]:",
          "",
          [this](double value) {
            if ( options_ && options_->min_rotation != (value *= M_PI / 180) ) {
              options_->min_rotation = value;
              Q_EMIT parameterChanged();
            }
          });

  max_rotation_ctl =
      add_numeric_box<double>("max_rotation [deg]:",
          "",
          [this](double value) {
            if ( options_ && options_->max_rotation != (value *= M_PI / 180) ) {
              options_->max_rotation = value;
              Q_EMIT parameterChanged();
            }
          });

  num_orientations_ctl =
      add_numeric_box<int>("num_orientations:",
          "",
          [this](int value) {
            if ( options_ && options_->num_orientations != value ) {
              options_->num_orientations = value;
              Q_EMIT parameterChanged();
            }
          });

  max_context_size_ctl =
      add_numeric_box<int>("max_context_size:",
          "",
          [this](double value) {
            if ( options_ && options_->max_context_size != value ) {
              options_->max_context_size = value;
              Q_EMIT parameterChanged();
            }
          });

  derotate_all_frames_ctl =
      add_checkbox("process all frames",
          "",
          [this](bool checked) {
            if ( options_ && options_->derotate_all_frames != checked ) {
              options_->derotate_all_frames = checked;
              Q_EMIT parameterChanged();
            }
          });

  updateControls();
}

void QJovianDerotationOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {
    jovian_detector_stdev_factor_ctl->setValue(options_->ellipse.stdev_factor);
    jovian_detector_pca_blur_ctl->setValue(options_->ellipse.pca_blur);
    jovian_detector_ellipse_offset_ctl->setValue(options_->ellipse.offset);
    max_pyramid_level_ctl->setValue(options_->max_pyramid_level);
    min_rotation_ctl->setValue(options_->min_rotation * 180 / M_PI);
    max_rotation_ctl->setValue(options_->max_rotation * 180 / M_PI);
    num_orientations_ctl->setValue(options_->num_orientations);
    max_context_size_ctl->setValue(options_->max_context_size);
    derotate_all_frames_ctl->setChecked(options_->derotate_all_frames);

    setEnabled(true);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//QMasterFrameOptions::QMasterFrameOptions(QWidget * parent) :
//    Base("", parent)
//{
//
//  master_frame_selection_ctl =
//      add_widget<QMasterFrameSelectionControl>(); // "Master Frame Selection"
//
//  connect(master_frame_selection_ctl, &QSettingsWidget::parameterChanged,
//      this, &ThisClass::parameterChanged);
//
////  masterFrameSelectionMethod_ctl =
////      add_enum_combobox<master_frame_selection_method>("Mater frame selection:",
////          "",
////          [this](master_frame_selection_method v) {
////            if ( options_ && options_->master_frame_selection.master_selection_method != v ) {
////              options_->master_frame_selection.master_selection_method = v;
////              updateMasterSourceControlStates();
////              Q_EMIT parameterChanged();
////            }
////          },
////          [this](master_frame_selection_method * v) {
////            if ( options_ ) {
////              *v = options_->master_frame_selection.master_selection_method;
////              return true;
////            }
////            return false;
////          });
//
////
////  masterSource_ctl =
////      add_combobox<QInputSourceSelectionCombo>("Master file:",
////          "Specify input source for master frame",
////          [this](int index, QInputSourceSelectionCombo * combo) {
////            if( options_ ) {
////
////              options_->master_source_fiename =
////                  combo->sourcePathFilename(index);
////
////              if ( true ) {
////                c_update_controls_lock lock(this);
////                masterFrameIndex_ctl->setRange(0, combo->sourceSize(index) - 1);
////                options_->master_frame_index = masterFrameIndex_ctl->value();
////              }
////
////              Q_EMIT parameterChanged();
////            }
////          },
////          [this](int * index, QInputSourceSelectionCombo * combo) -> bool {
////            if( options_ ) {
////
////              * index = combo->sourceIndex(options_->master_source_fiename);
////
////              if ( true ) {
////                c_update_controls_lock lock(this);
////                masterFrameIndex_ctl->setRange(0, combo->sourceSize(*index) - 1);
////                masterFrameIndex_ctl->setValue(options_->master_frame_index);
////                options_->master_frame_index = masterFrameIndex_ctl->value();
////              }
////
////              return true;
////            }
////
////            return false;
////          });
////
//
//  /////////
//  // xxx
////  masterSource_ctl = add_widget<QMasterSourceSelectionCombo>("Master file:");
////  masterSource_ctl->setToolTip("Specify input source for master frame");
////  connect(masterSource_ctl, &QMasterSourceSelectionCombo::currentSourceChanged,
////      [this]() {
////        if( options_ && !updatingControls() ) {
////
////          const QMasterSourceSelectionCombo::InputSourceData data =
////              masterSource_ctl->currentInputSource();
////
////          options_->master_frame_selection.master_fiename =
////              data.source_pathfilename;
////
////          if ( true ) {
////            c_update_controls_lock lock(this);
////            masterFrameIndex_ctl->setRange(0, data.source_size-1);
////            options_->master_frame_selection.master_frame_index = masterFrameIndex_ctl->value();
////          }
////
////          Q_EMIT parameterChanged();
////        }
////      });
////
////  connect(this, &ThisClass::populatecontrols,
////      [this](){
////        if( options_ ) {
////
////          masterSource_ctl->setCurrentInputSource(options_->master_frame_selection.master_fiename);
////
////          if ( true ) {
////            c_update_controls_lock lock(this);
////            masterFrameIndex_ctl->setRange(0, masterSource_ctl->currentInputSource().source_size - 1);
////            masterFrameIndex_ctl->setValue(options_->master_frame_selection.master_frame_index);
////            options_->master_frame_selection.master_frame_index = masterFrameIndex_ctl->value();
////          }
////        }
////      });
////
////
////  masterFrameIndex_ctl =
////      add_spinbox("Master frame Index:",
////          "",
////          [this](int v) {
////            if ( options_ && options_->master_frame_selection.master_frame_index != v ) {
////              options_->master_frame_selection.master_frame_index = v;
////              Q_EMIT parameterChanged();
////            }
////          },
////          [this](int * v) {
////            if ( options_ ) {
////              *v = options_->master_frame_selection.master_frame_index;
////              return true;
////            }
////            return false;
////          });
//
//  // xxx
///////////////
//
////  masterFrameIndex_ctl = new QSpinBox(this);
////  masterFrameIndex_ctl->setKeyboardTracking(false);
////  masterFrameIndex_ctl->setFocusPolicy(Qt::StrongFocus);
////  connect(masterFrameIndex_ctl, SIGNAL(valueChanged(int)),
////      this, SLOT(onSpinBoxValueChanged(int)));
//
//  apply_input_frame_processors_ctl =
//      add_checkbox("Apply input processors:",
//          "",
//          [this](bool checked) {
//            if ( options_ && options_->apply_input_image_processor != checked ) {
//              options_->apply_input_image_processor = checked;
//              Q_EMIT parameterChanged();
//            }
//          },
//          [this](bool * checked) {
//            if ( options_ ) {
//              *checked = options_->apply_input_image_processor;
//              return true;
//            }
//            return false;
//          });
//
////  connect(apply_input_frame_processors_ctl, &QCheckBox::stateChanged,
////      this, &ThisClass::onApplyInputFramePprocessorCheckboxStateChanged);
////  apply_input_frame_processors_ctl = new QCheckBox(this);
////  connect(apply_input_frame_processors_ctl, &QCheckBox::stateChanged,
////      this, &ThisClass::onApplyInputFramePprocessorCheckboxStateChanged);
//
//  generateMasterFrame_ctl =
//      add_checkbox("Generate Master Frame:",
//          "",
//          [this](bool checked) {
//            if ( options_ && options_->generate_master_frame != checked ) {
//              options_->generate_master_frame = checked;
//              updateGenerateMasterFrameControlStates();
//              Q_EMIT parameterChanged();
//            }
//          },
//          [this](bool * checked) {
//            if ( options_ ) {
//              *checked = options_->generate_master_frame;
//              return true;
//            }
//            return false;
//          });
//
////  generateMasterFrame_ctl = new QCheckBox(this);
////  connect(generateMasterFrame_ctl, &QCheckBox::stateChanged,
////      this, &ThisClass::onGenerateMasterFrameCheckboxStateChanged);
//
//  maxFramesForMasterFrameGeneration_ctl =
//      add_numeric_box<int>("Max frames:",
//          "Max frames for master frame generation",
//          [this](int v) {
//            if ( options_ && options_->max_frames_to_generate_master_frame != v ) {
//              options_->max_frames_to_generate_master_frame = v;
//              Q_EMIT parameterChanged();
//            }
//          },
//          [this](int * v) {
//            if ( options_ ) {
//              * v = options_->max_frames_to_generate_master_frame;
//              return true;
//            }
//            return false;
//          });
//
////  featureScale_ctl =
////      add_numeric_box<double>("feature scale:",
////      "",
////      [this](double v) {
////        if ( options_ && options_->feature_scale != v ) {
////          options_->feature_scale = v;
////          Q_EMIT parameterChanged();
////        }
////      },
////      [this](double * v) {
////        if ( options_ ) {
////          *v = options_->feature_scale;
////          return true;
////        }
////        return false;
////      });
////
////  eccScale_ctl =
////      add_numeric_box<double>("ECC scale:",
////      "",
////      [this](double v) {
////        if ( options_ && options_->ecc_scale != v ) {
////          options_->ecc_scale = v;
////          Q_EMIT parameterChanged();
////        }
////      },
////      [this](double * v) {
////        if ( options_ ) {
////          *v = options_->ecc_scale;
////          return true;
////        }
////        return false;
////      });
//
//  eccFlowScale_ctl =
//      add_numeric_box<int>("ECC flow log scale:",
//          "",
//          [this](int v) {
//            if ( options_ && options_->eccflow_scale != v ) {
//              options_->eccflow_scale = v;
//              Q_EMIT parameterChanged();
//            }
//          },
//          [this](int * v) {
//            if ( options_ ) {
//              *v = options_->eccflow_scale;
//              return true;
//            }
//            return false;
//          });
//
//  eccflowMinImageSize_ctl =
//      add_numeric_box<int>("ECC Min image size:",
//          "",
//          [this](int v) {
//            if ( options_ && options_->eccflow_min_image_size != v ) {
//              options_->eccflow_min_image_size = v;
//              Q_EMIT parameterChanged();
//            }
//          },
//          [this](int * v) {
//            if ( options_ ) {
//              *v = options_->eccflow_min_image_size;
//              return true;
//            }
//            return false;
//          });
//
//  eccflowMaxPyramidLevel_ctl =
//      add_numeric_box<int>("ECC Max Pyr. Level:",
//          "",
//          [this](int v) {
//            if ( options_ && options_->eccflow_max_pyramid_level != v ) {
//              options_->eccflow_max_pyramid_level = v;
//              Q_EMIT parameterChanged();
//            }
//          },
//          [this](int * v) {
//            if ( options_ ) {
//              *v = options_->eccflow_max_pyramid_level;
//              return true;
//            }
//            return false;
//          });
//
//  master_unsharp_sigma_ctl =
//      add_numeric_box<double>("Master unsharp sigma:",
//          "",
//          [this](double v) {
//            if ( options_ && options_->unsharp_sigma != v ) {
//              options_->unsharp_sigma = v;
//              Q_EMIT parameterChanged();
//            }
//          },
//          [this](double * v) {
//            if ( options_ ) {
//              *v = options_->unsharp_sigma;
//              return true;
//            }
//            return false;
//          });
//
//  master_unsharp_alpha_ctl =
//      add_numeric_box<double>("Master unsharp alpha:",
//          "",
//          [this](double v) {
//            if ( options_ && options_->unsharp_alpha != v ) {
//              options_->unsharp_alpha = v;
//              Q_EMIT parameterChanged();
//            }
//          },
//          [this](double * v) {
//            if ( options_ ) {
//              *v = options_->unsharp_alpha;
//              return true;
//            }
//            return false;
//          });
//
////  master_sharpen_factor_ctl = new QNumericBox(this);
////  connect(master_sharpen_factor_ctl, &QNumericBox::textChanged,
////      this, &ThisClass::onMasterSharpenFactorChanged);
//
//  accumulated_sharpen_factor_ctl =
//      add_numeric_box<double>("Acc. Sharpen Factor:",
//          "",
//          [this](double v) {
//            if ( options_ && options_->accumulated_sharpen_factor != v ) {
//              options_->accumulated_sharpen_factor = v;
//              Q_EMIT parameterChanged();
//            }
//          },
//          [this](double * v) {
//            if ( options_ ) {
//              * v = options_->accumulated_sharpen_factor;
//              return true;
//            }
//            return false;
//          });
//
////  accumulated_sharpen_factor_ctl = new QNumericBox(this);
////  connect(accumulated_sharpen_factor_ctl, &QNumericBox::textChanged,
////      this, &ThisClass::onAccumulatedSharpenFactorChanged);
//
//  saveMasterFrame_ctl =
//      add_checkbox("Save Master Frame:",
//          "",
//          [this](bool checked) {
//            if ( options_ && options_->save_master_frame != checked ) {
//              options_->save_master_frame = checked;
//              Q_EMIT parameterChanged();
//            }
//          },
//          [this](bool * checked) {
//            if ( options_ ) {
//              *checked = options_->save_master_frame;
//              return true;
//            }
//            return false;
//          });
//
////  saveMasterFrame_ctl = new QCheckBox(this);
////  connect(saveMasterFrame_ctl, &QCheckBox::stateChanged,
////      this, &ThisClass::onSaveMasterFrameCheckboxStateChanged);
//
////  form->addRow("Master file:", masterSource_ctl);
////  form->addRow("Master frame Index:", masterFrameIndex_ctl);
////  form->addRow("Apply input processors:", apply_input_frame_processors_ctl);
////
////  form->addRow("Generate master frame:", generateMasterFrame_ctl);
////  form->addRow("Max frames:", maxFramesForMasterFrameGeneration_ctl);
////  form->addRow("eccflow support scale:", eccFlowScale_ctl);
////  form->addRow("Sharpen factor:", master_sharpen_factor_ctl);
////  form->addRow("Acc. sharpen factor:", accumulated_sharpen_factor_ctl);
////
////  //form->addRow("Compensate master flow:", compensateMasterFlow_ctl);
////  form->addRow("Save Master Frame", saveMasterFrame_ctl);
//
//  //form->addRow(applyToAll_ctl);
//
//  setEnabled(false);
//}
//
////void QMasterFrameOptions::set_current_pipeline(c_image_stacking_pipeline * current_pipeline)
////{
////  if ( !(current_pipeline_ = current_pipeline) ) {
////    options_ = nullptr;
////  }
////  else {
////    options_ = &current_pipeline_->master_frame_options();
////  }
////
////  updateControls();
////}
////
////c_image_stacking_pipeline * QMasterFrameOptions::current_pipeline() const
////{
////  return current_pipeline_;
////}
//
//void QMasterFrameOptions::set_master_frame_options(c_master_frame_options * options)
//{
//  options_ = options;
//  updateControls();
//}
//
//c_master_frame_options* QMasterFrameOptions::master_frame_options() const
//{
//  return options_;
//}
//
//void QMasterFrameOptions::refreshInputSources(c_image_processing_pipeline * pipeline)
//{
//  c_update_controls_lock lock(this);
//  master_frame_selection_ctl->refreshInputSources(pipeline);
//}
//
//void QMasterFrameOptions::setEnableExternalFile(bool v)
//{
//  master_frame_selection_ctl->setEnableExternalFile(v);
//}
//
//bool QMasterFrameOptions::enableExternalFile() const
//{
//  return master_frame_selection_ctl->enableExternalFile();
//}
//
//void QMasterFrameOptions::onupdatecontrols()
//{
//  if( !options_ ) {
//    setEnabled(false);
//    //masterSource_ctl->clear();
//    master_frame_selection_ctl->set_options(nullptr);
//  }
//  else {
//
//    Base::populatecontrols();
//
//    //updateMasterSourceControlStates();
//    master_frame_selection_ctl->set_options(&options_->master_frame_selection);
//    updateGenerateMasterFrameControlStates();
//
////    generateMasterFrame_ctl->setChecked(options_->generate_master_frame);
////    maxFramesForMasterFrameGeneration_ctl->setValue(options_->max_frames_to_generate_master_frame);
////    apply_input_frame_processors_ctl->setChecked(options_->apply_input_frame_processors);
////    eccFlowScale_ctl->setValue(options_->eccflow_scale);
////    master_sharpen_factor_ctl->setValue(options_->master_sharpen_factor);
////    accumulated_sharpen_factor_ctl->setValue(options_->accumulated_sharpen_factor);
////
////    //compensateMasterFlow_ctl->setChecked(options_->compensate_master_flow);
////    saveMasterFrame_ctl->setChecked(options_->save_master_frame);
////
////
////    maxFramesForMasterFrameGeneration_ctl->setEnabled(options_->generate_master_frame);
////    eccFlowScale_ctl->setEnabled(options_->generate_master_frame);
////    master_sharpen_factor_ctl->setEnabled(options_->generate_master_frame);
//    //compensateMasterFlow_ctl->setEnabled(options_->generate_master_frame);
//
//    // Populate Master Source Combo
////    masterSource_ctl->clear();
////
////    if( current_pipeline_ ) {
////
////      const c_input_sequence::sptr &input_sequence =
////          current_pipeline_->input_sequence();
////
////      if( input_sequence ) {
////
////        for( int i = 0, n = input_sequence->sources().size(); i < n; ++i ) {
////          QString source_file_name = input_sequence->source(i)->filename().c_str();
////          masterSource_ctl->addItem(QFileInfo(source_file_name).fileName(), source_file_name);
////        }
////      }
////
////      const std::string &master_source =
////          current_pipeline_->master_source();
////
////      if( !master_source.empty() ) {
////        if( !input_sequence || input_sequence->indexof(master_source) < 0 ) {
////          QString source_file_name = master_source.c_str();
////          masterSource_ctl->addItem(QString("* %1").arg(QFileInfo(source_file_name).fileName()), source_file_name);
////        }
////      }
////
////      masterSource_ctl->addItem("Browse...");
////
////      // Select Current Index In Master Source Combo
////      if ( !master_source.empty() ) {
////        masterSource_ctl->setCurrentIndex(masterSource_ctl->findData(master_source.c_str()));
////      }
////      else {
////        masterSource_ctl->setCurrentIndex(0);
////        if ( masterSource_ctl->count() > 1 ) {
////          current_pipeline_->set_master_source(masterSource_ctl->itemData(0).toString().toStdString());
////        }
////      }
////    }
//
//    // updateMasterFrameIndex();
//    //previousComboboxItemIndex = masterSource_ctl->currentIndex();
//
//    //masterFrameSelectionMethod_ctl->setValue(options_->master_selection_method);
//    //masterFrameIndex_ctl->setEnabled(options_->master_selection_method == master_frame_specific_index);
//
//    setEnabled(true);
//  }
//}
//
////void QMasterFrameOptions::updateMasterFrameIndex()
////{
//////  if ( !current_pipeline_ ||  current_pipeline_->master_source().empty() ) {
//////    masterFrameIndex_ctl->setEnabled(false);
//////  }
//////  else {
//////
//////    const c_input_sequence::sptr & input_sequence =
//////        current_pipeline_->input_sequence();
//////
//////    c_input_source::sptr source;
//////
//////    if ( input_sequence && (source = input_sequence->source(current_pipeline_->master_source())) ) {
//////      if ( current_pipeline_->master_frame_index() < 0 || current_pipeline_->master_frame_index() >= source->size() ) {
//////        current_pipeline_->set_master_frame_index(0);
//////      }
//////      masterFrameIndex_ctl->setRange(0, source->size() - 1);
//////      masterFrameIndex_ctl->setValue(current_pipeline_->master_frame_index());
//////      masterFrameIndex_ctl->setEnabled(true);
//////    }
//////    else if ( !(source = c_input_source::create(current_pipeline_->master_source())) ) {
//////      CF_ERROR("c_input_source::create(pathfilename=%s) fails", current_pipeline_->master_source().c_str());
//////      masterFrameIndex_ctl->setEnabled(false);
//////    }
//////    else {
//////      if ( current_pipeline_->master_frame_index() < 0 || current_pipeline_->master_frame_index() >= source->size() ) {
//////        current_pipeline_->set_master_frame_index(0);
//////      }
//////      masterFrameIndex_ctl->setRange(0, source->size() - 1);
//////      masterFrameIndex_ctl->setValue(current_pipeline_->master_frame_index());
//////      masterFrameIndex_ctl->setEnabled(true);
//////    }
//////  }
////}
//
////void QMasterFrameOptions::onMasterSourceComboCurrentIndexChanged(int index)
////{
////  if ( current_pipeline_ && !updatingControls() && index >= 0 ) {
////
////    setUpdatingControls(true);
////
////    const int cn = masterSource_ctl->count();
////    if ( index == cn - 1 ) { // "Browse..."
////
////      QString selectedFileName = browseForMasterFrame();
////
////      if ( selectedFileName.isEmpty() ) {
////        masterSource_ctl->setCurrentIndex(previousComboboxItemIndex);
////      }
////      else {
////        current_pipeline_->set_master_source(selectedFileName.toStdString());
////        masterSource_ctl->insertItem(masterSource_ctl->count()-1, QString("* %1").arg(QFileInfo(selectedFileName).fileName()), selectedFileName);
////        masterSource_ctl->setCurrentIndex(masterSource_ctl->count()-2);
////      }
////    }
////    else {
////      QString selectedFileName = masterSource_ctl->itemData(index).toString();
////      if ( selectedFileName.isEmpty() ) {
////        masterSource_ctl->setCurrentIndex(previousComboboxItemIndex);
////      }
////      else {
////        current_pipeline_->set_master_source(selectedFileName.toStdString());
////      }
////    }
////
////    updateMasterFrameIndex();
////
////    previousComboboxItemIndex = masterSource_ctl->currentIndex();
////    setUpdatingControls(false);
////
////    Q_EMIT parameterChanged();
////  }
////}
//
////void QMasterFrameOptions::updateMasterSourceControlStates()
////{
////  if ( options_) {
////
////    switch (options_->master_frame_selection.master_selection_method) {
////      case master_frame_specific_index:
////        masterFrameIndex_ctl->setEnabled(true);
////        break;
////      case master_frame_middle_index:
////        masterFrameIndex_ctl->setEnabled(false);
////        break;
////      case master_frame_best_of_100_in_middle:
////        masterFrameIndex_ctl->setEnabled(false);
////        break;
////      default:
////        break;
////    }
////  }
////}
//
////void QMasterFrameOptions::onSpinBoxValueChanged(int value)
////{
////  if ( current_pipeline_ && !updatingControls() ) {
////    int currentComboboxIndex = masterSource_ctl->currentIndex();
////    if ( currentComboboxIndex >= 0 && currentComboboxIndex < masterSource_ctl->count() - 1 ) {
////      current_pipeline_->set_master_frame_index(value);
////      Q_EMIT parameterChanged();
////    }
////  }
////}
//
//void QMasterFrameOptions::updateGenerateMasterFrameControlStates()
//{
//  if( options_ ) {
//    maxFramesForMasterFrameGeneration_ctl->setEnabled(options_->generate_master_frame);
////    featureScale_ctl->setEnabled(options_->generate_master_frame);
////    eccScale_ctl->setEnabled(options_->generate_master_frame);
//    eccFlowScale_ctl->setEnabled(options_->generate_master_frame);
//    //    master_unsharp_sigma_ctl->setEnabled(options_->generate_master_frame);
//    //    master_unsharp_alpha_ctl->setEnabled(options_->generate_master_frame);
//  }
//}
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//QImageRegistrationOptions::QImageRegistrationOptions(QWidget * parent) :
//    Base("QImageRegistrationOptions", parent)
//{
//  motion_type_ctl =
//      add_enum_combobox<IMAGE_MOTION_TYPE>("Motion type:",
//          "",
//          [this](IMAGE_MOTION_TYPE value) {
//            if ( options_ && options_->motion_type != value ) {
//              options_->motion_type = value;
//              camera_matrix_groupbox_ctl->setVisible(options_->motion_type == IMAGE_MOTION_EPIPOLAR_DEROTATION);
//              Q_EMIT parameterChanged();
//            }
//          });
//
//  registration_channel_ctl =
//      add_enum_combobox<color_channel_type>("Registration channel:",
//          "",
//          [this](color_channel_type value) {
//            if ( options_ && options_->registration_channel != value ) {
//              options_->registration_channel = value;
//              Q_EMIT parameterChanged();
//            }
//          });
//
//  interpolation_method_ctl =
//      add_enum_combobox<ECC_INTERPOLATION_METHOD>("Interpolation method:",
//          "",
//          [this](ECC_INTERPOLATION_METHOD value) {
//            if ( options_ && options_->interpolation != value) {
//              options_->interpolation = value;
//              Q_EMIT parameterChanged();
//            }
//          });
//
//  border_mode_ctl =
//      add_enum_combobox<ECC_BORDER_MODE>("Border mode:",
//          "",
//          [this](ECC_BORDER_MODE value) {
//            if ( options_ && options_->border_mode != value ) {
//              options_->border_mode = value;
//              Q_EMIT parameterChanged();
//            }
//          });
//
//  border_value_ctl =
//      add_numeric_box<cv::Scalar>("Border Value",
//          "",
//          [this](const cv::Scalar & value) {
//            if ( options_ && options_->border_value != value ) {
//              options_->border_value = value;
//              Q_EMIT parameterChanged();
//            }
//          });
//
//  accumulateAndCompensateTurbulentFlow_ctl =
//      add_checkbox("accumulate and compensate turbulent flow",
//          "",
//          [this](bool checked) {
//            if ( options_ && options_->accumulate_and_compensate_turbulent_flow != checked ) {
//              options_->accumulate_and_compensate_turbulent_flow = checked;
//              Q_EMIT parameterChanged();
//            }
//          });
//
//  ///
//
//  camera_matrix_groupbox_ctl =
//      add_expandable_groupbox("Camera Matrix",
//          camera_matrix_ctl = new QCameraIntrinsicsEditBox(this));
//
//  //set_options
////  connect(camera_matrix_ctl, &QCameraIntrinsicsEditBox::parameterChanged,
////      [this]() {
////        if ( options_ && !updatingControls() ) {
//////          if ( camera_matrix_ctl->getMatrix(&options_->feature_registration.estimate_options.epipolar_derotation.camera_matrix) ) {
//////            Q_EMIT parameterChanged();
//////          }
////
////          options_->feature_registration.estimate_options.epipolar_derotation.camera_intrinsics =
////              camera_matrix_ctl->options();
////
////          if (  (&) ) {
////            Q_EMIT parameterChanged();
////          }
////        }
////      });
//
//  camera_matrix_groupbox_ctl->setVisible(false);
//  add_expandable_groupbox("Master Frame Options",
//      masterFrameOptions_ctl = new QMasterFrameOptions(this));
//
//  connect(masterFrameOptions_ctl, &QSettingsWidget::parameterChanged,
//      this, &ThisClass::parameterChanged);
//
//  add_expandable_groupbox("Feature Registration Options",
//      featureRegistrationOptions_ctl = new QFeatureBasedRegistrationOptions(this));
//  connect(featureRegistrationOptions_ctl, &QSettingsWidget::parameterChanged,
//      this, &ThisClass::parameterChanged);
//
//  add_expandable_groupbox("ECC Registration Options",
//      eccOptions_ctl = new QEccRegistrationOptions(this));
//  connect(eccOptions_ctl, &QSettingsWidget::parameterChanged,
//      this, &ThisClass::parameterChanged);
//
//  add_expandable_groupbox("Jovian Derotation Options",
//      jovianDerotationOptions_ctl = new QJovianDerotationOptions(this));
//  connect(jovianDerotationOptions_ctl, &QSettingsWidget::parameterChanged,
//      this, &ThisClass::parameterChanged);
//
//  add_expandable_groupbox("ECC Flow Registration Options",
//      eccFlowOptions_ctl = new QEccFlowRegistrationOptions(this));
//  connect(eccFlowOptions_ctl, &QSettingsWidget::parameterChanged,
//      this, &ThisClass::parameterChanged);
//
//  updateControls();
//}
//
//void QImageRegistrationOptions::set_registration_options(c_image_registration_options * options)
//{
//  options_ = options;
//  updateControls();
//}
//
//c_image_registration_options* QImageRegistrationOptions::registration_options() const
//{
//  return options_;
//}
//
//void QImageRegistrationOptions::refreshInputSources(c_image_processing_pipeline * pipeline)
//{
//  return masterFrameOptions_ctl->refreshInputSources(pipeline);
//}
//
//void QImageRegistrationOptions::setEnableExternalFile(bool v)
//{
//  masterFrameOptions_ctl->setEnableExternalFile(v);
//}
//
//bool QImageRegistrationOptions::enableExternalFile() const
//{
//  return masterFrameOptions_ctl->enableExternalFile();
//}
//
//void QImageRegistrationOptions::onupdatecontrols()
//{
//  if( !options_ ) {
//    setEnabled(false);
//
//    masterFrameOptions_ctl->set_master_frame_options(nullptr);
//    featureRegistrationOptions_ctl->set_options(nullptr);
//    eccOptions_ctl->set_options(nullptr);
//    eccFlowOptions_ctl->set_options(nullptr);
//    jovianDerotationOptions_ctl->set_options(nullptr);
//    camera_matrix_ctl->set_options(nullptr);
//    camera_matrix_groupbox_ctl->setVisible(false);
//  }
//  else {
//
//    motion_type_ctl->setValue(options_->motion_type);
//    registration_channel_ctl->setValue(options_->registration_channel);
//    interpolation_method_ctl->setValue(options_->interpolation);
//    border_mode_ctl->setValue(options_->border_mode);
//    border_value_ctl->setValue(options_->border_value);
//    camera_matrix_ctl->set_options(
//        &options_->feature_registration.estimate_options.epipolar_derotation.camera_intrinsics);
//    //camera_matrix_ctl->setMatrix(cv::Mat(options_->feature_registration.estimate_options.epipolar_derotation.camera_matrix));
//    camera_matrix_groupbox_ctl->setVisible(options_->motion_type == IMAGE_MOTION_EPIPOLAR_DEROTATION);
//    accumulateAndCompensateTurbulentFlow_ctl->setChecked(options_->accumulate_and_compensate_turbulent_flow);
//
//    masterFrameOptions_ctl->set_master_frame_options(&options_->master_frame_options);
//    featureRegistrationOptions_ctl->set_options(&options_->feature_registration);
//    eccOptions_ctl->set_options(&options_->ecc);
//    eccFlowOptions_ctl->set_options(&options_->eccflow);
//    jovianDerotationOptions_ctl->set_options(&options_->jovian_derotation);
//
//    setEnabled(true);
//  }
//}
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//QFrameRegistrationOptions::QFrameRegistrationOptions(QWidget * parent) :
//    Base("QFrameRegistrationOptions", parent)
//{
//  enable_frame_registration_ctl =
//      add_checkbox("Enable image registration",
//          "",
//          [this](bool checked) {
//            if ( options_ && options_->enabled != checked ) {
//              options_->enabled = checked;
//              update_controls_visibility();
//              Q_EMIT parameterChanged();
//            }
//          });
//
//  imageRegistrationOptions_ctl =
//      add_widget<QImageRegistrationOptions>();
//
//  connect(imageRegistrationOptions_ctl, &QSettingsWidget::parameterChanged,
//      this, &ThisClass::parameterChanged);
//
//  update_controls_visibility();
//}
//
////void QFrameRegistrationOptions::set_current_pipeline(c_image_stacking_pipeline * current_pipeline)
////{
////  current_pipeline_ = current_pipeline;
////  options_ = current_pipeline_ ? &current_pipeline_->registration_options() : nullptr;
////  updateControls();
////}
////
////c_image_stacking_pipeline * QFrameRegistrationOptions::current_pipeline() const
////{
////  return current_pipeline_;
////}
//
//void QFrameRegistrationOptions::set_registration_options(c_image_registration_options * options)
//{
//  options_ = options;
//  updateControls();
//}
//
//c_image_registration_options* QFrameRegistrationOptions::registration_options() const
//{
//  return options_;
//}
//
//void QFrameRegistrationOptions::refreshInputSources(c_image_processing_pipeline * pipeline)
//{
//  imageRegistrationOptions_ctl->refreshInputSources(pipeline);
//}
//
//void QFrameRegistrationOptions::setEnableExternalFile(bool v)
//{
//  imageRegistrationOptions_ctl->setEnableExternalFile(v);
//}
//
//bool QFrameRegistrationOptions::enableExternalFile() const
//{
//  return imageRegistrationOptions_ctl->enableExternalFile();
//}
//
//void QFrameRegistrationOptions::onupdatecontrols()
//{
//  imageRegistrationOptions_ctl->set_registration_options(options_);
//
//  if( !options_ ) {
//    setEnabled(false);
//  }
//  else {
//
//    enable_frame_registration_ctl->setChecked(options_->enabled);
//
//    update_controls_visibility();
//
//    setEnabled(true);
//  }
//}
//
//void QFrameRegistrationOptions::update_controls_visibility()
//{
//  if( !options_ ) {
//    imageRegistrationOptions_ctl->setVisible(false);
//    //accumulateAndCompensateTurbulentFlow_ctl->setEnabled(false);
//  }
//  else {
//    imageRegistrationOptions_ctl->setVisible(options_->enabled);
//    //accumulateAndCompensateTurbulentFlow_ctl->setEnabled(options_->enable_frame_registration);
//  }
//}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

