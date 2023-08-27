/*
 * QFrameRegistrationSettings.cc
 *
 *  Created on: Feb 9, 2021
 *      Author: amyznikov
 */

#include "QFrameRegistrationOptions.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QFeatureBasedRegistrationOptions::QFeatureBasedRegistrationOptions(QWidget * parent) :
    Base("QFeatureBasedRegistrationOptions", parent)
{
  enableFeatureBasedRegistration_ctl =
      add_checkbox("Enable feature based registration",
          "",
          [this](bool checked) {
            if ( options_ && options_->enabled != checked ) {
              options_->enabled = checked;
              update_controls_state();
              Q_EMIT parameterChanged();
            }
          });

  controls.append(scale_ctl =
      add_numeric_box<double>("Image scale",
          "",
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
          "",
          [this](bool checked) {
            if ( options_ && options_->enabled != checked ) {
              options_->enabled = checked;
              update_controls_state();
              Q_EMIT parameterChanged();
            }
          });

  controls.append(scale_ctl =
      add_numeric_box<double>("image scale",
          "",
          [this](double value) {
            if ( options_ && options_->scale != value ) {
              options_->scale = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(eps_ctl =
      add_numeric_box<double>("eps",
          "",
          [this](double value) {
            if ( options_ && options_->eps != value ) {
              options_->eps = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(min_rho_ctl =
      add_numeric_box<double>("min_rho",
          "",
          [this](double value) {
            if ( options_ && options_->min_rho != value ) {
              options_->min_rho = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(input_smooth_sigma_ctl =
      add_numeric_box<double>("input_smooth_sigma",
          "",
          [this](double value) {
            if ( options_ && options_->input_smooth_sigma != value ) {
              options_->input_smooth_sigma = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(reference_smooth_sigma_ctl =
      add_numeric_box<double>("reference_smooth_sigma",
          "",
          [this](double value) {
            if ( options_ && options_->reference_smooth_sigma != value ) {
              options_->reference_smooth_sigma = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(update_step_scale_ctl =
      add_numeric_box<double>("update_step_scale",
          "",
          [this](double value) {
            if ( options_ && options_->update_step_scale != value ) {
              options_->update_step_scale = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(normalization_noise_ctl =
      add_numeric_box<double>("normalization_noise",
          "",
          [this](double value) {
            if ( options_ && options_->normalization_noise != value ) {
              options_->normalization_noise = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(normalization_scale_ctl =
      add_numeric_box<int>("normalization_scale",
          "",
          [this](int value) {
            if ( options_ && options_->normalization_scale != value ) {
              options_->normalization_scale = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(max_iterations_ctl =
      add_numeric_box<int>("max_iterations",
          "",
          [this](int value) {
            if ( options_ && options_->max_iterations != value ) {
              options_->max_iterations = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(enable_ecch_ctl =
      add_checkbox("enable_ecch",
          "",
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
          "",
          [this](bool checked) {
            if ( options_ && options_->ecch_estimate_translation_first != checked ) {
              options_->ecch_estimate_translation_first = checked;
              Q_EMIT parameterChanged();
            }
          }));


  controls.append(ecch_minimum_image_size_ctl =
      add_numeric_box<int>("ecch_minimum_image_size",
          "",
          [this](int value) {
            if ( options_ && options_->ecch_minimum_image_size != value ) {
              options_->ecch_minimum_image_size = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(replace_planetary_disk_with_mask_ctl =
      add_checkbox("replace_planetary_disk_with_mask",
          "",
          [this](bool checked) {
            if ( options_ && options_->replace_planetary_disk_with_mask != checked ) {
              options_->replace_planetary_disk_with_mask = checked;
              planetary_disk_mask_stdev_factor_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(planetary_disk_mask_stdev_factor_ctl =
      add_numeric_box<double>("stdev_factor",
          "",
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
          "",
          [this](bool checked) {
            if ( options_ && options_->enabled != checked ) {
              options_->enabled = checked;
              update_controls_state();
              Q_EMIT parameterChanged();
            }
          });



  controls.append(support_scale_ctl =
      add_numeric_box<int>("support_scale",
          "",
          [this](int value) {
            if ( options_ && options_->support_scale != value ) {
              options_->support_scale = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(input_smooth_sigma_ctl =
      add_numeric_box<double>("input_smooth_sigma",
          "",
          [this](double value) {
            if ( options_ && options_->input_smooth_sigma != value ) {
              options_->input_smooth_sigma = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(reference_smooth_sigma_ctl =
      add_numeric_box<double>("reference_smooth_sigma",
          "",
          [this](double value) {
            if ( options_ && options_->reference_smooth_sigma != value ) {
              options_->reference_smooth_sigma = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(update_multiplier_ctl =
      add_numeric_box<double>("update_multiplier",
          "",
          [this](double value) {
            if ( options_ && options_->update_multiplier != value ) {
              options_->update_multiplier = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(max_iterations_ctl =
      add_numeric_box<int>("max_iterations",
          "",
          [this](int value) {
            if ( options_ && options_->max_iterations != value ) {
              options_->max_iterations = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(normalization_scale_ctl =
      add_numeric_box<int>("normalization_scale",
          "",
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
          "",
          [this](bool checked) {
            if ( options_ && options_->enabled != checked ) {
              options_->enabled = checked;
              update_controls_state();
              Q_EMIT parameterChanged();
            }
          });

  controls.append(jovian_detector_stdev_factor_ctl =
      add_numeric_box<double>("stdev_factor:",
          "",
          [this](double value) {
            if ( options_ && options_->ellipse.stdev_factor != value ) {
              options_->ellipse.stdev_factor = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(jovian_detector_pca_blur_ctl =
      add_numeric_box<double>("pca_blur:",
          "",
          [this](double value) {
            if ( options_ && options_->ellipse.pca_blur != value ) {
              options_->ellipse.pca_blur = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(max_pyramid_level_ctl =
      add_numeric_box<int>("max pyramid level:",
          "",
          [this](int value) {
            if ( options_ && options_->max_pyramid_level != value ) {
              options_->max_pyramid_level = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(min_rotation_ctl =
      add_numeric_box<double>("min_rotation [deg]:",
          "",
          [this](double value) {
            if ( options_ && options_->min_rotation != (value *= M_PI / 180) ) {
              options_->min_rotation = value;
              Q_EMIT parameterChanged();
            }
          }));

  controls.append(max_rotation_ctl =
      add_numeric_box<double>("max_rotation [deg]:",
          "",
          [this](double value) {
            if ( options_ && options_->max_rotation != (value *= M_PI / 180) ) {
              options_->max_rotation = value;
              Q_EMIT parameterChanged();
            }
          }));



  controls.append(num_orientations_ctl =
      add_numeric_box<int>("num_orientations:",
          "",
          [this](int value) {
            if ( options_ && options_->num_orientations != value ) {
              options_->num_orientations = value;
              Q_EMIT parameterChanged();
            }
          }));


  controls.append(max_context_size_ctl =
      add_numeric_box<int>("max_context_size:",
          "",
          [this](double value) {
            if ( options_ && options_->max_context_size != value ) {
              options_->max_context_size = value;
              Q_EMIT parameterChanged();
            }
          }));


  controls.append(derotate_all_frames_ctl =
      add_checkbox("process all frames",
          "",
          [this](bool checked) {
            if ( options_ && options_->derotate_all_frames != checked ) {
              options_->derotate_all_frames = checked;
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
    jovian_detector_stdev_factor_ctl->setValue(options_->ellipse.stdev_factor);
    jovian_detector_pca_blur_ctl->setValue(options_->ellipse.pca_blur);
    max_pyramid_level_ctl->setValue(options_->max_pyramid_level);
    min_rotation_ctl->setValue(options_->min_rotation * 180 / M_PI);
    max_rotation_ctl->setValue(options_->max_rotation * 180 / M_PI);
    num_orientations_ctl->setValue(options_->num_orientations);
    max_context_size_ctl->setValue(options_->max_context_size);
    derotate_all_frames_ctl->setChecked(options_->derotate_all_frames);
    update_controls_state();

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

QMasterSourceSelectionCombo::QMasterSourceSelectionCombo(QWidget * parent) :
    Base(parent)
{
  QHBoxLayout * layout = new QHBoxLayout(this);
  layout->setContentsMargins(0,0,0,0);


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

  if ( !pipeline ) {
    setEnabled(false);
    return;
  }

  const c_input_sequence::sptr & input_sequence =
      pipeline->input_sequence();

  if ( input_sequence ) {

    setEnabled(false);

    for ( const c_input_source::sptr & source : input_sequence->sources() ) {

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
  if ( !updatingControls() ) {
    Q_EMIT currentSourceChanged();
  }
}

void QMasterSourceSelectionCombo::onBrowseButtonClicked()
{
  static QString filter;

  if( filter.isEmpty() ) {

    filter.append("Regular images (");
    for( const std::string &s : c_regular_image_input_source::suffixes() ) {
      filter.append(QString("*%1 ").arg(QString(s.c_str())));
    }
    filter.append(");;");

#if HAVE_LIBRAW
      filter.append("RAW/DSLR images (");
      for ( const std::string & s : c_raw_image_input_source::suffixes() ) {
        filter.append(QString("*%1 ").arg(s.c_str()));
      }
      filter.append(");;");
  #endif

#if HAVE_CFITSIO
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QMasterFrameOptions::QMasterFrameOptions(QWidget * parent) :
  Base("", parent)
{
  masterFrameSelectionMethod_ctl =
      add_enum_combobox<master_frame_selection_method>("Mater frame selection:",
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

//
//  masterSource_ctl =
//      add_combobox<QInputSourceSelectionCombo>("Master file:",
//          "Specify input source for master frame",
//          [this](int index, QInputSourceSelectionCombo * combo) {
//            if( options_ ) {
//
//              options_->master_source_fiename =
//                  combo->sourcePathFilename(index);
//
//              if ( true ) {
//                c_update_controls_lock lock(this);
//                masterFrameIndex_ctl->setRange(0, combo->sourceSize(index) - 1);
//                options_->master_frame_index = masterFrameIndex_ctl->value();
//              }
//
//              Q_EMIT parameterChanged();
//            }
//          },
//          [this](int * index, QInputSourceSelectionCombo * combo) -> bool {
//            if( options_ ) {
//
//              * index = combo->sourceIndex(options_->master_source_fiename);
//
//              if ( true ) {
//                c_update_controls_lock lock(this);
//                masterFrameIndex_ctl->setRange(0, combo->sourceSize(*index) - 1);
//                masterFrameIndex_ctl->setValue(options_->master_frame_index);
//                options_->master_frame_index = masterFrameIndex_ctl->value();
//              }
//
//              return true;
//            }
//
//            return false;
//          });
//

  masterSource_ctl = add_widget<QMasterSourceSelectionCombo>("Master file:");
  masterSource_ctl->setToolTip("Specify input source for master frame");
  connect(masterSource_ctl, &QMasterSourceSelectionCombo::currentSourceChanged,
      [this]() {
        if( options_ && !updatingControls() ) {

          const QMasterSourceSelectionCombo::InputSourceData data =
              masterSource_ctl->currentInputSource();

          options_->master_fiename =
              data.source_pathfilename;

          if ( true ) {
            c_update_controls_lock lock(this);
            masterFrameIndex_ctl->setRange(0, data.source_size-1);
            options_->master_frame_index = masterFrameIndex_ctl->value();
          }

          Q_EMIT parameterChanged();
        }
      });

  connect(this, &ThisClass::populatecontrols,
      [this](){
        if( options_ ) {

          masterSource_ctl->setCurrentInputSource(options_->master_fiename);

          if ( true ) {
            c_update_controls_lock lock(this);
            masterFrameIndex_ctl->setRange(0, masterSource_ctl->currentInputSource().source_size - 1);
            masterFrameIndex_ctl->setValue(options_->master_frame_index);
            options_->master_frame_index = masterFrameIndex_ctl->value();
          }
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



//  masterFrameIndex_ctl = new QSpinBox(this);
//  masterFrameIndex_ctl->setKeyboardTracking(false);
//  masterFrameIndex_ctl->setFocusPolicy(Qt::StrongFocus);
//  connect(masterFrameIndex_ctl, SIGNAL(valueChanged(int)),
//      this, SLOT(onSpinBoxValueChanged(int)));


  apply_input_frame_processors_ctl =
      add_checkbox("Apply input processors:",
          "",
          [this](bool checked) {
            if ( options_ && options_->apply_input_frame_processors != checked ) {
              options_->apply_input_frame_processors = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_  ) {
              *checked = options_->apply_input_frame_processors;
              return true;
            }
            return false;
          });

//  connect(apply_input_frame_processors_ctl, &QCheckBox::stateChanged,
//      this, &ThisClass::onApplyInputFramePprocessorCheckboxStateChanged);
//  apply_input_frame_processors_ctl = new QCheckBox(this);
//  connect(apply_input_frame_processors_ctl, &QCheckBox::stateChanged,
//      this, &ThisClass::onApplyInputFramePprocessorCheckboxStateChanged);


  generateMasterFrame_ctl =
      add_checkbox("Generate Master Frame:",
          "",
          [this](bool checked) {
            if ( options_ && options_->generate_master_frame != checked ) {
              options_->generate_master_frame = checked;
              updateGenerateMasterFrameControlStates();
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              *checked = options_->generate_master_frame;
              return true;
            }
            return false;
          });

//  generateMasterFrame_ctl = new QCheckBox(this);
//  connect(generateMasterFrame_ctl, &QCheckBox::stateChanged,
//      this, &ThisClass::onGenerateMasterFrameCheckboxStateChanged);

  maxFramesForMasterFrameGeneration_ctl =
      add_numeric_box<int>("Max frames:",
          "Max frames for master frame generation",
          [this](int v) {
            if ( options_ && options_->max_frames_to_generate_master_frame != v ) {
              options_->max_frames_to_generate_master_frame = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( options_ ) {
              * v = options_->max_frames_to_generate_master_frame;
              return true;
            }
            return false;
          });

//  featureScale_ctl =
//      add_numeric_box<double>("feature scale:",
//      "",
//      [this](double v) {
//        if ( options_ && options_->feature_scale != v ) {
//          options_->feature_scale = v;
//          Q_EMIT parameterChanged();
//        }
//      },
//      [this](double * v) {
//        if ( options_ ) {
//          *v = options_->feature_scale;
//          return true;
//        }
//        return false;
//      });
//
//  eccScale_ctl =
//      add_numeric_box<double>("ECC scale:",
//      "",
//      [this](double v) {
//        if ( options_ && options_->ecc_scale != v ) {
//          options_->ecc_scale = v;
//          Q_EMIT parameterChanged();
//        }
//      },
//      [this](double * v) {
//        if ( options_ ) {
//          *v = options_->ecc_scale;
//          return true;
//        }
//        return false;
//      });

  eccFlowScale_ctl =
      add_numeric_box<int>("ECC flow log scale:",
      "",
      [this](int v) {
        if ( options_ && options_->eccflow_scale != v ) {
          options_->eccflow_scale = v;
          Q_EMIT parameterChanged();
        }
      },
      [this](int * v) {
        if ( options_ ) {
          *v = options_->eccflow_scale;
          return true;
        }
        return false;
      });


  master_sharpen_factor_ctl =
      add_numeric_box<double>("Master Sharpen Factor:",
          "",
          [this](double v) {
            if ( options_ && options_->master_sharpen_factor != v ) {
              options_->master_sharpen_factor = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( options_ ) {
              *v = options_->master_sharpen_factor;
              return true;
            }
            return false;
          });

//  master_sharpen_factor_ctl = new QNumericBox(this);
//  connect(master_sharpen_factor_ctl, &QNumericBox::textChanged,
//      this, &ThisClass::onMasterSharpenFactorChanged);

  accumulated_sharpen_factor_ctl =
      add_numeric_box<double>("Acc. Sharpen Factor:",
      "",
      [this](double v) {
        if ( options_ && options_->accumulated_sharpen_factor != v ) {
          options_->accumulated_sharpen_factor = v;
          Q_EMIT parameterChanged();
        }
      },
      [this](double * v) {
        if ( options_ ) {
          * v = options_->accumulated_sharpen_factor;
          return true;
        }
        return false;
      });

//  accumulated_sharpen_factor_ctl = new QNumericBox(this);
//  connect(accumulated_sharpen_factor_ctl, &QNumericBox::textChanged,
//      this, &ThisClass::onAccumulatedSharpenFactorChanged);

  saveMasterFrame_ctl =
      add_checkbox("Save Master Frame:",
          "",
          [this](bool checked) {
            if ( options_ && options_->save_master_frame != checked ) {
              options_->save_master_frame = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              *checked = options_->save_master_frame;
              return true;
            }
            return false;
          });

//  saveMasterFrame_ctl = new QCheckBox(this);
//  connect(saveMasterFrame_ctl, &QCheckBox::stateChanged,
//      this, &ThisClass::onSaveMasterFrameCheckboxStateChanged);


//  form->addRow("Master file:", masterSource_ctl);
//  form->addRow("Master frame Index:", masterFrameIndex_ctl);
//  form->addRow("Apply input processors:", apply_input_frame_processors_ctl);
//
//  form->addRow("Generate master frame:", generateMasterFrame_ctl);
//  form->addRow("Max frames:", maxFramesForMasterFrameGeneration_ctl);
//  form->addRow("eccflow support scale:", eccFlowScale_ctl);
//  form->addRow("Sharpen factor:", master_sharpen_factor_ctl);
//  form->addRow("Acc. sharpen factor:", accumulated_sharpen_factor_ctl);
//
//  //form->addRow("Compensate master flow:", compensateMasterFlow_ctl);
//  form->addRow("Save Master Frame", saveMasterFrame_ctl);

  //form->addRow(applyToAll_ctl);

  setEnabled(false);
}

//void QMasterFrameOptions::set_current_pipeline(c_image_stacking_pipeline * current_pipeline)
//{
//  if ( !(current_pipeline_ = current_pipeline) ) {
//    options_ = nullptr;
//  }
//  else {
//    options_ = &current_pipeline_->master_frame_options();
//  }
//
//  updateControls();
//}
//
//c_image_stacking_pipeline * QMasterFrameOptions::current_pipeline() const
//{
//  return current_pipeline_;
//}

void QMasterFrameOptions::set_master_frame_options(c_master_frame_options * options)
{
  options_ = options;
  updateControls();
}

c_master_frame_options * QMasterFrameOptions::master_frame_options() const
{
  return options_;
}

void QMasterFrameOptions::refreshInputSources(c_image_processing_pipeline * pipeline)
{
  c_update_controls_lock lock(this);
  masterSource_ctl->refreshInputSources(pipeline);
}

void QMasterFrameOptions::setEnableExternalFile(bool v)
{
  masterSource_ctl->setEnableExternalFile(v);
}

bool QMasterFrameOptions::enableExternalFile() const
{
  return masterSource_ctl->enableExternalFile();
}


void QMasterFrameOptions::onupdatecontrols()
{
  if ( !options_ ) {
    setEnabled(false);
    //masterSource_ctl->clear();
  }
  else {

    Base::populatecontrols();

    updateMasterSourceControlStates();
    updateGenerateMasterFrameControlStates();

//    generateMasterFrame_ctl->setChecked(options_->generate_master_frame);
//    maxFramesForMasterFrameGeneration_ctl->setValue(options_->max_frames_to_generate_master_frame);
//    apply_input_frame_processors_ctl->setChecked(options_->apply_input_frame_processors);
//    eccFlowScale_ctl->setValue(options_->eccflow_scale);
//    master_sharpen_factor_ctl->setValue(options_->master_sharpen_factor);
//    accumulated_sharpen_factor_ctl->setValue(options_->accumulated_sharpen_factor);
//
//    //compensateMasterFlow_ctl->setChecked(options_->compensate_master_flow);
//    saveMasterFrame_ctl->setChecked(options_->save_master_frame);
//
//
//    maxFramesForMasterFrameGeneration_ctl->setEnabled(options_->generate_master_frame);
//    eccFlowScale_ctl->setEnabled(options_->generate_master_frame);
//    master_sharpen_factor_ctl->setEnabled(options_->generate_master_frame);
    //compensateMasterFlow_ctl->setEnabled(options_->generate_master_frame);

    // Populate Master Source Combo
//    masterSource_ctl->clear();
//
//    if( current_pipeline_ ) {
//
//      const c_input_sequence::sptr &input_sequence =
//          current_pipeline_->input_sequence();
//
//      if( input_sequence ) {
//
//        for( int i = 0, n = input_sequence->sources().size(); i < n; ++i ) {
//          QString source_file_name = input_sequence->source(i)->filename().c_str();
//          masterSource_ctl->addItem(QFileInfo(source_file_name).fileName(), source_file_name);
//        }
//      }
//
//      const std::string &master_source =
//          current_pipeline_->master_source();
//
//      if( !master_source.empty() ) {
//        if( !input_sequence || input_sequence->indexof(master_source) < 0 ) {
//          QString source_file_name = master_source.c_str();
//          masterSource_ctl->addItem(QString("* %1").arg(QFileInfo(source_file_name).fileName()), source_file_name);
//        }
//      }
//
//      masterSource_ctl->addItem("Browse...");
//
//      // Select Current Index In Master Source Combo
//      if ( !master_source.empty() ) {
//        masterSource_ctl->setCurrentIndex(masterSource_ctl->findData(master_source.c_str()));
//      }
//      else {
//        masterSource_ctl->setCurrentIndex(0);
//        if ( masterSource_ctl->count() > 1 ) {
//          current_pipeline_->set_master_source(masterSource_ctl->itemData(0).toString().toStdString());
//        }
//      }
//    }

    // updateMasterFrameIndex();
    //previousComboboxItemIndex = masterSource_ctl->currentIndex();

    //masterFrameSelectionMethod_ctl->setValue(options_->master_selection_method);
    //masterFrameIndex_ctl->setEnabled(options_->master_selection_method == master_frame_specific_index);

    setEnabled(true);
  }
}

//void QMasterFrameOptions::updateMasterFrameIndex()
//{
////  if ( !current_pipeline_ ||  current_pipeline_->master_source().empty() ) {
////    masterFrameIndex_ctl->setEnabled(false);
////  }
////  else {
////
////    const c_input_sequence::sptr & input_sequence =
////        current_pipeline_->input_sequence();
////
////    c_input_source::sptr source;
////
////    if ( input_sequence && (source = input_sequence->source(current_pipeline_->master_source())) ) {
////      if ( current_pipeline_->master_frame_index() < 0 || current_pipeline_->master_frame_index() >= source->size() ) {
////        current_pipeline_->set_master_frame_index(0);
////      }
////      masterFrameIndex_ctl->setRange(0, source->size() - 1);
////      masterFrameIndex_ctl->setValue(current_pipeline_->master_frame_index());
////      masterFrameIndex_ctl->setEnabled(true);
////    }
////    else if ( !(source = c_input_source::create(current_pipeline_->master_source())) ) {
////      CF_ERROR("c_input_source::create(pathfilename=%s) fails", current_pipeline_->master_source().c_str());
////      masterFrameIndex_ctl->setEnabled(false);
////    }
////    else {
////      if ( current_pipeline_->master_frame_index() < 0 || current_pipeline_->master_frame_index() >= source->size() ) {
////        current_pipeline_->set_master_frame_index(0);
////      }
////      masterFrameIndex_ctl->setRange(0, source->size() - 1);
////      masterFrameIndex_ctl->setValue(current_pipeline_->master_frame_index());
////      masterFrameIndex_ctl->setEnabled(true);
////    }
////  }
//}

//void QMasterFrameOptions::onMasterSourceComboCurrentIndexChanged(int index)
//{
//  if ( current_pipeline_ && !updatingControls() && index >= 0 ) {
//
//    setUpdatingControls(true);
//
//    const int cn = masterSource_ctl->count();
//    if ( index == cn - 1 ) { // "Browse..."
//
//      QString selectedFileName = browseForMasterFrame();
//
//      if ( selectedFileName.isEmpty() ) {
//        masterSource_ctl->setCurrentIndex(previousComboboxItemIndex);
//      }
//      else {
//        current_pipeline_->set_master_source(selectedFileName.toStdString());
//        masterSource_ctl->insertItem(masterSource_ctl->count()-1, QString("* %1").arg(QFileInfo(selectedFileName).fileName()), selectedFileName);
//        masterSource_ctl->setCurrentIndex(masterSource_ctl->count()-2);
//      }
//    }
//    else {
//      QString selectedFileName = masterSource_ctl->itemData(index).toString();
//      if ( selectedFileName.isEmpty() ) {
//        masterSource_ctl->setCurrentIndex(previousComboboxItemIndex);
//      }
//      else {
//        current_pipeline_->set_master_source(selectedFileName.toStdString());
//      }
//    }
//
//    updateMasterFrameIndex();
//
//    previousComboboxItemIndex = masterSource_ctl->currentIndex();
//    setUpdatingControls(false);
//
//    Q_EMIT parameterChanged();
//  }
//}

void QMasterFrameOptions::updateMasterSourceControlStates()
{
  if ( options_) {

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

//void QMasterFrameOptions::onSpinBoxValueChanged(int value)
//{
//  if ( current_pipeline_ && !updatingControls() ) {
//    int currentComboboxIndex = masterSource_ctl->currentIndex();
//    if ( currentComboboxIndex >= 0 && currentComboboxIndex < masterSource_ctl->count() - 1 ) {
//      current_pipeline_->set_master_frame_index(value);
//      Q_EMIT parameterChanged();
//    }
//  }
//}

void QMasterFrameOptions::updateGenerateMasterFrameControlStates()
{
  if( options_ ) {
    maxFramesForMasterFrameGeneration_ctl->setEnabled(options_->generate_master_frame);
//    featureScale_ctl->setEnabled(options_->generate_master_frame);
//    eccScale_ctl->setEnabled(options_->generate_master_frame);
    eccFlowScale_ctl->setEnabled(options_->generate_master_frame);
    master_sharpen_factor_ctl->setEnabled(options_->generate_master_frame);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QImageRegistrationOptions::QImageRegistrationOptions(QWidget * parent) :
    Base("QImageRegistrationOptions", parent)
{
  motion_type_ctl =
      add_enum_combobox<IMAGE_MOTION_TYPE>("Motion type:",
          "",
          [this](IMAGE_MOTION_TYPE value) {
            if ( options_ && options_->motion_type != value ) {
              options_->motion_type = value;
              Q_EMIT parameterChanged();
            }
          });

  registration_channel_ctl =
      add_enum_combobox<color_channel_type>("Registration channel:",
          "",
          [this](color_channel_type value) {
            if ( options_ && options_->registration_channel != value ) {
              options_->registration_channel = value;
              Q_EMIT parameterChanged();
            }
          });

  interpolation_method_ctl =
      add_enum_combobox<ECC_INTERPOLATION_METHOD>("Interpolation method:",
          "",
          [this](ECC_INTERPOLATION_METHOD value) {
            if ( options_ && options_->interpolation != value) {
              options_->interpolation = value;
              Q_EMIT parameterChanged();
            }
          });

  border_mode_ctl =
      add_enum_combobox<ECC_BORDER_MODE>("Border mode:",
          "",
          [this](ECC_BORDER_MODE value) {
            if ( options_ && options_->border_mode != value ) {
              options_->border_mode = value;
              Q_EMIT parameterChanged();
            }
          });

  border_value_ctl =
      add_numeric_box<cv::Scalar>("Border Value",
          "",
          [this](const cv::Scalar & value) {
            if ( options_ && options_->border_value != value ) {
              options_->border_value = value;
              Q_EMIT parameterChanged();
            }
          });

  accumulateAndCompensateTurbulentFlow_ctl =
      add_checkbox("accumulate and compensate turbulent flow",
          "",
          [this](bool checked) {
            if ( options_ && options_->accumulate_and_compensate_turbulent_flow != checked ) {
              options_->accumulate_and_compensate_turbulent_flow = checked;
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

  add_expandable_groupbox("Jovian Derotation Options",
      jovianDerotationOptions_ctl = new QJovianDerotationOptions(this));
  connect(jovianDerotationOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("ECC Flow Registration Options",
      eccFlowOptions_ctl = new QEccFlowRegistrationOptions(this));
  connect(eccFlowOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  updateControls();
}

void QImageRegistrationOptions::set_registration_options(c_image_registration_options * options)
{
  options_ = options;
  updateControls();
}

c_image_registration_options* QImageRegistrationOptions::registration_options() const
{
  return options_;
}

void QImageRegistrationOptions::refreshInputSources(c_image_processing_pipeline * pipeline)
{
  return masterFrameOptions_ctl->refreshInputSources(pipeline);
}

void QImageRegistrationOptions::setEnableExternalFile(bool v)
{
  masterFrameOptions_ctl->setEnableExternalFile(v);
}

bool QImageRegistrationOptions::enableExternalFile() const
{
  return masterFrameOptions_ctl->enableExternalFile();
}

void QImageRegistrationOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);

    masterFrameOptions_ctl->set_master_frame_options(nullptr);
    featureRegistrationOptions_ctl->set_registration_options(nullptr);
    eccOptions_ctl->set_registration_options(nullptr);
    eccFlowOptions_ctl->set_registration_options(nullptr);
    jovianDerotationOptions_ctl->set_derotation_options(nullptr);

  }
  else {

    motion_type_ctl->setValue(options_->motion_type);
    registration_channel_ctl->setValue(options_->registration_channel);
    interpolation_method_ctl->setValue(options_->interpolation);
    border_mode_ctl->setValue(options_->border_mode);
    border_value_ctl->setValue(options_->border_value);
    accumulateAndCompensateTurbulentFlow_ctl->setChecked(options_->accumulate_and_compensate_turbulent_flow);

    masterFrameOptions_ctl->set_master_frame_options(&options_->master_frame_options);
    featureRegistrationOptions_ctl->set_registration_options(&options_->feature_registration);
    eccOptions_ctl->set_registration_options(&options_->ecc);
    eccFlowOptions_ctl->set_registration_options(&options_->eccflow);
    jovianDerotationOptions_ctl->set_derotation_options(&options_->jovian_derotation);

    setEnabled(true);
  }
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QFrameRegistrationOptions::QFrameRegistrationOptions(QWidget * parent) :
    Base("QFrameRegistrationOptions", parent)
{
  enable_frame_registration_ctl =
      add_checkbox("Enable image registration",
          "",
          [this](bool checked) {
            if ( options_ && options_->enable_frame_registration != checked ) {
              options_->enable_frame_registration = checked;
              update_controls_visibility();
              Q_EMIT parameterChanged();
            }
          });

  imageRegistrationOptions_ctl =
      add_widget<QImageRegistrationOptions>();

  connect(imageRegistrationOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);


  update_controls_visibility();
}

//void QFrameRegistrationOptions::set_current_pipeline(c_image_stacking_pipeline * current_pipeline)
//{
//  current_pipeline_ = current_pipeline;
//  options_ = current_pipeline_ ? &current_pipeline_->registration_options() : nullptr;
//  updateControls();
//}
//
//c_image_stacking_pipeline * QFrameRegistrationOptions::current_pipeline() const
//{
//  return current_pipeline_;
//}

void QFrameRegistrationOptions::set_registration_options(c_image_registration_options * options)
{
  options_ = options;
  updateControls();
}

c_image_registration_options* QFrameRegistrationOptions::registration_options() const
{
  return options_;
}

void QFrameRegistrationOptions::refreshInputSources(c_image_processing_pipeline * pipeline)
{
  imageRegistrationOptions_ctl->refreshInputSources(pipeline);
}

void QFrameRegistrationOptions::setEnableExternalFile(bool v)
{
  imageRegistrationOptions_ctl->setEnableExternalFile(v);
}

bool QFrameRegistrationOptions::enableExternalFile() const
{
  return imageRegistrationOptions_ctl->enableExternalFile();
}


void QFrameRegistrationOptions::onupdatecontrols()
{
  imageRegistrationOptions_ctl->set_registration_options(options_);

  if( !options_ ) {
    setEnabled(false);
  }
  else {

    enable_frame_registration_ctl->setChecked(options_-> enable_frame_registration);

    update_controls_visibility();

    setEnabled(true);
  }
}

void QFrameRegistrationOptions::update_controls_visibility()
{
  if( !options_ ) {
    imageRegistrationOptions_ctl->setVisible(false);
    //accumulateAndCompensateTurbulentFlow_ctl->setEnabled(false);
  }
  else {
    imageRegistrationOptions_ctl->setVisible(options_->enable_frame_registration);
    //accumulateAndCompensateTurbulentFlow_ctl->setEnabled(options_->enable_frame_registration);
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

