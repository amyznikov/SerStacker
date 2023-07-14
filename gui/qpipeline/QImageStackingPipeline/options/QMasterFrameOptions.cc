/*
 * QMasterFrameOptions.cc
 *
 *  Created on: Feb 9, 2021
 *      Author: amyznikov
 */

#include "QMasterFrameOptions.h"

QMasterFrameOptions::QMasterFrameOptions(QWidget * parent) :
  Base("QStackingMasterFrameSettings", parent)
{
  masterSource_ctl = new QComboBox();
  masterSource_ctl->setEditable(false);
  masterSource_ctl->setDuplicatesEnabled(true);
  masterSource_ctl->setSizeAdjustPolicy(QComboBox::AdjustToContents);
  masterSource_ctl->setFocusPolicy(Qt::StrongFocus);

  connect(masterSource_ctl, SIGNAL(currentIndexChanged(int)),
      this, SLOT(onMasterSourceComboCurrentIndexChanged(int)));



  masterFrameSelectionMethod_ctl =
      add_enum_combobox<master_frame_selection_method>("Frame selection:",
          "",
          [this](master_frame_selection_method v) {
            onMasterFrameSeletionMethodChaned(v);
          });


  masterFrameIndex_ctl = new QSpinBox(this);
  masterFrameIndex_ctl->setKeyboardTracking(false);
  masterFrameIndex_ctl->setFocusPolicy(Qt::StrongFocus);
  connect(masterFrameIndex_ctl, SIGNAL(valueChanged(int)),
      this, SLOT(onSpinBoxValueChanged(int)));


  apply_input_frame_processors_ctl = new QCheckBox(this);
  connect(apply_input_frame_processors_ctl, &QCheckBox::stateChanged,
      this, &ThisClass::onApplyInputFramePprocessorCheckboxStateChanged);


  generateMasterFrame_ctl = new QCheckBox(this);
  connect(generateMasterFrame_ctl, &QCheckBox::stateChanged,
      this, &ThisClass::onGenerateMasterFrameCheckboxStateChanged);

  maxFramesForMasterFrameGeneration_ctl = new QNumericBox(this);
  connect(maxFramesForMasterFrameGeneration_ctl, &QNumericBox::textChanged,
      this, &ThisClass::onMaxFramesForMasterFrameGenerationChanged);


  eccFlowScale_ctl = new QNumericBox(this);
  connect(eccFlowScale_ctl, &QNumericBox::textChanged,
      this, &ThisClass::onEccFlowScaleChanged);

  master_sharpen_factor_ctl = new QNumericBox(this);
  connect(master_sharpen_factor_ctl, &QNumericBox::textChanged,
      this, &ThisClass::onMasterSharpenFactorChanged);

  accumulated_sharpen_factor_ctl = new QNumericBox(this);
  connect(accumulated_sharpen_factor_ctl, &QNumericBox::textChanged,
      this, &ThisClass::onAccumulatedSharpenFactorChanged);



//  compensateMasterFlow_ctl = new QCheckBox(this);
//  connect(compensateMasterFlow_ctl, &QCheckBox::stateChanged,
//      this, &ThisClass::onAccumulateMasterFlowCheckboxStateChanged);


  saveMasterFrame_ctl = new QCheckBox(this);
  connect(saveMasterFrame_ctl, &QCheckBox::stateChanged,
      this, &ThisClass::onSaveMasterFrameCheckboxStateChanged);


  form->addRow("Master file:", masterSource_ctl);
  form->addRow("Master frame Index:", masterFrameIndex_ctl);
  form->addRow("Apply input processors:", apply_input_frame_processors_ctl);

  form->addRow("Generate master frame:", generateMasterFrame_ctl);
  form->addRow("Max frames:", maxFramesForMasterFrameGeneration_ctl);
  form->addRow("eccflow support scale:", eccFlowScale_ctl);
  form->addRow("Sharpen factor:", master_sharpen_factor_ctl);
  form->addRow("Acc. sharpen factor:", accumulated_sharpen_factor_ctl);

  //form->addRow("Compensate master flow:", compensateMasterFlow_ctl);
  form->addRow("Save Master Frame", saveMasterFrame_ctl);

  //form->addRow(applyToAll_ctl);


  setEnabled(false);
}

void QMasterFrameOptions::set_current_pipeline(c_image_stacking_pipeline * current_pipeline)
{
  if ( !(current_pipeline_ = current_pipeline) ) {
    options_ = nullptr;
  }
  else {
    options_ = &current_pipeline_->master_frame_options();
  }

  updateControls();
}

c_image_stacking_pipeline * QMasterFrameOptions::current_pipeline() const
{
  return current_pipeline_;
}

void QMasterFrameOptions::onupdatecontrols()
{
  if ( !current_pipeline_ ) {
    setEnabled(false);
    masterSource_ctl->clear();
  }
  else {

    generateMasterFrame_ctl->setChecked(options_->generate_master_frame);
    maxFramesForMasterFrameGeneration_ctl->setValue(options_->max_frames_to_generate_master_frame);
    apply_input_frame_processors_ctl->setChecked(options_->apply_input_frame_processors);
    eccFlowScale_ctl->setValue(options_->eccflow_scale);
    master_sharpen_factor_ctl->setValue(options_->master_sharpen_factor);
    accumulated_sharpen_factor_ctl->setValue(options_->accumulated_sharpen_factor);

    //compensateMasterFlow_ctl->setChecked(options_->compensate_master_flow);
    saveMasterFrame_ctl->setChecked(options_->save_master_frame);


    maxFramesForMasterFrameGeneration_ctl->setEnabled(options_->generate_master_frame);
    eccFlowScale_ctl->setEnabled(options_->generate_master_frame);
    master_sharpen_factor_ctl->setEnabled(options_->generate_master_frame);
    //compensateMasterFlow_ctl->setEnabled(options_->generate_master_frame);

    // Populate Master Source Combo
    masterSource_ctl->clear();

    if( current_pipeline_ ) {

      const c_input_sequence::sptr &input_sequence =
          current_pipeline_->input_sequence();

      if( input_sequence ) {

        for( int i = 0, n = input_sequence->sources().size(); i < n; ++i ) {
          QString source_file_name = input_sequence->source(i)->filename().c_str();
          masterSource_ctl->addItem(QFileInfo(source_file_name).fileName(), source_file_name);
        }
      }

      const std::string &master_source =
          current_pipeline_->master_source();

      if( !master_source.empty() ) {
        if( !input_sequence || input_sequence->indexof(master_source) < 0 ) {
          QString source_file_name = master_source.c_str();
          masterSource_ctl->addItem(QString("* %1").arg(QFileInfo(source_file_name).fileName()), source_file_name);
        }
      }

      masterSource_ctl->addItem("Browse...");

      // Select Current Index In Master Source Combo
      if ( !master_source.empty() ) {
        masterSource_ctl->setCurrentIndex(masterSource_ctl->findData(master_source.c_str()));
      }
      else {
        masterSource_ctl->setCurrentIndex(0);
        if ( masterSource_ctl->count() > 1 ) {
          current_pipeline_->set_master_source(masterSource_ctl->itemData(0).toString().toStdString());
        }
      }
    }

    updateMasterFrameIndex();
    previousComboboxItemIndex = masterSource_ctl->currentIndex();

    masterFrameSelectionMethod_ctl->setValue(options_->master_selection_method);
    masterFrameIndex_ctl->setEnabled(options_->master_selection_method == master_frame_specific_index);

    setEnabled(true);
  }
}

void QMasterFrameOptions::updateMasterFrameIndex()
{
  if ( !current_pipeline_ ||  current_pipeline_->master_source().empty() ) {
    masterFrameIndex_ctl->setEnabled(false);
  }
  else {

    const c_input_sequence::sptr & input_sequence =
        current_pipeline_->input_sequence();

    c_input_source::sptr source;

    if ( input_sequence && (source = input_sequence->source(current_pipeline_->master_source())) ) {
      if ( current_pipeline_->master_frame_index() < 0 || current_pipeline_->master_frame_index() >= source->size() ) {
        current_pipeline_->set_master_frame_index(0);
      }
      masterFrameIndex_ctl->setRange(0, source->size() - 1);
      masterFrameIndex_ctl->setValue(current_pipeline_->master_frame_index());
      masterFrameIndex_ctl->setEnabled(true);
    }
    else if ( !(source = c_input_source::create(current_pipeline_->master_source())) ) {
      CF_ERROR("c_input_source::create(pathfilename=%s) fails", current_pipeline_->master_source().c_str());
      masterFrameIndex_ctl->setEnabled(false);
    }
    else {
      if ( current_pipeline_->master_frame_index() < 0 || current_pipeline_->master_frame_index() >= source->size() ) {
        current_pipeline_->set_master_frame_index(0);
      }
      masterFrameIndex_ctl->setRange(0, source->size() - 1);
      masterFrameIndex_ctl->setValue(current_pipeline_->master_frame_index());
      masterFrameIndex_ctl->setEnabled(true);
    }
  }
}

void QMasterFrameOptions::onMasterSourceComboCurrentIndexChanged(int index)
{
  if ( current_pipeline_ && !updatingControls() && index >= 0 ) {

    setUpdatingControls(true);

    const int cn = masterSource_ctl->count();
    if ( index == cn - 1 ) { // "Browse..."

      QString selectedFileName = browseForMasterFrame();

      if ( selectedFileName.isEmpty() ) {
        masterSource_ctl->setCurrentIndex(previousComboboxItemIndex);
      }
      else {
        current_pipeline_->set_master_source(selectedFileName.toStdString());
        masterSource_ctl->insertItem(masterSource_ctl->count()-1, QString("* %1").arg(QFileInfo(selectedFileName).fileName()), selectedFileName);
        masterSource_ctl->setCurrentIndex(masterSource_ctl->count()-2);
      }
    }
    else {
      QString selectedFileName = masterSource_ctl->itemData(index).toString();
      if ( selectedFileName.isEmpty() ) {
        masterSource_ctl->setCurrentIndex(previousComboboxItemIndex);
      }
      else {
        current_pipeline_->set_master_source(selectedFileName.toStdString());
      }
    }

    updateMasterFrameIndex();

    previousComboboxItemIndex = masterSource_ctl->currentIndex();
    setUpdatingControls(false);

    Q_EMIT parameterChanged();
  }
}

void QMasterFrameOptions::onMasterFrameSeletionMethodChaned(master_frame_selection_method v)
{
  if ( current_pipeline_ && !updatingControls() ) {

    options_->master_selection_method = v;

    switch (v) {
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

void QMasterFrameOptions::onSpinBoxValueChanged(int value)
{
  if ( current_pipeline_ && !updatingControls() ) {
    int currentComboboxIndex = masterSource_ctl->currentIndex();
    if ( currentComboboxIndex >= 0 && currentComboboxIndex < masterSource_ctl->count() - 1 ) {
      current_pipeline_->set_master_frame_index(value);
      Q_EMIT parameterChanged();
    }
  }
}

void QMasterFrameOptions::onGenerateMasterFrameCheckboxStateChanged(int state)
{
  if ( current_pipeline_ && !updatingControls() ) {
    options_->generate_master_frame = state == Qt::Checked;

    maxFramesForMasterFrameGeneration_ctl->setEnabled(options_->generate_master_frame);
    eccFlowScale_ctl->setEnabled(options_->generate_master_frame);
    master_sharpen_factor_ctl->setEnabled(options_->generate_master_frame);

    //compensateMasterFlow_ctl->setEnabled(options_->generate_master_frame);

    Q_EMIT parameterChanged();
  }
}

void QMasterFrameOptions::onMaxFramesForMasterFrameGenerationChanged()
{
  if ( current_pipeline_ && !updatingControls() ) {
    int v = 0;
    if ( fromString(maxFramesForMasterFrameGeneration_ctl->text(), &v) &&
        v != options_->max_frames_to_generate_master_frame ) {
      options_->max_frames_to_generate_master_frame = v;
      Q_EMIT parameterChanged();
    }
  }
}

void QMasterFrameOptions::onEccFlowScaleChanged()
{
  if ( current_pipeline_ && !updatingControls() ) {
    int v = 0;
    if ( fromString(eccFlowScale_ctl->text(), &v) &&
        v != options_->max_frames_to_generate_master_frame ) {
      options_->eccflow_scale = v;
      Q_EMIT parameterChanged();
    }
  }
}

void QMasterFrameOptions::onMasterSharpenFactorChanged()
{
  if ( options_ && !updatingControls() ) {
    double v = 0;
    if ( fromString(master_sharpen_factor_ctl->text(), &v) &&
        v != options_->master_sharpen_factor ) {
      options_->master_sharpen_factor = v;
      Q_EMIT parameterChanged();
    }
  }
}

void QMasterFrameOptions::onAccumulatedSharpenFactorChanged()
{
  if ( current_pipeline_ && !updatingControls() ) {
    double v = 0;
    if ( fromString(accumulated_sharpen_factor_ctl->text(), &v) &&
        v != options_->accumulated_sharpen_factor ) {
      options_->accumulated_sharpen_factor = v;
      Q_EMIT parameterChanged();
    }
  }
}

//void QMasterFrameOptions::onAccumulateMasterFlowCheckboxStateChanged(int state)
//{
//  if ( options_ && !updatingControls() ) {
//    //options_->compensate_master_flow = state == Qt::Checked;
//    Q_EMIT parameterChanged();
//  }
//}

void QMasterFrameOptions::onSaveMasterFrameCheckboxStateChanged(int state)
{
  if ( current_pipeline_ && !updatingControls() ) {
    options_->save_master_frame = state == Qt::Checked;
    Q_EMIT parameterChanged();
  }
}

void QMasterFrameOptions::onApplyInputFramePprocessorCheckboxStateChanged(int state)
{
  if ( current_pipeline_ && !updatingControls() ) {
    options_->apply_input_frame_processors = state == Qt::Checked;
    Q_EMIT parameterChanged();
  }
}


QString QMasterFrameOptions::browseForMasterFrame()
{
  static QString filter;

  if ( filter.isEmpty() ) {

    filter.append("Regular images (");
    for ( const std::string & s : c_regular_image_input_source::suffixes() ) {
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


  QString proposedMasterSourcePath;
  if ( current_pipeline_ ) {
    if ( !current_pipeline_->master_source().empty() ) {
      proposedMasterSourcePath  = current_pipeline_->master_source().c_str();
    }
    else {
      // FIXME: need access to stack output directory
    }
  }

  if ( proposedMasterSourcePath.isEmpty() ) {
    proposedMasterSourcePath = settings.value(lastSourcesDirectoryKeyName).toString();
  }

  QString selectedFile = QFileDialog::getOpenFileName(this,
      "Select master frame",
      proposedMasterSourcePath,
      filter,
      &selectedFilter);

  if ( !selectedFile.isEmpty() ) {

    settings.setValue(lastSourcesDirectoryKeyName,
        QFileInfo(selectedFile).absolutePath());

    settings.setValue(lastMasterFrameSelectionFilter,
        selectedFilter);
  }

  return selectedFile;
}

QString QMasterFrameOptions::browseForMasterFFTSPath()
{
  if ( current_pipeline_ ) {
    return QFileDialog::getExistingDirectory(this, "Select directory",
        current_pipeline_->master_source().c_str(),
        QFileDialog::DontUseNativeDialog | QFileDialog::ShowDirsOnly);
  }

  return QString();
}

