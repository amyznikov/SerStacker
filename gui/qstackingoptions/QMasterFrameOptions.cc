/*
 * QMasterFrameOptions.cc
 *
 *  Created on: Feb 9, 2021
 *      Author: amyznikov
 */

#include "QMasterFrameOptions.h"
#include <gui/widgets/addctrl.h>
#include <core/debug.h>

#define ICON_close          "close"
#define ICON_check_all      "check_all"

//static const char borderless_style[] = ""
//    "QToolButton { border: none; } "
//    "QToolButton::menu-indicator { image: none; }"
//    "";
//
//static QIcon getIcon(const QString & name)
//{
//  return QIcon(QString(":/qstackingoptions/icons/%1").arg(name));
//}


QMasterFrameOptions::QMasterFrameOptions(QWidget * parent)
  : Base("QStackingMasterFrameSettings", parent)
{

  Q_INIT_RESOURCE(qstackingoptions_resources);

  masterSource_ctl = new QComboBox();
  masterSource_ctl->setEditable(false);
  masterSource_ctl->setDuplicatesEnabled(true);
  masterSource_ctl->setSizeAdjustPolicy(QComboBox::AdjustToContents);
  connect(masterSource_ctl, SIGNAL(currentIndexChanged(int)),
      this, SLOT(onMasterSourceComboCurrentIndexChanged(int)));


  masterFrameIndex_ctl = new QSpinBox(this);
  masterFrameIndex_ctl->setKeyboardTracking(false);
  connect(masterFrameIndex_ctl, SIGNAL(valueChanged(int)),
      this, SLOT(onSpinBoxValueChanged(int)));


  apply_input_frame_processor_ctl = new QCheckBox(this);
  connect(apply_input_frame_processor_ctl, &QCheckBox::stateChanged,
      this, &ThisClass::onApplyInputFramePprocessorCheckboxStateChanged);


  generateMasterFrame_ctl = new QCheckBox(this);
  connect(generateMasterFrame_ctl, &QCheckBox::stateChanged,
      this, &ThisClass::onGenerateMasterFrameCheckboxStateChanged);

  maxFramesForMasterFrameGeneration_ctl = new QNumberEditBox(this);
  connect(maxFramesForMasterFrameGeneration_ctl, &QNumberEditBox::textChanged,
      this, &ThisClass::onMaxFramesForMasterFrameGenerationChanged);


  eccFlowScale_ctl = new QNumberEditBox(this);
  connect(eccFlowScale_ctl, &QNumberEditBox::textChanged,
      this, &ThisClass::onEccFlowScaleChanged);

  master_sharpen_factor_ctl = new QNumberEditBox(this);
  connect(master_sharpen_factor_ctl, &QNumberEditBox::textChanged,
      this, &ThisClass::onMasterSharpenFactorChanged);

  accumulated_sharpen_factor_ctl = new QNumberEditBox(this);
  connect(accumulated_sharpen_factor_ctl, &QNumberEditBox::textChanged,
      this, &ThisClass::onAccumulatedSharpenFactorChanged);



//  compensateMasterFlow_ctl = new QCheckBox(this);
//  connect(compensateMasterFlow_ctl, &QCheckBox::stateChanged,
//      this, &ThisClass::onAccumulateMasterFlowCheckboxStateChanged);


  saveMasterFrame_ctl = new QCheckBox(this);
  connect(saveMasterFrame_ctl, &QCheckBox::stateChanged,
      this, &ThisClass::onSaveMasterFrameCheckboxStateChanged);




//  applyToAll_ctl = new QToolButton(this);
//  applyToAll_ctl->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
//  applyToAll_ctl->setIconSize(QSize(16,16));
//  applyToAll_ctl->setStyleSheet(borderless_style);
//  applyToAll_ctl->setIcon(getIcon(ICON_check_all));
//  applyToAll_ctl->setText("Copy these parameters to all currently selected in treeview");
//  connect(applyToAll_ctl, &QToolButton::clicked,
//      [this]() {
//        if ( options_ ) {
//          emit applyMasterFrameSettingsToAllRequested(*options_);
//        }
//      });


  form->addRow("Master file:", masterSource_ctl);
  form->addRow("Master frame Index:", masterFrameIndex_ctl);
  form->addRow("Apply input frame processor:", apply_input_frame_processor_ctl);

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

void QMasterFrameOptions::set_master_frame_options(c_master_frame_options * options, const c_input_sequence::ptr & input_sequence)
{
  options_ = options;
  input_sequence_ = input_sequence;
  updateControls();
}

const c_master_frame_options * QMasterFrameOptions::master_frame_options() const
{
  return options_;
}

const c_input_sequence::ptr & QMasterFrameOptions::input_sequence() const
{
  return input_sequence_;
}


void QMasterFrameOptions::onupdatecontrols()
{
  if ( !options_ ) {
    setEnabled(false);
    masterSource_ctl->clear();
  }
  else {

    generateMasterFrame_ctl->setChecked(options_->generate_master_frame);
    maxFramesForMasterFrameGeneration_ctl->setValue(options_->max_input_frames_to_generate_master_frame);
    apply_input_frame_processor_ctl->setChecked(options_->apply_input_frame_processor);
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

    if ( input_sequence_ ) {
      for ( int i = 0, n = input_sequence_->sources().size(); i < n; ++i ) {
        QString source_file_name = input_sequence_->source(i)->filename().c_str();
        masterSource_ctl->addItem(QFileInfo(source_file_name).fileName(), source_file_name);
      }
    }

    if ( !options_->master_source_path.empty() ) {
      if ( !input_sequence_ || input_sequence_->indexof(options_->master_source_path) < 0 ) {
        QString source_file_name = options_->master_source_path.c_str();
        masterSource_ctl->addItem(QString("* %1").arg(QFileInfo(source_file_name).fileName()), source_file_name);
      }
    }

    masterSource_ctl->addItem("Browse...");

    // Select Current Index In Master Source Combo
    if ( !options_->master_source_path.empty() ) {
      masterSource_ctl->setCurrentIndex(masterSource_ctl->findData(options_->master_source_path.c_str()));
    }
    else {
      masterSource_ctl->setCurrentIndex(0);
      if ( masterSource_ctl->count() > 1 ) {
        options_->master_source_path = masterSource_ctl->itemData(0).toString().toStdString();
      }
    }

    updateMasterFrameIndex();
    previousComboboxItemIndex = masterSource_ctl->currentIndex();

    setEnabled(true);
  }
}

void QMasterFrameOptions::updateMasterFrameIndex()
{
  if ( options_->master_source_path.empty() ) {
    masterFrameIndex_ctl->setEnabled(false);
  }
  else {

    c_input_source::ptr source;

    if ( input_sequence_ && (source = input_sequence_->source(options_->master_source_path)) ) {
      if ( options_->master_frame_index < 0 || options_->master_frame_index >= source->size() ) {
        options_->master_frame_index = 0;
      }
      masterFrameIndex_ctl->setRange(0, source->size() - 1);
      masterFrameIndex_ctl->setValue(options_->master_frame_index);
      masterFrameIndex_ctl->setEnabled(true);
    }
    else if ( !(source = c_input_source::create(options_->master_source_path)) ) {
      CF_ERROR("c_input_source::create(pathfilename=%s) fails", options_->master_source_path.c_str());
      masterFrameIndex_ctl->setEnabled(false);
    }
    else {
      if ( options_->master_frame_index < 0 || options_->master_frame_index >= source->size() ) {
        options_->master_frame_index = 0;
      }
      masterFrameIndex_ctl->setRange(0, source->size() - 1);
      masterFrameIndex_ctl->setValue(options_->master_frame_index);
      masterFrameIndex_ctl->setEnabled(true);
    }
  }
}

void QMasterFrameOptions::onMasterSourceComboCurrentIndexChanged(int index)
{
  if ( options_ && !updatingControls() && index >= 0 ) {

    setUpdatingControls(true);

    const int cn = masterSource_ctl->count();
    if ( index == cn - 1 ) { // "Browse..."

      QString selectedFileName = browseForMasterFrame();

      if ( selectedFileName.isEmpty() ) {
        masterSource_ctl->setCurrentIndex(previousComboboxItemIndex);
      }
      else {
        options_->master_source_path = selectedFileName.toStdString();
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
        options_->master_source_path = selectedFileName.toStdString();
      }
    }

    updateMasterFrameIndex();

    previousComboboxItemIndex = masterSource_ctl->currentIndex();
    setUpdatingControls(false);
  }
}

void QMasterFrameOptions::onSpinBoxValueChanged(int value)
{
  if ( options_ && !updatingControls() ) {
    int currentComboboxIndex = masterSource_ctl->currentIndex();
    if ( currentComboboxIndex >= 0 && currentComboboxIndex < masterSource_ctl->count() - 1 ) {
      options_->master_frame_index = value;
      emit parameterChanged();
    }
  }
}

void QMasterFrameOptions::onGenerateMasterFrameCheckboxStateChanged(int state)
{
  if ( options_ && !updatingControls() ) {
    options_->generate_master_frame = state == Qt::Checked;

    maxFramesForMasterFrameGeneration_ctl->setEnabled(options_->generate_master_frame);
    eccFlowScale_ctl->setEnabled(options_->generate_master_frame);
    master_sharpen_factor_ctl->setEnabled(options_->generate_master_frame);

    //compensateMasterFlow_ctl->setEnabled(options_->generate_master_frame);

    emit parameterChanged();
  }
}

void QMasterFrameOptions::onMaxFramesForMasterFrameGenerationChanged()
{
  if ( options_ && !updatingControls() ) {
    int v = 0;
    if ( fromString(maxFramesForMasterFrameGeneration_ctl->text(), &v) &&
        v != options_->max_input_frames_to_generate_master_frame ) {
      options_->max_input_frames_to_generate_master_frame = v;
      emit parameterChanged();
    }
  }
}

void QMasterFrameOptions::onEccFlowScaleChanged()
{
  if ( options_ && !updatingControls() ) {
    int v = 0;
    if ( fromString(eccFlowScale_ctl->text(), &v) &&
        v != options_->max_input_frames_to_generate_master_frame ) {
      options_->eccflow_scale = v;
      emit parameterChanged();
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
      emit parameterChanged();
    }
  }
}

void QMasterFrameOptions::onAccumulatedSharpenFactorChanged()
{
  if ( options_ && !updatingControls() ) {
    double v = 0;
    if ( fromString(accumulated_sharpen_factor_ctl->text(), &v) &&
        v != options_->accumulated_sharpen_factor ) {
      options_->accumulated_sharpen_factor = v;
      emit parameterChanged();
    }
  }
}

//void QMasterFrameOptions::onAccumulateMasterFlowCheckboxStateChanged(int state)
//{
//  if ( options_ && !updatingControls() ) {
//    //options_->compensate_master_flow = state == Qt::Checked;
//    emit parameterChanged();
//  }
//}

void QMasterFrameOptions::onSaveMasterFrameCheckboxStateChanged(int state)
{
  if ( options_ && !updatingControls() ) {
    options_->save_master_frame = state == Qt::Checked;
    emit parameterChanged();
  }
}

void QMasterFrameOptions::onApplyInputFramePprocessorCheckboxStateChanged(int state)
{
  if ( options_ && !updatingControls() ) {
    options_->apply_input_frame_processor = state == Qt::Checked;
    emit parameterChanged();
  }
}


QString QMasterFrameOptions::browseForMasterFrame()
{
  static QString filter;

  if ( filter.isEmpty() ) {

    filter.append("Regular images (");
    for ( const std::string & s : c_regular_image_input_source::suffixes() ) {
      filter.append(QString("*%1 ").arg(s.c_str()));
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
  if ( options_ ) {
    if ( !options_->master_source_path.empty() ) {
      proposedMasterSourcePath  = options_->master_source_path.c_str();
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
  if ( options_ ) {
    return QFileDialog::getExistingDirectory(this, "Select directory",
        options_->master_source_path.c_str(), QFileDialog::DontUseNativeDialog | QFileDialog::ShowDirsOnly);
  }
  return QString();
}

