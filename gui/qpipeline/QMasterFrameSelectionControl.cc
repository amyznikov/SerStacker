/*
 * QMasterFrameSelectionOptions.cc
 *
 *  Created on: May 23, 2024
 *      Author: amyznikov
 */

#include "QMasterFrameSelectionControl.h"

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


