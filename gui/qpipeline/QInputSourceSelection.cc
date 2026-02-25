/*
 * QInputSourceSelection.cc
 *
 *  Created on: Feb 22, 2026
 *      Author: amyznikov
 */

#include "QInputSourceSelection.h"
#include <core/io/image/c_fits_input_source.h>
#include <core/io/image/c_raw_image_input_source.h>
#include <core/io/image/c_regular_image_input_source.h>
#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////////////////////////

QMasterFrameSelection::QMasterFrameSelection(QWidget * parent) :
  Base(parent)
{
  setContentsMargins(0,0,0,0);
  form->setContentsMargins(0,0,0,0);

  /** Must be called before creating source combo */
  QObject::connect(this, &ThisClass::populatecontrols,
      this, &ThisClass::populateInputSources);

  QObject::connect(this, &ThisClass::enablecontrols,
      this, &ThisClass::refreshControlStates);

  master_frame_selection_method_ctl =
      add_enum_combobox<master_frame_selection_method>("Master frame selection:",
          "Specify the method how to select master frame",
          [this](master_frame_selection_method v) {
            if ( _opts && _opts->master_selection_method != v ) {
              _opts->master_selection_method = v;
              refreshControlStates();
              Q_EMIT parameterChanged();
            }
          },
          [this](master_frame_selection_method * v) {
            if ( _opts ) {
              *v = _opts->master_selection_method;
              return true;
            }
            return false;
          });


  input_source_ctl =
      add_combobox<QComboBox>("Master file",
          "Specify master file source to extract master frame",
          false,
          [this](int index, QComboBox * combo) {
            if ( _opts && index >= 0 ) {
              QSignalBlocker block(combo);

              const QString selectedFullPathFileName = combo->itemData(index).toString();
              if ( !selectedFullPathFileName.isEmpty() ) {
                _opts->master_fiename = selectedFullPathFileName.toStdString();
                Q_EMIT parameterChanged();
              }
              else if ( browseForExternalSource() ) {
                Q_EMIT parameterChanged();
              }
              else {
                combo->setCurrentIndex(combo->findData(QString::fromStdString(_opts->master_fiename)));
              }
            }
          },
          [this](int * index, QComboBox * combo) {
            if ( _opts ) {
              * index = combo->findData(QString::fromStdString(_opts->master_fiename));
              return true;
            }
            return false;
          });


  master_frame_index_ctl =
      add_numeric_box<int>("Master frame index:",
          "Specify frame index in master source fite to use as master frame",
          [this](int v) {
            if ( _opts && _opts->master_frame_index != v )
            _opts->master_frame_index = v;
            Q_EMIT parameterChanged();

          },
          [this](int * v) {
            if ( _opts ) {
              *v = _opts->master_frame_index;
              return true;
            }
            return false;
          });


  updateControls();
}

void QMasterFrameSelection::refreshControlStates()
{
  QSignalBlocker block(master_frame_index_ctl);
  master_frame_index_ctl->setEnabled(_opts && _opts->master_selection_method == master_frame_specific_index);
}

void QMasterFrameSelection::populateInputSources()
{
  const c_input_sequence * input_sequence = _opts ? _opts->input_sequence : nullptr;

  if ( input_sequence ) {

    QSignalBlocker block(input_source_ctl);

    input_source_ctl->clear();

    const std::vector<c_input_source::sptr> & sources = input_sequence->sources();
    for ( const c_input_source::sptr & source : sources ) {
      const QString sourcePathFileName = QString::fromStdString(source->cfilename());
      input_source_ctl->addItem(QFileInfo(sourcePathFileName).fileName(), QVariant::fromValue(sourcePathFileName));
    }

    const QString currentSourcePathFileName = QString::fromStdString(_opts->master_fiename);
    if ( currentSourcePathFileName.isEmpty() ) {
      if( !sources.empty() ) {
        input_source_ctl->setCurrentIndex(0);
        _opts->master_fiename = input_source_ctl->currentData().toString().toStdString();
      }
    }
    else {
      const int currentSourceIndex = input_source_ctl->findData(currentSourcePathFileName);
      if ( currentSourceIndex >= 0 ) {
        input_source_ctl->setCurrentIndex(currentSourceIndex);
      }
      else {
        const QString currentSourceFileName = QFileInfo(currentSourcePathFileName).fileName();
        input_source_ctl->addItem(currentSourceFileName, QVariant::fromValue(currentSourcePathFileName));
        input_source_ctl->setCurrentIndex(input_source_ctl->count()-1);
      }
    }

    input_source_ctl->addItem("Browse...");
  }
}

bool QMasterFrameSelection::browseForExternalSource()
{
  static QString filter;

  if( filter.isEmpty() ) {

    static const auto append_filter =
        [](const std::vector<std::string> & suffixes) {
          for( int i = 0, n = suffixes.size(); i < n; ++i ) {
            const std::string &s = suffixes[i];
            if ( i != 0 ) { filter.append(QString(" ")); }
            filter.append(QString("*%1").arg(QString(s.c_str())));
          }
        };

#if have_regular_image_input_source
    filter.append("Regular images (");
    append_filter(c_regular_image_input_source::suffixes());
    filter.append(");;");
#endif

#if have_raw_image_input_source
      filter.append("RAW/DSLR images (");
      append_filter(c_raw_image_input_source::suffixes());
      filter.append(");;");
  #endif

#if have_fits_input_source
      filter.append("FITS files (");
      append_filter(c_fits_input_source::suffixes());
      filter.append(");;");
#endif

    filter.append("All Files (*)");
  }

  static const QString lastSourcesDirectoryKeyName = "lastSourcesDirectory";
  static const QString lastMasterFrameSelectionFilter = "lastMasterFrameSelectionFilter";

  QSettings settings;

  QString selectedFilter = settings.value(lastMasterFrameSelectionFilter).toString();
  QString proposedMasterSourcePath = settings.value(lastSourcesDirectoryKeyName).toString();

  QString selectedFile =
      QFileDialog::getOpenFileName(this,
          "Select master frame",
          proposedMasterSourcePath,
          filter,
          &selectedFilter);

  if( selectedFile.isEmpty() ) {
    return false;
  }

  settings.setValue(lastSourcesDirectoryKeyName, QFileInfo(selectedFile).absolutePath());
  settings.setValue(lastMasterFrameSelectionFilter, selectedFilter);

  c_input_source::sptr source =
      c_input_source::create(selectedFile.toStdString());

  if( !source ) {
    QMessageBox::critical(this, "ERROR",
        qsprintf("Can not open input source '%s'\n"
            "Check debug log for details",
            selectedFile.toUtf8().constData()));
    return false;
  }

  const int sourceIndex = input_source_ctl->findData(selectedFile);
  if ( sourceIndex >= 0 ) {
    input_source_ctl->setCurrentIndex(sourceIndex);
  }
  else {
    const QString selectedSourceFileName = QFileInfo(selectedFile).fileName();
    const int index = std::max(0, input_source_ctl->count() - 1);
    input_source_ctl->insertItem(index,  selectedSourceFileName, QVariant::fromValue(selectedFile));
    input_source_ctl->setCurrentIndex(index);
  }

  _opts->master_fiename = selectedFile.toStdString();

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QStereoInputSourceSelection::QStereoInputSourceSelection(QWidget * parent) :
  Base(parent)
{
  setContentsMargins(0,0,0,0);
  form->setContentsMargins(0,0,0,0);

  /** Must be called before creating source combos */
  QObject::connect(this, &ThisClass::populatecontrols,
      this, &ThisClass::populateInputSources);

  QObject::connect(this, &ThisClass::enablecontrols,
      this, &ThisClass::refreshControlStates);

  layout_type_ctl =
      add_enum_combobox<stereo_input_frame_layout_type>("stereo frame layout",
          "Specify how stereo frames are laid out inside of single video frame",
          [this](stereo_input_frame_layout_type v) {
            if ( _opts && _opts->layout_type != v ) {
              _opts->layout_type = v;
              refreshControlStates();
              Q_EMIT parameterChanged();
            }
          },
          [this](stereo_input_frame_layout_type * v) {
            if ( _opts ) {
              *v = _opts->layout_type;
              return true;
            }
            return false;
          });

  left_stereo_source_ctl =
      add_combobox<QComboBox>("left stereo source",
          "",
          false,
          [this](int index, QComboBox * combo) {
            if ( _opts && index >= 0 ) {
              _opts->left_stereo_source = combo->itemData(index).toString().toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](int * index, QComboBox * combo) {
            if ( _opts ) {
              * index = combo->findData(QString::fromStdString(_opts->left_stereo_source));
              return true;
            }
            return false;
          });

  right_stereo_source_ctl =
      add_combobox<QComboBox>("right stereo source",
          "",
          false,
          [this](int index, QComboBox * combo) {
            if ( _opts && index >= 0 ) {
              _opts->right_stereo_source = combo->itemData(index).toString().toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](int * index, QComboBox * combo) {
            if ( _opts ) {
              * index = combo->findData(QString::fromStdString(_opts->right_stereo_source));
              return true;
            }
            return false;
          });

  swap_cameras_ctl =
      add_checkbox("swap cameras", "",
          [this](bool v) {
            if ( _opts && _opts->swap_cameras != v ) {
              _opts->swap_cameras = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * v) {
            if ( _opts ) {
              *v = _opts->swap_cameras;
              return true;
            }
            return false;
          });


  updateControls();
}

void QStereoInputSourceSelection::refreshControlStates()
{
  if ( _opts ) {
    if ( _opts->layout_type == stereo_frame_layout_separate_sources ) {
      right_stereo_source_ctl->setEnabled(true);
    }
    else {
      right_stereo_source_ctl->setEnabled(false);
    }
  }
}

void QStereoInputSourceSelection::populateInputSources()
{
  const c_input_sequence * input_sequence = _opts ? _opts->input_sequence : nullptr;

  if ( input_sequence ) {
    QSignalBlocker block_left(left_stereo_source_ctl);
    QSignalBlocker block_right(right_stereo_source_ctl);

    left_stereo_source_ctl->clear();
    right_stereo_source_ctl->clear();

    const std::vector<c_input_source::sptr> & sources = input_sequence->sources();
    for ( const c_input_source::sptr & source : sources ) {
      const QString sourcePathFileName = QString::fromStdString(source->cfilename());
      left_stereo_source_ctl->addItem(QFileInfo(sourcePathFileName).fileName(), QVariant::fromValue(sourcePathFileName));
      right_stereo_source_ctl->addItem(QFileInfo(sourcePathFileName).fileName(), QVariant::fromValue(sourcePathFileName));
    }

    const QString currentLeftSourcePathFileName = QString::fromStdString(_opts->left_stereo_source);
    if ( !currentLeftSourcePathFileName.isEmpty() ) {

      const int currentLeftSourceIndex = left_stereo_source_ctl->findData(currentLeftSourcePathFileName);
      if ( currentLeftSourceIndex >= 0 ) {
        left_stereo_source_ctl->setCurrentIndex(currentLeftSourceIndex);
      }
      else {
        const QString currentLeftSourceFileName = QFileInfo(currentLeftSourcePathFileName).fileName();
        left_stereo_source_ctl->addItem(currentLeftSourceFileName, QVariant::fromValue(currentLeftSourcePathFileName));
      }
    }
    else if( !sources.empty() ) {
      _opts->left_stereo_source = sources.front()->filename();
    }

    const QString currentRightSourcePathFileName = QString::fromStdString(_opts->right_stereo_source);
    if ( !currentRightSourcePathFileName.isEmpty() ) {

      const int currentRightSourceIndex = right_stereo_source_ctl->findData(currentRightSourcePathFileName);
      if ( currentRightSourceIndex >= 0 ) {
        right_stereo_source_ctl->setCurrentIndex(currentRightSourceIndex);
      }
      else {
        const QString currentRightSourceFileName = QFileInfo(currentRightSourcePathFileName).fileName();
        right_stereo_source_ctl->addItem(currentRightSourceFileName, QVariant::fromValue(currentRightSourcePathFileName));
      }
    }
    else if ( !sources.empty() ) {
      if ( sources.size() > 1 && _opts->layout_type == stereo_frame_layout_separate_sources ) {
        _opts->right_stereo_source = sources[1]->filename();
      }
      else {
        _opts->right_stereo_source = sources[0]->filename();
      }
    }
  }
}
