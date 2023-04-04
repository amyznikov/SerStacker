/*
 * QGenericImageProcessorOutputOptions.cc
 *
 *  Created on: Apr 4, 2023
 *      Author: amyznikov
 */

#include "QGenericImageProcessorOutputOptions.h"

QGenericImageProcessorOutputOptions::QGenericImageProcessorOutputOptions(QWidget * parent) :
    Base("", parent)
{
  output_directory_ctl =
      add_browse_for_path("",
          "Output Directory",
          QFileDialog::AcceptOpen,
          QFileDialog::Directory,
          [this](const QString & path) {
            if ( options_ && options_->output_directory != path.toStdString() ) {
              options_->output_directory = path.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * path) {
            if ( options_ ) {
              *path = options_->output_directory.c_str();
              return true;
            }
            return false;
          });

  save_processed_frames_ctl =
      add_checkbox("Save processed frames:",
          "",
          [this](bool checked) {
            if ( options_ && options_->save_processed_frames != checked ) {
              options_->save_processed_frames = checked;
              processed_frames_filename_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              *checked = options_->save_processed_frames;
              return true;
            }
            return false;
          });

  processed_frames_filename_ctl =
      add_textbox("Output filename:",
          "",
          [this](const QString & value) {
            if ( options_ && options_->processed_frames_filename != value.toStdString() ) {
              options_->processed_frames_filename = value.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * value) {
            if ( options_ ) {
              *value = options_->processed_frames_filename.c_str();
              return true;
            }
            return false;
          });

  processed_frames_filename_ctl->setPlaceholderText("auto");

  updateControls();
}

void QGenericImageProcessorOutputOptions::set_options(c_generic_image_processor_output_options * options)
{
  options_ = options;
  updateControls();
}

c_generic_image_processor_output_options * QGenericImageProcessorOutputOptions::options() const
{
  return options_;
}

void QGenericImageProcessorOutputOptions::onupdatecontrols()
{
  if ( !options_ ) {
    setEnabled(false);
  }
  else {
    Base::onupdatecontrols();
    processed_frames_filename_ctl->setEnabled(options_->save_processed_frames);
    setEnabled(true);
  }
}

