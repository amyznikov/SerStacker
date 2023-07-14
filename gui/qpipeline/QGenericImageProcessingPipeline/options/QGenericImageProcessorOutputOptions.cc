/*
 * QGenericImageProcessorOutputOptions.cc
 *
 *  Created on: Apr 4, 2023
 *      Author: amyznikov
 */

#include "QGenericImageProcessorOutputOptions.h"

QGenericImageProcessorOutputOptions::QGenericImageProcessorOutputOptions(QWidget * parent) :
    Base(parent)
{
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

void QGenericImageProcessorOutputOptions::onupdatecontrols()
{
  Base::onupdatecontrols();
  if ( options_ ) {
    // processed_frames_filename_ctl->setEnabled(options_->save_processed_frames);
  }
}

