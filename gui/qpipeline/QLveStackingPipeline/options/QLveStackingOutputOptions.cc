/*
 * QLveStackingOutputOptions.cc
 *
 *  Created on: Jul 16, 2023
 *      Author: amyznikov
 */

#include "QLveStackingOutputOptions.h"

QLveStackingOutputOptions::QLveStackingOutputOptions(QWidget * parent) :
    Base(parent)
{
  save_accumuated_file_ctl =
      add_checkbox("Save processed frames:",
          "",
          [this](bool checked) {
            if ( options_ && options_->save_accumuated_file != checked ) {
              options_->save_accumuated_file = checked;
              output_accumuated_file_name_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              *checked = options_->save_accumuated_file;
              return true;
            }
            return false;
          });

  output_accumuated_file_name_ctl =
      add_textbox("Output filename:",
          "",
          [this](const QString & value) {
            if ( options_ && options_->output_accumuated_file_name != value.toStdString() ) {
              options_->output_accumuated_file_name = value.toStdString();
              output_accumuated_file_name_ctl->setEnabled(options_->save_accumuated_file);
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * value) {
            if ( options_ ) {
              *value = options_->output_accumuated_file_name.c_str();
              return true;
            }
            return false;
          });

  output_accumuated_file_name_ctl->setPlaceholderText("auto");

  updateControls();
}

void QLveStackingOutputOptions::onupdatecontrols()
{
  Base::onupdatecontrols();
  if ( options_ ) {
  }
}
