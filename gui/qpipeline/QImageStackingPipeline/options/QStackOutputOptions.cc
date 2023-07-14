/*
 * QStackOutputOptions.cc
 *
 *  Created on: Jun 23, 2021
 *      Author: amyznikov
 */

#include "QStackOutputOptions.h"
#include <gui/qimproc/QImageProcessorsCollection.h>

QStackOutputOptions::QStackOutputOptions(QWidget * parent) :
    Base(parent)
{
  if( QImageProcessorsCollection::empty() ) {
    QImageProcessorsCollection::load();
  }

  ///

  save_preprocessed_frames_ctl =
      add_named_checkbox("Save preprocessed frames",
          "",
          [this](bool checked) {
            if ( options_ && options_->save_preprocessed_frames != checked ) {
              options_->save_preprocessed_frames = checked;
              output_preprocessed_frames_path_ctl->setEnabled(options_->save_preprocessed_frames);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              * checked = options_->save_preprocessed_frames;
              return true;
            }
            return false;
          });


  output_preprocessed_frames_path_ctl =
      add_textbox("Output filename:",
          "",
          [this](const QString & value) {
            if ( options_ && options_->output_preprocessed_frames_filename != value.toStdString() ) {
              options_->output_preprocessed_frames_filename = value.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * value) {
            if ( options_ ) {
              *value = options_->output_preprocessed_frames_filename.c_str();
              return true;
            }
            return false;
          });

  output_preprocessed_frames_path_ctl->setPlaceholderText("auto");

  ///

  save_aligned_frames_ctl =
      add_named_checkbox("Save aligned frames",
          "",
          [this](bool checked) {
            if ( options_ && options_->save_aligned_frames != checked ) {
              options_->save_aligned_frames = checked;
              output_aligned_frames_path_ctl->setEnabled(options_->save_aligned_frames);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              * checked = options_->save_aligned_frames;
              return true;
            }
            return false;
          });

  output_aligned_frames_path_ctl =
      add_textbox("Output filename:",
          "",
          [this](const QString & value) {
            if ( options_ && options_->output_aligned_frames_filename != value.toStdString() ) {
              options_->output_aligned_frames_filename = value.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * value) {
            if ( options_ ) {
              *value = options_->output_aligned_frames_filename.c_str();
              return true;
            }
            return false;
          });

  output_aligned_frames_path_ctl->setPlaceholderText("auto");

  ///

  save_ecc_frames_ctl =
      add_named_checkbox("Save ecc frames",
          "",
          [this](bool checked) {
            if ( options_ && options_->save_ecc_frames != checked ) {
              options_->save_ecc_frames = checked;
              output_ecc_frames_path_ctl->setEnabled(options_->save_ecc_frames);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              * checked = options_->save_ecc_frames;
              return true;
            }
            return false;
          });


  output_ecc_frames_path_ctl =
      add_textbox("Output filename:",
          "",
          [this](const QString & value) {
            if ( options_ && options_->output_ecc_frames_filename != value.toStdString() ) {
              options_->output_ecc_frames_filename = value.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * value) {
            if ( options_ ) {
              *value = options_->output_ecc_frames_filename.c_str();
              return true;
            }
            return false;
          });

  output_ecc_frames_path_ctl->setPlaceholderText("auto");

  ///

  save_postprocessed_frames_ctl =
      add_named_checkbox("Save processed aligned frames",
          "",
          [this](
              bool checked) {
                if ( options_ && options_->save_processed_aligned_frames != checked ) {
                  options_->save_processed_aligned_frames = checked;
                  output_processed_aligned_frames_path_ctl->setEnabled(options_->save_processed_aligned_frames);
                  Q_EMIT parameterChanged();
                }
              },
          [this](bool * checked) {
            if ( options_ ) {
              * checked = options_->save_processed_aligned_frames;
              return true;
            }
            return false;
          });


  output_processed_aligned_frames_path_ctl =
      add_textbox("Output filename:",
          "",
          [this](const QString & value) {
            if ( options_ && options_->output_postprocessed_frames_filename != value.toStdString() ) {
              options_->output_postprocessed_frames_filename = value.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * value) {
            if ( options_ ) {
              *value = options_->output_postprocessed_frames_filename.c_str();
              return true;
            }
            return false;
          });

  output_processed_aligned_frames_path_ctl->setPlaceholderText("auto");

  ///

  save_accumulated_frames_ctl =
      add_named_checkbox("Save incremental accumulated frames",
          "",
          [this](
              bool checked) {
                if ( options_ && options_->save_incremental_frames != checked ) {
                  options_->save_incremental_frames = checked;
                  output_accumulated_frames_path_ctl->setEnabled(options_->save_incremental_frames);
                  Q_EMIT parameterChanged();
                }
              },
          [this](bool * checked) {
            if ( options_ ) {
              * checked = options_->save_incremental_frames;
              return true;
            }
            return false;
          });


  output_accumulated_frames_path_ctl =
      add_textbox("Output filename:",
          "",
          [this](const QString & value) {
            if ( options_ && options_->output_incremental_frames_filename != value.toStdString() ) {
              options_->output_incremental_frames_filename = value.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * value) {
            if ( options_ ) {
              *value = options_->output_incremental_frames_filename.c_str();
              return true;
            }
            return false;
          });

  output_accumulated_frames_path_ctl->setPlaceholderText("auto");

  ///

  save_accumulation_masks_ctl =
      add_named_checkbox("Save accumulation masks",
          "",
          [this](
              bool checked) {
                if ( options_ && options_->save_accumulation_masks != checked ) {
                  options_->save_accumulation_masks = checked;
                  output_accumulation_masks_path_ctl->setEnabled(options_->save_accumulation_masks);
                  Q_EMIT parameterChanged();
                }
              },
          [this](bool * checked) {
            if ( options_ ) {
              * checked = options_->save_accumulation_masks;
              return true;
            }
            return false;
          });

  output_accumulation_masks_path_ctl =
      add_textbox("Output filename:",
          "",
          [this](const QString & value) {
            if ( options_ && options_->output_accumulation_masks_filename != value.toStdString() ) {
              options_->output_accumulation_masks_filename = value.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * value) {
            if ( options_ ) {
              *value = options_->output_accumulation_masks_filename.c_str();
              return true;
            }
            return false;
          });

  output_accumulation_masks_path_ctl->setPlaceholderText("auto");

  ///

  write_image_mask_as_alpha_channel_ctl =
      add_checkbox("Write image mask as alpha channel",
          "",
          [this](
              bool checked) {
                if ( options_ && options_->write_image_mask_as_alpha_channel != checked ) {
                  options_->write_image_mask_as_alpha_channel = checked;
                  Q_EMIT parameterChanged();
                }
              },
          [this](bool * checked) {
            if ( options_ ) {
              * checked = options_->write_image_mask_as_alpha_channel;
              return true;
            }
            return false;
          });

  ///

  dump_reference_data_for_debug_ctl =
      add_checkbox("Dump reference data for debug",
          "",
          [this](bool checked) {
            if ( options_ && options_-> dump_reference_data_for_debug != checked ) {
              options_-> dump_reference_data_for_debug = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              * checked = options_->dump_reference_data_for_debug;
              return true;
            }
            return false;
          });

  ///

  debug_frame_registration_ctl =
      add_checkbox("Debug frame registration",
          "",
          [this](bool checked) {
            if ( options_ && options_-> debug_frame_registration != checked ) {
              options_->debug_frame_registration = checked;
              debug_frame_registration_frame_indexes_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              * checked = options_->debug_frame_registration;
              return true;
            }
            return false;
          });

  debug_frame_registration_frame_indexes_ctl =
      add_textbox("Frame indexes to debug registration:",
          "",
          [this](const QString & s) {
            if ( options_&& fromString(s, &options_->debug_frame_registration_frame_indexes)) {
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * v) {
            if ( options_ ) {
              * v = toQString(options_->debug_frame_registration_frame_indexes);
              return true;
            }
            return false;
          });

  ///

  setEnabled(false);
}

void QStackOutputOptions::onupdatecontrols()
{
  Base::onupdatecontrols();

  if( options_) {

//    output_directory_ctl->setCurrentPath(options_->output_directory.c_str(), false);
//
//    save_preprocessed_frames_ctl->setChecked(options_->save_preprocessed_frames);
//    output_preprocessed_frames_path_ctl->setCurrentPath(options_->output_preprocessed_frames_filename.c_str());
//    output_preprocessed_frames_path_ctl->setEnabled(options_->save_preprocessed_frames);
//
//    save_aligned_frames_ctl->setChecked(options_->save_aligned_frames);
//    output_aligned_frames_path_ctl->setCurrentPath(options_->output_aligned_frames_filename.c_str());
//    output_aligned_frames_path_ctl->setEnabled(options_->save_aligned_frames);
//
//    save_ecc_frames_ctl->setChecked(options_->save_ecc_frames);
//    output_ecc_frames_path_ctl->setCurrentPath(options_->output_ecc_frames_filename.c_str());
//    output_ecc_frames_path_ctl->setEnabled(options_->save_ecc_frames);
//
//    save_postprocessed_frames_ctl->setChecked(options_->save_processed_aligned_frames);
//    output_processed_aligned_frames_path_ctl->setCurrentPath(
//        options_->output_postprocessed_frames_filename.c_str());
//    output_processed_aligned_frames_path_ctl->setEnabled(options_->save_processed_aligned_frames);
//
//    save_accumulated_frames_ctl->setChecked(options_->save_incremental_frames);
//    output_accumulated_frames_path_ctl->setCurrentPath(options_->output_incremental_frames_filename.c_str());
//    output_accumulated_frames_path_ctl->setEnabled(options_->save_incremental_frames);
//
//    save_accumulation_masks_ctl->setChecked(options_->save_accumulation_masks);
//    output_accumulation_masks_path_ctl->setCurrentPath(options_->output_accumulation_masks_filename.c_str());
//    output_accumulation_masks_path_ctl->setEnabled(options_->save_accumulation_masks);
//
//    write_image_mask_as_alpha_channel_ctl->setChecked(options_->write_image_mask_as_alpha_channel);
//    dump_reference_data_for_debug_ctl->setChecked(options_->dump_reference_data_for_debug);
//
//    debug_frame_registration_ctl->setChecked(options_->debug_frame_registration);
//    debug_frame_registration_frame_indexes_ctl->setValue(options_->debug_frame_registration_frame_indexes);
//    debug_frame_registration_frame_indexes_ctl->setEnabled(options_->debug_frame_registration);
  }

}
