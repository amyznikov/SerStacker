/*
 * QStackOutputOptions.cc
 *
 *  Created on: Jun 23, 2021
 *      Author: amyznikov
 */

#include "QStackOutputOptions.h"
#include <gui/qimproc/QImageProcessorsCollection.h>
#include <core/debug.h>

QStackOutputOptions::QStackOutputOptions(QWidget * parent) :
    Base("QStackOutputOptions", parent)
{
  Q_INIT_RESOURCE(qstackingoptions_resources);

  if( QImageProcessorsCollection::empty() ) {
    QImageProcessorsCollection::load();
  }

  ///

  form->addRow(output_directory_ctl =
      new QBrowsePathCombo("Output directory:",
          QFileDialog::AcceptSave,
          QFileDialog::AnyFile,
          this));

  output_directory_ctl->setShowDirsOnly(true);

  connect(output_directory_ctl, &QBrowsePathCombo::pathChanged,
      [this]() {
        if ( current_pipeline_ && !updatingControls() ) {
          current_pipeline_->output_options().output_directoty =
              output_directory_ctl->currentPath().toStdString();
          Q_EMIT parameterChanged();
        }
      });

  ///

  save_preprocessed_frames_ctl =
      add_named_checkbox("Save preprocessed (input) frames",
          "",
          [this](
              bool checked) {
                if ( current_pipeline_ && current_pipeline_->output_options().save_preprocessed_frames != checked ) {
                  current_pipeline_->output_options().save_preprocessed_frames = checked;
                  output_preprocessed_frames_path_ctl->setEnabled(current_pipeline_->output_options().save_preprocessed_frames);
                  Q_EMIT parameterChanged();
                }
              });

  form->addRow(output_preprocessed_frames_path_ctl =
      new QBrowsePathCombo("Preprocessed frames file name:",
          QFileDialog::AcceptSave,
          QFileDialog::AnyFile,
          this));

  connect(output_preprocessed_frames_path_ctl, &QBrowsePathCombo::pathChanged,
      [this]() {
        if ( current_pipeline_ && !updatingControls() ) {
          current_pipeline_->output_options().output_preprocessed_frames_filename =
          output_preprocessed_frames_path_ctl->currentPath().toStdString();
          Q_EMIT parameterChanged();
        }
      });

  ///

  ///

  save_aligned_frames_ctl =
      add_named_checkbox("Save aligned frames",
          "",
          [this](bool checked) {
            if ( current_pipeline_ && current_pipeline_->output_options().save_aligned_frames != checked ) {
              current_pipeline_->output_options().save_aligned_frames = checked;
              output_aligned_frames_path_ctl->setEnabled(current_pipeline_->output_options().save_aligned_frames);
              Q_EMIT parameterChanged();
            }
          });

  form->addRow(output_aligned_frames_path_ctl =
      new QBrowsePathCombo("Aligned frames file name:",
          QFileDialog::AcceptSave,
          QFileDialog::AnyFile,
          this));

  connect(output_aligned_frames_path_ctl, &QBrowsePathCombo::pathChanged,
      [this]() {
        if ( current_pipeline_ && !updatingControls() ) {
          current_pipeline_->output_options().output_aligned_frames_filename =
          output_aligned_frames_path_ctl->currentPath().toStdString();
          Q_EMIT parameterChanged();
        }
      });

  ///

  ///

  save_ecc_frames_ctl =
      add_named_checkbox("Save ecc frames",
          "",
          [this](bool checked) {
            if ( current_pipeline_ && current_pipeline_->output_options().save_ecc_frames != checked ) {
              current_pipeline_->output_options().save_ecc_frames = checked;
              output_ecc_frames_path_ctl->setEnabled(current_pipeline_->output_options().save_ecc_frames);
              Q_EMIT parameterChanged();
            }
          });

  form->addRow(output_ecc_frames_path_ctl =
      new QBrowsePathCombo("ECC frames file name:",
          QFileDialog::AcceptSave,
          QFileDialog::AnyFile,
          this));

  connect(output_ecc_frames_path_ctl, &QBrowsePathCombo::pathChanged,
      [this]() {
        if ( current_pipeline_ && !updatingControls() ) {
          current_pipeline_->output_options().output_ecc_frames_filename =
          output_ecc_frames_path_ctl->currentPath().toStdString();
          Q_EMIT parameterChanged();
        }
      });

  ///

  ///

  save_postprocessed_frames_ctl =
      add_named_checkbox("Save processed aligned frames",
          "",
          [this](
              bool checked) {
                if ( current_pipeline_ && current_pipeline_->output_options().save_processed_aligned_frames != checked ) {
                  current_pipeline_->output_options().save_processed_aligned_frames = checked;
                  output_processed_aligned_frames_path_ctl->setEnabled(current_pipeline_->output_options().save_processed_aligned_frames);
                  Q_EMIT parameterChanged();
                }
              });

  form->addRow(output_processed_aligned_frames_path_ctl =
      new QBrowsePathCombo("Processed aligned frames file name:",
          QFileDialog::AcceptSave,
          QFileDialog::AnyFile,
          this));

  connect(output_processed_aligned_frames_path_ctl, &QBrowsePathCombo::pathChanged,
      [this]() {
        if ( current_pipeline_ && !updatingControls() ) {
          current_pipeline_->output_options().output_postprocessed_frames_filename =
          output_processed_aligned_frames_path_ctl->currentPath().toStdString();
          Q_EMIT parameterChanged();
        }
      });

  ///

  save_accumulated_frames_ctl =
      add_named_checkbox("Save (incremental) accumulated frames",
          "",
          [this](
              bool checked) {
                if ( current_pipeline_ && current_pipeline_->output_options().save_incremental_frames != checked ) {
                  current_pipeline_->output_options().save_incremental_frames = checked;
                  output_accumulated_frames_path_ctl->setEnabled(current_pipeline_->output_options().save_incremental_frames);
                  Q_EMIT parameterChanged();
                }
              });

  form->addRow(output_accumulated_frames_path_ctl =
      new QBrowsePathCombo("Accumulated frames file name:",
          QFileDialog::AcceptSave,
          QFileDialog::AnyFile,
          this));

  connect(output_accumulated_frames_path_ctl, &QBrowsePathCombo::pathChanged,
      [this]() {
        if ( current_pipeline_ && !updatingControls() ) {
          current_pipeline_->output_options().output_incremental_frames_filename =
          output_accumulated_frames_path_ctl->currentPath().toStdString();
          Q_EMIT parameterChanged();
        }
      });

  ///

  save_accumulation_masks_ctl =
      add_named_checkbox("Save accumulation masks",
          "",
          [this](
              bool checked) {
                if ( current_pipeline_ && current_pipeline_->output_options().save_accumulation_masks != checked ) {
                  current_pipeline_->output_options().save_accumulation_masks = checked;
                  output_accumulation_masks_path_ctl->setEnabled(current_pipeline_->output_options().save_accumulation_masks);
                  Q_EMIT parameterChanged();
                }
              });

  form->addRow(output_accumulation_masks_path_ctl =
      new QBrowsePathCombo("Accumulation masks file name:",
          QFileDialog::AcceptSave,
          QFileDialog::AnyFile,
          this));

  connect(output_accumulation_masks_path_ctl, &QBrowsePathCombo::pathChanged,
      [this]() {
        if ( current_pipeline_ && !updatingControls() ) {
          current_pipeline_->output_options().output_accumulation_masks_filename =
          output_accumulation_masks_path_ctl->currentPath().toStdString();
          Q_EMIT parameterChanged();
        }
      });

  ///

  write_image_mask_as_alpha_channel_ctl =
      add_checkbox("Write image mask as alpha channel",
          "",
          [this](
              bool checked) {
                if ( current_pipeline_ && current_pipeline_->output_options().write_image_mask_as_alpha_channel != checked ) {
                  current_pipeline_->output_options().write_image_mask_as_alpha_channel = checked;
                  Q_EMIT parameterChanged();
                }
              });

  ///

  dump_reference_data_for_debug_ctl =
      add_checkbox("Dump reference data for debug",
          "",
          [this](bool checked) {
            if ( current_pipeline_ && current_pipeline_->output_options(). dump_reference_data_for_debug != checked ) {
              current_pipeline_->output_options(). dump_reference_data_for_debug = checked;
              Q_EMIT parameterChanged();
            }
          });

  ///

  debug_frame_registration_ctl =
      add_checkbox("Debug frame registration",
          "",
          [this](bool checked) {
            if ( current_pipeline_ && current_pipeline_->output_options(). debug_frame_registration != checked ) {
              current_pipeline_->output_options().debug_frame_registration = checked;
              debug_frame_registration_frame_indexes_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          });

  debug_frame_registration_frame_indexes_ctl =
      add_textbox("Frame indexes to debug registration:",
          "",
          [this](const QString & s) {
            if ( current_pipeline_ ) {
              if ( fromString(s, &current_pipeline_->output_options().debug_frame_registration_frame_indexes)) {
                Q_EMIT parameterChanged();
              }
            }
          });

  ///

//
//  applyToAll_ctl = new QToolButton(this);
//  applyToAll_ctl->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
//  applyToAll_ctl->setIconSize(QSize(16,16));
//  //applyToAll_ctl->setStyleSheet(borderless_style);
//  applyToAll_ctl->setIcon(getIcon(ICON_check_all));
//  applyToAll_ctl->setText("Copy these parameters to all currently selected in treeview");
//  form->addRow(applyToAll_ctl);
//  connect(applyToAll_ctl, &QToolButton::clicked,
//      [this]() {
//        if ( options_ ) {
//          Q_EMIT applyOutputOptionsToAllRequested(options_->output_options());
//        }
//      });
//
//
//  ///

  setEnabled(false);
}

void QStackOutputOptions::set_current_pipeline(const c_image_stacking_pipeline::sptr & current_pipeline)
{
  this->current_pipeline_ = current_pipeline;
  updateControls();
}

const c_image_stacking_pipeline::sptr& QStackOutputOptions::current_pipeline() const
{
  return this->current_pipeline_;
}

void QStackOutputOptions::onupdatecontrols()
{
  if( !current_pipeline_ ) {
    setEnabled(false);
  }
  else {

    c_image_stacking_output_options &output_options =
        current_pipeline_->output_options();

    output_directory_ctl->setCurrentPath(output_options.output_directoty.c_str(), false);

    save_preprocessed_frames_ctl->setChecked(output_options.save_preprocessed_frames);
    output_preprocessed_frames_path_ctl->setCurrentPath(output_options.output_preprocessed_frames_filename.c_str());
    output_preprocessed_frames_path_ctl->setEnabled(output_options.save_preprocessed_frames);

    save_aligned_frames_ctl->setChecked(output_options.save_aligned_frames);
    output_aligned_frames_path_ctl->setCurrentPath(output_options.output_aligned_frames_filename.c_str());
    output_aligned_frames_path_ctl->setEnabled(output_options.save_aligned_frames);

    save_ecc_frames_ctl->setChecked(output_options.save_ecc_frames);
    output_ecc_frames_path_ctl->setCurrentPath(output_options.output_ecc_frames_filename.c_str());
    output_ecc_frames_path_ctl->setEnabled(output_options.save_ecc_frames);

    save_postprocessed_frames_ctl->setChecked(output_options.save_processed_aligned_frames);
    output_processed_aligned_frames_path_ctl->setCurrentPath(
        output_options.output_postprocessed_frames_filename.c_str());
    output_processed_aligned_frames_path_ctl->setEnabled(output_options.save_processed_aligned_frames);

    save_accumulated_frames_ctl->setChecked(output_options.save_incremental_frames);
    output_accumulated_frames_path_ctl->setCurrentPath(output_options.output_incremental_frames_filename.c_str());
    output_accumulated_frames_path_ctl->setEnabled(output_options.save_incremental_frames);

    save_accumulation_masks_ctl->setChecked(output_options.save_accumulation_masks);
    output_accumulation_masks_path_ctl->setCurrentPath(output_options.output_accumulation_masks_filename.c_str());
    output_accumulation_masks_path_ctl->setEnabled(output_options.save_accumulation_masks);

    write_image_mask_as_alpha_channel_ctl->setChecked(output_options.write_image_mask_as_alpha_channel);
    dump_reference_data_for_debug_ctl->setChecked(output_options.dump_reference_data_for_debug);

    debug_frame_registration_ctl->setChecked(output_options.debug_frame_registration);
    debug_frame_registration_frame_indexes_ctl->setValue(output_options.debug_frame_registration_frame_indexes);
    debug_frame_registration_frame_indexes_ctl->setEnabled(output_options.debug_frame_registration);

    setEnabled(true);
  }

}
