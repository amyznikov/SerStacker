/*
 * QStackOutputOptions.cc
 *
 *  Created on: Jun 23, 2021
 *      Author: amyznikov
 */

#include "QStackOutputOptions.h"
#include <gui/qimproc/QImageProcessorsCollection.h>
#include <core/debug.h>

#define ICON_close          "close"
#define ICON_check_all      "check_all"

//static const char borderless_style[] = ""
//    "QToolButton { border: none; } "
//    "QToolButton::menu-indicator { image: none; }"
//    "";

static QIcon getIcon(const QString & name)
{
  return QIcon(QString(":/qstackingoptions/icons/%1").arg(name));
}



QStackOutputOptions::QStackOutputOptions(QWidget * parent)
  : Base("QStackingDebugOptions", parent)
{
  Q_INIT_RESOURCE(qstackingoptions_resources);

  if ( QImageProcessorsCollection::empty() ) {
    QImageProcessorsCollection::load();
  }

  ///

  form->addRow(output_directory_ctl =
      new QBrowsePathCombo("Output directory:",
          QFileDialog::AnyFile,
          this));
  output_directory_ctl->setShowDirsOnly(true);


  connect(output_directory_ctl, &QBrowsePathCombo::pathChanged,
      [this] () {
        if ( options_ && !updatingControls() ) {
          options_->output_options().output_directory =
              output_directory_ctl->currentPath().toStdString();
          emit parameterChanged();
        }
      });



  ///

  save_preprocessed_frames_ctl =
      add_named_checkbox("Save preprocessed (input) frames",
          [this](bool checked) {
            if ( options_ && options_->output_options().save_preprocessed_frames != checked ) {
              options_->output_options().save_preprocessed_frames = checked;
              output_preprocessed_frames_path_ctl->setEnabled(options_->output_options().save_preprocessed_frames);
              emit parameterChanged();
            }
          });

  form->addRow(output_preprocessed_frames_path_ctl =
      new QBrowsePathCombo("Preprocessd frames file name:",
          QFileDialog::AnyFile,
          this));

  connect(output_preprocessed_frames_path_ctl, &QBrowsePathCombo::pathChanged,
      [this] () {
        if ( options_ && !updatingControls() ) {
          options_->output_options().output_preprocessed_frames_filename =
              output_preprocessed_frames_path_ctl->currentPath().toStdString();
          emit parameterChanged();
        }
      });

  ///


  ///

  save_aligned_frames_ctl =
      add_named_checkbox("Save aligned frames",
          [this](bool checked) {
            if ( options_ && options_->output_options().save_aligned_frames != checked ) {
              options_->output_options().save_aligned_frames = checked;
              output_aligned_frames_path_ctl->setEnabled(options_->output_options().save_aligned_frames);
              emit parameterChanged();
            }
          });

  form->addRow(output_aligned_frames_path_ctl =
      new QBrowsePathCombo("Aligned frames file name:",
          QFileDialog::AnyFile,
          this));

  connect(output_aligned_frames_path_ctl, &QBrowsePathCombo::pathChanged,
      [this] () {
        if ( options_ && !updatingControls() ) {
          options_->output_options().output_aligned_frames_filename =
              output_aligned_frames_path_ctl->currentPath().toStdString();
          emit parameterChanged();
        }
      });

  ///

  ///

  save_ecc_frames_ctl =
      add_named_checkbox("Save ecc frames",
      [this](bool checked) {
        if ( options_ && options_->output_options().save_ecc_frames != checked ) {
          options_->output_options().save_ecc_frames = checked;
          output_ecc_frames_path_ctl->setEnabled(options_->output_options().save_ecc_frames);
          emit parameterChanged();
        }
      });

  form->addRow(output_ecc_frames_path_ctl =
    new QBrowsePathCombo("ECC frames file name:",
        QFileDialog::AnyFile,
        this));

  connect(output_ecc_frames_path_ctl, &QBrowsePathCombo::pathChanged,
    [this] () {
      if ( options_ && !updatingControls() ) {
        options_->output_options().output_ecc_frames_filename =
            output_ecc_frames_path_ctl->currentPath().toStdString();
        emit parameterChanged();
      }
    });


  ///

  ///

  save_postprocessed_frames_ctl =
      add_named_checkbox("Save postprocessed frames",
          [this](bool checked) {
            if ( options_ && options_->output_options().save_postprocessed_frames != checked  ) {
              options_->output_options().save_postprocessed_frames = checked;
              output_postprocessed_frames_path_ctl->setEnabled(options_->output_options().save_postprocessed_frames);
              emit parameterChanged();
            }
          });

  form->addRow(output_postprocessed_frames_path_ctl =
      new QBrowsePathCombo("Postprocessed frames file name:",
          QFileDialog::AnyFile,
          this));

  connect(output_postprocessed_frames_path_ctl, &QBrowsePathCombo::pathChanged,
      [this] () {
        if ( options_ && !updatingControls() ) {
          options_->output_options().output_postprocessed_frames_filename =
              output_postprocessed_frames_path_ctl->currentPath().toStdString();
          emit parameterChanged();
        }
      });


  ///

  save_accumulation_masks_ctl =
      add_named_checkbox("Save accumulation masks",
          [this](bool checked) {
            if ( options_ && options_->output_options().save_accumulation_masks != checked  ) {
              options_->output_options().save_accumulation_masks = checked;
              output_accumulation_masks_path_ctl->setEnabled(options_->output_options().save_accumulation_masks);
              emit parameterChanged();
            }
          });

  form->addRow(output_accumulation_masks_path_ctl =
      new QBrowsePathCombo("Accumulation masks file name:",
          QFileDialog::AnyFile,
          this));

  connect(output_accumulation_masks_path_ctl, &QBrowsePathCombo::pathChanged,
      [this] () {
        if ( options_ && !updatingControls() ) {
          options_->output_options().output_accumulation_masks_filename =
              output_accumulation_masks_path_ctl->currentPath().toStdString();
          emit parameterChanged();
        }
      });

  ///


  write_image_mask_as_alpha_channel_ctl =
      add_checkbox("Write image mask as alpha channel",
          [this](bool checked) {
            if ( options_ && options_->output_options().write_image_mask_as_alpha_channel != checked ) {
              options_->output_options().write_image_mask_as_alpha_channel = checked;
              emit parameterChanged();
            }
          });


  ///


  dump_reference_data_for_debug_ctl =
      add_checkbox("Dump reference data for debug",
          [this](bool checked) {
            if ( options_ && options_->output_options(). dump_reference_data_for_debug != checked ) {
              options_->output_options(). dump_reference_data_for_debug = checked;
              emit parameterChanged();
            }
          });

  ///

  debug_frame_registration_ctl =
      add_checkbox("Debug frame registration",
          [this](bool checked) {
            if ( options_ && options_->output_options(). debug_frame_registration != checked ) {
              options_->output_options().debug_frame_registration = checked;
              debug_frame_registration_frame_indexes_ctl->setEnabled(checked);
              emit parameterChanged();
            }
          });

  debug_frame_registration_frame_indexes_ctl =
      add_textbox("Frame indexes to debug registration:",
          [this](const QString & s) {
            if ( options_ ) {
              if ( fromString(s, &options_->output_options().debug_frame_registration_frame_indexes)) {
                emit parameterChanged();
              }
            }
          });


  ///


  applyToAll_ctl = new QToolButton(this);
  applyToAll_ctl->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  applyToAll_ctl->setIconSize(QSize(16,16));
  //applyToAll_ctl->setStyleSheet(borderless_style);
  applyToAll_ctl->setIcon(getIcon(ICON_check_all));
  applyToAll_ctl->setText("Copy these parameters to all currently selected in treeview");
  form->addRow(applyToAll_ctl);
  connect(applyToAll_ctl, &QToolButton::clicked,
      [this]() {
        if ( options_ ) {
          emit applyOutputOptionsToAllRequested(options_->output_options());
        }
      });


  ///

  setEnabled(false);
}

void QStackOutputOptions::set_stacking_options(const c_image_stacking_options::ptr & options)
{
  this->options_ = options;
  updateControls();
}

const c_image_stacking_options::ptr & QStackOutputOptions::stacking_options() const
{
  return this->options_;
}

void QStackOutputOptions::onupdatecontrols()
{
  if ( !options_ ) {
    setEnabled(false);
  }
  else {

    c_image_stacking_output_options & output_options =
        options_->output_options();

    output_directory_ctl->setCurrentPath(output_options.output_directory.c_str(), false);

    save_preprocessed_frames_ctl->setChecked(output_options.save_preprocessed_frames);
    output_preprocessed_frames_path_ctl->setCurrentPath(output_options.output_preprocessed_frames_filename.c_str());
    output_preprocessed_frames_path_ctl->setEnabled(output_options.save_preprocessed_frames);

    save_aligned_frames_ctl->setChecked(output_options.save_aligned_frames);
    output_aligned_frames_path_ctl->setCurrentPath(output_options.output_aligned_frames_filename.c_str());
    output_aligned_frames_path_ctl->setEnabled(output_options.save_aligned_frames);

    save_ecc_frames_ctl->setChecked(output_options.save_ecc_frames);
    output_ecc_frames_path_ctl->setCurrentPath(output_options.output_ecc_frames_filename.c_str());
    output_ecc_frames_path_ctl->setEnabled(output_options.save_ecc_frames);

    save_postprocessed_frames_ctl->setChecked(output_options.save_postprocessed_frames);
    output_postprocessed_frames_path_ctl->setCurrentPath(output_options.output_postprocessed_frames_filename.c_str());
    output_postprocessed_frames_path_ctl->setEnabled(output_options.save_postprocessed_frames);

    save_accumulation_masks_ctl->setChecked(output_options.save_accumulation_masks);
    output_accumulation_masks_path_ctl->setCurrentPath(output_options.output_accumulation_masks_filename.c_str());
    output_accumulation_masks_path_ctl->setEnabled(output_options.save_accumulation_masks);

    write_image_mask_as_alpha_channel_ctl->setChecked(output_options.write_image_mask_as_alpha_channel);
    dump_reference_data_for_debug_ctl->setChecked(output_options.dump_reference_data_for_debug);

    debug_frame_registration_ctl->setChecked(
        output_options.debug_frame_registration);
    debug_frame_registration_frame_indexes_ctl->setValue(
        output_options.debug_frame_registration_frame_indexes);
    debug_frame_registration_frame_indexes_ctl->setEnabled(
        output_options.debug_frame_registration);


    setEnabled(true);
  }

}
