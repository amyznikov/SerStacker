/*
 * QStackOutputOptions.cc
 *
 *  Created on: Jun 23, 2021
 *      Author: amyznikov
 */

#include "QStackOutputOptions.h"
#include <gui/widgets/addctrl.h>
#include <gui/widgets/settings.h>
#include <core/debug.h>

#define ICON_close          "close"
#define ICON_check_all      "check_all"

static const char borderless_style[] = ""
    "QToolButton { border: none; } "
    "QToolButton::menu-indicator { image: none; }"
    "";

static QIcon getIcon(const QString & name)
{
  return QIcon(QString(":/qstackingoptions/icons/%1").arg(name));
}



QStackOutputOptions::QStackOutputOptions(QWidget * parent)
  : Base("QStackingDebugOptions", parent)
{
  Q_INIT_RESOURCE(qstackingoptions_resources);

  form->addRow(output_directory_ctl =
      new QBrowsePathCombo("Output directory:",
          QFileDialog::DirectoryOnly,
          this));

  connect(output_directory_ctl, &QBrowsePathCombo::pathChanged,
      [this] () {
        if ( options_ && !updatingControls() ) {
          options_->output_directory = output_directory_ctl->currentPath().toStdString();
          emit parameterChanged();
        }
      });



  write_image_mask_as_alpha_channel_ctl = add_checkbox(form, "Write image mask as alpha channel",
      [this](int state) {
        if ( options_ && !updatingControls() ) {
          const bool status = state == Qt::Checked;
          if ( status != options_->write_image_mask_as_alpha_channel ) {
            options_->write_image_mask_as_alpha_channel = status;
            emit parameterChanged();
          }
        }
      });



  write_aligned_video_ctl = add_checkbox(form, "Write aligned video",
      [this](int state) {
        if ( options_ && !updatingControls() ) {
          const bool status = state == Qt::Checked;
          if ( status != options_->write_aligned_video ) {
            options_->write_aligned_video = status;
            emit parameterChanged();
          }

          output_aligned_video_filename_ctl->setEnabled(options_->write_aligned_video);
        }
      });


  form->addRow(output_aligned_video_filename_ctl =
      new QBrowsePathCombo("Aligned video file name:",
          QFileDialog::AnyFile,
          this));

  connect(output_aligned_video_filename_ctl, &QBrowsePathCombo::pathChanged,
      [this] () {
        if ( options_ && !updatingControls() ) {
          options_->output_aligned_video_filename = output_aligned_video_filename_ctl->currentPath().toStdString();
          emit parameterChanged();
        }
      });

  dump_reference_frames_for_debug_ctl = add_checkbox(form, "Dump reference frames for debug",
      [this](int state) {
        if ( options_ && !updatingControls() ) {
          const bool status = state == Qt::Checked;
          if ( status != options_->dump_reference_frames_for_debug ) {
            options_->dump_reference_frames_for_debug = status;
            emit parameterChanged();
          }
        }
      });



  applyToAll_ctl = new QToolButton(this);
  applyToAll_ctl->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  applyToAll_ctl->setIconSize(QSize(16,16));
  applyToAll_ctl->setStyleSheet(borderless_style);
  applyToAll_ctl->setIcon(getIcon(ICON_check_all));
  applyToAll_ctl->setText("Copy these parameters to all currently selected in treeview");
  form->addRow(applyToAll_ctl);
  connect(applyToAll_ctl, &QToolButton::clicked,
      [this]() {
        if ( options_ ) {
          emit applyOutputOptionsToAllRequested(*options_);
        }
      });


  setEnabled(false);
}

void QStackOutputOptions::set_debug_options(c_image_stacking_output_options * options)
{
  this->options_ = options;
  updateControls();
}

const c_image_stacking_output_options * QStackOutputOptions::debug_options() const
{
  return this->options_;
}

void QStackOutputOptions::onupdatecontrols()
{
  if ( !options_ ) {
    setEnabled(false);
  }
  else {
    output_directory_ctl->setCurrentPath(options_->output_directory.c_str(), false);
    write_image_mask_as_alpha_channel_ctl->setChecked(options_->write_image_mask_as_alpha_channel);
    write_aligned_video_ctl->setChecked(options_->write_aligned_video);
    output_aligned_video_filename_ctl->setCurrentPath(options_->output_aligned_video_filename.c_str());
    output_aligned_video_filename_ctl->setEnabled(options_->write_aligned_video);
    dump_reference_frames_for_debug_ctl->setChecked(options_->dump_reference_frames_for_debug);
    setEnabled(true);
  }

}
