/*
 * QStackOutputOptions.cc
 *
 *  Created on: Jun 23, 2021
 *      Author: amyznikov
 */

#include "QStackOutputOptions.h"
#include <gui/qimproc/QImageProcessorsCollection.h>
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

  if ( QImageProcessorsCollection::empty() ) {
    QImageProcessorsCollection::load();
  }

  connect(QImageProcessorsCollection::instance(), &QImageProcessorsCollection::collectionChanged,
      this, &ThisClass::populateAvailableImageProcessors);


  ///

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


  ///


  save_processed_frames_ctl =
      add_checkbox("Save processed frames",
          [this](int state) {
            if ( options_ ) {
              if ( options_->save_processed_frames != (state == Qt::Checked) ) {
                options_->save_processed_frames = (state == Qt::Checked);
                emit parameterChanged();
              }
            }
          });

  frame_processor_selector_ctl =
      add_combobox( "Post Process Individual Frames:",
          [this](int index) {
            if ( options_ ) {
              if ( index < 1 ||(index = QImageProcessorsCollection::indexof(
                  frame_processor_selector_ctl->currentText()) ) < 0 ) {
                options_->frame_processor.reset();
              }
              else {
                options_->frame_processor = QImageProcessorsCollection::item(index);
              }
            }
      });

  frame_processor_selector_ctl->setEditable(false);
  frame_processor_selector_ctl->setMinimumContentsLength(12);
  frame_processor_selector_ctl->setSizeAdjustPolicy(QComboBox::AdjustToContents);


  ///


  accumulated_image_processor_selector_ctl =
      add_combobox( "Post Process Accumulated Image:",
          [this](int index) {
            if ( options_ ) {
              if ( index < 1 ||(index = QImageProcessorsCollection::indexof(
                  accumulated_image_processor_selector_ctl->currentText()) ) < 0 ) {
                options_->accumuated_image_processor.reset();
              }
              else {
                options_->accumuated_image_processor = QImageProcessorsCollection::item(index);
              }
            }
      });

  accumulated_image_processor_selector_ctl->setEditable(false);
  accumulated_image_processor_selector_ctl->setMinimumContentsLength(12);
  accumulated_image_processor_selector_ctl->setSizeAdjustPolicy(QComboBox::AdjustToContents);


  ///

  write_aligned_video_ctl = add_checkbox("Write aligned video",
      [this](int state) {
        if ( options_ ) {
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


  ///

  write_image_mask_as_alpha_channel_ctl = add_checkbox("Write image mask as alpha channel",
      [this](int state) {
        if ( options_ ) {
          const bool status = state == Qt::Checked;
          if ( status != options_->write_image_mask_as_alpha_channel ) {
            options_->write_image_mask_as_alpha_channel = status;
            emit parameterChanged();
          }
        }
      });


  ///

  dump_reference_data_for_debug_ctl = add_checkbox("Dump reference data for debug",
      [this](int state) {
        if ( options_ ) {
          const bool status = state == Qt::Checked;
          if ( status != options_->dump_reference_data_for_debug ) {
            options_->dump_reference_data_for_debug = status;
            emit parameterChanged();
          }
        }
      });


  ///

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


  ///

  populateAvailableImageProcessors();

  setEnabled(false);
}

void QStackOutputOptions::set_output_options(c_image_stacking_output_options * options)
{
  this->options_ = options;
  updateControls();
}

const c_image_stacking_output_options * QStackOutputOptions::output_options() const
{
  return this->options_;
}

void QStackOutputOptions::onupdatecontrols()
{
  if ( !options_ ) {
    setEnabled(false);
  }
  else {
    int current_index;

    output_directory_ctl->setCurrentPath(options_->output_directory.c_str(), false);

    save_processed_frames_ctl->setChecked(options_->save_processed_frames);

    write_image_mask_as_alpha_channel_ctl->setChecked(options_->write_image_mask_as_alpha_channel);
    write_aligned_video_ctl->setChecked(options_->write_aligned_video);

    output_aligned_video_filename_ctl->setCurrentPath(options_->output_aligned_video_filename.c_str());
    output_aligned_video_filename_ctl->setEnabled(options_->write_aligned_video);
    dump_reference_data_for_debug_ctl->setChecked(options_->dump_reference_data_for_debug);


    current_index = 0;
    if ( options_->frame_processor ) {
      if ( (current_index = frame_processor_selector_ctl->findText(options_->frame_processor->cname())) < 0 ) {
        options_->frame_processor.reset();
        current_index = 0;
      }
    }
    frame_processor_selector_ctl->setCurrentIndex(current_index);


    current_index = 0;
    if ( options_->accumuated_image_processor ) {

      current_index = accumulated_image_processor_selector_ctl->findText(
              options_->accumuated_image_processor->cname());

      if ( current_index < 0 ) {
        options_->accumuated_image_processor.reset();
        current_index = 0;
      }
    }
    accumulated_image_processor_selector_ctl->setCurrentIndex(current_index);


    setEnabled(true);
  }

}

void QStackOutputOptions::populateAvailableImageProcessors()
{
  const bool oldUpdatingControlsFlag =
      updatingControls();

  setUpdatingControls(true);

  const QString current_accumulated_processor_name =
      accumulated_image_processor_selector_ctl->currentText();

  const QString current_frame_processor_name =
      frame_processor_selector_ctl->currentText();

  accumulated_image_processor_selector_ctl->clear();
  accumulated_image_processor_selector_ctl->addItem("None");

  frame_processor_selector_ctl->clear();
  frame_processor_selector_ctl->addItem("None");

  for ( int i = 0, n = QImageProcessorsCollection::size(); i < n; ++i ) {

    const c_image_processor::ptr processor =
        QImageProcessorsCollection::item(i);

    if( processor ) {

      accumulated_image_processor_selector_ctl->
          addItem(processor->cname(), processor->cfilename());

      frame_processor_selector_ctl->
          addItem(processor->cname(), processor->cfilename());

    }
  }

  if ( !current_accumulated_processor_name.isEmpty() ) {
    const int index =
        accumulated_image_processor_selector_ctl->findText(current_accumulated_processor_name);

    if ( index >= 0 ) {
      accumulated_image_processor_selector_ctl->setCurrentIndex(index);
    }
  }

  if ( !current_frame_processor_name.isEmpty() ) {
    const int index =
        frame_processor_selector_ctl->findText(current_frame_processor_name);

    if ( index >= 0 ) {
      frame_processor_selector_ctl->setCurrentIndex(index);
    }
  }

  setUpdatingControls(oldUpdatingControlsFlag);
}
