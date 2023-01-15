/*
 * QImageStackingInputOptions.cc
 *
 *  Created on: Jul 20, 2021
 *      Author: amyznikov
 */

#include "QImageStackingInputOptions.h"
#include <gui/widgets/addctrl.h>
#include <gui/widgets/settings.h>
#include <core/debug.h>

#define ICON_check_all      "check_all"

//static const char borderless_style[] = ""
//    "QToolButton { border: none; } "
//    "QToolButton::menu-indicator { image: none; }"
//    "";

static QIcon getIcon(const QString & name)
{
  return QIcon(QString(":/qstackingoptions/icons/%1").arg(name));
}

QImageStackingInputOptions::QImageStackingInputOptions(QWidget * parent)
  : Base("QImageStackingInputOptions", parent)
{
  start_frame_index_ctl =
      add_numeric_box<int>("Start frame index:",
          [this](int v) {
            if ( options_ && options_->start_frame_index != v ) {
              options_->start_frame_index = v;
              emit parameterChanged();
            }
          });


  max_input_frames_ctl =
      add_numeric_box<int>("Max frames:",
          [this](int v) {
            if ( options_ && options_->max_input_frames != v ) {
              options_->max_input_frames = v;
              emit parameterChanged();
            }
          });

  enable_remove_bad_pixels_ctl =
      add_checkbox("Detect Bad Pixels",
          [this](bool checked) {
            if ( options_ && checked != options_->filter_bad_pixels ) {
              options_->filter_bad_pixels = checked;
              emit parameterChanged();
            }
          });

  bad_pixels_variation_threshold_ctl =
      add_numeric_box<double>("Bad Pixels Variation Threshold",
          [this](double v) {
            if ( options_ && options_->hot_pixels_variation_threshold != v ) {
              options_->hot_pixels_variation_threshold = v;
              emit parameterChanged();
            }
          });

  drop_bad_asi_frames_ctl =
      add_checkbox("Drop Corrupted ASI frames",
          [this](bool checked) {
            if ( options_ && checked != options_->drop_bad_asi_frames ) {
              options_->drop_bad_asi_frames = checked;
              emit parameterChanged();
            }
          });

  enable_color_maxtrix_ctl =
      add_checkbox("Apply color matrix if available",
          [this](bool checked) {
            if ( options_ && checked != options_->enable_color_maxtrix ) {
              options_->enable_color_maxtrix = checked;
              emit parameterChanged();
            }
          });


  anscombe_ctl =
      add_enum_combobox<anscombe_method>("Anscombe Transform:",
          [this](anscombe_method v) {
            if ( options_ && v != options_->anscombe ) {
              options_->anscombe = v;
              emit parameterChanged();
            }
          });



  form->addRow(darkbayer_filename_ctl =
      new QBrowsePathCombo("DarkBayer:",
          QFileDialog::AcceptOpen,
          QFileDialog::ExistingFile,
          this));

  connect(darkbayer_filename_ctl, &QBrowsePathCombo::pathChanged,
      [this] () {
        if ( options_ && !updatingControls() ) {
          options_->darkbayer_filename =
              darkbayer_filename_ctl->currentPath().toStdString();
          emit parameterChanged();
        }
      });



  form->addRow(missing_pixel_mask_filename_ctl =
      new QBrowsePathCombo("MIssing pixels mask:",
          QFileDialog::AcceptOpen,
          QFileDialog::ExistingFile,
          this));


  connect(missing_pixel_mask_filename_ctl, &QBrowsePathCombo::pathChanged,
      [this] () {
        if ( options_ && !updatingControls() ) {
          options_->missing_pixel_mask_filename =
              missing_pixel_mask_filename_ctl->currentPath().toStdString();
          emit parameterChanged();
        }
      });


  missing_pixels_marked_black_ctl =
      add_checkbox("Missing pixels marked as black",
          [this](bool checked) {
            if ( options_ && checked != options_->missing_pixels_marked_black ) {
              options_->missing_pixels_marked_black = checked;
              emit parameterChanged();
            }
          });

  inpaint_missing_pixels_ctl =
      add_checkbox("Inpaint missing pixels with feasible values",
          [this](bool checked) {
            if ( options_ && checked != options_->inpaint_missing_pixels ) {
              options_->inpaint_missing_pixels = checked;
              emit parameterChanged();
            }
          });

  applyToAll_ctl = new QToolButton(this);
  applyToAll_ctl->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  applyToAll_ctl->setIconSize(QSize(16,16));
  //applyToAll_ctl->setStyleSheet(borderless_style);
  applyToAll_ctl->setIcon(getIcon(ICON_check_all));
  applyToAll_ctl->setText("Copy these parameters to all currently selected in treeview");
  connect(applyToAll_ctl, &QToolButton::clicked,
      [this]() {
        if ( options_ ) {
          emit applyInputOptionsToAllRequested(*options_);
        }
      });

  form->addRow(applyToAll_ctl);
}

void QImageStackingInputOptions::set_input_options(c_input_options * options)
{
  options_ = options;
  updateControls();
}

c_input_options * QImageStackingInputOptions::input_options() const
{
  return options_;
}

void QImageStackingInputOptions::onupdatecontrols()
{
  if ( !options_) {
    setEnabled(false);
  }
  else {

    enable_remove_bad_pixels_ctl->setChecked(options_->filter_bad_pixels);
    bad_pixels_variation_threshold_ctl->setValue(options_->hot_pixels_variation_threshold);
    drop_bad_asi_frames_ctl->setChecked(options_->drop_bad_asi_frames);

    enable_color_maxtrix_ctl->setChecked(options_->enable_color_maxtrix);
    anscombe_ctl->setCurrentItem(options_->anscombe);

    darkbayer_filename_ctl->setCurrentPath(options_->darkbayer_filename.c_str(), false);
    missing_pixel_mask_filename_ctl->setCurrentPath(options_->missing_pixel_mask_filename.c_str(), false);
    missing_pixels_marked_black_ctl->setChecked(options_->missing_pixels_marked_black);
    inpaint_missing_pixels_ctl->setChecked(options_->inpaint_missing_pixels);

    start_frame_index_ctl->setValue(options_->start_frame_index);
    max_input_frames_ctl->setValue(options_->max_input_frames);

    setEnabled(true);
  }

  Base::onupdatecontrols();
}
