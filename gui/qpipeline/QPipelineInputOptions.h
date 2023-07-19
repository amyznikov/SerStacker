/*
 * QPipelineInputOptions.h
 *
 *  Created on: Jul 12, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QPipelineInputOptions_h__
#define __QPipelineInputOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/QBrowsePathCombo.h>
#include <core/pipeline/c_image_processing_pipeline.h>

template<class pipeline_type>
class QPipelineInputOptions :
    public QSettingsWidget
{
public:
  typedef QPipelineInputOptions ThisClass;
  typedef QSettingsWidget Base;

  QPipelineInputOptions(QWidget * parent = nullptr) :
      Base("", parent)
  {
    start_frame_index_ctl =
        add_numeric_box<int>("Start frame index:",
            "",
            [this](int v) {
              if ( pipeline_ && pipeline_->input_options().start_frame_index != v ) {
                pipeline_->input_options().start_frame_index = v;
                Q_EMIT parameterChanged();
              }
            },
            [this](int * v) {
              if ( pipeline_ ) {
                *v = pipeline_->input_options().start_frame_index;
                return true;
              }
              return false;
            });

    max_input_frames_ctl =
        add_numeric_box<int>("Max frames:",
            "",
            [this](int v) {
              if ( pipeline_ && pipeline_->input_options().max_input_frames != v ) {
                pipeline_->input_options().max_input_frames = v;
                Q_EMIT parameterChanged();
              }
            },
            [this](int * v) {
              if ( pipeline_ ) {
                *v = pipeline_->input_options().max_input_frames;
                return true;
              }
              return false;
            });

    debayer_method_ctl =
        add_enum_combobox<DEBAYER_ALGORITHM>("DEBAYER:",
            "",
            [this](DEBAYER_ALGORITHM v) {
              if ( pipeline_ && pipeline_->input_options().debayer_method != v ) {
                pipeline_->input_options().debayer_method = v;
                Q_EMIT parameterChanged();
              }
            },
            [this](DEBAYER_ALGORITHM * v) {
              if ( pipeline_ ) {
                *v = pipeline_->input_options().debayer_method;
                return true;
              }
              return false;
            });

    enable_color_maxtrix_ctl =
        add_checkbox("Enable color matrix:",
            "",
            [this](bool checked) {
              if ( pipeline_ && pipeline_->input_options().enable_color_maxtrix != checked ) {
                pipeline_->input_options().enable_color_maxtrix = checked;
                Q_EMIT parameterChanged();
              }
            },
            [this](bool * checked) {
              if ( pipeline_ ) {
                * checked = pipeline_->input_options().enable_color_maxtrix;
                return true;
              }
              return false;
            });

    inpaint_missing_pixels_ctl =
        add_checkbox("Inpaint missing pixels:",
            "",
            [this](bool checked) {
              if ( pipeline_ && pipeline_->input_options().inpaint_missing_pixels != checked ) {
                pipeline_->input_options().inpaint_missing_pixels = checked;
                missing_pixel_mask_filename_ctl->setEnabled(pipeline_->input_options().inpaint_missing_pixels);
                Q_EMIT parameterChanged();
              }
            },
            [this](bool * checked) {
              if ( pipeline_ ) {
                * checked = pipeline_->input_options().inpaint_missing_pixels;
                return true;
              }
              return false;
            });

    missing_pixels_marked_black_ctl =
        add_checkbox("missing pixels marked black:",
            "",
            [this](bool checked) {
              if ( pipeline_ && pipeline_->input_options().missing_pixels_marked_black != checked ) {
                pipeline_->input_options().missing_pixels_marked_black = checked;
                missing_pixels_marked_black_ctl->setEnabled(pipeline_->input_options().inpaint_missing_pixels);
                Q_EMIT parameterChanged();
              }
            },
            [this](bool * checked) {
              if ( pipeline_ ) {
                * checked = pipeline_->input_options().missing_pixels_marked_black;
                return true;
              }
              return false;
            });

    missing_pixel_mask_filename_ctl =
        add_browse_for_path("",
            "Missing Pixels Mask",
            QFileDialog::AcceptOpen,
            QFileDialog::ExistingFile,
            [this](const QString & v) {
              if ( pipeline_ && pipeline_->input_options().missing_pixel_mask_filename != v.toStdString() ) {
                pipeline_->input_options().missing_pixel_mask_filename = v.toStdString();
                missing_pixel_mask_filename_ctl->setEnabled(pipeline_->input_options().inpaint_missing_pixels);
                Q_EMIT parameterChanged();
              }
            },
            [this](QString * v) {
              if ( pipeline_ ) {
                * v = pipeline_->input_options().missing_pixel_mask_filename.c_str();
                return true;
              }
              return false;
            });

    darkbayer_filename_ctl =
        add_browse_for_path("",
            "Dark Frame",
            QFileDialog::AcceptOpen,
            QFileDialog::ExistingFile,
            [this](const QString & v) {
              if ( pipeline_ && pipeline_->input_options().darkbayer_filename != v.toStdString() ) {
                pipeline_->input_options().darkbayer_filename = v.toStdString();
                Q_EMIT parameterChanged();
              }
            },
            [this](QString * v) {
              if ( pipeline_ ) {
                * v = pipeline_->input_options().darkbayer_filename.c_str();
                return true;
              }
              return false;
            });

    flatbayer_filename_ctl =
        add_browse_for_path("",
            "Flat Frame",
            QFileDialog::AcceptOpen,
            QFileDialog::ExistingFile,
            [this](const QString & v) {
              if ( pipeline_ && pipeline_->input_options().flatbayer_filename != v.toStdString() ) {
                pipeline_->input_options().flatbayer_filename = v.toStdString();
                Q_EMIT parameterChanged();
              }
            },
            [this](QString * v) {
              if ( pipeline_ ) {
                * v = pipeline_->input_options().flatbayer_filename.c_str();
                return true;
              }
              return false;
            });

    filter_bad_pixels_ctl =
        add_checkbox("Detect bad pixels:",
        "",
        [this](bool checked) {
          if ( pipeline_ && pipeline_->input_options().filter_bad_pixels != checked ) {
            pipeline_->input_options().filter_bad_pixels = checked;
            bad_pixels_variation_threshold_ctl->setEnabled(pipeline_->input_options().filter_bad_pixels);
            Q_EMIT parameterChanged();
          }
        },
        [this](bool * checked) {
          if ( pipeline_ ) {
            * checked = pipeline_->input_options().filter_bad_pixels;
            return true;
          }
          return false;
        });

    bad_pixels_variation_threshold_ctl =
        add_numeric_box<double>("bad_pixels_variation:",
            "",
            [this](double v) {
              if ( pipeline_ && pipeline_->input_options().bad_pixels_variation_threshold != v ) {
                pipeline_->input_options().bad_pixels_variation_threshold = v;
                bad_pixels_variation_threshold_ctl->setEnabled(pipeline_->input_options().filter_bad_pixels);
                Q_EMIT parameterChanged();
              }
            },
            [this](double * v) {
              if ( pipeline_ ) {
                *v = pipeline_->input_options().bad_pixels_variation_threshold;
                return true;
              }
              return false;
            });

  }

  void set_pipeline(pipeline_type * pipeline)
  {
    pipeline_ = pipeline;
    updateControls();
  }

  const pipeline_type * pipeline() const
  {
    return pipeline_;
  }

protected:
  void onupdatecontrols() override
  {
    if ( !pipeline_ ) {
      setEnabled(false);
    }
    else {
      Q_EMIT Base::populatecontrols();
      update_pipeline_controls();
      setEnabled(true);
    }
  }

  // placeholder for overrides
  virtual void update_pipeline_controls()
  {
    if( pipeline_ && !pipeline_->is_running() ) {

      const bool is_live =
          !pipeline_->input_sequence();

      start_frame_index_ctl->setEnabled(!is_live);
      max_input_frames_ctl->setEnabled(!is_live);
    }
  }

protected:
  pipeline_type * pipeline_ = nullptr;

  QNumericBox * start_frame_index_ctl = nullptr;
  QNumericBox * max_input_frames_ctl = nullptr;
  QEnumComboBox<DEBAYER_ALGORITHM> * debayer_method_ctl = nullptr;
  QCheckBox * enable_color_maxtrix_ctl = nullptr;

  QCheckBox * inpaint_missing_pixels_ctl = nullptr;
  QCheckBox * missing_pixels_marked_black_ctl = nullptr;
  QBrowsePathCombo * missing_pixel_mask_filename_ctl = nullptr;

  QBrowsePathCombo * darkbayer_filename_ctl = nullptr;
  QBrowsePathCombo * flatbayer_filename_ctl = nullptr;

  QCheckBox * detect_bad_asi_frames_ctl = nullptr;

  QCheckBox * filter_bad_pixels_ctl = nullptr;
  QNumericBox * bad_pixels_variation_threshold_ctl = nullptr;

  QCheckBox * enable_bground_normalization_ctl = nullptr;

  // c_histogram_normalization_options background_normalization_options;

};

#endif /* __QPipelineInputOptions_h__ */
