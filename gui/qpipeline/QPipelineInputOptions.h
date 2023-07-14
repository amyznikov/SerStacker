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

    enable_color_maxtrix_ctl =
        add_checkbox("Enable color maxtrix:",
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
  QCheckBox * enable_color_maxtrix_ctl = nullptr;
  QCheckBox * inpaint_missing_pixels_ctl = nullptr;
};

#endif /* __QPipelineInputOptions_h__ */
