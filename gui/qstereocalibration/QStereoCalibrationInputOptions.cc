/*
 * QStereoCalibrationInputOptions.cc
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#include "QStereoCalibrationInputOptions.h"

QStereoCalibrationInputOptions::QStereoCalibrationInputOptions(QWidget * parent) :
    Base("QStereoCalibrationInputOptions", parent)
{

  layout_type_ctl =
      add_enum_combobox<stereo_calibration_input_frame_layout_type>("Stereo frame layout:",
          [this](stereo_calibration_input_frame_layout_type value) {
            if ( pipeline_ && pipeline_->input_options().layout_type != value ) {
              pipeline_->input_options().layout_type = value;
              updatesourcecontrols();
              Q_EMIT parameterChanged();
            }
          },
          [this](stereo_calibration_input_frame_layout_type * value) {
            if ( pipeline_ ) {
              * value = pipeline_->input_options().layout_type;
              return true;
            }
            return false;
          });


  swap_cameras_ctl =
      add_checkbox("Swap cameras:",
          [this](bool checked) {
            if ( pipeline_ && pipeline_->input_options().swap_cameras != checked ) {
              pipeline_->input_options().swap_cameras = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( pipeline_ ) {
              * checked = pipeline_->input_options().swap_cameras;
              return true;
            }
            return false;
          });


  left_source_ctl =
      add_combobox<QComboBox>("Left camera source:",

          [this](int index, QComboBox * combo) {
            if ( pipeline_ ) {
              pipeline_->input_options().left_stereo_source =
                  combo->itemData(index).toString().toStdString();
              Q_EMIT parameterChanged();
            }
          },

          [this](int * index, QComboBox * combo) {

            * index = -1;

            if ( pipeline_ && pipeline_->input_sequence() ) {

              const c_stereo_calibration_input_options & opts =
                  pipeline_->input_options();

              if( !opts.left_stereo_source.empty() ) {

                const std::vector<c_input_source::sptr> &sources =
                    pipeline_->input_sequence()->sources();

                for( int i = 0, n = sources.size(); i < n; ++i ) {
                  if( opts.left_stereo_source == sources[i]->filename() ) {
                    *index = i;
                    return true;
                  }
                }
              }

              return true;
            }
            return false;
          });

  left_source_ctl->setEditable(false);


  right_source_ctl =
      add_combobox<QComboBox>("Right camera source:",

          [this](int index, QComboBox * combo) {
            if ( pipeline_ ) {
              pipeline_->input_options().right_stereo_source =
                  combo->itemData(index).toString().toStdString();
              Q_EMIT parameterChanged();
            }
          },

          [this](int * index, QComboBox * combo) {
            * index = -1;

            if ( pipeline_ && pipeline_->input_sequence() ) {

              const c_stereo_calibration_input_options & opts =
                  pipeline_->input_options();

              if( !opts.right_stereo_source.empty() ) {

                const std::vector<c_input_source::sptr> &sources =
                    pipeline_->input_sequence()->sources();

                for( int i = 0, n = sources.size(); i < n; ++i ) {
                  if( opts.right_stereo_source == sources[i]->filename() ) {
                    *index = i;
                    return true;
                  }
                }
              }

              return true;
            }
            return false;
          });

  right_source_ctl->setEditable(false);

  start_frame_index_ctl =
      add_numeric_box<int>("start_frame_index:",
          [this](int value) {
            if ( pipeline_ ) {
              pipeline_->input_options().start_frame_index = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( pipeline_ ) {
              * value = pipeline_->input_options().start_frame_index;
              return true;
            }
            return false;
          });

  max_input_frames_ctl =
      add_numeric_box<int>("max_input_frames:",
          [this](int value) {
            if ( pipeline_ ) {
              pipeline_->input_options().max_input_frames = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( pipeline_ ) {
              * value = pipeline_->input_options().max_input_frames;
              return true;
            }
            return false;
          });

  inpaint_missing_pixels_ctl =
      add_checkbox("inpaint_missing_pixels",
          [this](bool value) {
            if ( pipeline_ ) {
              pipeline_->input_options().inpaint_missing_pixels = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * value) {
            if ( pipeline_ ) {
              * value = pipeline_->input_options().inpaint_missing_pixels;
              return true;
            }
            return false;
          });

  enable_color_maxtrix_ctl =
      add_checkbox("enable_color_maxtrix",
          [this](bool value) {
            if ( pipeline_ ) {
              pipeline_->input_options().enable_color_maxtrix = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * value) {
            if ( pipeline_ ) {
              * value = pipeline_->input_options().enable_color_maxtrix;
              return true;
            }
            return false;
          });

  updateControls();
}

void QStereoCalibrationInputOptions::set_current_pipeline(const c_stereo_calibration_pipeline::sptr & pipeline)
{
  pipeline_ = pipeline;
  updateControls();
}

const c_stereo_calibration_pipeline::sptr& QStereoCalibrationInputOptions::current_pipeline() const
{
  return pipeline_;
}

void QStereoCalibrationInputOptions::populatesources()
{
  c_update_controls_lock lock(this);

  left_source_ctl->clear();
  right_source_ctl->clear();

  if ( !pipeline_ || !pipeline_->input_sequence() ) {
    left_source_ctl->setEnabled(false);
    right_source_ctl->setEnabled(false);
  }
  else {

    const c_input_sequence::sptr &input_sequence =
        pipeline_->input_sequence();

    const std::vector<c_input_source::sptr> &sources =
        input_sequence->sources();

    for( const c_input_source::sptr &source : sources ) {

      const QString filename =
          QFileInfo(source->cfilename()).fileName();

      left_source_ctl->addItem(filename, QString(source->cfilename()));
      right_source_ctl->addItem(filename, QString(source->cfilename()));
    }

    updatesourcecontrols();
  }
}

void QStereoCalibrationInputOptions::updatesourcecontrols()
{
  if( pipeline_->input_options().layout_type == stereo_calibration_frame_layout_separate_sources ) {
    left_source_ctl->setEnabled(true);
    right_source_ctl->setEnabled(true);
    swap_cameras_ctl->setEnabled(false);
  }
  else {
    left_source_ctl->setEnabled(true);
    right_source_ctl->setEnabled(false);
    swap_cameras_ctl->setEnabled(true);
  }

}

void QStereoCalibrationInputOptions::onupdatecontrols()
{
  populatesources();

  if( !pipeline_ || !pipeline_->input_sequence() ) {
    setEnabled(false);
  }
  else {

    Base::onupdatecontrols();
    updatesourcecontrols();

    setEnabled(true);
  }
}
