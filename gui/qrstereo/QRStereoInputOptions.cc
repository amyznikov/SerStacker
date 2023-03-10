/*
 * QRStereoOptions.cc
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#include "QRStereoInputOptions.h"


QPlainTextEdit * createCameraMatrixEditBox(QWidget * parent)
{
  QPlainTextEdit * edit =
      new QPlainTextEdit(parent);

  const int maxHeight =
      5 * QFontMetrics(edit->font()).lineSpacing();

  edit->setMaximumHeight(maxHeight);
  edit->setFixedHeight(maxHeight);
  edit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);
  edit->setLineWrapMode(QPlainTextEdit::NoWrap);
  edit->setWordWrapMode(QTextOption::WrapMode::NoWrap);

  return edit;
}



QRStereoInputOptions::QRStereoInputOptions(QWidget * parent) :
    Base("QRStereoCalibrationInputOptions", parent)
{

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

              const c_regular_stereo_input_options & opts =
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

              const c_regular_stereo_input_options & opts =
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

//  form->addRow("Left Camera Matrix:",
//      leftCameraMatrixEditBox_ctl = createCameraMatrixEditBox(this));
//
//  form->addRow("Right Camera Matrix:",
//      rightCameraMatrixEditBox_ctl = createCameraMatrixEditBox(this));

  updateControls();
}

void QRStereoInputOptions::set_current_pipeline(const c_regular_stereo_pipeline::sptr & pipeline)
{
  pipeline_ = pipeline;
  updateControls();
}

const c_regular_stereo_pipeline::sptr& QRStereoInputOptions::current_pipeline() const
{
  return pipeline_;
}

void QRStereoInputOptions::populatesources()
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

    left_source_ctl->setEnabled(true);
    right_source_ctl->setEnabled(true);
  }
}

void QRStereoInputOptions::onupdatecontrols()
{
  populatesources();

  if( !pipeline_ || !pipeline_->input_sequence() ) {
    setEnabled(false);
  }
  else {

    Base::onupdatecontrols();
    setEnabled(true);
  }
}
