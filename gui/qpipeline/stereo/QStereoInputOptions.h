/*
 * QStereoCalibrationInputOptions.h
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStereoInputOptions_h__
#define __QStereoInputOptions_h__

#include <gui/qpipeline/QPipelineInputOptions.h>
#include <core/io/c_stereo_input.h>

template<class pipeline_type>
class QStereoInputOptions :
    public QPipelineInputOptions<pipeline_type>
{
public:
  typedef QStereoInputOptions ThisClass;
  typedef QPipelineInputOptions<pipeline_type> Base;

  QStereoInputOptions(QWidget * parent = nullptr) :
      Base(parent)
  {
    layout_type_ctl =
        QSettingsWidget::add_enum_combobox<stereo_input_frame_layout_type>("Stereo frame layout:",
            "",
            [this](stereo_input_frame_layout_type value) {
              if ( this->pipeline_ && this->pipeline_->input_options().layout_type != value ) {
                this->pipeline_->input_options().layout_type = value;
                updatesourcecontrols();
                Q_EMIT this->parameterChanged();
              }
            },
            [this](stereo_input_frame_layout_type * value) {
              if ( this->pipeline_ ) {
                * value = this->pipeline_->input_options().layout_type;
                return true;
              }
              return false;
            });

    left_source_ctl =
        QSettingsWidget::add_combobox<QComboBox>("Left camera source:",
            "",
            [this](int index, QComboBox * combo) {
              if ( this->pipeline_ ) {
                this->pipeline_->input_options().left_stereo_source = combo->itemData(index).toString().toStdString();
                Q_EMIT this->parameterChanged();
              }
            },

            [this](int * index, QComboBox * combo) {

              * index = -1;

              if ( this->pipeline_ && this->pipeline_->input_sequence() ) {

                if( !this->pipeline_->input_options().left_stereo_source.empty() ) {

                  const std::vector<c_input_source::sptr> &sources = this->pipeline_->input_sequence()->sources();

                  for( int i = 0, n = sources.size(); i < n; ++i ) {
                    if( this->pipeline_->input_options().left_stereo_source == sources[i]->filename() ) {
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
        QSettingsWidget::add_combobox<QComboBox>("Right camera source:",
            "",
            [this](int index, QComboBox * combo) {
              if ( this->pipeline_ ) {
                this->pipeline_->input_options().right_stereo_source = combo->itemData(index).toString().toStdString();
                Q_EMIT this->parameterChanged();
              }
            },

            [this](int * index, QComboBox * combo) {

              * index = -1;

              if ( this->pipeline_ && this->pipeline_->input_sequence() ) {

                if( !this->pipeline_->input_options().right_stereo_source.empty() ) {

                  const std::vector<c_input_source::sptr> &sources = this->pipeline_->input_sequence()->sources();

                  for( int i = 0, n = sources.size(); i < n; ++i ) {
                    if( this->pipeline_->input_options().right_stereo_source == sources[i]->filename() ) {
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

    swap_cameras_ctl =
        QSettingsWidget::add_checkbox("Swap cameras:",
            "",
            [this](bool checked) {
              if ( this->pipeline_ && this->pipeline_->input_options().swap_cameras != checked ) {
                this->pipeline_->input_options().swap_cameras = checked;
                Q_EMIT this->parameterChanged();
              }
            },
            [this](bool * checked) {
              if ( this->pipeline_ ) {
                * checked = this->pipeline_->input_options().swap_cameras;
                return true;
              }
              return false;
            });

  }

protected:
  void update_pipeline_controls() override
  {
    Base::update_pipeline_controls();
    populatesources();
    updatesourcecontrols();
  }

  void populatesources()
  {
    QSettingsWidget::c_update_controls_lock lock(this);

    left_source_ctl->clear();
    right_source_ctl->clear();

    if( !this->pipeline_ || !this->pipeline_->input_sequence() ) {
      left_source_ctl->setEnabled(false);
      right_source_ctl->setEnabled(false);
    }
    else {

      const c_input_sequence::sptr &input_sequence =
          this->pipeline_->input_sequence();

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

  void updatesourcecontrols()
  {
    if ( this->pipeline_ ) {
      if( this->pipeline_->input_options().layout_type == stereo_frame_layout_separate_sources ) {
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
  }

protected:
  QEnumComboBox<stereo_input_frame_layout_type> * layout_type_ctl = nullptr;
  QComboBox * left_source_ctl = nullptr;
  QComboBox * right_source_ctl = nullptr;
  QCheckBox * swap_cameras_ctl = nullptr;
};

#endif /* __QStereoInputOptions_h__ */
