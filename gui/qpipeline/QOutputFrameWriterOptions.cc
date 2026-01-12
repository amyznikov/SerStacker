/*
 * QOutputFrameWriterOptions.cc
 *
 *  Created on: Nov 4, 2023
 *      Author: amyznikov
 */

#include "QOutputFrameWriterOptions.h"

QOutputFrameWriterOptions::QOutputFrameWriterOptions(QWidget * parent) :
    Base(parent)
{
  output_filename_ctl =
      add_textbox("Filename:",
          "Optional output file name pattern",
          [this](const QString & v) {
            if ( _options ) {
              _options->output_filename = v.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * v) {
            if ( _options ) {
              *v = _options->output_filename.c_str();
              return true;
            }
            return false;
          });

  output_ffmpeg_opts_ctl =
      add_ffmpeg_options_control("ffmpeg opts:",
          "Optional ffmpeg options for ffmpeg writers",
          [this](const QString & v) {
            if ( _options ) {
              _options->ffmpeg_opts = v.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * v) {
            if ( _options ) {
              *v = _options->ffmpeg_opts.c_str();
              return true;
            }
            return false;
          });

  output_image_processor_ctl =
      add_combobox<QImageProcessorSelectionCombo>("Image processor:",
          "Optional image processor before writing output frame",
          false,
          [this](int index, QImageProcessorSelectionCombo * combo) {
            if( _options ) {
              _options ->output_image_processor = combo->processor(index);
              Q_EMIT parameterChanged();
            }
          },
          [this](int * index, QImageProcessorSelectionCombo * combo) -> bool {
            if( _options ) {
              combo->setCurrentProcessor(_options->output_image_processor);
            }
            return false;
          });

  output_pixel_depth_ctl =
      add_enum_combobox<PIXEL_DEPTH>("Pixel Depth:",
          "Optional conversion to output pixel depth",
          [this](PIXEL_DEPTH v) {
            if( _options ) {
              _options ->output_pixel_depth = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](PIXEL_DEPTH * v) {
            if( _options ) {
              * v = _options ->output_pixel_depth;
              return true;
            }
            return false;
          });

  save_frame_mapping_ctl =
      add_checkbox("Save frame mapping",
          "",
          [this](bool checked) {
            if( _options ) {
              _options ->save_frame_mapping = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if( _options ) {
              * checked = _options ->save_frame_mapping;
              return true;
            }
            return false;

          });

  updateControls();
}

