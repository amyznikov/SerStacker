/*
 * QOutputFrameWriterOptions.h
 *
 *  Created on: Nov 4, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QOutputFrameWriterOptions_h__
#define __QOutputFrameWriterOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/QBrowsePathCombo.h>
#include <gui/qimproc/QImageProcessorsCollection.h>
#include <core/pipeline/c_output_frame_writer.h>

class QOutputFrameWriterOptions :
    public QSettingsWidget // Template<c_output_frame_writer_options>
{
  Q_OBJECT;
public:
  typedef QOutputFrameWriterOptions ThisClass;
  typedef QSettingsWidget Base; // Template<c_output_frame_writer_options> Base;

  QOutputFrameWriterOptions(QWidget * parent = nullptr);

  void set_output_frame_writer_options(c_output_frame_writer_options * options)
  {
    _options = options;
    updateControls();
  }

  c_output_frame_writer_options * output_frame_writer_options() const
  {
    return _options;
  }

protected:
  c_output_frame_writer_options * _options =  nullptr;
  QLineEditBox * output_filename_ctl =  nullptr;
  QFFmpegOptionsControl * output_ffmpeg_opts_ctl = nullptr;
  QImageProcessorSelectionCombo * output_image_processor_ctl = nullptr;
  QEnumComboBox<PIXEL_DEPTH> * output_pixel_depth_ctl = nullptr;
  QCheckBox * save_frame_mapping_ctl = nullptr;
};

#endif /* __QOutputFrameWriterOptions_h__ */
