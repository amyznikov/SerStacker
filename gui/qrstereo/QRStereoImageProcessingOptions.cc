/*
 * QRStereoImageProcessingOptions.cc
 *
 *  Created on: Mar 16, 2023
 *      Author: amyznikov
 */

#include "QRStereoImageProcessingOptions.h"

QRStereoImageProcessingOptions::QRStereoImageProcessingOptions(QWidget * parent) :
  Base("QRStereoIImageProcessingOptions", parent)
{
  input_image_processor_ctl =
      add_combobox<QImageProcessorSelectionCombo>("Input image processor:",
          "",
          [this](int index, QImageProcessorSelectionCombo * combo) {
            if( pipeline_ ) {
              pipeline_->image_processing_options().input_image_processor =
                  combo->processor(index);
              Q_EMIT parameterChanged();
            }
          });

  stereo_match_preprocessor_ctl =
      add_combobox<QImageProcessorSelectionCombo>("Stereo match preprocessor:",
          "",
          [this](int index, QImageProcessorSelectionCombo * combo) {
            if( pipeline_ ) {
              pipeline_->image_processing_options().stereo_match_preprocessor =
                  combo->processor(index);
              Q_EMIT parameterChanged();
            }
          });

  output_image_processor_ctl =
      add_combobox<QImageProcessorSelectionCombo>("Output image processor:",
          "",
          [this](int index, QImageProcessorSelectionCombo * combo) {
            if( pipeline_ ) {
              pipeline_->image_processing_options().output_image_processor =
                  combo->processor(index);
              Q_EMIT parameterChanged();
            }
          });
}

void QRStereoImageProcessingOptions::set_current_pipeline(const c_regular_stereo_pipeline::sptr & pipeline)
{
  pipeline_ = pipeline;
  updateControls();
}

const c_regular_stereo_pipeline::sptr & QRStereoImageProcessingOptions::current_pipeline() const
{
  return pipeline_;
}

void QRStereoImageProcessingOptions::onupdatecontrols()
{
  if( !pipeline_ ) {
    setEnabled(false);
  }
  else {

    c_regular_stereo_image_processing_options &options =
        pipeline_->image_processing_options();

    if( !input_image_processor_ctl->setCurrentProcessor(options.input_image_processor) ) {
      options.input_image_processor.reset();
    }

    if( !stereo_match_preprocessor_ctl->setCurrentProcessor(options.stereo_match_preprocessor) ) {
      options.stereo_match_preprocessor.reset();
    }

    if( !output_image_processor_ctl->setCurrentProcessor(options.output_image_processor) ) {
      options.output_image_processor.reset();
    }


    setEnabled(true);
  }

}

void QRStereoImageProcessingOptions::populatesources()
{

}
