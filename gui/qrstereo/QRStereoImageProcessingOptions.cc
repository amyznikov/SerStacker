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
            if( options_ ) {
              options_->input_image_processor =
                  combo->processor(index);
              Q_EMIT parameterChanged();
            }
          });

  remapped_image_processor_ctl =
      add_combobox<QImageProcessorSelectionCombo>("Remapped image processor:",
          "",
          [this](int index, QImageProcessorSelectionCombo * combo) {
            if( options_ ) {
              options_->remapped_image_processor =
                  combo->processor(index);
              Q_EMIT parameterChanged();
            }
          });

  output_image_processor_ctl =
      add_combobox<QImageProcessorSelectionCombo>("Output image processor:",
          "",
          [this](int index, QImageProcessorSelectionCombo * combo) {
            if( options_ ) {
              options_->output_image_processor =
                  combo->processor(index);
              Q_EMIT parameterChanged();
            }
          });
}

void QRStereoImageProcessingOptions::set_options(c_regular_stereo_image_processing_options * options)
{
  options_ = options;
  updateControls();
}

c_regular_stereo_image_processing_options* QRStereoImageProcessingOptions::options() const
{
  return options_;
}

void QRStereoImageProcessingOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {

    if( !input_image_processor_ctl->setCurrentProcessor(options_->input_image_processor) ) {
      options_->input_image_processor.reset();
    }

    if( !remapped_image_processor_ctl->setCurrentProcessor(options_->remapped_image_processor) ) {
      options_->remapped_image_processor.reset();
    }

    if( !output_image_processor_ctl->setCurrentProcessor(options_->output_image_processor) ) {
      options_->output_image_processor.reset();
    }

    setEnabled(true);
  }

}
