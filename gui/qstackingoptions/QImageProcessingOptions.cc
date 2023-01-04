/*
 * QImageProcessingOptions.cc
 *
 *  Created on: Jul 11, 2022
 *      Author: amyznikov
 */

#include "QImageProcessingOptions.h"

QImageProcessingOptions::QImageProcessingOptions(QWidget * parent) :
    Base("QImageProcessingOptions", parent)
{
  input_image_processor_ctl =
      add_combobox<QImageProcessorSelectionCombo>(
          "Input image processor:",
          [this](int index, QImageProcessorSelectionCombo * combo) {
            if( options_ ) {
              options_->input_image_processor =
              combo->processor(index);
              emit parameterChanged();
            }
          });

  ecc_image_processor_ctl =
      add_combobox<QImageProcessorSelectionCombo>(
          "ECC image processor:",
          [this](int index, QImageProcessorSelectionCombo * combo) {
            if( options_ ) {
              options_->ecc_image_processor =
              combo->processor(index);
              emit parameterChanged();
            }
          });

  aligned_image_processor_ctl =
      add_combobox<QImageProcessorSelectionCombo>(
          "Aligned image processor:",
          [this](int index, QImageProcessorSelectionCombo * combo) {
            if( options_ ) {
              options_->aligned_image_processor =
              combo->processor(index);
              emit parameterChanged();
            }
          });

  accumulated_image_processor_ctl =
      add_combobox<QImageProcessorSelectionCombo>(
          "Accumulated image processor:",
          [this](int index, QImageProcessorSelectionCombo * combo) {
            if( options_ ) {
              options_->accumulated_image_processor =
              combo->processor(index);
              emit parameterChanged();
            }
          });

}

void QImageProcessingOptions::set_image_processing_options(c_image_processing_options * options)
{
  options_ = options;
  updateControls();
}

c_image_processing_options* QImageProcessingOptions::image_processing_options() const
{
  return options_;
}

void QImageProcessingOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {

    if( !input_image_processor_ctl->setCurrentProcessor(options_->input_image_processor) ) {
      options_->input_image_processor.reset();
    }

    if( !ecc_image_processor_ctl->setCurrentProcessor(options_->ecc_image_processor) ) {
      options_->ecc_image_processor.reset();
    }

    if( !aligned_image_processor_ctl->setCurrentProcessor(options_->aligned_image_processor) ) {
      options_->aligned_image_processor.reset();
    }

    if( !accumulated_image_processor_ctl->setCurrentProcessor(options_->accumulated_image_processor) ) {
      options_->accumulated_image_processor.reset();
    }

    setEnabled(true);
  }
}
