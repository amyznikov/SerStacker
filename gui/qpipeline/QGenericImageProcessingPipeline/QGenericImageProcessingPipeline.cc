/*
 * QGenericImageProcessingPipeline.cc
 *
 *  Created on: Jun 29, 2023
 *      Author: amyznikov
 */

#include "QGenericImageProcessingPipeline.h"



QGenericImageProcessingSettingsWidget::QGenericImageProcessingSettingsWidget(QWidget * parent) :
  ThisClass("", parent)
{
}

QGenericImageProcessingSettingsWidget::QGenericImageProcessingSettingsWidget(const QString & prefix, QWidget * parent) :
    Base(prefix, parent)
{
  ///

  add_expandable_groupbox("Input options",
      inputOptions_ctl = new QGenericImageProcessorInputOptions());

  connect(inputOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  ///

  add_expandable_groupbox("Output options",
      outputOptions_ctl = new QGenericImageProcessorOutputOptions());

  connect(outputOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  ///

  frame_processor_ctl =
      add_combobox<QImageProcessorSelectionCombo>("Image processor:",
          "",
          [this](int index, QImageProcessorSelectionCombo * combo) {
            if( pipeline_ ) {
              pipeline_->processing_options().image_processor =
                  combo->processor(index);
              Q_EMIT parameterChanged();
            }
          });

  ///

  updateControls();
}

void QGenericImageProcessingSettingsWidget::update_pipeline_controls()
{

  Base::update_pipeline_controls();

  if( !pipeline_ ) {
    inputOptions_ctl->set_pipeline(nullptr);
    outputOptions_ctl->set_output_options(nullptr);
  }
  else {

    inputOptions_ctl->set_pipeline(pipeline_);
    outputOptions_ctl->set_output_options(&pipeline_->output_options());

    c_generic_image_processor_options &processing_options =
        pipeline_->processing_options();

    if( !frame_processor_ctl->setCurrentProcessor(processing_options.image_processor) ) {
      processing_options.image_processor.reset();
    }
  }
}
