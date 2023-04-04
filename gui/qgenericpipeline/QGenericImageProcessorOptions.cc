/*
 * QGenericImageProcessorOptions.cc
 *
 *  Created on: Apr 4, 2023
 *      Author: amyznikov
 */

#include "QGenericImageProcessorOptions.h"

QGenericImageProcessorOptions::QGenericImageProcessorOptions(QWidget * parent) :
    Base("", parent)
{
  frame_processor_ctl =
      add_combobox<QImageProcessorSelectionCombo>("image processor:",
          "",
          [this](int index, QImageProcessorSelectionCombo * combo) {
            if( pipeline_ ) {
              pipeline_->processing_options().image_processor =
                  combo->processor(index);
              Q_EMIT parameterChanged();
            }
          });

  add_expandable_groupbox("Output options",
      outputOptions_ctl = new QGenericImageProcessorOutputOptions());

  connect(outputOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  updateControls();
}

void QGenericImageProcessorOptions::set_pipeline(c_generic_image_processor * pipeline)
{
  pipeline_ = pipeline;
  updateControls();
}

c_generic_image_processor* QGenericImageProcessorOptions::pipeline() const
{
  return pipeline_;
}

void QGenericImageProcessorOptions::onupdatecontrols()
{
  if ( !pipeline_ ) {
    setEnabled(false);
  }
  else {

    c_generic_image_processor_options & processing_options =
        pipeline_->processing_options();

    if( !frame_processor_ctl->setCurrentProcessor(processing_options.image_processor) ) {
      processing_options.image_processor.reset();
    }


    c_generic_image_processor_output_options & output_options =
        pipeline_->output_options();

    outputOptions_ctl->set_options(&output_options);

    Base::onupdatecontrols();
    setEnabled(true);
  }

}
