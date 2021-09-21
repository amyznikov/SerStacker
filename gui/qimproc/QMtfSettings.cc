/*
 * QMtfSettings.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "QMtfSettings.h"

const QMtfSettings::ClassFactory QMtfSettings::classFactory;

QMtfSettings::QMtfSettings(const c_mtf_routine::ptr & routine, QWidget * parent)
  : Base(&classFactory, routine, parent)
{
  mtf_ctl = add_widget("", new QMtfControl(this));

  updateControls();

  connect(mtf_ctl, &QMtfControl::mtfChanged,
      [this]() {
        if ( routine_ && routine_->enabled() ) {
          emit parameterChanged();
        }
      });
}

void QMtfSettings::onupdatecontrols()
{
  if ( !routine_ ) {
    setEnabled(false);
    mtf_ctl->setMtf(nullptr);
  }
  else {

    mtf_ctl->setMtf(routine_->mtf());

    routine_->set_preprocess_notify_callback(
        [this](c_image_processor_routine * obj, cv::InputArray image, cv::InputArray mask) {
          if ( mtf_ctl->isAutoMtfActionEnabled() ) {

            mtf_ctl->setUpdatingControls(true);
            mtf_ctl->setInputImage(image, mask);
            mtf_ctl->setUpdatingControls(false);

          }
        });

    routine_->set_postprocess_notify_callback(
        [this](c_image_processor_routine * obj, cv::InputArray image, cv::InputArray mask) {
          mtf_ctl->setOutputImage(image, mask);
        });

    setEnabled(true);
  }

  Base::onupdatecontrols();
}
