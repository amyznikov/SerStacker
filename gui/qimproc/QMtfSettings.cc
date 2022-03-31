/*
 * QMtfSettings.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "QMtfSettings.h"
#include <core/proc/histogram.h>

const QMtfSettings::ClassFactory QMtfSettings::classFactory;

QMtfSettings::QMtfSettings(const c_mtf_routine::ptr & routine, QWidget * parent)
  : Base(&classFactory, routine, parent),
    displayFunction_(this)
{
  displayFunction_.set_mtf(&routine->mtf());
  mtf_ctl = add_widget("", new QMtfControl(this));
  mtf_ctl->setDisplayFunction(&displayFunction_);

  updateControls();

  connect(&displayFunction_, &QImageDisplayFunction::update,
      [this]() {
        if ( routine_->enabled() && !mtf_ctl->updatingControls() ) {
          emit parameterChanged();
        }
      });

  routine_->set_preprocess_notify_callback(
      [this](c_image_processor_routine * obj, cv::InputArray image, cv::InputArray mask) {
        mtf_ctl->setUpdatingControls(true);
        mtf_ctl->setInputImage(image, mask, true);
        mtf_ctl->setUpdatingControls(false);
      });


  routine_->set_postprocess_notify_callback(
      [this](c_image_processor_routine * obj, cv::InputArray image, cv::InputArray mask) {
        mtf_ctl->updateOutputHistogram();
      });
}

void QMtfSettings::onupdatecontrols()
{
  Base::onupdatecontrols();
}
