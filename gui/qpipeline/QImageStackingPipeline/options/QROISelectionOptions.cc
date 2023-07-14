/*
 * QROISelectionOptions.cc
 *
 *  Created on: Jul 17, 2021
 *      Author: amyznikov
 */

#include "QROISelectionOptions.h"
#include <core/settings/opencv_settings.h>

QROISelectionOptions::QROISelectionOptions(QWidget * parent) :
  Base("QROISelectionOptions", parent)
{

  selectionMethod_ctl =
      add_enum_combobox<roi_selection_method>("ROI selection:",
          "",
          [this](roi_selection_method v) {
            if ( options_ && v != options_->method ) {
              options_->method = v;
              updateROIControls();
              emit parameterChanged();
            }
          });

  planetaryDiskSize_ctl =
      add_numeric_box<cv::Size>("Crop size WxH [px]:",
          "",
          [this](const cv::Size & v) {
            if ( options_ && v != options_->planetary_disk_crop_size ) {
              options_->planetary_disk_crop_size = v;
              emit parameterChanged();
            }
          });

  rectangeROI_ctl =
      add_numeric_box<QString>("Rectangle L;T;WxH [px]:",
          "",
          [this](const QString & v) {

            if ( options_ ) {

              int x, y, cx, cy, n;

              const QByteArray a = v.toUtf8();
              const char * s = a.data();

              if ( (n = sscanf(s, "%d ; %d ; %d x %d", &x, &y, &cx, &cy)) == 4 ) {
                options_ ->rectangle_roi_selection.x = x;
                options_ ->rectangle_roi_selection.y = y;
                options_ ->rectangle_roi_selection.width = cx;
                options_ ->rectangle_roi_selection.height = cy;
                emit parameterChanged();
              }
              else if ((n = sscanf(s, "%d ; %d ; %d ; %d", &x, &y, &cx, &cy)) == 4) {
                options_ ->rectangle_roi_selection.x = x;
                options_ ->rectangle_roi_selection.y = y;
                options_ ->rectangle_roi_selection.width = cx - x;
                options_ ->rectangle_roi_selection.height = cy - y;
                emit parameterChanged();
              }
              else {
                CF_ERROR("Parse error in ROI specification: s='%s' n=%d", s, n);
              }
            }

          });

  planetaryDiskGbSigma_ctl =
      add_numeric_box<double>("gbsigma:",
          "",
          [this](double v) {
            if ( options_ && v != options_->planetary_disk_gbsigma ) {
              options_->planetary_disk_gbsigma = v;
              emit parameterChanged();
            }
          });

  planetaryDiskStdevFactor_ctl =
      add_numeric_box<double>("stdev_factor:",
          "",
          [this](double v) {
            if ( options_ && v != options_->planetary_disk_stdev_factor ) {
              options_->planetary_disk_stdev_factor = v;
              emit parameterChanged();
            }
          });

  setEnabled(false);

  updateControls();
}

void QROISelectionOptions::set_roi_selection_options(c_roi_selection_options * options)
{
  options_ = options;
  updateControls();
}

const c_roi_selection_options * QROISelectionOptions::roi_selection_options() const
{
  return options_;
}


void QROISelectionOptions::onupdatecontrols()
{
  if ( !options_ ) {
    setEnabled(false);
  }
  else {

    selectionMethod_ctl->setCurrentItem(options_->method);
    updateROIControls();
    setEnabled(true);
  }

}

void QROISelectionOptions::updateROIControls()
{
  if ( !options_ || options_->method == roi_selection_none ) {

    planetaryDiskSize_ctl->setVisible(false);
    form->labelForField(planetaryDiskSize_ctl)->setVisible(false);

    rectangeROI_ctl->setVisible(false);
    form->labelForField(rectangeROI_ctl)->setVisible(false);

    planetaryDiskGbSigma_ctl->setVisible(false);
    form->labelForField(planetaryDiskGbSigma_ctl)->setVisible(false);

    planetaryDiskStdevFactor_ctl->setVisible(false);
    form->labelForField(planetaryDiskStdevFactor_ctl)->setVisible(false);
  }
  else {
    switch ( options_->method ) {
    case roi_selection_planetary_disk :

      rectangeROI_ctl->setVisible(false);
      form->labelForField(rectangeROI_ctl)->setVisible(false);

      planetaryDiskSize_ctl->setValue(options_->planetary_disk_crop_size);
      planetaryDiskSize_ctl->setVisible(true);

      planetaryDiskGbSigma_ctl->setValue(options_->planetary_disk_gbsigma);
      planetaryDiskGbSigma_ctl->setVisible(true);
      form->labelForField(planetaryDiskGbSigma_ctl)->setVisible(true);

      planetaryDiskStdevFactor_ctl->setValue(options_->planetary_disk_stdev_factor);
      planetaryDiskStdevFactor_ctl->setVisible(true);
      form->labelForField(planetaryDiskStdevFactor_ctl)->setVisible(true);

      break;

    case roi_selection_rectange_crop :

      planetaryDiskSize_ctl->setVisible(false);
      form->labelForField(planetaryDiskSize_ctl)->setVisible(false);

      planetaryDiskGbSigma_ctl->setVisible(false);
      form->labelForField(planetaryDiskGbSigma_ctl)->setVisible(false);

      planetaryDiskStdevFactor_ctl->setVisible(false);
      form->labelForField(planetaryDiskStdevFactor_ctl)->setVisible(false);


      rectangeROI_ctl->setText(QString("%1;%2;%3x%4")
          .arg(options_->rectangle_roi_selection.x)
          .arg(options_->rectangle_roi_selection.y)
          .arg(options_->rectangle_roi_selection.width)
          .arg(options_->rectangle_roi_selection.height));

      rectangeROI_ctl->setVisible(true);
      form->labelForField(rectangeROI_ctl)->setVisible(true);
      break;

    default:
      break;
    }
  }
}
