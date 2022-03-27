/*
 * QROISelectionOptions.cc
 *
 *  Created on: Jul 17, 2021
 *      Author: amyznikov
 */

#include "QROISelectionOptions.h"
#include <gui/widgets/addctrl.h>
#include <core/debug.h>

#define ICON_check_all      "check_all"

static const char borderless_style[] = ""
    "QToolButton { border: none; } "
    "QToolButton::menu-indicator { image: none; }"
    "";

static QIcon getIcon(const QString & name)
{
  return QIcon(QString(":/qstackingoptions/icons/%1").arg(name));
}
//
//
//QString toString(enum roi_selection_method v)
//{
//  return QString::fromStdString(toStdString(v));
//}
//
//enum roi_selection_method fromString(const QString  & s, enum roi_selection_method defval )
//{
//  return fromStdString(s.toStdString(), defval);
//}



QROISelectionOptions::QROISelectionOptions(QWidget * parent)
  : Base("QROISelectionOptions", parent)
{

  selectionMethod_ctl = add_enum_combobox<roi_selection_method>(
      "ROI selection:",
      [this](roi_selection_method v) {
        if ( options_ && v != options_->method ) {
          options_->method = v;
          updateROIControls();
          emit parameterChanged();
        }
      });

  planetaryDiskSize_ctl =
      add_numeric_box<cv::Size>("Crop size WxH [px]:",
          [this](const cv::Size & v) {
            if ( options_ && v != options_->planetary_disk_crop_size ) {
              options_->planetary_disk_crop_size = v;
              emit parameterChanged();
            }
          });

  rectangeROI_ctl =
      add_numeric_box<QString>("Rectangle L;T;WxH [px]:",
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


  applyToAll_ctl = new QToolButton(this);
  applyToAll_ctl->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  applyToAll_ctl->setIconSize(QSize(16,16));
  applyToAll_ctl->setStyleSheet(borderless_style);
  applyToAll_ctl->setIcon(getIcon(ICON_check_all));
  applyToAll_ctl->setText("Copy these parameters to all currently selected in treeview");
  connect(applyToAll_ctl, &QToolButton::clicked,
      [this]() {
        if ( options_ ) {
          emit applyROISelectionOptionsToAllRequested(*options_);
        }
      });

  form->addRow(applyToAll_ctl);

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
  }
  else {
    switch ( options_->method ) {
    case roi_selection_planetary_disk :

      rectangeROI_ctl->setVisible(false);
      form->labelForField(rectangeROI_ctl)->setVisible(false);


      planetaryDiskSize_ctl->setValue(options_->planetary_disk_crop_size);
      planetaryDiskSize_ctl->setVisible(true);
      form->labelForField(planetaryDiskSize_ctl)->setVisible(true);
      break;

    case roi_selection_rectange_crop :

      planetaryDiskSize_ctl->setVisible(false);
      form->labelForField(planetaryDiskSize_ctl)->setVisible(false);


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
