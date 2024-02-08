/*
 * QGlViewPlanarGridSettings.cc
 *
 *  Created on: Feb 8, 2024
 *      Author: amyznikov
 */

#include "QGLViewPlanarGridSettings.h"

QGLViewPlanarGridSettings::QGLViewPlanarGridSettings(QWidget * parent) :
  Base(parent)
{
  visible_ctl =
      add_checkbox("Visible",
          "Set checked to draw this grid",
          [this](bool checked) {
            if ( options_ && options_->visible != checked ) {
              options_->visible = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              *checked = options_->visible;
              return true;
            }
            return false;
          });

  name_ctl =
      add_textbox("Grid Name",
          "Optional name for this grid to be displayed in quick menu",
          [this](const QString & v) {
            if ( options_ && options_->name != v ) {
              options_->name = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * v) {
            if ( options_ ) {
              *v = options_->name;
              return true;
            }
            return false;
          });


  typedef decltype(OptionsType::step)
      step_type;

  step_ctl =
      add_numeric_box<step_type>("Step Size",
          "Set grid step size",
          [this](const step_type v) {
            if ( options_ && options_->step != v ) {
              options_->step = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](step_type * v) {
            if ( options_ ) {
              *v = options_->step;
              return true;
            }
            return false;
          });

  typedef decltype(OptionsType::max_distance)
      max_distance_type;

  max_distance_ctl =
      add_numeric_box<max_distance_type>("Max distance",
          "Max distance for the grid.\n"
          "Set <= 0 for auto selection based on far plane setting",
          [this](const max_distance_type v) {
            if ( options_ && options_->max_distance != v ) {
              options_->max_distance = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](max_distance_type * v) {
            if ( options_ ) {
              *v = options_->max_distance;
              return true;
            }
            return false;
          });


  bgColor_ctl =
      add_widget<QColorPickerButton>("Grid Color");

  connect(this, &ThisClass::populatecontrols,
      [this]() {
        if ( options_ ) {
          bgColor_ctl->setColor(options_->color);
        }
      });
  connect(bgColor_ctl, &QColorPickerButton::colorSelected,
      [this]() {
        if ( options_ ) {
          options_->color = bgColor_ctl->color();
          Q_EMIT parameterChanged();
        }
      });


  opaqueness_ctl =
      add_sliderspinbox<int>("Opaqueness",
          "Opaqueness (alpha-channel) the grid in range 0..255",
          [this](const int v) {
            if ( options_ && options_->opaqueness != v ) {
              options_->opaqueness = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( options_ ) {
              *v = options_->opaqueness;
              return true;
            }
            return false;
          });

  opaqueness_ctl->setRange(0, 255);


  updateControls();
}

