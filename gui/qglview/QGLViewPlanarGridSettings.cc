/*
 * QGlViewPlanarGridSettings.cc
 *
 *  Created on: Feb 8, 2024
 *      Author: amyznikov
 */

#include "QGLViewPlanarGridSettings.h"
#include <core/debug.h>

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

  typedef decltype(OptionsType::maxDistance)
      max_distance_type;

  max_distance_ctl =
      add_numeric_box<max_distance_type>("Max distance",
          "Max distance for the grid.\n"
          "Set <= 0 for auto selection based on far plane setting",
          [this](const max_distance_type v) {
            if ( options_ && options_->maxDistance != v ) {
              options_->maxDistance = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](max_distance_type * v) {
            if ( options_ ) {
              *v = options_->maxDistance;
              return true;
            }
            return false;
          });

  typedef decltype(OptionsType::Rotation)
      Rotation_type;

  rotation_ctl =
      add_numeric_box<Rotation_type>("Rotation",
          "Rotation Euler Angles in degrees [pitch;yaw;roll]",
          [this](const Rotation_type v) {
            if ( options_  ) {
              options_->Rotation = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](Rotation_type * v) {
            if ( options_ ) {
              *v = options_->Rotation;
              return true;
            }
            return false;
          });



  typedef decltype(OptionsType::Translation)
      Translation_type;

  translation_ctl =
      add_numeric_box<Translation_type>("Translation",
          "Translation [Tx;Ty;Tz].",
          [this](const Translation_type v) {
            if ( options_ ) {
              options_->Translation = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](Translation_type * v) {
            if ( options_ ) {
              *v = options_->Translation;
              return true;
            }
            return false;
          });


  gridColor_ctl =
      add_widget<QColorPickerButton>("Grid Color");

  connect(gridColor_ctl, &QColorPickerButton::colorSelected,
      [this]() {
        if ( options_ ) {
          options_->gridColor = gridColor_ctl->color();
          Q_EMIT parameterChanged();
        }
      });
  connect(this, &ThisClass::populatecontrols,
      [this]() {
        if ( options_ ) {
          gridColor_ctl->setColor(options_->gridColor);
        }
      });


  gridOpaqueness_ctl =
      add_sliderspinbox<int>("Grid Opaqueness",
          "Opaqueness (alpha-channel) the grid in range 0..255",
          [this](const int v) {
            if ( options_ && options_->gridOpaqueness != v ) {
              options_->gridOpaqueness = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( options_ ) {
              *v = options_->gridOpaqueness;
              return true;
            }
            return false;
          });

  gridOpaqueness_ctl->setRange(0, 255);


  fillColor_ctl =
      add_widget<QColorPickerButton>("Fill Color");

  connect(fillColor_ctl, &QColorPickerButton::colorSelected,
      [this]() {
        if ( options_ ) {
          options_->fillColor = fillColor_ctl->color();
          Q_EMIT parameterChanged();
        }
      });
  connect(this, &ThisClass::populatecontrols,
      [this]() {
        if ( options_ ) {
          fillColor_ctl->setColor(options_->fillColor);
        }
      });


  fillOpaqueness_ctl =
      add_sliderspinbox<int>("Fill Opaqueness",
          "Opaqueness (alpha-channel) the grid fill in range 0..255",
          [this](const int v) {
            if ( options_ && options_->fillOpaqueness != v ) {
              options_->fillOpaqueness = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( options_ ) {
              *v = options_->fillOpaqueness;
              return true;
            }
            return false;
          });

  fillOpaqueness_ctl->setRange(0, 255);

  updateControls();
}

