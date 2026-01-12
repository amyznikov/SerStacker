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
            if ( _options && _options->visible != checked ) {
              _options->visible = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( _options ) {
              *checked = _options->visible;
              return true;
            }
            return false;
          });

  name_ctl =
      add_textbox("Grid Name",
          "Optional name for this grid to be displayed in quick menu",
          [this](const QString & v) {
            if ( _options && _options->name != v ) {
              _options->name = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * v) {
            if ( _options ) {
              *v = _options->name;
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
            if ( _options && _options->step != v ) {
              _options->step = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](step_type * v) {
            if ( _options ) {
              *v = _options->step;
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
            if ( _options && _options->maxDistance != v ) {
              _options->maxDistance = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](max_distance_type * v) {
            if ( _options ) {
              *v = _options->maxDistance;
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
            if ( _options  ) {
              _options->Rotation = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](Rotation_type * v) {
            if ( _options ) {
              *v = _options->Rotation;
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
            if ( _options ) {
              _options->Translation = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](Translation_type * v) {
            if ( _options ) {
              *v = _options->Translation;
              return true;
            }
            return false;
          });


  gridColor_ctl =
      add_widget<QColorPickerButton>("Grid Color");

  connect(gridColor_ctl, &QColorPickerButton::colorSelected,
      [this]() {
        if ( _options ) {
          _options->gridColor = gridColor_ctl->color();
          Q_EMIT parameterChanged();
        }
      });
  connect(this, &ThisClass::populatecontrols,
      [this]() {
        if ( _options ) {
          gridColor_ctl->setColor(_options->gridColor);
        }
      });


  gridOpaqueness_ctl =
      add_sliderspinbox<int>("Grid Opaqueness",
          "Opaqueness (alpha-channel) the grid in range 0..255",
          [this](const int v) {
            if ( _options && _options->gridOpaqueness != v ) {
              _options->gridOpaqueness = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _options ) {
              *v = _options->gridOpaqueness;
              return true;
            }
            return false;
          });

  gridOpaqueness_ctl->setRange(0, 255);


  fillColor_ctl =
      add_widget<QColorPickerButton>("Fill Color");

  connect(fillColor_ctl, &QColorPickerButton::colorSelected,
      [this]() {
        if ( _options ) {
          _options->fillColor = fillColor_ctl->color();
          Q_EMIT parameterChanged();
        }
      });
  connect(this, &ThisClass::populatecontrols,
      [this]() {
        if ( _options ) {
          fillColor_ctl->setColor(_options->fillColor);
        }
      });


  fillOpaqueness_ctl =
      add_sliderspinbox<int>("Fill Opaqueness",
          "Opaqueness (alpha-channel) the grid fill in range 0..255",
          [this](const int v) {
            if ( _options && _options->fillOpaqueness != v ) {
              _options->fillOpaqueness = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _options ) {
              *v = _options->fillOpaqueness;
              return true;
            }
            return false;
          });

  fillOpaqueness_ctl->setRange(0, 255);

  updateControls();
}

