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
            if ( _opts && _opts->visible != checked ) {
              _opts->visible = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( _opts ) {
              *checked = _opts->visible;
              return true;
            }
            return false;
          });

  name_ctl =
      add_textbox("Grid Name",
          "Optional name for this grid to be displayed in quick menu",
          [this](const QString & v) {
            if ( _opts && _opts->name != v ) {
              _opts->name = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * v) {
            if ( _opts ) {
              *v = _opts->name;
              return true;
            }
            return false;
          });


  typedef decltype(OptsType::step)
      step_type;

  step_ctl =
      add_numeric_box<step_type>("Step Size",
          "Set grid step size",
          [this](const step_type v) {
            if ( _opts && _opts->step != v ) {
              _opts->step = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](step_type * v) {
            if ( _opts ) {
              *v = _opts->step;
              return true;
            }
            return false;
          });

  typedef decltype(OptsType::maxDistance)
      max_distance_type;

  max_distance_ctl =
      add_numeric_box<max_distance_type>("Max distance",
          "Max distance for the grid.\n"
          "Set <= 0 for auto selection based on far plane setting",
          [this](const max_distance_type v) {
            if ( _opts && _opts->maxDistance != v ) {
              _opts->maxDistance = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](max_distance_type * v) {
            if ( _opts ) {
              *v = _opts->maxDistance;
              return true;
            }
            return false;
          });

  typedef decltype(OptsType::Rotation)
      Rotation_type;

  rotation_ctl =
      add_numeric_box<Rotation_type>("Rotation",
          "Rotation Euler Angles in degrees [pitch;yaw;roll]",
          [this](const Rotation_type v) {
            if ( _opts  ) {
              _opts->Rotation = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](Rotation_type * v) {
            if ( _opts ) {
              *v = _opts->Rotation;
              return true;
            }
            return false;
          });



  typedef decltype(OptsType::Translation)
      Translation_type;

  translation_ctl =
      add_numeric_box<Translation_type>("Translation",
          "Translation [Tx;Ty;Tz].",
          [this](const Translation_type v) {
            if ( _opts ) {
              _opts->Translation = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](Translation_type * v) {
            if ( _opts ) {
              *v = _opts->Translation;
              return true;
            }
            return false;
          });


  gridColor_ctl =
      add_widget<QColorPickerButton>("Grid Color");

  connect(gridColor_ctl, &QColorPickerButton::colorSelected,
      [this]() {
        if ( _opts ) {
          _opts->gridColor = gridColor_ctl->color();
          Q_EMIT parameterChanged();
        }
      });
  connect(this, &ThisClass::populatecontrols,
      [this]() {
        if ( _opts ) {
          gridColor_ctl->setColor(_opts->gridColor);
        }
      });


  gridOpaqueness_ctl =
      add_sliderspinbox<int>("Grid Opaqueness",
          "Opaqueness (alpha-channel) the grid in range 0..255",
          [this](const int v) {
            if ( _opts && _opts->gridOpaqueness != v ) {
              _opts->gridOpaqueness = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _opts ) {
              *v = _opts->gridOpaqueness;
              return true;
            }
            return false;
          });

  gridOpaqueness_ctl->setRange(0, 255);


  fillColor_ctl =
      add_widget<QColorPickerButton>("Fill Color");

  connect(fillColor_ctl, &QColorPickerButton::colorSelected,
      [this]() {
        if ( _opts ) {
          _opts->fillColor = fillColor_ctl->color();
          Q_EMIT parameterChanged();
        }
      });
  connect(this, &ThisClass::populatecontrols,
      [this]() {
        if ( _opts ) {
          fillColor_ctl->setColor(_opts->fillColor);
        }
      });


  fillOpaqueness_ctl =
      add_sliderspinbox<int>("Fill Opaqueness",
          "Opaqueness (alpha-channel) the grid fill in range 0..255",
          [this](const int v) {
            if ( _opts && _opts->fillOpaqueness != v ) {
              _opts->fillOpaqueness = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( _opts ) {
              *v = _opts->fillOpaqueness;
              return true;
            }
            return false;
          });

  fillOpaqueness_ctl->setRange(0, 255);

  updateControls();
}

