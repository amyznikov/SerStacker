/*
 * QGraphicsTargetShapeSettings.cc
 *
 *  Created on: Jan 18, 2023
 *      Author: amyznikov
 */

#include "QGraphicsTargetShapeSettings.h"

QGraphicsTargetShapeSettings::QGraphicsTargetShapeSettings(QWidget * parent) :
  Base(parent)
{
  lockPosition_ctl =
      add_checkbox("Lock position",
          "Lock object position on this scene ",
          [this](bool checked) {
            if ( _opts ) {
              _opts->setLockPosition(checked);
            }
          },
          [this](bool * checked) {
            if ( _opts ) {
              * checked = _opts->lockPosition();
              return true;

            }
            return false;
          });

  fixOnSceneCenter_ctl =
      add_checkbox("Fix on scene center",
          "Uncheck this button and use "
              "Shift + LeftMouseButton + MouseMove to position this object on scene.",
          [this](bool checked) {
            if ( _opts ) {
              _opts->setFixOnSceneCenter(checked);
            }
          },
          [this](bool * checked) {
            if ( _opts ) {
              * checked = _opts->fixOnSceneCenter();
              return true;

            }
            return false;
          });


//  baseRadius_ctl =
//      add_numeric_box<double>("Base radius:",
//          "",
//          [this](double v) {
//            if ( _opts ) {
//              _opts->setBaseRadius(v);
//                saveSettings();
//            }
//          },
//          [this](double * v) {
//            if ( _opts ) {
//              *v = _opts->baseRadius();
//              return true;
//            }
//            return false;
//          });

  baseRadius_ctl =
      add_sliderspinbox<int>("Base radius:",
          "Radius of first (internal) circle",
          [this](int v) {
            if ( _opts ) {
              _opts->setBaseRadius(v);
            }
          },
          [this](int * v) {
            if ( _opts ) {
              *v = _opts->baseRadius();
              return true;
            }
            return false;
          });

  baseRadius_ctl->setRange(0, 1000);


  numRings_ctl =
      add_spinbox("Num rings:",
          "",
          [this](int v) {
            if ( _opts ) {
              _opts->setNumRings(v);
            }
          },
          [this](int * v) {
            if ( _opts ) {
              *v = _opts->numRings();
              return true;
            }
            return false;
          });
  numRings_ctl->setMinimum(1);


  showDiagonalRays_ctl =
      add_checkbox("Show diagonal rays",
          "",
          [this](bool checked) {
            if ( _opts ) {
              _opts->setShowDiagonalRays(checked);
            }
          },
          [this](bool * checked) {
            if ( _opts ) {
              * checked = _opts->showDiagonalRays();
              return true;
            }
            return false;
          });


  penColor_ctl =
      add_color_picker_button("Pen Color",
          "",
          [this](const QColor&v) {
            if ( _opts ) {
              _opts->setPenColor(v);
            }
          },
          [this](QColor * v) {
            if ( _opts ) {
              * v =  _opts->penColor();
              return true;
            }
            return false;
          });

  penWidth_ctl =
      add_spinbox( "Pen Width:",
          "",
          [this](int v) {
            if ( _opts ) {
              _opts->setPenWidth(v);
            }
          },
          [this](int * v) {
            if ( _opts ) {
              *v = _opts->penWidth();
              return true;
            }
            return false;
          });

  updateControls();
}


void QGraphicsTargetShapeSettings::onload(const QSettings & settings, const QString & prefix)
{
  if ( _opts ) {

    const QString PREFIX = prefix.isEmpty() ? "QGraphicsTargetShapeSettings" : prefix;

    Base::onload(settings, prefix);

    bool lockPosition = _opts->lockPosition();
    if( load_parameter(settings, PREFIX, "fixOnSceneCenter", &lockPosition) ) {
      _opts->setLockPosition(lockPosition);
    }

    bool fixOnSceneCenter = _opts->fixOnSceneCenter();
    if( load_parameter(settings, PREFIX, "fixOnSceneCenter", &fixOnSceneCenter) ) {
      _opts->setFixOnSceneCenter(fixOnSceneCenter);
    }

    double baseRadius = _opts->baseRadius();
    if ( load_parameter(settings, PREFIX, "baseRadius", &baseRadius) ) {
      _opts->setBaseRadius(baseRadius);
    }

    int numRings = _opts->numRings();
    if ( load_parameter(settings, PREFIX, "numRings", &numRings) ) {
      _opts->setNumRings(numRings);
    }

    bool showDiagonalRays = _opts->showDiagonalRays();
    if ( load_parameter(settings, PREFIX, "showDiagonalRays", &showDiagonalRays) ) {
      _opts->setShowDiagonalRays(showDiagonalRays);
    }

    int penWidth = _opts->penWidth();
    if ( load_parameter(settings, PREFIX, "penWidth", &penWidth) ) {
      _opts->setPenWidth(penWidth);
    }

    QColor penColor = _opts->penColor();
    if ( load_parameter(settings, PREFIX, "penColor", &penColor) ) {
      _opts->setPenColor(penColor);
    }
  }
}

void QGraphicsTargetShapeSettings::onsave(QSettings & settings, const QString & prefix)
{
  if ( _opts ) {

    const QString PREFIX = prefix.isEmpty() ? "QGraphicsTargetShapeSettings" : prefix;

    Base::onsave(settings, PREFIX);

    save_parameter(settings, PREFIX, "fixOnSceneCenter", _opts->fixOnSceneCenter());
    save_parameter(settings, PREFIX, "fixOnSceneCenter", _opts->fixOnSceneCenter());
    save_parameter(settings, PREFIX, "baseRadius", _opts->baseRadius());
    save_parameter(settings, PREFIX, "numRings", _opts->numRings());
    save_parameter(settings, PREFIX, "showDiagonalRays", _opts->showDiagonalRays());
    save_parameter(settings, PREFIX, "penWidth", _opts->penWidth());
    save_parameter(settings, PREFIX, "penColor", _opts->penColor());
  }
}

