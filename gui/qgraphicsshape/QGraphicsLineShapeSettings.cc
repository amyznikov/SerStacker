/*
 * QGraphicsLineShapeSettings.cc
 *
 *  Created on: Jan 19, 2023
 *      Author: amyznikov
 */

#include "QGraphicsLineShapeSettings.h"

QGraphicsLineShapeSettings::QGraphicsLineShapeSettings(QWidget * parent) :
    Base(parent)
{
  lockP1_ctl =
      add_checkbox("Lock P1",
          "",
          [this](bool checked) {
            if ( _opts ) {
              _opts->setLockP1(checked);
            }
          },
          [this](bool * checked) {
            if ( _opts ) {
              * checked = _opts->lockP1();
              return true;
            }
            return false;
          });

  lockP2_ctl =
      add_checkbox("Lock P2",
          "",
          [this](bool checked) {
            if ( _opts ) {
              _opts->setLockP2(checked);
            }
          },
          [this](bool * checked) {
            if ( _opts ) {
              * checked = _opts->lockP2();
              return true;
            }
            return false;
          });


  snapToPixelGrid_ctl =
      add_checkbox("Snap To Pixels",
          "",
          [this](bool checked) {
            if ( _opts ) {
              _opts->setSnapToPixelGrid(checked);
            }
          },
          [this](bool * checked) {
            if ( _opts ) {
              * checked = _opts->snapToPixelGrid();
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
      add_spinbox("Pen Width:",
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

  arrowSize_ctl =
      add_spinbox("Arrow Size:",
          "",
          [this](int v) {
            if ( _opts ) {
              _opts->setArrowSize(v);
            }
          },
          [this](int * v) {
            if ( _opts ) {
              *v = _opts->arrowSize();
              return true;
            }
            return false;
          });


  updateControls();
}

void QGraphicsLineShapeSettings::onload(const QSettings & settings, const QString & prefix)
{
  if ( _opts ) {

    const QString PREFIX = prefix.isEmpty() ? "QGraphicsLineShapeSettings" : prefix;

    Base::onload(settings, prefix);

    bool lockP1 = _opts->lockP1();
    if( load_parameter(settings, PREFIX, "lockP1", &lockP1) ) {
      _opts->setLockP1(lockP1);
    }

    bool lockP2 = _opts->lockP2();
    if( load_parameter(settings, PREFIX, "lockP2", &lockP2) ) {
      _opts->setLockP2(lockP2);
    }

    int penWidth = _opts->penWidth();
    if ( load_parameter(settings, PREFIX, "penWidth", &penWidth) ) {
      _opts->setPenWidth(penWidth);
    }

    double arrowSize = _opts->arrowSize();
    if ( load_parameter(settings, PREFIX, "arrowSize", &arrowSize) ) {
      _opts->setArrowSize(arrowSize);
    }

    QColor penColor = _opts->penColor();
    if ( load_parameter(settings, PREFIX, "penColor", &penColor) ) {
      _opts->setPenColor(penColor);
    }
  }

}

void QGraphicsLineShapeSettings::onsave(QSettings & settings, const QString & prefix)
{
  if ( _opts ) {

    const QString PREFIX = prefix.isEmpty() ? "QGraphicsLineShapeSettings" : prefix;

    Base::onsave(settings, PREFIX);

    save_parameter(settings, PREFIX, "lockP1", _opts->lockP1());
    save_parameter(settings, PREFIX, "lockP2", _opts->lockP2());
    save_parameter(settings, PREFIX, "penWidth", _opts->penWidth());
    save_parameter(settings, PREFIX, "arrowSize", _opts->arrowSize());
    save_parameter(settings, PREFIX, "penColor", _opts->penColor());
  }
}


