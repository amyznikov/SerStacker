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


  startPoint_ctl =
    add_numeric_box<QPointF>("Start Point [px]",
        "Set Start Point as x;y",
        [this](const QPointF & v) {
          if ( _opts ) {
            _opts->setSceneStartPoint(v);
          }
        },
        [this](QPointF * v) {
          if ( _opts ) {
            *v = _opts->sceneStartPoint();
            return true;
          }
          return false;
        });

  endPoint_ctl =
    add_numeric_box<QPointF>("End Point [px]",
        "Set End Point as x;y",
        [this](const QPointF & v) {
          if ( _opts ) {
            _opts->setSceneEndPoint(v);
          }
        },
        [this](QPointF * v) {
          if ( _opts ) {
            *v = _opts->sceneEndPoint();
            return true;
          }
          return false;
        });


  updateControls();
}

