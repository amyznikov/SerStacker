/*
 * QGraphicsRectShapeSettings.cc
 *
 *  Created on: Jan 18, 2023
 *      Author: amyznikov
 */

#include "QGraphicsRectShapeSettings.h"

QGraphicsRectShapeSettings::QGraphicsRectShapeSettings(QWidget * parent) :
    Base(parent)
{

  snapToPixelGrid_ctl =
      add_checkbox("Snap to grid",
          "Snap to pixels grid",
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

  fixOnSceneCenter_ctl =
      add_checkbox("Fix on scene center",
          "Uncheck this button and use "
              "Shift + LeftMouseButton + MouseMove to position this object on scene."
              "Ctrl + LeftMouseButton + MouseMove also allows to resize the object",
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

  rect_ctl =
      add_numeric_box<QRectF>("RECT:",
          "Set Rectangle",
          [this](const QRectF & v) {
            if ( _opts ) {
              _opts->setSceneRect(v);
            }
          },
          [this](QRectF * v) {
            if ( _opts ) {
              *v = _opts->sceneRect();
              return true;
            }
            return false;
          });

  updateControls();
}
