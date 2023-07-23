/*
 * QGraphicsTargetShapeSettings.cc
 *
 *  Created on: Jan 18, 2023
 *      Author: amyznikov
 */

#include "QGraphicsTargetShapeSettings.h"

QGraphicsTargetShapeSettings::QGraphicsTargetShapeSettings(QWidget * parent) :
  ThisClass("QGraphicsTargetShapeSettings", parent)
{
}

QGraphicsTargetShapeSettings::QGraphicsTargetShapeSettings(const QString &prefix, QWidget * parent) :
  Base(prefix, parent)
{
  fixOnSceneCenter_ctl =
      add_checkbox("Fix on scene center",
          "Uncheck this button and use "
              "Shift + LeftMouseButton + MouseMove to position this object on scene.",
          [this](bool checked) {
            if ( shape_ ) {
              shape_->setFixOnSceneCenter(checked);
            }
            save_parameter(PREFIX, "fixOnSceneCenter", shape_->fixOnSceneCenter());
          },
          [this](bool * checked) {
            if ( shape_ ) {
              * checked = shape_->fixOnSceneCenter();
              return true;

            }
            return false;
          });


  baseRadius_ctl =
      add_numeric_box<double>("Base radius:",
          "",
          [this](double v) {
            if ( shape_ ) {
              shape_->setBaseRadius(v);
              save_parameter(PREFIX, "baseRadius", shape_->baseRadius());
            }
          },
          [this](double * v) {
            if ( shape_ ) {
              *v = shape_->baseRadius();
              return true;
            }
            return false;
          });


  numRings_ctl =
      add_numeric_box<int>("Num rings:",
          "",
          [this](int v) {
            if ( shape_ ) {
              shape_->setNumRings(v);
              save_parameter(PREFIX, "numRings", shape_->numRings());
            }
          },
          [this](int * v) {
            if ( shape_ ) {
              *v = shape_->numRings();
              return true;
            }
            return false;
          });


  showDiagonalRays_ctl =
      add_checkbox("Show diagonal rays",
          "",
          [this](bool checked) {
            if ( shape_ ) {
              shape_->setShowDiagonalRays(checked);
              save_parameter(PREFIX, "showDiagonalRays", shape_->showDiagonalRays());
            }
          },
          [this](bool * checked) {
            if ( shape_ ) {
              * checked = shape_->showDiagonalRays();
              return true;
            }
            return false;
          });


  penColor_ctl =
      add_widget<QColorPickerButton>("Pen Color");

  connect(penColor_ctl, &QColorPickerButton::colorSelected,
      [this]() {
        if ( shape_ ) {
          shape_->setPenColor(penColor_ctl->color());
          save_parameter(PREFIX, "penColor", shape_->penColor());
        }
      });

  penWidth_ctl =
      add_spinbox( "Pen Width:",
          "",
          [this](int v) {
            if ( shape_ ) {
              shape_->setPenWidth(v);
              save_parameter(PREFIX, "penWidth", shape_->penWidth());
            }
          },
          [this](int * v) {
            if ( shape_ ) {
              *v = shape_->penWidth();
              return true;
            }
            return false;
          });

  updateControls();
}

void QGraphicsTargetShapeSettings::setShape(QGraphicsTargetShape * shape)
{
  shape_ = shape;
  updateControls();
}

QGraphicsTargetShape * QGraphicsTargetShapeSettings::shape() const
{
  return shape_;
}

void QGraphicsTargetShapeSettings::onupdatecontrols()
{
  if ( !shape_ ) {
    setEnabled(false);
  }
  else {

    penColor_ctl->setColor(shape_->penColor());

    Base::onupdatecontrols();

    setEnabled(true);
  }
}

void QGraphicsTargetShapeSettings::onload(QSettings & settings)
{
  Base::onload(settings);

  if ( shape_ ) {

    bool fixOnSceneCenter = shape_->fixOnSceneCenter();
    if( load_parameter(settings, PREFIX, "fixOnSceneCenter", &fixOnSceneCenter) ) {
      shape_->setFixOnSceneCenter(fixOnSceneCenter);
    }

    double baseRadius = shape_->baseRadius();
    if ( load_parameter(settings, PREFIX, "baseRadius", &baseRadius) ) {
      shape_->setBaseRadius(baseRadius);
    }

    int numRings = shape_->numRings();
    if ( load_parameter(settings, PREFIX, "numRings", &numRings) ) {
      shape_->setNumRings(numRings);
    }

    bool showDiagonalRays = shape_->showDiagonalRays();
    if ( load_parameter(settings, PREFIX, "showDiagonalRays", &showDiagonalRays) ) {
      shape_->setShowDiagonalRays(showDiagonalRays);
    }

    int penWidth = shape_->penWidth();
    if ( load_parameter(settings, PREFIX, "penWidth", &penWidth) ) {
      shape_->setPenWidth(penWidth);
    }

    QColor penColor = shape_->penColor();
    if ( load_parameter(settings, PREFIX, "penColor", &penColor) ) {
      shape_->setPenColor(penColor);
    }
  }
}


QGraphicsTargetShapeSettingsDialogBox::QGraphicsTargetShapeSettingsDialogBox(QWidget * parent) :
    ThisClass("Shape Options", parent)
{
}

QGraphicsTargetShapeSettingsDialogBox::QGraphicsTargetShapeSettingsDialogBox(const QString & title, QWidget * parent) :
    ThisClass("Shape Options", nullptr, parent)
{
}

QGraphicsTargetShapeSettingsDialogBox::QGraphicsTargetShapeSettingsDialogBox(const QString & title, QGraphicsTargetShape * shape, QWidget * parent) :
    Base(parent)
{
  setWindowTitle(title);

  QVBoxLayout * vbox =
      new QVBoxLayout(this);

  vbox->addWidget(settigs_ctl =
      new QGraphicsTargetShapeSettings(this));

  settigs_ctl->setShape(shape);

  loadParameters();
}


void QGraphicsTargetShapeSettingsDialogBox::setShape(QGraphicsTargetShape * shape)
{
  settigs_ctl->setShape(shape);
}

QGraphicsTargetShape * QGraphicsTargetShapeSettingsDialogBox::shape() const
{
  return settigs_ctl->shape();
}

void QGraphicsTargetShapeSettingsDialogBox::loadParameters()
{
  return settigs_ctl->loadParameters();
}

void QGraphicsTargetShapeSettingsDialogBox::showEvent(QShowEvent * e)
{
  Base::showEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QGraphicsTargetShapeSettingsDialogBox::hideEvent(QHideEvent * e)
{
  Base::hideEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}


