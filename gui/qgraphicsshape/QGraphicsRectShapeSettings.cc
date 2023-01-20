/*
 * QGraphicsRectShapeSettings.cc
 *
 *  Created on: Jan 18, 2023
 *      Author: amyznikov
 */

#include "QGraphicsRectShapeSettings.h"

QGraphicsRectShapeSettings::QGraphicsRectShapeSettings(QWidget * parent) :
  ThisClass("QGraphicsRectShapeSettings", parent)
{
}

QGraphicsRectShapeSettings::QGraphicsRectShapeSettings(const QString &prefix, QWidget * parent) :
    Base(prefix, parent)
{
  fixOnSceneCenter_ctl =
      add_checkbox("Fix on scene center",
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

  penColor_ctl =
      add_widget<QColorPickerButton>(
          "Pen Color");

  connect(penColor_ctl, &QColorPickerButton::colorSelected,
      [this]() {
        if ( shape_ ) {
          shape_->setPenColor(penColor_ctl->color());
          save_parameter(PREFIX, "penColor", shape_->penColor());
        }
      });

  penWidth_ctl =
      add_spinbox(""
          "Pen Width:",
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

void QGraphicsRectShapeSettings::setShape(QGraphicsRectShape * shape)
{
  shape_ = shape;
  updateControls();
}

QGraphicsRectShape * QGraphicsRectShapeSettings::shape() const
{
  return shape_;
}

void QGraphicsRectShapeSettings::onupdatecontrols()
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

void QGraphicsRectShapeSettings::onload(QSettings & settings)
{
  Base::onload(settings);

  if ( shape_ ) {

    bool fixOnSceneCenter = shape_->fixOnSceneCenter();
    if( load_parameter(settings, PREFIX, "fixOnSceneCenter", &fixOnSceneCenter) ) {
      shape_->setFixOnSceneCenter(fixOnSceneCenter);
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


QGraphicsRectShapeSettingsDialogBox::QGraphicsRectShapeSettingsDialogBox(QWidget * parent) :
    ThisClass("Shape Options", parent)
{
}

QGraphicsRectShapeSettingsDialogBox::QGraphicsRectShapeSettingsDialogBox(const QString & title, QWidget * parent) :
    ThisClass("Shape Options", nullptr, parent)
{
}

QGraphicsRectShapeSettingsDialogBox::QGraphicsRectShapeSettingsDialogBox(const QString & title, QGraphicsRectShape * shape, QWidget * parent) :
    Base(parent)
{
  setWindowTitle(title);

  QVBoxLayout * lv =
      new QVBoxLayout(this);

  lv->setSizeConstraint(QLayout::SetFixedSize);

  lv->addWidget(settigs_ctl =
      new QGraphicsRectShapeSettings(this));

  settigs_ctl->setShape(shape);

  loadParameters();
}


void QGraphicsRectShapeSettingsDialogBox::setShape(QGraphicsRectShape * shape)
{
  settigs_ctl->setShape(shape);
}

QGraphicsRectShape * QGraphicsRectShapeSettingsDialogBox::shape() const
{
  return settigs_ctl->shape();
}

void QGraphicsRectShapeSettingsDialogBox::loadParameters()
{
  return settigs_ctl->loadParameters();
}

void QGraphicsRectShapeSettingsDialogBox::showEvent(QShowEvent * e)
{
  Base::showEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QGraphicsRectShapeSettingsDialogBox::hideEvent(QHideEvent * e)
{
  Base::hideEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

