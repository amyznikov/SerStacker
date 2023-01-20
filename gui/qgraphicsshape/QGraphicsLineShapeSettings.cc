/*
 * QGraphicsLineShapeSettings.cc
 *
 *  Created on: Jan 19, 2023
 *      Author: amyznikov
 */

#include "QGraphicsLineShapeSettings.h"

QGraphicsLineShapeSettings::QGraphicsLineShapeSettings(QWidget * parent) :
  ThisClass("QGraphicsLineShapeSettings", parent)
{
}

QGraphicsLineShapeSettings::QGraphicsLineShapeSettings(const QString &prefix, QWidget * parent) :
    Base(prefix, parent)
{
  lockP1_ctl =
      add_checkbox("Lock P1",
          [this](bool checked) {
            if ( shape_ ) {
              shape_->setLockP1(checked);
            }
            save_parameter(PREFIX, "lockP1", shape_->lockP1());
          },
          [this](bool * checked) {
            if ( shape_ ) {
              * checked = shape_->lockP1();
              return true;
            }
            return false;
          });

  lockP2_ctl =
      add_checkbox("Lock P2",
          [this](bool checked) {
            if ( shape_ ) {
              shape_->setLockP2(checked);
            }
            save_parameter(PREFIX, "lockP2", shape_->lockP2());
          },
          [this](bool * checked) {
            if ( shape_ ) {
              * checked = shape_->lockP2();
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

void QGraphicsLineShapeSettings::setShape(QGraphicsLineShape * shape)
{
  shape_ = shape;
  updateControls();
}

QGraphicsLineShape * QGraphicsLineShapeSettings::shape() const
{
  return shape_;
}

void QGraphicsLineShapeSettings::onupdatecontrols()
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

void QGraphicsLineShapeSettings::onload(QSettings & settings)
{
  Base::onload(settings);

  if ( shape_ ) {

    bool lockP1 = shape_->lockP1();
    if( load_parameter(settings, PREFIX, "lockP1", &lockP1) ) {
      shape_->setLockP1(lockP1);
    }

    bool lockP2 = shape_->lockP2();
    if( load_parameter(settings, PREFIX, "lockP2", &lockP2) ) {
      shape_->setLockP2(lockP2);
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


QGraphicsLineShapeSettingsDialogBox::QGraphicsLineShapeSettingsDialogBox(QWidget * parent) :
    ThisClass("Shape Options", parent)
{
}

QGraphicsLineShapeSettingsDialogBox::QGraphicsLineShapeSettingsDialogBox(const QString & title, QWidget * parent) :
    ThisClass("Shape Options", nullptr, parent)
{
}

QGraphicsLineShapeSettingsDialogBox::QGraphicsLineShapeSettingsDialogBox(const QString & title, QGraphicsLineShape * shape, QWidget * parent) :
    Base(parent)
{
  setWindowTitle(title);

  QVBoxLayout * vbox =
      new QVBoxLayout(this);

  vbox->addWidget(settigs_ctl =
      new QGraphicsLineShapeSettings(this));

  settigs_ctl->setShape(shape);

  loadParameters();
}


void QGraphicsLineShapeSettingsDialogBox::setShape(QGraphicsLineShape * shape)
{
  settigs_ctl->setShape(shape);
}

QGraphicsLineShape * QGraphicsLineShapeSettingsDialogBox::shape() const
{
  return settigs_ctl->shape();
}

void QGraphicsLineShapeSettingsDialogBox::loadParameters()
{
  return settigs_ctl->loadParameters();
}

void QGraphicsLineShapeSettingsDialogBox::showEvent(QShowEvent * e)
{
  Base::showEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QGraphicsLineShapeSettingsDialogBox::hideEvent(QHideEvent * e)
{
  Base::hideEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}
