/*
 * QTestGLViewSettings.cc
 *
 *  Created on: Feb 2, 2023
 *      Author: amyznikov
 */

#include "QTestGLViewSettings.h"

namespace qgltest {

QTestGLViewSettings::QTestGLViewSettings(QWidget * parent) :
    Base("QTestGLViewSettings", parent)
{
  eye_x_ctl =
      add_sliderspinbox<double>("EyeX",
          [this](double value) {
            if ( glView_ ) {
              glView_->setEyeX(value);
            }
          },
          [this](double * value) -> bool {
            if ( glView_ ) {
              *value = glView_->eyeX();
              return true;
            }
            return false;
          });

  eye_y_ctl =
      add_sliderspinbox<double>("EyeY",
          [this](double value) {
            if ( glView_ ) {
              glView_->setEyeY(value);
            }
          },
          [this](double * value) -> bool {
            if ( glView_ ) {
              *value = glView_->eyeY();
              return true;
            }
            return false;
          });

  eye_z_ctl =
      add_sliderspinbox<double>("EyeZ",
          [this](double value) {
            if ( glView_ ) {
              glView_->setEyeZ(value);
            }
          },
          [this](double * value) -> bool {
            if ( glView_ ) {
              *value = glView_->eyeZ();
              return true;
            }
            return false;
          });


  eye_x_ctl->setRange(-500, 500);
  eye_y_ctl->setRange(-500, 500);
  eye_z_ctl->setRange(-500, 500);

  updateControls();
}

void QTestGLViewSettings::setGLView(QTestGLView * view)
{
  glView_ = view;

  connect(glView_, &QTestGLView::viewPointChanged,
      [this]() {
        updateControls();
      });


  updateControls();
}

QTestGLView *  QTestGLViewSettings::glView() const
{
  return glView_;
}

void QTestGLViewSettings::onupdatecontrols()
{
  if ( !glView_ ) {
    setEnabled(false);
  }
  else {
    Base::populatecontrols();
    setEnabled(true);
  }
}

} /* namespace qgltest */
