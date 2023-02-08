/*
 * QFrameAccumulationOptions.cc
 *
 *  Created on: Jul 13, 2021
 *      Author: amyznikov
 */

#include "QFrameAccumulationOptions.h"
#include <gui/widgets/style.h>
#include <core/debug.h>


#define ICON_check_all      ":/qstackingoptions/icons/check_all"

QFrameAccumulationOptions::QFrameAccumulationOptions(QWidget * parent) :
    Base("QFrameAccumulationOptions", parent)
{
  accumulation_method_ctl =
      add_enum_combobox<frame_accumulation_method>(
          "Accumulation Method:",
          [this](frame_accumulation_method v) {
            if ( options_ && v != options_->accumulation_method ) {
              options_->accumulation_method = v;
              updatecurrentoptionswidget();
              Q_EMIT parameterChanged();
            }
          });

  stack_ctl = new QStackedWidget(this);
  stack_ctl->setSizePolicy(QSizePolicy::Policy::Maximum, QSizePolicy::Policy::Maximum);

  stack_ctl->addWidget(lpg_options_ctl =
      new QLPGSharpnessMeasureOptions(this));

  stack_ctl->addWidget(focus_stack_options_ctl =
      new QFocusStackingOptions(this));

  form->addRow(stack_ctl);

  connect(lpg_options_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  connect(focus_stack_options_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  updateControls();
}


void QFrameAccumulationOptions::set_accumulation_options(c_frame_accumulation_options * options)
{
  this->options_ = options;
  lpg_options_ctl->set_accumulation_options(options);
  focus_stack_options_ctl->set_accumulation_options(options);
  updateControls();
}

const c_frame_accumulation_options * QFrameAccumulationOptions::accumulation_options() const
{
  return this->options_;
}

void QFrameAccumulationOptions::onupdatecontrols()
{
  if ( !options_ ) {
    setEnabled(false);
  }
  else {
    accumulation_method_ctl->setCurrentItem(options_->accumulation_method);
    updatecurrentoptionswidget();
    setEnabled(true);
  }

  Base::onupdatecontrols();
}

void QFrameAccumulationOptions::updatecurrentoptionswidget()
{
  switch (accumulation_method_ctl->currentItem()) {
    case frame_accumulation_weighted_average:
      lpg_options_ctl->updateControls();
      stack_ctl->setCurrentWidget(lpg_options_ctl);
      stack_ctl->setEnabled(true);
      break;
    case frame_accumulation_focus_stack:
      focus_stack_options_ctl->updateControls();
      stack_ctl->setCurrentWidget(focus_stack_options_ctl);
      stack_ctl->setEnabled(true);
      break;
    default:
      stack_ctl->setEnabled(false);
      break;
  }
}


QLPGSharpnessMeasureOptions::QLPGSharpnessMeasureOptions(QWidget * parent) :
    Base("QLPGSharpnessMeasureOptions", parent)
{
  k_ctl =
      add_numeric_box<double>("k:",
          [this](double v) {
            if ( options_ && v != options_->m_.k() ) {
              options_->m_.set_k(v);
              Q_EMIT parameterChanged();
            }
          });

  dscale_ctl =
      add_numeric_box<double>("dscale:",
          [this](double v) {
            if ( options_ && v != options_->m_.dscale() ) {
              options_->m_.set_dscale(v);
              Q_EMIT parameterChanged();
            }
          });

  uscale_ctl =
      add_numeric_box<double>("uscale:",
          [this](double v) {
            if ( options_ && v != options_->m_.uscale() ) {
              options_->m_.set_uscale(v);
              Q_EMIT parameterChanged();
            }
          });

  square_ctl =
      add_checkbox("squared:",
          [this](bool checked) {
            if ( options_ &&  options_->m_.squared() != checked ) {
              options_->m_.set_squared(checked);
              Q_EMIT parameterChanged();
            }
          });

  avgc_ctl =
      add_checkbox("avgc:",
          [this](bool checked) {
            if ( options_ &&  options_->m_.avgchannel() != checked ) {
              options_->m_.set_avgchannel(checked);
              Q_EMIT parameterChanged();
            }
          });

  updateControls();
}


void QLPGSharpnessMeasureOptions::set_accumulation_options(c_frame_accumulation_options * options)
{
  options_ = options;
  updateControls();
}

const c_frame_accumulation_options * QLPGSharpnessMeasureOptions::accumulation_options() const
{
  return options_;
}

void QLPGSharpnessMeasureOptions::onupdatecontrols()
{
  if ( !options_ ) {
    setEnabled(false);
  }
  else {

    k_ctl->setValue(options_->m_.k());
    dscale_ctl->setValue(options_->m_.dscale());
    uscale_ctl->setValue(options_->m_.uscale());
    square_ctl->setChecked(options_->m_.squared());
    avgc_ctl->setChecked(options_->m_.avgchannel());

    setEnabled(true);
  }
}


QFocusStackingOptions::QFocusStackingOptions(QWidget * parent) :
    Base("QFocusStackingOptions", parent)
{
}

void QFocusStackingOptions::set_accumulation_options(c_frame_accumulation_options * options)
{
  options_ = options;
  updateControls();
}

const c_frame_accumulation_options * QFocusStackingOptions::accumulation_options() const
{
  return options_;
}

void QFocusStackingOptions::onupdatecontrols()
{
  if ( !options_ ) {
    setEnabled(false);
  }
  else {
    setEnabled(true);
  }

  Base::onupdatecontrols();
}

