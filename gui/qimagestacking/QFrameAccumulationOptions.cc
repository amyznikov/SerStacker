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
      add_enum_combobox<frame_accumulation_method>("Accumulation Method:",
          "",
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
  lpg_options_ctl->set_measure_options(options ? &options->m_ : nullptr);
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



QFocusStackingOptions::QFocusStackingOptions(QWidget * parent) :
    Base("QFocusStackingOptions", parent)
{

  fusion_ctl =
      add_enum_combobox<c_laplacian_pyramid_focus_stacking::fusing_policy>("fusion method:",
          "",
          [this](c_laplacian_pyramid_focus_stacking::fusing_policy v) {
            if ( options_ && options_->fs_.fusing_policy != v ) {
              options_->fs_.fusing_policy = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](c_laplacian_pyramid_focus_stacking::fusing_policy * v) {
            if ( options_ ) {
              *v = options_->fs_.fusing_policy;
              return true;

            }
            return false;
          });


  avgchannel_ctl =
      add_checkbox("Average color channels:",
          "",
          [this](bool checked) {
            if ( options_ && options_->fs_.avgchannel != checked ) {
              options_->fs_.avgchannel = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              * checked = options_->fs_.avgchannel;
              return true;
            }
            return false;
          });

  enable_inpaint_ctl =
      add_checkbox("Inpaint mask holes",
          "",
          [this](bool checked) {
            if ( options_ && options_->fs_.inpaint_mask_holes != checked ) {
              options_->fs_.inpaint_mask_holes = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              * checked = options_->fs_.inpaint_mask_holes;
              return true;
            }
            return false;
          });

  kradius_ctl =
      add_numeric_box<int>("kradius [pix]",
          "",
          [this](int v) {
            if ( options_ && options_->fs_.kradius != v ) {
              options_->fs_.kradius = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * v) {
            if ( options_ ) {
              *v = options_->fs_.kradius;
              return true;
            }
            return false;
          });

  ksigma_ctl =
      add_numeric_box<double>("ksigma [pix]",
          "",
          [this](double v) {
            if ( options_ && options_->fs_.ksigma != v ) {
              options_->fs_.ksigma = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) -> bool {
            if ( options_ ) {
              *v = options_->fs_.ksigma;
              return true;
            }
            return false;
          });


  updateControls();
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

