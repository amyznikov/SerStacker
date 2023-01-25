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

//static const char borderless_style[] = ""
//    "QToolButton { border: none; } "
//    "QToolButton::menu-indicator { image: none; }"
//    "";

//static QIcon getIcon(const QString & name)
//{
//  return QIcon(QString(":/qstackingoptions/icons/%1").arg(name));
//}

QFrameAccumulationOptions::QFrameAccumulationOptions(QWidget * parent) :
    Base("QFrameAccumulationOptions", parent)
{

  accumulation_method_ctl = add_enum_combobox<frame_accumulation_method>(
      "Accumulation Method:",
      [this](frame_accumulation_method v) {
        if ( options_ && v != options_->accumulation_method ) {
          options_->accumulation_method = v;
          emit parameterChanged();
        }
      });

//  lksize_ctl =
//      add_numeric_box<int>("lksize:",
//          [this](int v) {
//            if ( options_ && v != options_->lksize ) {
//              options_->lksize = v;
//              emit parameterChanged();
//            }
//          });
//
//  scale_size_ctl =
//      add_numeric_box<int>("scale:",
//          [this](int v) {
//            if ( options_ && v != options_->scale_size ) {
//              options_->scale_size = v;
//              emit parameterChanged();
//            }
//          });
//
  lw_ctl =
      add_numeric_box<double>("lw:",
          [this](double v) {
            if ( options_ && v != options_->lpg_.laplacian_weight() ) {
              options_->lpg_.set_laplacian_weight(v);
              emit parameterChanged();
            }
          });

  gw_ctl =
      add_numeric_box<double>("gw:",
          [this](double v) {
            if ( options_ && v != options_->lpg_.gradient_weight() ) {
              options_->lpg_.set_gradient_weight(v);
              emit parameterChanged();
            }
          });

  dscale_ctl =
      add_numeric_box<double>("dscale:",
          [this](double v) {
            if ( options_ && v != options_->lpg_.dscale() ) {
              options_->lpg_.set_dscale(v);
              emit parameterChanged();
            }
          });

  uscale_ctl =
      add_numeric_box<double>("uscale:",
          [this](double v) {
            if ( options_ && v != options_->lpg_.uscale() ) {
              options_->lpg_.set_uscale(v);
              emit parameterChanged();
            }
          });

  avgc_ctl =
      add_checkbox("avgc:",
          [this](bool checked) {
            if ( options_ &&  options_->lpg_.avgchannel() != checked ) {
              options_->lpg_.set_avgchannel(checked);
              emit parameterChanged();
            }
          });

  setEnabled(false);
}


void QFrameAccumulationOptions::set_accumulation_options(c_frame_accumulation_options * options)
{
  this->options_ = options;
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

    lw_ctl->setValue(options_->lpg_.laplacian_weight());
    gw_ctl->setValue(options_->lpg_.gradient_weight());
    dscale_ctl->setValue(options_->lpg_.dscale());
    uscale_ctl->setValue(options_->lpg_.uscale());
    avgc_ctl->setChecked(options_->lpg_.avgchannel());

    setEnabled(true);
  }

  Base::onupdatecontrols();
}
