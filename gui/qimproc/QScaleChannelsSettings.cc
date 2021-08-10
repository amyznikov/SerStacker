/*
 * QScaleChannelsSettings.cc
 *
 *  Created on: Aug 10, 2021
 *      Author: amyznikov
 */

#include "QScaleChannelsSettings.h"

const QScaleChannelsSettings::ClassFactory QScaleChannelsSettings::classFactory;

//bool fromString(const QString & text, cv::Scalar * v)
//{
//  const QByteArray a = text.toUtf8().data();
//  const char * s = a.data();
//
//  const int n = sscanf(s, "%lf[ :;]%lf[ :;]%lf[ :;]%lf[ :;]",
//      &v->val[0],
//      &v->val[1],
//      &v->val[2],
//      &v->val[3]);
//
//  return n > 0;
//}
//
//QString ToString(const cv::Scalar & v)
//{
//  return QString("%1;%2;%3;%4").arg(v[0]).arg(v[1]).arg(v[2]).arg(v[3]);
//}

QScaleChannelsSettings::QScaleChannelsSettings(const c_scale_channels_routine::ptr & routine, QWidget * parent)
  : Base(&classFactory, routine, parent)
{
  bias_r_ctl = add_numeric_box("R Bias:",
      &c_scale_channels_routine::bias_r,
      &c_scale_channels_routine::set_bias_r);

  stretch_r_ctl = add_numeric_box("R Stretch: ",
      &c_scale_channels_routine::stretch_r,
      &c_scale_channels_routine::set_stretch_r);

  bias_g_ctl = add_numeric_box("G Bias:",
      &c_scale_channels_routine::bias_g,
      &c_scale_channels_routine::set_bias_g);

  stretch_g_ctl = add_numeric_box("G Stretch: ",
      &c_scale_channels_routine::stretch_g,
      &c_scale_channels_routine::set_stretch_g);

  bias_b_ctl = add_numeric_box("B Bias:",
      &c_scale_channels_routine::bias_b,
      &c_scale_channels_routine::set_bias_b);

  stretch_b_ctl = add_numeric_box("B Stretch: ",
      &c_scale_channels_routine::stretch_b,
      &c_scale_channels_routine::set_stretch_b);

  bias_a_ctl = add_numeric_box("A Bias:",
      &c_scale_channels_routine::bias_a,
      &c_scale_channels_routine::set_bias_a);

  stretch_a_ctl = add_numeric_box("A Stretch: ",
      &c_scale_channels_routine::stretch_a,
      &c_scale_channels_routine::set_stretch_a);

  updateControls();
}

void QScaleChannelsSettings::onupdatecontrols()
{
  if ( !routine_ ) {
    setEnabled(false);
  }
  else {

    bias_r_ctl->setText(toString(routine_->bias_r()));
    stretch_r_ctl->setText(toString(routine_->stretch_r()));

    bias_g_ctl->setText(toString(routine_->bias_g()));
    stretch_g_ctl->setText(toString(routine_->stretch_g()));

    bias_b_ctl->setText(toString(routine_->bias_b()));
    stretch_b_ctl->setText(toString(routine_->stretch_b()));

    bias_a_ctl->setText(toString(routine_->bias_a()));
    stretch_a_ctl->setText(toString(routine_->stretch_a()));

    setEnabled(true);
  }

  Base::onupdatecontrols();
}
