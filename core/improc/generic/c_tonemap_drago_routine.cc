/*
 * c_tonemap_drago_routine.cc
 *
 *  Created on: Oct 11, 2026
 *      Author: amyznikov
 */

#include "c_tonemap_drago_routine.h"
#include <core/proc/divide.h>
#include <core/proc/multiply.h>

bool c_tonemap_drago_routine::serialize(c_config_setting settings, bool save)
{
  float gamma = 1.0f;
  float saturation = 1.0f;
  float bias = 0.85f;

  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, gamma);
    SERIALIZE_OPTION(settings, save, *this, saturation);
    SERIALIZE_OPTION(settings, save, *this, bias);
    return true;
  }
  return false;
}

void c_tonemap_drago_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "gamma", CTL_CONTEXT(ctx, gamma), "");
  ctlbind(ctls, "saturation", CTL_CONTEXT(ctx, saturation), "");
  ctlbind_slider_spinbox(ctls, "bias", CTL_CONTEXT(ctx, bias), 0.0, 1.0, 1e-3, "");
}

bool c_tonemap_drago_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( !tonemap || gamma != old_gamma || saturation != old_saturation || old_bias != bias) {
    tonemap = cv::createTonemapDrago(gamma, saturation, bias);
    old_gamma = gamma;
    old_saturation = saturation;
    old_bias = bias;
    if ( !tonemap ) {
      CF_ERROR("cv::createTonemapDrago() fails");
      return false;
    }
  }

  const cv::Mat src = image.getMat();

  std::vector<cv::Mat> channels;
  cv::split(src, channels);
  cv::Mat Y_src = (channels[0] + channels[1] + channels[2]) / 3.0f;

  cv::Mat Y_gray;
  std::vector<cv::Mat> gray_layers = {Y_src, Y_src, Y_src};
  cv::merge(gray_layers, Y_gray);

  tonemap->process(Y_gray, Y_gray);

  std::vector<cv::Mat> dst_layers;
  cv::split(Y_gray, dst_layers);
  cv::Mat Y_dst = dst_layers[0];

  // K = Y_dst / Y_src
  cv::Mat K;
  divideImages(Y_dst, Y_src, K);

  cv::Mat processed;
  multiplyImages(src, K, processed, src.depth());

  image.move(processed);

//  const cv::Mat src = image.getMat();
//
//  cv::Mat Y_src, Y_gray, Y_dst, K, processed;
//  cv::cvtColor(image, Y_src, cv::COLOR_BGR2GRAY);
//  cv::cvtColor(Y_src, Y_gray, cv::COLOR_GRAY2BGR);
//
//  tonemap->process(Y_gray, Y_gray);
//
//  cv::cvtColor(Y_gray, Y_dst, cv::COLOR_BGR2GRAY);
//
//  divideImages(Y_dst, Y_src, K);
//  multiplyImages(src, K, processed, src.depth());
//  image.move(processed);

  return true;
}
