/*
 * c_image_gradient_routine.cc
 *
 *  Created on: Jan 11, 2024
 *      Author: amyznikov
 */

#include "c_image_gradient_routine.h"
#include <core/proc/gradient.h>
#include <core/debug.h>

const std::string & c_image_gradient_routine::input_image_name() const
{
  return input_image_name_;
}

void c_image_gradient_routine::set_input_image_name(const std::string & v)
{
  input_image_name_ = v;
}

const std::string & c_image_gradient_routine::output_image_name() const
{
  return output_image_name_;
}

void c_image_gradient_routine::set_output_image_name(const std::string & v)
{
  output_image_name_ = v;
}

void c_image_gradient_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_CTRL(ctls, input_image_name, "input_image", "input image name");
  BIND_CTRL(ctls, output_image_name, "output_image", "output image name");
}

bool c_image_gradient_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, input_image_name);
    SERIALIZE_PROPERTY(settings, save, *this, output_image_name);
    return true;
  }

  return false;
}

bool c_image_gradient_routine::process(c_video_frame * frame)
{

  CF_ERROR("ERROR: the code of c_image_gradient_routine is broken now. Fix it and try again");
  return false;

//  cv::Mat input_image, output_image;
//  cv::Mat input_mask, output_mask;
//
//  if ( !frame->get_image(input_image_name_, input_image, input_mask) ) {
//    CF_ERROR("frame->get_image('%s') fails", input_image_name_.c_str());
//    return false;
//  }
//
//  compute_gradient(input_image, output_image,
//      1, 0,
//      2);
//
//  frame->set_image(output_image_name_, output_image, output_mask);
//
//  return true;
}
