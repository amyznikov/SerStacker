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
  return _input_image_name;
}

void c_image_gradient_routine::set_input_image_name(const std::string & v)
{
  _input_image_name = v;
}

const std::string & c_image_gradient_routine::output_image_name() const
{
  return _output_image_name;
}

void c_image_gradient_routine::set_output_image_name(const std::string & v)
{
  _output_image_name = v;
}

//void c_image_gradient_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
//{
//  BIND_CTRL(ctls, input_image_name, "input_image", "input image name");
//  BIND_CTRL(ctls, output_image_name, "output_image", "output image name");
//}

bool c_image_gradient_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, input_image_name);
    SERIALIZE_PROPERTY(settings, save, *this, output_image_name);
    return true;
  }

  return false;
}

bool c_image_gradient_routine::process(c_data_frame::sptr & frame)
{

  cv::Mat input_image, output_image;
  cv::Mat input_mask, output_mask;

  if ( !frame->get_image(_input_image_name, input_image, input_mask, cv::noArray()) ) {
    CF_ERROR("frame->get_image('%s') fails", _input_image_name.c_str());
    return false;
  }

  compute_gradient(input_image, output_image,
      1, 0,
      2);

  frame->add_image(_output_image_name.empty() ? "GRADIENT" :
      _output_image_name,
      output_image,
      output_mask,
      cv::noArray());

  return true;
}
