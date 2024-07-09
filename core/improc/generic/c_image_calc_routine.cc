/*
 * c_image_calc_routine.cc
 *
 *  Created on: May 9, 2023
 *      Author: amyznikov
 */

#include "c_image_calc_routine.h"
#include <core/io/load_image.h>
#include <core/ssprintf.h>



template<>
const c_enum_member * members_of<c_image_calc_routine::Function>()
{
  static const c_enum_member members[] = {
      { c_image_calc_routine::Function_None, "None", " No function, return just current image"},
      { c_image_calc_routine::Function_add, "add", "cv::add(Ic, Ia)"},
      { c_image_calc_routine::Function_subtract, "subtract", "cv::subtract(Ic, Ia"},
      { c_image_calc_routine::Function_absdiff, "absdiff", "cv::absdiff(Ic, Ia"},
      { c_image_calc_routine::Function_multiply, "multiply", "cv::multiply(Ic, Ia"},
      { c_image_calc_routine::Function_divide, "divide", "cv::divide(Ic, Ia)"},
      { c_image_calc_routine::Function_max, "max", "cv::max(Ic, Ia)"},
      { c_image_calc_routine::Function_min, "min", "cv::min(Ic, Ia)"},

      {c_image_calc_routine::Function_None}
  };

  return members;
}


void c_image_calc_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, function, "Select function to apply\n");
  BIND_PCTRL(ctls, argname, "Saved artifact name");
}

bool c_image_calc_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, function);
    SERIALIZE_PROPERTY(settings, save, *this, argname);
    return true;
  }
  return false;
}


bool c_image_calc_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( function_ == Function_None ) {
    return true;
  }

  if( argname_.empty() ) {
    CF_ERROR("No second image name specified");
    return false;
  }

  cv::Mat second_image;
  cv::Mat second_mask;

  if ( !base::get_artifact(argname_, second_image, second_mask)) {
    CF_ERROR("get_artifact('%s') fails", argname_.c_str());
    return false;
  }

  if( image.size() != second_image.size() ) {
    CF_ERROR("Current image (%dx%d) and argument image (%dx%d) sizes not match",
        image.cols(), image.rows(), second_image.cols, second_image.rows);
    return false;
  }

  if( image.channels() != second_image.channels() ) {
    CF_ERROR("Current image channels (%dd) and argument image channels (%d) not equal",
        image.channels(), second_image.channels());
    return false;
  }

  switch (function_) {
    case Function_None:
      break;
    case Function_add:
      cv::add(image.getMat(), second_image, image, cv::noArray(),
          std::max(image.depth(), second_image.depth()));
      break;
    case Function_subtract:
      cv::subtract(image.getMat(), second_image, image, cv::noArray(),
          std::max(image.depth(), second_image.depth()));
      break;
    case Function_absdiff:
      if ( image.depth() == second_image.depth() ) {
        cv::absdiff(image.getMat(), second_image, image);
      }
      else if (image.depth() > second_image.depth()) {
        cv::Mat tmp;
        image.getMat().convertTo(tmp, second_image.depth());
        cv::absdiff(tmp, second_image, image);
      }
      else {
        cv::Mat tmp;
        second_image.convertTo(tmp, image.depth());
        cv::absdiff(image.getMat(), tmp, image);
      }
      break;
    case Function_multiply:
      cv::multiply(image.getMat(), second_image, image, 1,
          std::max(image.depth(), second_image.depth()));
      break;
    case Function_divide:
      cv::divide(image.getMat(), second_image, image, 1,
          std::max(image.depth(), second_image.depth()));
      break;
    case Function_max:
      cv::max(image.getMat(), second_image, image.getMatRef());
      break;
    case Function_min:
      cv::min(image.getMat(), second_image, image.getMatRef());
      break;
    default:
      CF_ERROR("Unhandled function %d requested", function_);
      return false;
  }

  return true;
}


