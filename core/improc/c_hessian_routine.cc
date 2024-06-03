/*
 * c_hessian_routine.cc
 *
 *  Created on: May 31, 2024
 *      Author: amyznikov
 */

#include "c_hessian_routine.h"
#include <core/proc/gradient.h>
#include <core/debug.h>

template<>
const c_enum_member* members_of<c_hessian_routine::OutputType>()
{

  static const c_enum_member members[] = {
      { c_hessian_routine::OutputGxx, "Gxx", "Gxx" },
      { c_hessian_routine::OutputGyy, "Gyy", "Gyy" },
      { c_hessian_routine::OutputGxy, "Gxy", "Gxy" },
      { c_hessian_routine::OutputDet, "Determinant", "Determinant" },
      { c_hessian_routine::OutputMaxEigenValues, "MaxEigenValues", "Max Eigen Values" },
      { c_hessian_routine::OutputMinEigenValues, "MinEigenValues", "Min Eigen Values" },

      { c_hessian_routine::OutputDet },
  };

  return members;
}

void c_hessian_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, output_type, "Output type");
  BIND_PCTRL(ctls, border_type, "Border type");
}

bool c_hessian_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, output_type);
    SERIALIZE_PROPERTY(settings, save, *this, border_type);
    return true;
  }
  return false;
}

bool c_hessian_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat gxx, gxy, gyy;

  INSTRUMENT_REGION("hessian");

  compute_second_sobel_derivatives(image.getMat(),
      gxx,
      gxy,
      gyy,
      CV_32F,
      border_type_);


  switch (output_type_) {
    case OutputGxx:
      gxx.copyTo(image);
      break;
    case OutputGyy:
      gyy.copyTo(image);
      break;
    case OutputGxy:
      gxy.copyTo(image);
      break;
    case OutputMaxEigenValues:
      compute_hessian_eigenvalues(gxx, gxy, gyy, image, cv::noArray());
      break;
    case OutputMinEigenValues:
      compute_hessian_eigenvalues(gxx, gxy, gyy, cv::noArray(), image);
      break;
    case OutputDet:
    default:
      cv::subtract(gxx.mul(gyy), gxy.mul(gxy), image);
      break;
  }


  return true;
}

