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
      { c_hessian_routine::OutputGx, "Gx", "Gx" },
      { c_hessian_routine::OutputGy, "Gy", "Gy" },
      { c_hessian_routine::OutputGxx, "Gxx", "Gxx" },
      { c_hessian_routine::OutputGyy, "Gyy", "Gyy" },
      { c_hessian_routine::OutputGxy, "Gxy", "Gxy" },
      { c_hessian_routine::OutputGxxGyy, "GxxGyy", "Gxx*Gyy" },
      { c_hessian_routine::OutputGxyGxy, "GxyGxy", "Gxy*Gxy" },
      { c_hessian_routine::OutputDet, "Determinant", "Det = Gxx*Gyy - Gxy*Gxy" },
      { c_hessian_routine::OutputLap, "Laplacian", "Laplacian = Gxx+Gyy" },
      { c_hessian_routine::OutputDetLap, "DetLaplacian", "DetLaplacian = -Laplacian * Determinant" },
      { c_hessian_routine::OutputGaussianCurvature, "GaussianCurvature", "GaussianCurvature = Det / (1 + Gx^2 + Gy^2)^2" },

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

  switch (output_type_) {
    case OutputGx: {
      compute_sobel_gradients(image.getMat(),
          image,
          cv::noArray(),
          CV_32F,
          border_type_);
      break;
    }
    case OutputGy: {
      compute_sobel_gradients(image.getMat(),
          cv::noArray(),
          image,
          CV_32F,
          border_type_);
      break;
    }
    case OutputGxx: {
      compute_second_sobel_derivatives(image.getMat(),
          image,
          cv::noArray(),
          cv::noArray(),
          CV_32F,
          border_type_);
      break;
    }

    case OutputGyy: {
      compute_second_sobel_derivatives(image.getMat(),
          cv::noArray(),
          image,
          cv::noArray(),
          CV_32F,
          border_type_);
      break;
    }

    case OutputGxy: {
      compute_second_sobel_derivatives(image.getMat(),
          cv::noArray(),
          cv::noArray(),
          image,
          CV_32F,
          border_type_);
      break;
    }
    case OutputGxxGyy: {
      cv::Mat gxx, gyy;
      compute_second_sobel_derivatives(image.getMat(),
          gxx,
          gyy,
          cv::noArray(),
          CV_32F,
          border_type_);
      cv::multiply(gxx, gyy, image);
      break;
    }
    case OutputGxyGxy: {
      cv::Mat gxy;
      compute_second_sobel_derivatives(image.getMat(),
          cv::noArray(),
          cv::noArray(),
          gxy,
          CV_32F,
          border_type_);
      cv::multiply(gxy, gxy, image);
      break;
    }

    case OutputLap: {
      cv::Mat gxx, gyy;
      compute_second_sobel_derivatives(image.getMat(),
          gxx,
          gyy,
          cv::noArray(),
          CV_32F,
          border_type_);
      cv::add(gxx, gyy, image);
      break;
    }

    case OutputDetLap: {
      cv::Mat gxx, gyy, gxy;
      cv::Mat det, lap;

      compute_second_sobel_derivatives(image.getMat(),
          gxx,
          gyy,
          gxy,
          CV_32F,
          border_type_);

      cv::subtract(gxx.mul(gyy), gxy.mul(gxy), det);
      cv::add(gxx, gyy, lap);
      det.setTo(0, lap > 0);
      det.copyTo(image);
//
////      lap.setTo(0, lap > 0);
////      det.setTo(0, det < 0);
//      cv::multiply(det, lap, image, -1);
      break;
    }

    case OutputGaussianCurvature: {
      cv::Mat gx, gy, gxx, gyy, gxy;
      cv::Mat grad, det;

      compute_sobel_gradients(image.getMat(),
          gx,
          gy,
          CV_32F,
          border_type_);

      cv::add(gx.mul(gx), gy.mul(gy), grad);
      cv::add(grad, cv::Scalar::all(1), grad);

      compute_second_sobel_derivatives(image.getMat(),
          gxx,
          gyy,
          gxy,
          CV_32F,
          border_type_);

      cv::subtract(gxx.mul(gyy), gxy.mul(gxy), det);

      //cv::divide(det, grad.mul(grad), image);
      cv::divide(det, grad, image);
      break;
    }

    case OutputMaxEigenValues: {
      cv::Mat gxx, gyy, gxy;

      compute_second_sobel_derivatives(image.getMat(),
          gxx,
          gyy,
          gxy,
          CV_32F,
          border_type_);

      compute_hessian_eigenvalues(gxx, gxy, gyy,
          image,
          cv::noArray());
      break;
    }
    case OutputMinEigenValues: {
      cv::Mat gxx, gyy, gxy;

      compute_second_sobel_derivatives(image.getMat(),
          gxx,
          gyy,
          gxy,
          CV_32F,
          border_type_);

      compute_hessian_eigenvalues(gxx, gxy, gyy,
          cv::noArray(),
          image);
      break;
    }

    case OutputDet:
    default: {

      cv::Mat gxx, gyy, gxy;

      compute_second_sobel_derivatives(image.getMat(),
          gxx,
          gyy,
          gxy,
          CV_32F,
          border_type_);

      cv::subtract(gxx.mul(gyy), gxy.mul(gxy),
          image);
      break;
    }
  }


  return true;
}

