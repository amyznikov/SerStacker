/*
 * c_hessian_routine.cc
 *
 *  Created on: May 31, 2024
 *      Author: amyznikov
 */

#include "c_hessian_routine.h"


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



static void compute_second_sobel_derivatives(cv::InputArray src,
    cv::OutputArray gxx,
    cv::OutputArray gxy,
    cv::OutputArray gyy,
    int ddepth,
    int borderType)
{
  static thread_local cv::Mat Kx1, Ky1, Kx2, Ky2;
  if( Kx1.empty() ) {

    cv::getDerivKernels(Kx1, Ky1, 1, 1, 3, true, CV_32F);
    cv::getDerivKernels(Kx2, Ky2, 2, 0, 3, true, CV_32F);
    //    Kx1 *= M_SQRT2;
    //    Ky1 *= M_SQRT2;
  }

  if( ddepth < 0 ) {
    ddepth = std::max(src.depth(), CV_32F);
  }

  cv::sepFilter2D(src, gxx, ddepth, Kx2, Ky2, cv::Point(-1, -1), 0, borderType);
  cv::sepFilter2D(src, gyy, ddepth, Ky2, Kx2, cv::Point(-1, -1), 0, borderType);
  cv::sepFilter2D(src, gxy, ddepth, Kx1, Ky1, cv::Point(-1, -1), 0, borderType);
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
    case OutputMinEigenValues: {

      // Eigenvectors and eigenvalues of real symmetric matrices.pdf
      // A = | a b |
      //     | b d |
      //

      const cv::Size image_size =
          image.size();

      const int cn =
          image.channels();

      image.create(image_size, CV_MAKETYPE(CV_32F, cn));

      cv::Mat & dest =
          image.getMatRef();

      for ( int y = 0; y < image_size.height; ++y ) {

        const float * gxxp = gxx.ptr<const float>(y);
        const float * gxyp = gxy.ptr<const float>(y);
        const float * gyyp = gyy.ptr<const float>(y);
        float * dstp = dest.ptr<float>(y);

        for ( int x = 0; x < image_size.width; ++x ) {

          for ( int c = 0; c < cn; ++c ) {

            const int i =
                x * cn + c;

            const float a =
                gxxp[i];

            const float b =
                gxyp[i];

            const float d =
                gyyp[i];

            const float T =
                a + d;

            const float D =
                std::sqrt((a - d) * (a - d) + 4 * b * b);

            const float mu1 =
                (T + D) * 0.5f;

            const float mu2 =
                (T - D) * 0.5f;

            if ( output_type_ == OutputMaxEigenValues ) {

              if ( std::abs(mu1) > std::abs(mu2) ) {
                dstp[i] = mu1;
              }
              else {
                dstp[i] = mu2;
              }

            }
            else {

              if ( std::abs(mu1) < std::abs(mu2) ) {
                dstp[i] = mu1;
              }
              else {
                dstp[i] = mu2;
              }

            }

          }
        }
      }

      break;
    }

    case OutputDet:
    default:
      cv::subtract(gxx.mul(gyy), gxy.mul(gxy), image);
      break;
  }


  return true;
}

