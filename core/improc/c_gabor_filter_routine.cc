/*
 * c_gabor_filter_routine.cc
 *
 *  Created on: May 2, 2024
 *      Author: amyznikov
 */

#include "c_gabor_filter_routine.h"
#include <core/debug.h>

static void generate_filter_bank(std::vector<cv::Mat> & filters, int num_theta,
    const cv::Size & ksize, double sigma, double min_theta, double max_theta,
    double lambd, double gamma, double psi,
    int ktype)
{

  filters.clear();

  if( num_theta > 0 ) {

    filters.resize(num_theta);

    if ( ktype < 0 ) {
      ktype = CV_32F;
    }

    min_theta *= CV_PI / 180;
    max_theta *= CV_PI / 180;
    psi *= CV_PI / 180;

    if ( std::abs(lambd) < FLT_EPSILON ) {
      lambd = sigma * gamma;
    }

    for( int i = 0; i < num_theta; ++i ) {

      const double theta =
          min_theta + (max_theta - min_theta) * i / num_theta;

      cv::normalize(cv::getGaborKernel(ksize, sigma, theta, lambd, gamma, psi, ktype),
          filters[i], 1, 0,
          cv::NORM_L1);
    }
  }
}


void c_gabor_filter_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  // BIND_PCTRL(ctls, scales, "");
  BIND_CTRL(ctls, ksize, "ksize", "Size of the filter.");
  BIND_CTRL(ctls, sigma, "sigma [px]", "Standard deviation of the Gaussian envelope");
  BIND_CTRL(ctls, min_theta, "min_theta [deg]", "Orientation of the normal to the parallel stripes of a Gabor function, [deg]");
  BIND_CTRL(ctls, max_theta, "max_theta [deg]", "Orientation of the normal to the parallel stripes of a Gabor function, [deg]");
  BIND_CTRL(ctls, num_theta, "num_theta", "Num orientations");
  BIND_CTRL(ctls, lambd, "lambd", "Wavelength of the sinusoidal factor");
  BIND_CTRL(ctls, gamma, "gamma", "Spatial aspect ratio");
  BIND_CTRL(ctls, psi, "psi [deg]", "Phase offset");
  BIND_CTRL(ctls, ktype, "ktype", "Type of filter coefficients. It can be CV_32F or CV_64F");
  BIND_CTRL(ctls, display_kernels, "display_kernels", "Set checked to display filters bank");
}

bool c_gabor_filter_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, ksize);
    SERIALIZE_PROPERTY(settings, save, *this, sigma);
    SERIALIZE_PROPERTY(settings, save, *this, min_theta);
    SERIALIZE_PROPERTY(settings, save, *this, max_theta);
    SERIALIZE_PROPERTY(settings, save, *this, num_theta);
    SERIALIZE_PROPERTY(settings, save, *this, lambd);
    SERIALIZE_PROPERTY(settings, save, *this, gamma);
    SERIALIZE_PROPERTY(settings, save, *this, psi);
    SERIALIZE_PROPERTY(settings, save, *this, ktype);
    SERIALIZE_PROPERTY(settings, save, *this, display_kernels);
    return true;
  }
  return false;
}

bool c_gabor_filter_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{

  if( update_filter_bank_ ) {

    generate_filter_bank(filters_, num_theta_,
        ksize_, sigma_, min_theta_, max_theta_,
        lambd_, gamma_, psi_,
        ktype_);
  }


  if ( display_kernels_ ) {

    mask.release();

    if ( filters_.empty() ) {
      image.release();
    }
    else {

      image.create(ksize_.height, ksize_.width * filters_.size(),
          filters_.front().depth());

      cv::Mat & dst =
          image.getMatRef();

      for ( size_t i = 0, n = filters_.size(); i < n; ++i ) {
        filters_[i].copyTo(dst(cv::Rect(i * ksize_.width, 0, ksize_.width, ksize_.height)));
      }
    }
  }
  else {

    cv::Mat filtered_image, max_image;//, min_image;

    const double delta = 0;
    const cv::BorderTypes borderType = cv::BORDER_REPLICATE;

    for ( size_t i = 0, n = filters_.size(); i < n; ++i ) {

      if( max_image.empty() ) {
        cv::filter2D(image, max_image, -1, filters_[i], cv::Point(-1, -1), delta, borderType);
        //max_image.copyTo(min_image);
      }
      else {
        cv::filter2D(image, filtered_image, -1, filters_[i], cv::Point(-1,-1), delta, borderType);
        cv::max(filtered_image, max_image, max_image);
        //cv::min(filtered_image, min_image, min_image);
      }
    }

    //cv::subtract(max_image, min_image, image);
    image.move(max_image);
  }


  return true;
}
