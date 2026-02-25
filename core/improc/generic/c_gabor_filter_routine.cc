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

void c_gabor_filter_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "ksize", ctx(&this_class::_ksize), "Size of the filter");
   ctlbind(ctls, "sigma", ctx(&this_class::_sigma), "Standard deviation of the Gaussian envelope");
   ctlbind(ctls, "min_theta", ctx(&this_class::_min_theta), "Orientation of the normal to the parallel stripes of a Gabor function, [deg]");
   ctlbind(ctls, "max_theta", ctx(&this_class::_max_theta), "Orientation of the normal to the parallel stripes of a Gabor function, [deg]");
   ctlbind(ctls, "num_theta", ctx(&this_class::_num_theta), "Num orientations");
   ctlbind(ctls, "lambd", ctx(&this_class::_lambd), "Wavelength of the sinusoidal factor");
   ctlbind(ctls, "gamma", ctx(&this_class::_gamma), "Spatial aspect ratio");
   ctlbind(ctls, "psi", ctx(&this_class::_psi), "Phase offset");
   ctlbind(ctls, "ktype", ctx(&this_class::_ktype), "Type of filter coefficients. It can be CV_32F or CV_64F");
   ctlbind(ctls, "display_kernels", ctx(&this_class::_display_kernels), "");
   ctlbind(ctls, "update_filter_bank", ctx(&this_class::_update_filter_bank), "Set checked to display filters bank");
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

  if( _update_filter_bank ) {

    generate_filter_bank(_filters, _num_theta,
        _ksize, _sigma, _min_theta, _max_theta,
        _lambd, _gamma, _psi,
        _ktype);
  }


  if ( _display_kernels ) {

    mask.release();

    if ( _filters.empty() ) {
      image.release();
    }
    else {

      image.create(_ksize.height, _ksize.width * _filters.size(),
          _filters.front().depth());

      cv::Mat & dst =
          image.getMatRef();

      for ( size_t i = 0, n = _filters.size(); i < n; ++i ) {
        _filters[i].copyTo(dst(cv::Rect(i * _ksize.width, 0, _ksize.width, _ksize.height)));
      }
    }
  }
  else {

    cv::Mat filtered_image, max_image;//, min_image;

    const double delta = 0;
    const cv::BorderTypes borderType = cv::BORDER_REPLICATE;

    for ( size_t i = 0, n = _filters.size(); i < n; ++i ) {

      if( max_image.empty() ) {
        cv::filter2D(image, max_image, -1, _filters[i], cv::Point(-1, -1), delta, borderType);
        //max_image.copyTo(min_image);
      }
      else {
        cv::filter2D(image, filtered_image, -1, _filters[i], cv::Point(-1,-1), delta, borderType);
        cv::max(filtered_image, max_image, max_image);
        //cv::min(filtered_image, min_image, min_image);
      }
    }

    //cv::subtract(max_image, min_image, image);
    image.move(max_image);
  }


  return true;
}
