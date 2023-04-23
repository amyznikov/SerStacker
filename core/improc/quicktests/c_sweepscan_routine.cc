/*
 * c_sweepscan_routine.cc
 *
 *  Created on: Apr 17, 2023
 *      Author: amyznikov
 */

#include "c_sweepscan_routine.h"
#include <core/debug.h>


void c_sweepscan_routine::set_output_type(OutputType v)
{
  return sm_.set_output_type(v);
}

c_sweepscan_routine::OutputType c_sweepscan_routine::output_type() const
{
  return sm_.output_type();
}

void c_sweepscan_routine::set_max_disparity(int v)
{
  return sm_.set_max_disparity(v);
}

int c_sweepscan_routine::max_disparity() const
{
  return sm_.max_disparity();
}

void c_sweepscan_routine::set_max_scale(int v)
{
  return sm_.set_max_scale(v);
}

void c_sweepscan_routine::set_ssflags(int v)
{
  return sm_.set_ssflags(v);
}

int c_sweepscan_routine::ssflags() const
{
  return sm_.ssflags();
}

void c_sweepscan_routine::set_ss_sigma(double v)
{
  return sm_.set_ss_sigma(v);
}

double c_sweepscan_routine::ss_sigma() const
{
  return sm_.ss_sigma();
}


void c_sweepscan_routine::set_ss_radius(int v)
{
  return sm_.set_ss_radius(v);
}

int c_sweepscan_routine::ss_radius() const
{
  return sm_.ss_radius();
}

int c_sweepscan_routine::max_scale() const
{
  return sm_.max_scale();
}

void c_sweepscan_routine::set_kernel_sigma(double v)
{
  return sm_.set_kernel_sigma(v);
}

double c_sweepscan_routine::kernel_sigma() const
{
  return sm_.kernel_sigma();
}

void c_sweepscan_routine::set_kernel_radius(int v)
{
  return sm_.set_kernel_radius(v);
}

int c_sweepscan_routine::kernel_radius() const
{
  return sm_.kernel_radius();
}

void c_sweepscan_routine::set_normalization_scale(int v)
{
  return sm_.set_normalization_scale(v);
}

int c_sweepscan_routine::normalization_scale() const
{
  return sm_.normalization_scale();
}

void c_sweepscan_routine::set_debug_directory(const std::string & v)
{
  return sm_.set_debug_directory(v);
}

const std::string& c_sweepscan_routine::debug_directory() const
{
  return sm_.debug_directory();
}

void c_sweepscan_routine::set_debug_points(const std::vector<cv::Point> & v)
{
  return sm_.set_debug_points(v);
}

const std::vector<cv::Point>& c_sweepscan_routine::debug_points() const
{
  return sm_.debug_points();
}

void c_sweepscan_routine::set_lpg_k(double v)
{
  return sm_.lpg().set_k(v);
}

double c_sweepscan_routine::lpg_k() const
{
  return sm_.lpg().k();
}

void c_sweepscan_routine::set_lpg_dscale(int v)
{
  return sm_.lpg().set_dscale(v);
}

int c_sweepscan_routine::lpg_dscale() const
{
  return sm_.lpg().dscale();
}

void c_sweepscan_routine::set_lpg_uscale(int v)
{
  return sm_.lpg().set_uscale(v);
}

int c_sweepscan_routine::lpg_uscale() const
{
  return sm_.lpg().uscale();
}

void c_sweepscan_routine::set_lpg_squared(bool v)
{
  return sm_.lpg().set_squared(v);
}

bool c_sweepscan_routine::lpg_squared() const
{
  return sm_.lpg().squared();
}

void c_sweepscan_routine::set_lpg_avgchannel(bool v)
{
  return sm_.lpg().set_avgchannel(v);
}

bool c_sweepscan_routine::lpg_avgchannel() const
{
  return sm_.lpg().avgchannel();
}

void c_sweepscan_routine::get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls)
{
  ADD_IMAGE_PROCESSOR_CTRL(ctls, output_type, "output_type");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, max_disparity, "max_disparity");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, ss_sigma, "ss_sigma");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, ss_radius, "ss_radius");
  ADD_IMAGE_PROCESSOR_FLAGS_CTRL(ctls, ssflags, "ssflags", sscmpflags, "ssflags");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, max_scale, "max_scale");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, kernel_sigma, "kernel_sigma");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, kernel_radius, "kernel_radius");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, normalization_scale, "normalization_scale") ;
  ADD_IMAGE_PROCESSOR_CTRL_BROWSE_FOR_DIRECTORY(ctls, debug_directory, "debug_directory");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, debug_points, "debug_points");

  ADD_IMAGE_PROCESSOR_CTRL_GROUP(ctls, "lpg", "lpg");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, lpg_k, "k");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, lpg_dscale, "dscale");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, lpg_uscale, "uscale");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, lpg_squared, "squared");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, lpg_avgchannel, "avgchanne");
  END_IMAGE_PROCESSOR_CTRL_GROUP(ctls);

}

bool c_sweepscan_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {

    SERIALIZE_PROPERTY(settings, save, *this, output_type);
    SERIALIZE_PROPERTY(settings, save, *this, max_disparity);
    SERIALIZE_PROPERTY(settings, save, *this, ssflags);
    SERIALIZE_PROPERTY(settings, save, *this, ss_sigma);
    SERIALIZE_PROPERTY(settings, save, *this, ss_radius);
    SERIALIZE_PROPERTY(settings, save, *this, max_scale);
    SERIALIZE_PROPERTY(settings, save, *this, kernel_sigma);
    SERIALIZE_PROPERTY(settings, save, *this, kernel_radius);
    SERIALIZE_PROPERTY(settings, save, *this, normalization_scale) ;
    SERIALIZE_PROPERTY(settings, save, *this, debug_directory);
    SERIALIZE_PROPERTY(settings, save, *this, debug_points);

    SERIALIZE_PROPERTY(settings, save, *this, lpg_k);
    SERIALIZE_PROPERTY(settings, save, *this, lpg_dscale);
    SERIALIZE_PROPERTY(settings, save, *this, lpg_uscale);
    SERIALIZE_PROPERTY(settings, save, *this, lpg_squared);
    SERIALIZE_PROPERTY(settings, save, *this, lpg_avgchannel);

    return true;
  }

  return false;
}

bool c_sweepscan_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat src_image =
      image.getMat();

  cv::Mat src_mask =
      mask.getMat();

  const cv::Size src_size =
      src_image.size();

  const cv::Size s(src_size.width / 2, src_size.height);

  const cv::Rect ROI[2] = {
      cv::Rect(0, 0, s.width, s.height),
      cv::Rect(s.width, 0, s.width, s.height),
  };

  for( int i = 0; i < 2; ++i ) {

    images_[i] = src_image(ROI[i]);

    if ( !src_mask.empty() ) {
      masks_[i] = src_mask(ROI[i]);
    }
    else {
      masks_[i].release();
    }
  }

  if( !sm_.match(images_[0], masks_[0], images_[1], masks_[1], m, &mm) ) {
    CF_ERROR("sm_.match() fails");
    return false;
  }

  m.copyTo(image);
  //image.move(m);
  if ( mask.needed() ) {
    //mask.move(mm);
    mask.release();
  }

  return true;
}
