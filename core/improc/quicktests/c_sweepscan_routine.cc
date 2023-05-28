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

void c_sweepscan_routine::set_disp12maxDiff(int v)
{
  return sm_.set_disp12maxDiff(v);
}

int c_sweepscan_routine::disp12maxDiff() const
{
  return sm_.disp12maxDiff();
}

void c_sweepscan_routine::set_texture_threshold(int v)
{
  return sm_.set_texture_threshold(v);
}

int c_sweepscan_routine::texture_threshold() const
{
  return sm_.texture_threshold();
}

void c_sweepscan_routine::set_max_scale(int v)
{
  return sm_.set_max_scale(v);
}

int c_sweepscan_routine::max_scale() const
{
  return sm_.max_scale();
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

void c_sweepscan_routine::get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls)
{
  ADD_IMAGE_PROCESSOR_CTRL(ctls, output_type, "output_type");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, max_disparity, "max_disparity");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, texture_threshold, "texture_threshold");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, disp12maxDiff, "disp12maxDiff");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, max_scale, "max_scale");
  ADD_IMAGE_PROCESSOR_CTRL_BROWSE_FOR_DIRECTORY(ctls, debug_directory, "debug_directory");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, debug_points, "debug_points");
}

bool c_sweepscan_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {

    SERIALIZE_PROPERTY(settings, save, *this, output_type);
    SERIALIZE_PROPERTY(settings, save, *this, max_disparity);
    SERIALIZE_PROPERTY(settings, save, *this, texture_threshold);
    SERIALIZE_PROPERTY(settings, save, *this, disp12maxDiff);
    SERIALIZE_PROPERTY(settings, save, *this, max_scale);
    SERIALIZE_PROPERTY(settings, save, *this, debug_directory);
    SERIALIZE_PROPERTY(settings, save, *this, debug_points);

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
