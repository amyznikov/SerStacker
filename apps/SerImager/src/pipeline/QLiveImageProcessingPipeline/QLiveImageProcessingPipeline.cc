/*
 * QLiveImageProcessingPipeline.cc
 *
 *  Created on: Apr 4, 2023
 *      Author: amyznikov
 */

#include "QLiveImageProcessingPipeline.h"

namespace serimager {

QLiveImageProcessingPipeline::QLiveImageProcessingPipeline(const QString & name, QObject * parent) :
    Base(name, parent)
{
}

bool QLiveImageProcessingPipeline::initialize_pipeline()
{
  if ( !Base::initialize_pipeline() ) {
    CF_ERROR("Base::initialize_pipeline() fails");
    return false;
  }

  const std::string output_path =
      create_output_path(output_options().output_directory);

  c_generic_image_processor::set_output_file_name(
      generate_output_file_name(output_path,
          output_options_.processed_frames_filename,
          "live",
          ".avi"));

  if ( !c_generic_image_processor::initialize() ) {
    CF_ERROR("c_generic_image_processor::initialize() fails");
    return false;
  }

  return true;
}

void QLiveImageProcessingPipeline::cleanup_pipeline()
{
  c_generic_image_processor::cleanup();
  Base::cleanup_pipeline();
}

bool QLiveImageProcessingPipeline::process_frame(const cv::Mat & image, COLORID colorid, int bpp)
{
  displayColorid_ =
      colorid == COLORID_MONO ? COLORID_MONO :
          COLORID_BGR;

  if( !Base::convert_image(image, colorid, bpp, &current_image_, displayColorid_, CV_8U) ) {
    CF_ERROR("convertInputImage() fails");
    return false;
  }

  if( !c_generic_image_processor::process_frame(current_image_, current_mask_) ) {
    CF_ERROR("process_current_frame() fails");
    return false;
  }

  return true;
}

bool QLiveImageProcessingPipeline::get_display_image(cv::Mat * displayImage, COLORID * colorid, int *bpp)
{
  *colorid = displayColorid_;
  *bpp = 8;

   if( !c_generic_image_processor::get_display_image(*displayImage, cv::noArray()) ) {
    CF_ERROR("c_camera_calibration::get_display_image() fails");
    return false;
  }

  return true;
}

bool QLiveImageProcessingPipeline::serialize(c_config_setting settings, bool save)
{
  if ( !Base::serialize(settings, save) ) {
    CF_ERROR("QLiveImageProcessingPipeline::Base::serialize() fails");
    return false;
  }

  if( !c_generic_image_processor::serialize(settings, save) ) {
    CF_ERROR("c_generic_image_processor::serialize() fails");
    return false;
  }

  return true;
}

} // namespace serimager
