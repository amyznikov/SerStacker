/*
 * QLiveRegularStereoPipeline.cc
 *
 *  Created on: Mar 27, 2023
 *      Author: amyznikov
 */

#include "QLiveRegularStereoPipeline.h"

#if 0

namespace serimager {

QLiveRegularStereoPipeline::QLiveRegularStereoPipeline(const QString & name, QObject * parent) :
    Base(name, parent)
{
}

bool QLiveRegularStereoPipeline::initialize_pipeline()
{
  if  ( !Base::initialize_pipeline() ) {
    CF_ERROR("QLiveRegularStereoPipeline::Base::initialize_pipeline() fails");
    return false;
  }

  if ( !c_regular_stereo::initialize() ) {
    CF_ERROR("c_regular_stereo::initialize() fails");
    return false;
  }

  return true;
}

void QLiveRegularStereoPipeline::cleanup_pipeline()
{
  c_regular_stereo::cleanup();
  Base::cleanup_pipeline();
}

bool QLiveRegularStereoPipeline::process_frame(const cv::Mat & image, COLORID colorid, int bpp)
{
  cv::Mat currentImage;
  cv::Mat frames[2];
  cv::Mat masks[2];
  cv::Rect roi[2];

  displayColorid_ =
      colorid == COLORID_MONO ? COLORID_MONO :
          COLORID_BGR;

  if( !Base::convert_image(image, colorid, bpp, &currentImage, displayColorid_, CV_8U) ) {
    CF_ERROR("convertInputImage() fails");
    return false;
  }


  roi[0] = cv::Rect(0, 0, currentImage.cols / 2, currentImage.rows);
  roi[1] = cv::Rect(currentImage.cols / 2, 0, currentImage.cols / 2, currentImage.rows);
  frames[0] = currentImage(roi[0]);
  frames[1] = currentImage(roi[1]);

  if ( !c_regular_stereo::process_stereo_frame(frames, masks) ) {
    CF_ERROR("c_regular_stereo::process_stereo_frame() fails");
    return false;
  }

  return true;
}

bool QLiveRegularStereoPipeline::get_display_image(cv::Mat * displayImage, COLORID * colorid, int *bpp)
{
  *colorid = displayColorid_;
  *bpp = 8;

  if( !c_regular_stereo::get_display_image(*displayImage, cv::noArray()) ) {
    CF_ERROR("c_regular_stereo::get_display_image() fails");
    return false;
  }

  return true;
}

bool QLiveRegularStereoPipeline::serialize(c_config_setting settings, bool save)
{
  if ( !Base::serialize(settings, save) ) {
    CF_ERROR("QLiveRegularStereoPipeline::Base::serialize() fails");
    return false;
  }

  if ( !c_regular_stereo::serialize(settings, save) ) {
    CF_ERROR("c_regular_stereo::serialize() fails");
    return false;
  }

  return true;
}


} /* namespace serimager */

#endif
