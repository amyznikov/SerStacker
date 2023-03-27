/*
 * QLiveRegularStereoPipeline.cc
 *
 *  Created on: Mar 27, 2023
 *      Author: amyznikov
 */

#include "QLiveRegularStereoPipeline.h"

namespace serimager {

QLiveRegularStereoPipeline::QLiveRegularStereoPipeline(const QString & name, QObject * parent) :
    Base(name, parent)
{
}

c_regular_stereo & QLiveRegularStereoPipeline::rstereo()
{
  return rstereo_;
}

const c_regular_stereo & QLiveRegularStereoPipeline::rstereo() const
{
  return rstereo_;
}

bool QLiveRegularStereoPipeline::initialize_pipeline()
{
  return false;
}

void QLiveRegularStereoPipeline::cleanup_pipeline()
{
  return ;
}

bool QLiveRegularStereoPipeline::process_frame(const cv::Mat & image, COLORID colorid, int bpp)
{
  return false;
}

bool QLiveRegularStereoPipeline::get_display_image(cv::Mat * displayImage, COLORID * colorid, int *bpp)
{
  return false;
}

bool QLiveRegularStereoPipeline::serialize(c_config_setting settings, bool save)
{
  if ( !Base::serialize(settings, save) ) {
    CF_ERROR("QLiveRegularStereoPipeline::Base::serialize() fails");
    return false;
  }

  return true;
}


} /* namespace serimager */
