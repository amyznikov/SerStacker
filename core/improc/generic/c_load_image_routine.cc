/*
 * c_load_image_routine.cc
 *
 *  Created on: May 10, 2024
 *      Author: amyznikov
 */

#include "c_load_image_routine.h"
#include <core/io/load_image.h>
#include <core/debug.h>

void c_load_image_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind_browse_for_file(ctls, "filename", ctx(&this_class::_filename), "Path to image file to load");
  ctlbind(ctls, "artifact_name", ctx(&this_class::_artifact_name), "Name for this image to save");
}

bool c_load_image_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _filename);
    SERIALIZE_OPTION(settings, save, *this, _artifact_name);
    return true;
  }
  return false;
}

bool c_load_image_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( _filename.empty() ) {
    CF_ERROR("No input file name specified");
    return false;
  }

  cv::Mat img, msk;

  if ( !load_image(_filename, img, msk) ) {
    CF_ERROR("load_image('%s') fails", _filename.c_str());
    return false;
  }

  if ( !_artifact_name.empty() ) {
    base::add_artifact(_artifact_name, img, msk);
  }
  else {
    img.copyTo(image);
    msk.copyTo(mask);
  }

  return true;
}
