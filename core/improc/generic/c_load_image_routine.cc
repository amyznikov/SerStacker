/*
 * c_load_image_routine.cc
 *
 *  Created on: May 10, 2024
 *      Author: amyznikov
 */

#include "c_load_image_routine.h"
#include <core/io/load_image.h>
#include <core/debug.h>

void c_load_image_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_BROWSE_FOR_EXISTING_FILE_CTRL(ctls, filename, "filename", "Path to image file to load");
  BIND_PCTRL(ctls, artifact_name, "Name for this image to save");
}

bool c_load_image_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, filename);
    SERIALIZE_PROPERTY(settings, save, *this, artifact_name);
    return true;
  }
  return false;
}

bool c_load_image_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( filename_.empty() ) {
    CF_ERROR("No input file name specified");
    return false;
  }

  cv::Mat img, msk;

  if ( !load_image(filename_, img, msk) ) {
    CF_ERROR("load_image('%s') fails", filename_.c_str());
    return false;
  }

  if ( !artifact_name_.empty() ) {
    base::add_artifact(artifact_name_, img, msk);
  }
  else {
    img.copyTo(image);
    msk.copyTo(mask);
  }

  return true;
}


