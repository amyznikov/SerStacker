/*
 * load_image.cc
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */


#include "load_image.h"
#include <core/readdir.h>
#include <core/debug.h>

bool load_image(cv::Mat & dst, const std::string & filename)
{
  const std::string suffix = get_file_suffix(filename);

  if ( strcasecmp(suffix.c_str(), ".flo") == 0 ) {
    return (dst = cv::readOpticalFlow(filename)).data != nullptr;
  }

  return (dst = cv::imread(filename, cv::IMREAD_UNCHANGED)).data != nullptr;
}

