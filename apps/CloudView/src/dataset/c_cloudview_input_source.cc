/*
 * c_cloudview_input_source.cc
 *
 *  Created on: Nov 18, 2023
 *      Author: amyznikov
 */

#include "c_cloudview_input_source.h"
#include "video/c_video_input_source.h"
#include "vlo/c_cloudview_vlo_input_source.h"
#include "text/c_text_input_source.h"
#include <core/readdir.h>
#include <core/debug.h>

namespace cloudview {

const std::string & c_cloudview_input_source::filename() const
{
  return filename_;
}

const char * c_cloudview_input_source::cfilename() const
{
  return filename_.c_str();
}

c_cloudview_input_source::sptr c_cloudview_input_source::load(const std::string & filename)
{
  sptr source;

  if( c_text_input_source::is_supported_suffix(filename) ) {

    if( (source = c_text_input_source::load(filename)) ) {
      return source;
    }

  }


  if( c_cloudview_vlo_input_source::is_supported_suffix(filename) ) {

    if( (source = c_cloudview_vlo_input_source::load(filename)) ) {
      return source;
    }

  }

  if( c_video_input_source::is_supported_suffix(filename) ) {

    if( (source = c_video_input_source::load(filename)) ) {
      return source;
    }

  }

  return source;
}


} /* namespace cloudview */
