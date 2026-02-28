/*
 * c_image_input_source.cc
 *
 *  Created on: Dec 15, 2023
 *      Author: amyznikov
 */

#include "c_image_input_source.h"
#include "c_video_frame.h"
#include <core/io/c_input_options.h>
#include <core/io/load_image.h>
#include <core/proc/bad_pixels.h>
#include <core/ssprintf.h>
#include <core/readdir.h>
#include <mutex>
#include <core/debug.h>


c_image_input_source::c_image_input_source(const std::string & filename) :
  base(filename)
{
}

bool c_image_input_source::read(c_data_frame::sptr & output_frame)
{
  if( !is_open() ) {
    errno = EBADF;
    return false;
  }

  c_video_frame * f =
      dynamic_cast<c_video_frame*>(output_frame.get());

  if( !f ) {
    output_frame.reset(f = new c_video_frame());
  }

  if( !((base*) this)->read(f->_input_image, &f->_colorid, &f->_bpc) ) {
    CF_ERROR("input_source_->read() fails");
    return false;
  }

  f->_input_mask.release();

  if( f->_colorid != COLORID_OPTFLOW && (f->_input_image.channels() == 4 || f->_input_image.channels() == 2) ) {
    if( !splitbgra(f->_input_image, f->_input_image, &f->_input_mask) ) {
      CF_WARNING("c_video_input_source: splitbgra() fails for colorid=%s image.channels=%d",
          toCString(f->_colorid), f->_input_image.channels());
    }
  }

  if( (f->_has_color_matrix = has_color_matrix()) ) {
    f->_color_matrix = color_matrix();
  }

  if ( _input_options ) {

    const c_video_input_options & opts =
        _input_options->video;

    if( opts.filter_bad_pixels && opts.bad_pixels_variation_threshold > 0 ) {

      if( !is_bayer_pattern(f->_colorid) ) {
        median_filter_hot_pixels(f->_input_image, opts.bad_pixels_variation_threshold, false);
      }
      else if( !extract_bayer_planes(f->_input_image, f->_input_image, f->_colorid) ) {
        CF_ERROR("ERROR: extract_bayer_planes() fails");
      }
      else {
        median_filter_hot_pixels(f->_input_image, opts.bad_pixels_variation_threshold, true);
        if( !nninterpolation(f->_input_image, f->_input_image, f->_colorid) ) {
          CF_ERROR("nninterpolation() fails");
        }
      }
    }
    else if( is_bayer_pattern(f->_colorid) ) {
      // In debayer() the DEBAYER_AVGC reduces the image size twice because of 2x2 binning
      debayer(f->_input_image, f->_input_image, f->_colorid, opts.debayer_method);
      if ( !f->_input_mask.empty() && f->_input_mask.size() != f->_input_image.size() ) {
        cv::resize(f->_input_mask, f->_input_mask, f->_input_image.size(), 0, 0, cv::INTER_NEAREST);
      }
    }
  }

  f->cleanup();

  return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
