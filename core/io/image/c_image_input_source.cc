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

  if( !((base*) this)->read(f->input_image_, &f->colorid_, &f->bpc_) ) {
    CF_ERROR("input_source_->read() fails");
    return false;
  }

  f->input_mask_.release();

  if( f->colorid_ != COLORID_OPTFLOW && (f->input_image_.channels() == 4 || f->input_image_.channels() == 2) ) {
    if( !splitbgra(f->input_image_, f->input_image_, &f->input_mask_) ) {
      CF_WARNING("c_video_input_source: splitbgra() fails for colorid=%s image.channels=%d",
          toCString(f->colorid_), f->input_image_.channels());
    }
  }

  if( (f->has_color_matrix_ = has_color_matrix()) ) {
    f->color_matrix_ = color_matrix();
  }

  if ( input_options_ ) {

    const c_video_input_options & opts =
        input_options_->video;

    if( opts.filter_bad_pixels && opts.bad_pixels_variation_threshold > 0 ) {

      if( !is_bayer_pattern(f->colorid_) ) {
        median_filter_hot_pixels(f->input_image_, opts.bad_pixels_variation_threshold, false);
      }
      else if( !extract_bayer_planes(f->input_image_, f->input_image_, f->colorid_) ) {
        CF_ERROR("ERROR: extract_bayer_planes() fails");
      }
      else {
        median_filter_hot_pixels(f->input_image_, opts.bad_pixels_variation_threshold, true);
        if( !nninterpolation(f->input_image_, f->input_image_, f->colorid_) ) {
          CF_ERROR("nninterpolation() fails");
        }
      }
    }
    else if( is_bayer_pattern(f->colorid_) ) {
      debayer(f->input_image_, f->input_image_, f->colorid_,
          opts.debayer_method);
    }
  }

  f->cleanup();

  return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
