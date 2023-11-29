/*
 * c_video_input_source.cc
 *
 *  Created on: Nov 26, 2023
 *      Author: amyznikov
 */

#include "c_video_input_source.h"
#include <core/io/load_image.h>
#include <core/ssprintf.h>
#include <core/debug.h>

namespace cloudview {

c_video_input_source::c_video_input_source()
{
}

bool c_video_input_source::is_supported_suffix(const std::string & filename)
{
  return c_input_source::suggest_source_type(filename) !=
      c_input_source::source_type::UNKNOWN;
}

c_video_input_source::sptr c_video_input_source::load(const std::string & filename)
{
  sptr obj(new this_class());

  if( !obj->open(filename) ) {
    CF_ERROR("c_video_input_source: obj->open(%s) fails", filename.c_str());
    obj.reset();
  }

  return obj;
}

bool c_video_input_source::open(const std::string & filename)
{
  close();

  if( !(input_source_ = c_input_source::create(filename)) ) {
    CF_ERROR("c_input_source::create('%s') fails", filename.c_str());
    return false;
  }

  if( !input_source_->open() ) {
    CF_ERROR("c_input_source::open('%s') fails", filename.c_str());
    close();
    return false;
  }

  return true;
}

void c_video_input_source::close()
{
  if ( input_source_ ) {
    input_source_->close();
    input_source_.reset();
  }
}

bool c_video_input_source::is_open()
{
  return input_source_ ? input_source_->is_open() : false;
}

ssize_t c_video_input_source::size()
{
  return input_source_ ? input_source_->size() : -1;
}

bool c_video_input_source::seek(ssize_t pos)
{
  return input_source_ ? input_source_->seek(pos) : -1;
}

ssize_t c_video_input_source::curpos()
{
  return input_source_ ? input_source_->curpos() : -1;
}

c_cloudview_data_frame::sptr c_video_input_source::read()
{
  if ( !is_open() ) {
    CF_ERROR("input_source_ is not open");
    return nullptr;
  }

  c_video_frame::sptr f(new c_video_frame());

  if( !input_source_->read(f->image, &f->colorid, &f->bpc) ) {
    CF_ERROR("input_source_->read() fails");
    return nullptr;
  }

  if( f->colorid != COLORID_OPTFLOW && (f->image.channels() == 4 || f->image.channels() == 2) ) {
    if( !splitbgra(f->image, f->image, &f->mask) ) {
      CF_WARNING("c_video_input_source: splitbgra() fails for colorid=%s image.channels=%d",
          toString(f->colorid), f->image.channels());
    }
  }

  if( (f->has_color_matrix = input_source_->has_color_matrix()) ) {
    // cv::transform(f->image, f->image, input_source_->color_matrix());
    f->color_matrix = input_source_->color_matrix();
  }

  return f;
}


} /* namespace cloudview */
