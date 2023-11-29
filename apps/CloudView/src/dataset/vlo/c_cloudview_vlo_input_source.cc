/*
 * c_vlo_input_sequence.cc
 *
 *  Created on: Nov 18, 2023
 *      Author: amyznikov
 */

#include "c_cloudview_vlo_input_source.h"
#include "c_vlo_data_frame.h"
#include <core/readdir.h>
#include <core/debug.h>

#if _MSC_VER
#ifndef strcasecmp
# define strcasecmp  stricmp
#endif
#endif

namespace cloudview {

c_cloudview_vlo_input_source::c_cloudview_vlo_input_source()
{
}

bool c_cloudview_vlo_input_source::is_supported_suffix(const std::string & filename)
{

  const std::string suffix =
      get_file_suffix(filename);


  for( const std::string & s : c_vlo_input_source::suffixes() ) {
    if( strcasecmp(suffix.c_str(), s.c_str()) == 0 ) {
      return true;
    }
  }

  return false;
}

c_cloudview_vlo_input_source::sptr c_cloudview_vlo_input_source::load(const std::string & filename)
{
  sptr obj(new this_class());

  if( !obj->open(filename) ) {
    CF_ERROR("c_cloudview_vlo_input_source: obj->open(%s) fails", filename.c_str());
    obj.reset();
  }

  return obj;
}


bool c_cloudview_vlo_input_source::open(const std::string & filename)
{
  close();

  if( !(input_source_ = c_vlo_input_source::create(filename)) ) {
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

void c_cloudview_vlo_input_source::close()
{
  if ( input_source_ ) {
    input_source_->close();
    input_source_.reset();
  }
}

bool c_cloudview_vlo_input_source::is_open()
{
  return input_source_ ? input_source_->is_open() : false;
}

ssize_t c_cloudview_vlo_input_source::size()
{
  return input_source_ ? input_source_->size() : -1;
}

bool c_cloudview_vlo_input_source::seek(ssize_t pos)
{
  return input_source_ ? input_source_->seek(pos) : -1;
}

ssize_t c_cloudview_vlo_input_source::curpos()
{
  return input_source_ ? input_source_->curpos() : -1;
}

c_cloudview_data_frame::sptr c_cloudview_vlo_input_source::read()
{
  if ( !is_open() ) {
    CF_ERROR("input_source_ is not open");
    return nullptr;
  }

  c_vlo_data_frame::sptr f(new c_vlo_data_frame());

  if( !input_source_->read(&f->current_scan_) ) {
    CF_ERROR("input_source_->read() fails");
    return nullptr;
  }


  return f;

}


} /* namespace cloudview */
