/*
 * c_text_input_source.cc
 *
 *  Created on: Dec 4, 2023
 *      Author: amyznikov
 */

#include "c_text_input_source.h"
#include <core/readdir.h>

namespace cloudview {

c_text_input_source::c_text_input_source()
{
}


const std::vector<std::string> & c_text_input_source::suffixes()
{
//  static std::mutex mtx;
//  mtx.lock();

  static const std::vector<std::string> suffixes_({
    ".txt",
    ".doc",
    ".ply",
    ".cfg",
    ".conf",
    ".config",
    ".sh",
    ".bash",
    ".zsh",
  });

  //  mtx.unlock();

  return suffixes_;
}


bool c_text_input_source::is_supported_suffix(const std::string & filename)
{
  const std::string suffix =
      get_file_suffix(filename);


  for( const std::string & s : suffixes() ) {
    if( strcasecmp(suffix.c_str(), s.c_str()) == 0 ) {
      return true;
    }
  }

  return false;
}

c_text_input_source::sptr c_text_input_source::load(const std::string & filename)
{
  sptr obj(new this_class());

  obj->filename_ = filename;

  return obj;
}

bool c_text_input_source::open(const std::string & filename)
{
  close();

  return true;
}

void c_text_input_source::close()
{
}

bool c_text_input_source::is_open()
{
  return true;
}

ssize_t c_text_input_source::size()
{
  return 1;
}

bool c_text_input_source::seek(ssize_t pos)
{
  return pos == 0;
}

ssize_t c_text_input_source::curpos()
{
  return 0;
}

c_cloudview_data_frame::sptr c_text_input_source::read()
{
  c_text_frame::sptr f(new c_text_frame());
  f->set_filename(filename_);
  return f;
}

} /* namespace cloudview */
