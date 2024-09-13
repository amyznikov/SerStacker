/*
 * c_textfile_input_source.cc
 *
 *  Created on: Jan 23, 2024
 *      Author: amyznikov
 */

#include "c_textfile_input_source.h"
#include "c_textfile_frame.h"

c_textfile_input_source::c_textfile_input_source(const std::string & filename) :
  base(/*source_type::TEXTFILE, */filename)
{
  size_ = 1;
}

c_textfile_input_source::sptr c_textfile_input_source::create(const std::string & filename)
{
  return sptr(new this_class(filename));
}

const std::vector<std::string> & c_textfile_input_source::suffixes()
{
  static const std::vector<std::string> suffixes_ = {
      ".txt",
      ".doc",
      ".md",
      ".xml",
      ".html",
      ".htm",
      ".rtf",
      ".tex",
      ".cfg",
      ".conf",
      ".config",
      ".rc",
      ".yml",
      ".sh",
      ".bash",
      ".zsh",
      ".gpx",
      // ".dat",
      // ".ply"
  };

  return suffixes_;
}

bool c_textfile_input_source::open()
{
  curpos_ = 0;
  return !filename_.empty();
}

void c_textfile_input_source::close()
{
  curpos_ = -1;
}

bool c_textfile_input_source::read(c_data_frame::sptr & output_frame)
{
  if( !is_open() ) {
    errno = EBADF;
    return false;
  }

  c_textfile_frame * f =
      dynamic_cast<c_textfile_frame*>(output_frame.get());

  if( !f ) {
    output_frame.reset(f = new c_textfile_frame());
  }

  f->filename_ = this->filename_;

  return true;
}

bool c_textfile_input_source::read(cv::Mat & output_frame, enum COLORID * output_colorid, int * output_bpc)
{
  return false;
}

bool c_textfile_input_source::seek(int pos)
{
  return ((curpos_ = pos) == 0);
}

int c_textfile_input_source::curpos()
{
  return curpos_;
}

bool c_textfile_input_source::is_open() const
{
  return curpos_ >= 0;
}

