/*
 * c_fits_input_source.cc
 *
 *  Created on: Apr 2, 2024
 *      Author: amyznikov
 */

#include "c_fits_input_source.h"
#include <core/readdir.h>


#if HAVE_CFITSIO

c_fits_input_source::c_fits_input_source(const std::string & filename) :
    base(/*c_input_source::FITS, */filename)
{
}

c_fits_input_source::sptr c_fits_input_source::create(const std::string & filename)
{
  if ( file_exists(filename) && !is_directory(filename) ) {
    c_fits_input_source::sptr obj(new c_fits_input_source(filename));
    obj->size_ = 1;
    return obj;
  }
  return nullptr;
}

const std::vector<std::string> & c_fits_input_source::suffixes()
{
  static const std::vector<std::string> suffixes_ = {
      ".fits",
      ".fit",
      ".fts"
  };

  return suffixes_;
}

bool c_fits_input_source::open()
{
  if ( !fits_.open(filename_) ) {
    return false;
  }
  curpos_ = 0;
  return true;
}

void c_fits_input_source::close()
{
  fits_.close();
  curpos_ = -1;
}

bool c_fits_input_source::seek(int pos)
{
  if ( pos != 0 ) {
    return false;
  }
  curpos_ = pos;
  return true;
}

int c_fits_input_source::curpos()
{
  return curpos_;
}

bool c_fits_input_source::read(cv::Mat & output_frame,
    enum COLORID * output_colorid,
    int * output_bpc)
{
  if ( curpos_ != 0 || !fits_.read(output_frame) ) {
    return false;
  }

  ++curpos_;

  if ( output_colorid ) {
    *output_colorid = suggest_colorid(
        output_frame.channels());
  }

  if ( output_bpc ) {
    *output_bpc = suggest_bpp(
        output_frame.depth());
  }

  return true;
}

bool c_fits_input_source::is_open() const
{
  return curpos_ >= 0;
}

#endif // HAVE_CFITSIO
