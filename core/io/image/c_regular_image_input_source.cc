/*
 * c_regular_image_input_source.cc
 *
 *  Created on: Apr 2, 2024
 *      Author: amyznikov
 */

#include "c_regular_image_input_source.h"
#include <core/io/load_image.h>
#include <core/proc/parse_timestamp.h>
#include <core/readdir.h>
//#include <core/debug.h>



c_regular_image_input_source::c_regular_image_input_source(const std::string & filename) :
    base(/*c_input_source::REGULAR_IMAGE, */filename)
{
}

c_regular_image_input_source::sptr c_regular_image_input_source::create(const std::string & filename)
{
  if ( file_exists(filename) && !is_directory(filename) ) {
    sptr obj(new this_class(filename));
    obj->size_ = 1;
    return obj;
  }
  return nullptr;
}

const std::vector<std::string> & c_regular_image_input_source::suffixes()
{
  static const std::vector<std::string> suffixes_ = {
      ".tif", ".tiff",
      ".png",
      ".exr",
      ".hdr", ".pic",
      ".jpg", ".jpeg", ".jp2",
      ".bmp", ".dib",
      ".ppm", ".pgm",
      ".webp",
      ".flo",
      ".pbm", ".pgm", ".ppm", ".pxm", ".pnm",  // Portable image format
      ".sr", ".ras",      // Sun rasters
      ".pfm",
  };

  return suffixes_;
}

bool c_regular_image_input_source::open()
{
  if ( file_readable(filename_) && !is_directory(filename_) ) {
    curpos_ = 0;
    return true;
  }
  return false;
}


void c_regular_image_input_source::close()
{
  curpos_ = -1;
}


bool c_regular_image_input_source::seek(int pos)
{
  if ( pos != 0 ) {
    return false;
  }
  curpos_ = pos;
  return true;
}

int c_regular_image_input_source::curpos()
{
  return curpos_;
}

bool c_regular_image_input_source::read(cv::Mat & output_frame,
    enum COLORID * output_colorid,
    int * output_bpc)
{
  if ( curpos_ != 0 ) {
    return false;
  }

  const std::string suffix =
      get_file_suffix(filename_);

  if ( strcasecmp(suffix.c_str(), ".flo") == 0 ) {

    if ( !(output_frame = cv::readOpticalFlow(filename_)).data ) {
      return false;
    }

    if ( output_colorid ) {
      *output_colorid = COLORID_OPTFLOW;
    }

    if ( output_bpc ) {
      *output_bpc = suggest_bpp(
          output_frame.depth());
    }

  }
  else {

    if ( !load_image(filename_, output_frame) ) {
      return false;
    }

    // CF_DEBUG("output_frame.channels()=%d", output_frame.channels());


    if ( output_colorid ) {
      *output_colorid = suggest_colorid(
          output_frame.channels());
    }

    if ( output_bpc ) {
      *output_bpc = suggest_bpp(
          output_frame.depth());
    }

  }

  _has_last_ts =
      parse_timestamp_from_filename(filename_,
          &_last_ts);

  ++curpos_;

  return true;
}

bool c_regular_image_input_source::is_open() const
{
  return curpos_ >= 0;
}

