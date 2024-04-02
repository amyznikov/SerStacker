/*
 * c_ffmpeg_input_source.cc
 *
 *  Created on: Apr 2, 2024
 *      Author: amyznikov
 */

#include "c_ffmpeg_input_source.h"

#if HAVE_FFMPEG

c_ffmpeg_input_source::c_ffmpeg_input_source(const std::string & filename) :
    base(filename)
{
}

c_ffmpeg_input_source::sptr c_ffmpeg_input_source::create(const std::string & filename)
{
  sptr obj(new this_class(filename));
  if ( obj->ffmpeg_.open(filename) ) {
    obj->size_ = obj->ffmpeg_.num_frames();
    obj->ffmpeg_.close();
    return obj;
  }
  return nullptr;
}

const std::vector<std::string> & c_ffmpeg_input_source::suffixes()
{
  static std::vector<std::string> suffixes_ =
      c_ffmpeg_reader::supported_input_formats();

  return suffixes_;
}


bool c_ffmpeg_input_source::open()
{
  return ffmpeg_.open(filename_);
}

void c_ffmpeg_input_source::close()
{
  return ffmpeg_.close();
}

bool c_ffmpeg_input_source::seek(int pos)
{
  return ffmpeg_.seek_frame(pos);
}

int c_ffmpeg_input_source::curpos()
{
  return ffmpeg_.curpos();
}

bool c_ffmpeg_input_source::read(cv::Mat & output_frame,
    enum COLORID * output_colorid,
    int * output_bpc)
{
  if ( ffmpeg_.read(output_frame) ) {

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
  return false;
}

bool c_ffmpeg_input_source::is_open() const
{
  return ffmpeg_.is_open();
}

#endif // HAVE_FFMPEG
