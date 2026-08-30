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
  if ( obj->_ffmpeg.open(filename) ) {
    obj->_size = obj->_ffmpeg.num_frames();
    obj->_ffmpeg.close();
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
  return _ffmpeg.open(_filename);
}

void c_ffmpeg_input_source::close()
{
  return _ffmpeg.close();
}

bool c_ffmpeg_input_source::seek(int pos)
{
  return _ffmpeg.seek_frame(pos);
}

int c_ffmpeg_input_source::curpos()
{
  return _ffmpeg.curpos();
}

bool c_ffmpeg_input_source::read(cv::OutputArray output_image,
    cv::OutputArray output_mask,
    enum COLORID * output_colorid,
    int * output_bpc)
{
  if ( _ffmpeg.read(output_image) ) {

    if ( output_colorid ) {
      *output_colorid = suggest_colorid(output_image.channels());
    }

    if ( output_bpc ) {
      *output_bpc = suggest_bpp(output_image.depth());
    }

    if ( output_mask.needed() ) {
      output_mask.release();
    }

    return true;
  }
  return false;
}

bool c_ffmpeg_input_source::is_open() const
{
  return _ffmpeg.is_open();
}

#endif // HAVE_FFMPEG
