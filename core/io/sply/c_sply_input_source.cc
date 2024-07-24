/*
 * c_sply_input_source.cc
 *
 *  Created on: Jul 24, 2024
 *      Author: amyznikov
 */

#include "c_sply_input_source.h"
#include "c_sply_data_frame.h"


c_sply_input_source::c_sply_input_source(const std::string & filename) :
  base(filename)
{
}

c_sply_input_source::sptr c_sply_input_source::create(const std::string & filename)
{
  this_class::sptr obj(new this_class(filename));
  if( obj->open() ) {
    obj->close();
    return obj;
  }
  return nullptr;
}

const std::vector<std::string> & c_sply_input_source::suffixes()
{
  static const std::vector<std::string> suffixes_ = {
      ".sply",
  };
  return suffixes_;
}


bool c_sply_input_source::open()
{
  if( _sply.open(filename_) && _sply.select_stream(0) ) {
    _sply.seek(0);
    size_ = _sply.num_frames();
    return true;
  }
  return false;
}

bool c_sply_input_source::is_open() const
{
  return _sply.is_open();
}

void c_sply_input_source::close()
{
  _sply.close();
}

bool c_sply_input_source::seek(int pos)
{
  return _sply.seek(pos);
}

int c_sply_input_source::curpos()
{
  return _sply.curpos();
}

bool c_sply_input_source::read(c_data_frame::sptr & output_frame)
{
  if( !is_open() ) {
    errno = EBADF;
    return false;
  }


  c_sply_data_frame * frame =
      dynamic_cast<c_sply_data_frame*>(output_frame.get());

  if( !frame ) {
    output_frame.reset(frame = new c_sply_data_frame());
  }

  frame->cleanup();

  return _sply.read(frame->points_, frame->colors_, frame->timestamps_);
}

bool c_sply_input_source::read(cv::Mat & output_frame,
    enum COLORID * output_colorid,
    int * output_bpc)
{
  return false;
}
