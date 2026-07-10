/*
 * c_ser_input_source.cc
 *
 *  Created on: Apr 2, 2024
 *      Author: amyznikov
 */

#include "c_ser_input_source.h"

c_ser_input_source::c_ser_input_source(const std::string & filename) :
    base(/*c_input_source::SER, */filename)
{
}

c_ser_input_source::sptr c_ser_input_source::create(const std::string & filename)
{
  sptr obj(new this_class(filename));
  if ( obj->_ser.open(filename) ) {
    obj->_size = obj->_ser.num_frames();
    obj->_ser.close();
    return obj;
  }
  return nullptr;
}

const std::vector<std::string> & c_ser_input_source::suffixes()
{
  static const std::vector<std::string> suffixes_ = {
      ".ser"
  };

  return suffixes_;
}

bool c_ser_input_source::open()
{
  return _ser.open(_filename);
}

void c_ser_input_source::close()
{
  _ser.close();
}

bool c_ser_input_source::seek(int pos)
{
  return _ser.seek(pos);
}

int c_ser_input_source::curpos()
{
  return _ser.curpos();
}

bool c_ser_input_source::read(cv::Mat & output_frame,
    enum COLORID * output_colorid,
    int * output_bpc)
{
  if ( _ser.read(output_frame) ) {

    if ( output_colorid ) {
      *output_colorid = _ser.color_id();
    }

    if ( output_bpc ) {
      *output_bpc = _ser.bits_per_plane();
    }

    return true;
  }

  return false;
}

bool c_ser_input_source::is_open() const
{
  return _ser.is_open();
}
