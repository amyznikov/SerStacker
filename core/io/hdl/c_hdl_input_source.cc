/*
 * c_hdl_input_source.cc
 *
 *  Created on: Feb 25, 2024
 *      Author: amyznikov
 */

#include "c_hdl_input_source.h"
#include "c_hdl_data_frame.h"

#if HAVE_PCAP

c_hdl_input_source::c_hdl_input_source(const std::string & filename) :
  base(filename)
{
}

c_hdl_input_source::sptr c_hdl_input_source::create(const std::string & filename)
{
  this_class::sptr obj(new this_class(filename));
  if( obj->open() ) {
    obj->close();
    return obj;
  }
  return nullptr;
}

const std::vector<std::string> & c_hdl_input_source::suffixes()
{
  static const std::vector<std::string> suffixes_ = {
      ".pcap",
      ".vpcap"
  };
  return suffixes_;
}


bool c_hdl_input_source::open()
{
  if( reader_.open(filename_, "") && reader_.select_stream(0) ) {
    reader_.seek(0);
    size_ = reader_.num_frames();
    return true;
  }
  return false;
}

bool c_hdl_input_source::is_open() const
{
  return reader_.is_open();
}

void c_hdl_input_source::close()
{
  reader_.close();
}

bool c_hdl_input_source::seek(int pos)
{
  return reader_.seek(pos);
}

int c_hdl_input_source::curpos()
{
  return reader_.curpos();
}

bool c_hdl_input_source::read(c_data_frame::sptr & output_frame)
{
  if( !is_open() ) {
    errno = EBADF;
    return false;
  }

  c_hdl_data_frame * hdl =
      dynamic_cast<c_hdl_data_frame*>(output_frame.get());

  if( !hdl ) {
    output_frame.reset(hdl = new c_hdl_data_frame());
  }

  hdl->cleanup();

  //c_hdl_specification lidar_specification_;
  //;
  if( (hdl->current_frame_ = reader_.read()) ) {

    hdl->current_lidar_ =
        *reader_.hdl_parser().lidar_specification();

    return true;
  }

  return false;
}

c_hdl_frame::sptr c_hdl_input_source::read()
{
  return reader_.read();
}

bool c_hdl_input_source::read(cv::Mat & output_frame,
    enum COLORID * output_colorid,
    int * output_bpc)
{
  return false;
}

#endif // HAVE_PCAP
