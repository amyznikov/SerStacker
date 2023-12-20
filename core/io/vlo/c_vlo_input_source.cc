/*
 * c_vlo_input_source.cc
 *
 *  Created on: Dec 15, 2023
 *      Author: amyznikov
 */

#include "c_vlo_input_source.h"

c_vlo_input_source::c_vlo_input_source(const std::string & filename) :
    base(c_input_source::VLO, filename)
{
}

c_vlo_input_source::sptr c_vlo_input_source::create(const std::string & filename)
{
  this_class::sptr obj(new this_class(filename));
  if( obj->vlo_.open(filename) ) {
    obj->size_ = obj->vlo_.num_frames();
    obj->vlo_.close();
    return obj;
  }
  return nullptr;
}

const std::vector<std::string> & c_vlo_input_source::suffixes()
{
  static const std::vector<std::string> suffixes_ = {
      ".vsb",
      ".dat"
  };
  return suffixes_;
}

bool c_vlo_input_source::open()
{
  return vlo_.open(filename_);
}

void c_vlo_input_source::close()
{
  return vlo_.close();
}


bool c_vlo_input_source::seek(int pos)
{
  return vlo_.seek(pos);
}

int c_vlo_input_source::curpos()
{
  return vlo_.curpos();
}


bool c_vlo_input_source::read(cv::Mat & output_frame,
    enum COLORID * output_colorid,
    int * output_bpc)
{
   if( read(&current_scan_) && !(output_frame = get_vlo_image(current_scan_, VLO_DATA_CHANNEL_AMBIENT)).empty()  ) {

    if ( output_colorid ) {
      *output_colorid =
          suggest_colorid(output_frame.channels());
    }

    if ( output_bpc ) {
      *output_bpc = suggest_bbp(
          output_frame.depth());
    }

    return true;
  }

  return false;
}
//
//bool c_vlo_input_source::read_cloud3d(cv::OutputArray points, cv::OutputArray colors)
//{
//  // vlo_.set_apply_ghost_filter(apply_ghost_filter_);
//  return vlo_.read_cloud3d(points, colors, read_channel_);
//}


bool c_vlo_input_source::is_open() const
{
  return vlo_.is_open();
}

void c_vlo_input_source::set_read_channel(VLO_DATA_CHANNEL v)
{
  read_channel_ = v;
}

VLO_DATA_CHANNEL c_vlo_input_source::read_channel() const
{
  return read_channel_;
}


c_vlo_processing_options * c_vlo_input_source::processing_options()
{
  return vlo_.processing_options();
}

VLO_VERSION c_vlo_input_source::version() const
{
  return vlo_.version();
}

bool c_vlo_input_source::read(c_vlo_scan * scan)
{
  return vlo_.read(scan);
}

bool c_vlo_input_source::read(c_data_frame::sptr & output_frame)
{
  if( !is_open() ) {
    errno = EBADF;
    return false;
  }

  c_vlo_frame * vlo =
      dynamic_cast<c_vlo_frame*>(output_frame.get());

  if( !vlo ) {
    output_frame.reset(vlo = new c_vlo_frame());
  }

  return vlo_.read(&vlo->current_scan_);
}

