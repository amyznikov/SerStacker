/*
 * c_vlo_input_sequence.cc
 *
 *  Created on: Nov 18, 2023
 *      Author: amyznikov
 */

#include "c_cloudview_vlo_input_source.h"

namespace cloudview {

c_cloudview_vlo_input_source::c_cloudview_vlo_input_source(const std::string & filename) :
    base(filename)
{
}

bool c_cloudview_vlo_input_source::open(const std::string & filename)
{
  return false;
}

void c_cloudview_vlo_input_source::close()
{

}

bool c_cloudview_vlo_input_source::is_open()
{
  return false;
}

ssize_t c_cloudview_vlo_input_source::size()
{
  return -1;
}

bool c_cloudview_vlo_input_source::seek(ssize_t pos)
{
  return false;
}

ssize_t c_cloudview_vlo_input_source::curpos()
{
  return -1;
}

c_cloudview_data_frame::sptr c_cloudview_vlo_input_source::read()
{
  return nullptr;
}


} /* namespace cloudview */
