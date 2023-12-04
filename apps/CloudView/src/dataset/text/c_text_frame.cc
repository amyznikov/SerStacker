/*
 * c_text_frame.cc
 *
 *  Created on: Dec 3, 2023
 *      Author: amyznikov
 */

#include "c_text_frame.h"

namespace cloudview {

c_text_frame::c_text_frame()
{
  add_data_item("Text", TEXT,
      c_cloudview_data_item::Type::text,
      "Input text");
}

void c_text_frame::set_filename(const std::string & v)
{
  filename_ = v;
}

std::string c_text_frame::get_filename()
{
  return filename_;
}

bool c_text_frame::get_text(int id, std::string & text)
{
  return false;
}

} /* namespace cloudview */
