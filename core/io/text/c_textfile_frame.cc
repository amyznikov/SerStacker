/*
 * c_textfile_frame.cc
 *
 *  Created on: Jan 23, 2024
 *      Author: amyznikov
 */

#include "c_textfile_frame.h"

c_textfile_frame::c_textfile_frame()
{
  viewTypes_.emplace(DataViewType_TextFile);
}

std::string c_textfile_frame::get_filename()
{
  return filename_;
}
