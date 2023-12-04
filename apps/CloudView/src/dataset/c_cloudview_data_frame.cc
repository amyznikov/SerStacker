/*
 * c_cloudview_data_frame.cc
 *
 *  Created on: Nov 18, 2023
 *      Author: amyznikov
 */

#include "c_cloudview_data_frame.h"

namespace cloudview {

void c_cloudview_data_frame::add_data_item(const std::string & name, int id,
    const c_cloudview_data_item::Type & type,
    const std::string & tooltip)
{
  items_.emplace_back(c_cloudview_data_item(name,
      type,
      id,
      tooltip));
}



} /* namespace cloudview */
