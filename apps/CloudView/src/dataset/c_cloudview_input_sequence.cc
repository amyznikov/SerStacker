/*
 * c_cloudview_input_sequence.cc
 *
 *  Created on: Nov 18, 2023
 *      Author: amyznikov
 */

#include "c_cloudview_input_sequence.h"
#include <core/debug.h>

namespace cloudview {

c_cloudview_input_sequence::c_cloudview_input_sequence()
{
}

const std::vector<c_cloudview_input_source::sptr> & c_cloudview_input_sequence::sources()
{
  return sources_;
}

bool c_cloudview_input_sequence::add_source(const c_cloudview_input_source::sptr & source)
{
  if ( source ) {
    sources_.emplace_back(source);
    return true;
  }

  CF_ERROR("c_cloudview_input_sequence: NULL source specified to add to input sequence");
  return false;
}

} /* namespace cloudview */
