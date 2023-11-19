/*
 * c_cloudview_input_sequence.h
 *
 *  Created on: Nov 18, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_cloudview_input_sequence_h__
#define __c_cloudview_input_sequence_h__

#include "c_cloudview_input_source.h"
#include <vector>

namespace cloudview {

class c_cloudview_input_sequence
{
public:
  typedef c_cloudview_input_sequence this_class;
  typedef std::shared_ptr<this_class> sptr;

  c_cloudview_input_sequence();

  const std::vector<c_cloudview_input_source::sptr> & sources();

  bool add_source(const c_cloudview_input_source::sptr & source);

protected:
  std::vector<c_cloudview_input_source::sptr> sources_;
};

} /* namespace cloudview */

#endif /* __c_cloudview_input_sequence_h__ */
