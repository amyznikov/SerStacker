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

  void set_name(const std::string & v);
  const std::string & name() const;

  const std::vector<c_cloudview_input_source::sptr> & sources();

protected:
  c_cloudview_input_sequence(const std::string & sequence_name);

protected:
  std::string sequence_name_;
  std::vector<c_cloudview_input_source::sptr> sources_;

};

} /* namespace cloudview */

#endif /* __c_cloudview_input_sequence_h__ */
