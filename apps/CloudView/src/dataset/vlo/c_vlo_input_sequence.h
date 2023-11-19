/*
 * c_vlo_input_sequence.h
 *
 *  Created on: Nov 18, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_vlo_input_sequence_h__
#define __c_vlo_input_sequence_h__

#include "c_cloudview_input_sequence.h"
#include <core/io/c_vlo_file.h>

namespace cloudview {

class c_vlo_input_sequence :
    public c_cloudview_input_sequence
{
public:
  typedef c_vlo_input_sequence this_class;
  typedef c_cloudview_input_sequence base;
  typedef std::shared_ptr<this_class> sptr;

  c_vlo_input_sequence(const std::string & sequence_name);

protected:

};

} /* namespace cloudview */

#endif /* __c_vlo_input_sequence_h__ */
