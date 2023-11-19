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

class c_cloudview_vlo_input_source :
    public c_cloudview_input_source
{
public:
  typedef c_cloudview_vlo_input_source this_class;
  typedef c_cloudview_input_source base;
  typedef std::shared_ptr<this_class> sptr;

  c_cloudview_vlo_input_source(const std::string & filename);

  bool open(const std::string & filename) override;
  void close() override;
  bool is_open()  override;
  ssize_t size()  override;
  bool seek(ssize_t pos)  override;
  ssize_t curpos()  override;

protected:

};

} /* namespace cloudview */

#endif /* __c_vlo_input_sequence_h__ */
