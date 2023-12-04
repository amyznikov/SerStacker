/*
 * c_text_input_source.h
 *
 *  Created on: Dec 4, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_text_input_source_h__
#define __c_text_input_source_h__

#include "c_cloudview_input_source.h"
#include "c_text_frame.h"
#include <stdio.h>

namespace cloudview {

class c_text_input_source :
    public c_cloudview_input_source
{
public:
  typedef c_text_input_source this_class;
  typedef c_cloudview_input_source base;
  typedef std::shared_ptr<this_class> sptr;


  c_text_input_source();

  static const std::vector<std::string> & suffixes();
  static bool is_supported_suffix(const std::string & filename);
  static sptr load(const std::string & filename);

  bool open(const std::string & filename) override;
  void close() override;
  bool is_open() override;
  ssize_t size() override;
  ssize_t curpos() override;
  bool seek(ssize_t pos) override;
  c_cloudview_data_frame::sptr read() override;


protected:
  std::string filename_;

};

} /* namespace cloudview */

#endif /* __c_text_input_source_h__ */
