/*
 * c_video_input_source.h
 *
 *  Created on: Nov 26, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_cloudview_video_input_source_h__
#define __c_cloudview_video_input_source_h__

#include "c_cloudview_input_source.h"
#include "c_video_frame.h"
#include <core/io/c_input_source.h>

namespace cloudview {

class c_video_input_source :
    public c_cloudview_input_source
{
public:
  typedef c_video_input_source this_class;
  typedef c_cloudview_input_source base;
  typedef std::shared_ptr<this_class> sptr;

  c_video_input_source();

  bool open(const std::string & filename) override;
  void close() override;
  bool is_open() override;
  ssize_t size() override;
  ssize_t curpos() override;
  bool seek(ssize_t pos) override;
  c_cloudview_data_frame::sptr read() override;

  static bool is_supported_suffix(const std::string & filename);
  static sptr load(const std::string & filename);

protected:
  c_input_source::sptr input_source_;
};

} /* namespace cloudview */

#endif /* __c_cloudview_video_input_source_h__ */
