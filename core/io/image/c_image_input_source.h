/*
 * c_image_input_source.h
 *
 *  Created on: Dec 15, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_image_input_source_h__
#define __c_image_input_source_h__

#include <core/io/c_input_source.h>

class c_image_input_source :
    public c_input_source
{
public:
  typedef c_image_input_source this_class;
  typedef c_input_source base;
  typedef std::shared_ptr<this_class> sptr;

  bool read(c_data_frame::sptr & output_frame) override;

protected:
  c_image_input_source(const std::string & filename);
};

#endif /* __c_video_input_source_h__ */
