/*
 * c_textfile_frame.h
 *
 *  Created on: Jan 23, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_textfile_frame_h__
#define __c_textfile_frame_h__

#include <core/io/c_data_frame.h>

class c_textfile_frame :
    public c_data_frame
{
public:
  typedef c_textfile_frame this_class;
  typedef c_data_frame base;
  typedef std::shared_ptr<this_class> sptr;

  c_textfile_frame();

  std::string get_filename() override;

protected:
  friend class c_textfile_input_source;
  std::string filename_;
};

#endif /* __c_textfile_frame_h__ */
