/*
 * c_text_frame.h
 *
 *  Created on: Dec 3, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_text_frame_h__
#define __c_text_frame_h__

#include "c_cloudview_data_frame.h"

namespace cloudview {

class c_text_frame :
    public c_cloudview_data_frame
{
public:
  typedef c_text_frame this_class;
  typedef c_cloudview_data_frame base;
  typedef std::shared_ptr<this_class> sptr;

  enum
  {
    TEXT = 0,
  };

  c_text_frame();

  void set_filename(const std::string & v);
  std::string get_filename() override;

  bool get_text(int id, std::string & text) override;

protected:
  std::string filename_;
};

} /* namespace cloudview */

#endif /* __c_text_frame_h__ */
