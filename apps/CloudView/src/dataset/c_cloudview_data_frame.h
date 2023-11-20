/*
 * c_cloudview_data_frame.h
 *
 *  Created on: Nov 18, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_cloudview_data_frame_h__
#define __c_cloudview_data_frame_h__

#include <memory>

namespace cloudview {

class c_cloudview_data_frame
{
public:
  typedef c_cloudview_data_frame this_class;
  typedef std::shared_ptr<this_class> sptr;

  c_cloudview_data_frame();
};

} /* namespace cloudview */

#endif /* __c_cloudview_data_frame_h__ */
