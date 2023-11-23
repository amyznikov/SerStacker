/*
 * c_cloudview_dataframe_processor.h
 *
 *  Created on: Nov 22, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_cloudview_dataframe_processor_h__
#define __c_cloudview_dataframe_processor_h__

#include "c_cloudview_data_frame.h"

namespace cloudview {

class c_cloudview_dataframe_processor
{
public:
  typedef c_cloudview_dataframe_processor this_class;
  typedef std::shared_ptr<this_class> sptr;

  c_cloudview_dataframe_processor();
  virtual ~c_cloudview_dataframe_processor();

  bool process(const c_cloudview_data_frame::sptr & frame)
  {
    return false;
  }

protected:

};

} // namespace cloudview

#endif /* __c_cloudview_dataframe_processor_h__ */
