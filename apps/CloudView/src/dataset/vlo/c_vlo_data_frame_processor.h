/*
 * c_vlo_data_frame_processor.h
 *
 *  Created on: Dec 9, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_vlo_data_frame_processor_h__
#define __c_vlo_data_frame_processor_h__

#include "c_cloudview_dataframe_processor.h"
#include "c_vlo_data_frame.h"

namespace cloudview {

class c_vlo_data_frame_processor :
    public c_cloudview_dataframe_processor
{
public:
  typedef c_vlo_data_frame_processor this_class;
  typedef c_cloudview_dataframe_processor base;
  typedef std::shared_ptr<this_class> sptr;

  c_vlo_data_frame_processor();

  bool process(const c_cloudview_data_frame::sptr & dataframe) override;
  virtual bool process(c_vlo_data_frame * vlo);

protected:

};

} /* namespace cloudview */

#endif /* __c_vlo_data_frame_processor_h__ */
