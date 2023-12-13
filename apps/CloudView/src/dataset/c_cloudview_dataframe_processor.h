/*
 * c_cloudview_data_frame_processor.h
 *
 *  Created on: Dec 9, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_cloudview_data_frame_processor_h__
#define __c_cloudview_data_frame_processor_h__

#include "c_cloudview_data_frame.h"

namespace cloudview {

class c_cloudview_dataframe_processor
{
public:
  typedef c_cloudview_dataframe_processor this_class;
  typedef std::shared_ptr<this_class> sptr;

  c_cloudview_dataframe_processor();
  virtual ~c_cloudview_dataframe_processor() = default;

  bool enabled() const;
  void set_enabled(bool v);

  virtual bool process(const c_cloudview_data_frame::sptr & dataframe);

protected:
  virtual void onstatechanged();

protected:
  bool enabled_ = false;
};


class c_cloudview_processor
{
public:
  typedef c_cloudview_processor this_class;
  typedef std::shared_ptr<this_class> sptr;

  c_cloudview_processor();
  virtual ~c_cloudview_processor() = default;

  virtual bool process(const c_cloudview_data_frame::sptr & dataframe);

protected:
  std::vector<c_cloudview_dataframe_processor::sptr> processors_;
};

} /* namespace cloudview */

#endif /* __c_cloudview_data_frame_processor_h__ */
