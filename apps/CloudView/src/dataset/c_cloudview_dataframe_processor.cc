/*
 * c_cloudview_data_frame_processor.cc
 *
 *  Created on: Dec 9, 2023
 *      Author: amyznikov
 */

#include "c_cloudview_dataframe_processor.h"

#include <core/debug.h>

namespace cloudview {

c_cloudview_dataframe_processor::c_cloudview_dataframe_processor()
{
}

bool c_cloudview_dataframe_processor::enabled() const
{
  return enabled_;
}

void c_cloudview_dataframe_processor::set_enabled(bool v)
{
  if( enabled_ != v ) {
    enabled_ = v;
    onstatechanged();
  }
}

void c_cloudview_dataframe_processor::onstatechanged()
{
}


bool c_cloudview_dataframe_processor::process(const c_cloudview_data_frame::sptr & dataframe)
{
  CF_DEBUG("c_cloudview_data_frame_processor: abstract function not implemented");
  return false;
}


c_cloudview_processor::c_cloudview_processor()
{
}

bool c_cloudview_processor::process(const c_cloudview_data_frame::sptr & dataframe)
{
  for( const auto & processor : processors_ ) {
    if( processor && processor->enabled() && !processor->process(dataframe) ) {
      CF_ERROR("processor->process() fails");
      return false;
    }
  }

  return true;
}


} /* namespace cloudview */
