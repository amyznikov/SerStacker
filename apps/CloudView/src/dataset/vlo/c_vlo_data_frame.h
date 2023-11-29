/*
 * c_vlo_data_frame.h
 *
 *  Created on: Nov 20, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_vlo_data_frame_h__
#define __c_vlo_data_frame_h__

#include "c_cloudview_data_frame.h"
#include <core/io/c_vlo_file.h>

namespace cloudview {

class c_vlo_data_frame :
    public c_cloudview_data_frame
{
public:
  typedef c_vlo_data_frame this_class;
  typedef c_cloudview_data_frame base;
  typedef std::shared_ptr<this_class> sptr;

public:
  c_vlo_scan current_scan_;
};

} /* namespace cloudview */

#endif /* __c_vlo_data_frame_h__ */
