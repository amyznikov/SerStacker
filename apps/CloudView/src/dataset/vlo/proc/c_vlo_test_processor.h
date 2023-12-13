/*
 * c_vlo_test_processor.h
 *
 *  Created on: Dec 10, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_vlo_test_processor_h__
#define __c_vlo_test_processor_h__

#include "c_vlo_data_frame_processor.h"

namespace cloudview {

class c_vlo_test_processor :
    public c_vlo_data_frame_processor
{
public:
  c_vlo_test_processor();

  bool process(c_vlo_data_frame * vlo) override;

protected:

};

} /* namespace cloudview */

#endif /* __c_vlo_test_processor_h__ */
