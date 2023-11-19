/*
 * c_vlo_dataset.h
 *
 *  Created on: Nov 18, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_vlo_dataset_h__
#define __c_vlo_dataset_h__

#include "c_cloudview_dataset.h"

namespace cloudview {

class c_vlo_dataset :
    public c_cloudview_dataset
{
public:
  typedef c_vlo_dataset this_class;
  typedef c_cloudview_dataset base;
  typedef std::shared_ptr<this_class> sptr;

  c_vlo_dataset();
  c_vlo_dataset(const std::string & dataset_path);

protected:
};

} // namespace cloudview
#endif /* __c_vlo_dataset_h__ */
