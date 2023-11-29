/*
 * c_vlo_dataset.cc
 *
 *  Created on: Nov 18, 2023
 *      Author: amyznikov
 */

#include "c_vlo_dataset.h"
#include "c_cloudview_vlo_input_source.h"

namespace cloudview {

c_vlo_dataset::c_vlo_dataset(const type * _type,
    const std::string & dataset_name) :
    base(_type, dataset_name)
{
}

c_cloudview_input_source::sptr c_vlo_dataset::create_input_source(const std::string & filename) const
{
  return nullptr;//  c_cloudview_input_source::sptr(new c_cloudview_vlo_input_source(filename));
}

} // namespace cloudview
