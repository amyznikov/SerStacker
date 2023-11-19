/*
 * c_vlo_dataset.cc
 *
 *  Created on: Nov 18, 2023
 *      Author: amyznikov
 */

#include "c_vlo_dataset.h"

namespace cloudview {

c_vlo_dataset::c_vlo_dataset(const type * _type,
    const std::string & dataset_name) :
    base(_type, dataset_name)
{
}

} // namespace cloudview
