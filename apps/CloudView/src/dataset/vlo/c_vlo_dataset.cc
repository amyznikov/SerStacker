/*
 * c_vlo_dataset.cc
 *
 *  Created on: Nov 18, 2023
 *      Author: amyznikov
 */

#include "c_vlo_dataset.h"

namespace cloudview {

c_vlo_dataset::c_vlo_dataset() :
    base("")
{
}

c_vlo_dataset::c_vlo_dataset(const std::string & dataset_path) :
    base(dataset_path)
{
}

} // namespace cloudview
