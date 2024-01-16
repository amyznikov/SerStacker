/*
 * c_cloudview_data_frame.cc
 *
 *  Created on: Nov 18, 2023
 *      Author: amyznikov
 */

#include "c_data_frame.h"
#include <core/ssprintf.h>

template<>
const c_enum_member* members_of<DataViewType>()
{
  static const c_enum_member members[] = {
      { DataViewType_Image, "Image", "Image" },
      { DataViewType_PointCloud, "PointCloud", "PointCloud" },
      { DataViewType_Image },
  };

  return members;
}


template<>
const c_enum_member* members_of<c_data_frame::SELECTION_MASK_MODE>()
{
  static const c_enum_member members[] = {
      { c_data_frame::SELECTION_MASK_REPLACE, "REPLACE", "REPLACE" },
      { c_data_frame::SELECTION_MASK_AND, "AND", "AND" },
      { c_data_frame::SELECTION_MASK_OR, "OR", "OR" },
      { c_data_frame::SELECTION_MASK_XOR, "XOR", "XOR" },
      { c_data_frame::SELECTION_MASK_REPLACE }
  };

  return members;
}

void c_data_frame::add_display_channel(int id, const std::string & name,
    const std::string & tooltip,
    double minval,
    double maxval)
{
  const DataDisplayChannel c = {
      .name = name,
      .tooltip = tooltip,
      .minval = minval,
      .maxval = maxval
  };

  displayChanenls_.emplace(id, c);
}
