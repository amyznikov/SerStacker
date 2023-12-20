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
  static constexpr c_enum_member members[] = {
      { DataViewType_Image, "Image", "Image" },
      { DataViewType_PointCloud, "PointCloud", "PointCloud" },
      { DataViewType_Image },
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
