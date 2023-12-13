/*
 * c_cloudview_data_frame.cc
 *
 *  Created on: Nov 18, 2023
 *      Author: amyznikov
 */

#include "c_cloudview_data_frame.h"
#include <core/ssprintf.h>

template<>
const c_enum_member* members_of<cloudview::ViewType>()
{
  using namespace cloudview;

  static constexpr c_enum_member members[] = {
      { ViewType_Image, "Image", "Image" },
      { ViewType_PointCloud, "PointCloud", "PointCloud" },
      { ViewType_Image },
  };

  return members;
}

namespace cloudview {


void c_cloudview_data_frame::add_display_channel(int id, const std::string & name,
    const std::string & tooltip,
    double minval,
    double maxval)
{
  const DisplayChannel c = {
      .name = name,
      .tooltip = tooltip,
      .minval = minval,
      .maxval = maxval
  };

  displayChanenls_.emplace(id, c);
}


} /* namespace cloudview */
