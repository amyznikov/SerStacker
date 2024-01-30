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
      { DataViewType_TextFile, "TextFile", "TextFile" },
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

void c_data_frame::add_display_channel(const std::string & name,
    const std::string & tooltip,
    double minval,
    double maxval)
{
  const DataDisplayChannel c = {
      .tooltip = tooltip,
      .minval = minval,
      .maxval = maxval
  };

  displayChannels_.emplace(name, c);
}

bool c_data_frame::set_data(DataViewType viewType,
    const std::string & channelName,
    cv::InputArray image,
    cv::InputArray data,
    cv::InputArray mask)
{
  const auto pos =
      displayChannels_.find(channelName);

  if ( pos != displayChannels_.end() ) {

    image.copyTo(pos->second.image);
    data.copyTo(pos->second.data);
    mask.copyTo(pos->second.mask);

  }
  else {

    DataDisplayChannel c = {
        .tooltip = "",
        .minval = -1,
        .maxval = -1
    };

    image.copyTo(c.image);
    data.copyTo(c.data);
    mask.copyTo(c.mask);

    displayChannels_.emplace(channelName, c);
  }

  return true;
}
