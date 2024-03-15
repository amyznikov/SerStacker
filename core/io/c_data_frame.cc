/*
 * c_cloudview_data_frame.cc
 *
 *  Created on: Nov 18, 2023
 *      Author: amyznikov
 */

#include "c_data_frame.h"
#include <core/proc/reduce_channels.h>
#include <core/ssprintf.h>
#include <core/debug.h>

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

    if( mask.empty() || mask.channels() == 1 ) {
      mask.copyTo(pos->second.mask);
    }
    else {
      reduce_color_channels(mask, pos->second.mask,
          cv::REDUCE_MAX);
    }

  }
  else {

    DataDisplayChannel c = {
        .tooltip = channelName,
        .minval = -1,
        .maxval = -1
    };

    image.copyTo(c.image);
    data.copyTo(c.data);

    if ( !mask.empty() ) {
      if ( mask.channels() == 1 ) {
        mask.copyTo(c.mask);
      }
      else {
        reduce_color_channels(mask, c.mask,
            cv::REDUCE_MAX);
      }
    }

    displayChannels_.emplace(channelName, c);
  }

  return true;
}


void c_data_frame::update_selection(cv::InputArray mask, SELECTION_MASK_MODE mode)
{
  static const auto dup_channels =
      [](const cv::Mat & src, cv::Mat & dst, int cn) -> bool {

        std::vector<cv::Mat> channels(cn);

        for( int i = 0; i < cn; ++i ) {
          channels[i] = src;
        }

        cv::merge(channels, dst);
        return true;
      };


  if( selection_mask_.empty() || mode == SELECTION_MASK_REPLACE ) {
    mask.getMat().copyTo(selection_mask_);
  }
  else if ( !mask.empty() ) {

    cv::Mat cmask;

    if ( selection_mask_.channels() == mask.channels() ) {
      cmask = mask.getMat();
    }
    else if ( selection_mask_.channels() == 1 ) {
      dup_channels(selection_mask_, selection_mask_, mask.channels());
      cmask = mask.getMat();
    }
    else if ( mask.channels() == 1 ) {
      dup_channels(mask.getMat(), cmask, selection_mask_.channels());
    }
    else {

      CF_ERROR("Unsupported combination of mask channels:\n"
          "selection_mask: %dx%d channels=%d\n"
          "mask: %dx%d channels=%d\n"
          "",
          selection_mask_.cols, selection_mask_.rows, selection_mask_.channels(),
          mask.cols(), mask.rows(), mask.channels());

      return;
    }

    switch (mode) {
      case SELECTION_MASK_AND:
        cv::bitwise_and(cmask, selection_mask_, selection_mask_);
        break;
      case SELECTION_MASK_OR:
        cv::bitwise_or(cmask, selection_mask_, selection_mask_);
        break;
      case SELECTION_MASK_XOR:
        cv::bitwise_xor(cmask, selection_mask_, selection_mask_);
        break;
      default:
        CF_ERROR("Not implemented mask operation detected: mode=%d", mode);
        break;
    }
  }
}

