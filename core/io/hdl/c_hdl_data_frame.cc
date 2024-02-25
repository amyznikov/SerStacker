/*
 * c_hdl_data_frame.cc
 *
 *  Created on: Feb 25, 2024
 *      Author: amyznikov
 */

#include "c_hdl_data_frame.h"

c_hdl_data_frame::c_hdl_data_frame()
{
  cleanup();
}

void c_hdl_data_frame::cleanup()
{
  setup_default_channels();
}

void c_hdl_data_frame::setup_default_channels()
{
  displayChannels_.clear();

  add_display_channel("AMBIENT",
      "AMBIENT",
      0,
      65535);

  add_display_channel("DISTANCES",
      "3-channel 2D Image with distances to points",
      0, 30000);

  add_display_channel("DEPTH",
      "3-channel 2D Image with depths (horizontal distances) to points",
      0, 30000);

  add_display_channel("HEIGHT",
      "3-channel 2D Image with height (vertical 3D coordinate) of points",
      -100, 1000);

  add_display_channel("AREA",
      "3-channel 2D Image with pixel intensities measured as AREA",
      0, 10000);

  add_display_channel("PEAK",
      "3-channel 2D Image with pixel intensities measured as PEAK",
      0, 135);

  add_display_channel("SELECTION_MASK",
      "1- or 3-channel binary 2D Image representing current selection mask",
      0, 255);

  viewTypes_.clear();
//  viewTypes_.emplace(DataViewType_Image);
  viewTypes_.emplace(DataViewType_PointCloud);
}


bool c_hdl_data_frame::get_data(DataViewType * selectedViewType,
    const std::string & channelName,
    cv::OutputArray image,
    cv::OutputArray data,
    cv::OutputArray mask)
{

  * selectedViewType =
      DataViewType_PointCloud;

  if ( !current_frame_ ) {
    return false;
  }

  if ( * selectedViewType == DataViewType_PointCloud ) {

    convert_to_cartesian(current_frame_->points, image);

    data.create(image.size(), CV_32FC1);

    cv::Mat1f colors =
        data.getMatRef();

    for ( int i = 0, n = current_frame_->points.size(); i < n; ++i ) {

      const c_hdl_point & p =
          current_frame_->points[i];

      colors[i][0] = p.intensity;
    }

  }

  return true;
}


