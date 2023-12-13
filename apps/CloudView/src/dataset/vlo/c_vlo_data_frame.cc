/*
 * c_vlo_data_frame.cc
 *
 *  Created on: Nov 20, 2023
 *      Author: amyznikov
 */

#include "c_vlo_data_frame.h"
#include <core/ssprintf.h>
#include <core/debug.h>



namespace cloudview {

c_vlo_data_frame::c_vlo_data_frame()
{
  add_display_channel(AMBIENT, "AMBIENT",
      "AMBIENT",
      0,
      65535);

  add_display_channel(DISTANCES, "DISTANCES",
      "3-channel 2D Image with distances to points",
      0, 300);

  add_display_channel(DEPTH, "DEPTH",
      "3-channel 2D Image with depths (horizontal distances) to points",
      0, 300);

  add_display_channel(AREA, "AREA",
      "3-channel 2D Image with pixel intensities measured as AREA",
      0, 10000);

  add_display_channel(PEAK, "PEAK",
      "3-channel 2D Image with pixel intensities measured as PEAK",
      0, 135);

  add_display_channel(WIDTH, "WIDTH",
      "3-channel 2D Image with pixel intensities measured as WIDTH",
      0, 100);

  add_display_channel(GHOSTS, "GHOSTS",
      "3-channel binary 2D Image with Reflectors and Ghosts labeled as 255",
      0, 255);
}


bool c_vlo_data_frame::getViewData(ViewType * selectedViewType, int selectedDisplayId,
    cv::OutputArray image,
    cv::OutputArray data,
    cv::OutputArray mask)
{
  switch (*selectedViewType) {
    case ViewType_PointCloud:
      return get_point_cloud(selectedDisplayId,
          image, data);

    case ViewType_Image:
    default:
      *selectedViewType = ViewType_Image;
      return get_image(selectedDisplayId,
          image, mask);
  }

  return false;
}

bool c_vlo_data_frame::get_image(int id, cv::OutputArray image, cv::OutputArray mask)
{
  c_vlo_file::DATA_CHANNEL channel = (c_vlo_file::DATA_CHANNEL) (id);

  if ( image.needed() ) {
    //image.assign(c_vlo_file::get_image(current_scan_, channel));
    c_vlo_file::get_image(current_scan_, channel).copyTo(image);
  }

  if ( mask.needed() ) {
    mask.release();
  }
  return true;
}

bool c_vlo_data_frame::get_point_cloud(int id, cv::OutputArray points, cv::OutputArray colors)
{
  c_vlo_file::DATA_CHANNEL channel = (c_vlo_file::DATA_CHANNEL) (id);
  c_vlo_file::get_cloud3d(current_scan_, channel, points, colors);
  return true;
}

void c_vlo_data_frame::getSupportedViewTypes(std::set<ViewType> * viewTypes)
{
  viewTypes->emplace(ViewType_Image);
  viewTypes->emplace(ViewType_PointCloud);
}


} /* namespace cloudview */
