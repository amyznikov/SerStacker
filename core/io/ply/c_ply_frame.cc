/*
 * c_ply_frame.cc
 *
 *  Created on: Jan 24, 2024
 *      Author: amyznikov
 */

#include "c_ply_frame.h"

c_ply_frame::c_ply_frame()
{
  viewTypes_.emplace(DataViewType_TextFile);
  viewTypes_.emplace(DataViewType_PointCloud);

  add_display_channel("COLORS",
      "Point Color",
      -1, -1);

  add_display_channel("DISTANCES",
      "3-channel 2D Image with distances to points",
      -1, -1);

  add_display_channel("X",
      "X coordinate",
      -1, -1);

  add_display_channel("Y",
      "Y coordinate",
      -1, -1);

  add_display_channel("Z",
      "Z coordinate",
      -1, -1);
}

std::string c_ply_frame::get_filename()
{
  return filename_;
}

bool c_ply_frame::get_data(DataViewType * selectedViewType,
    const std::string & channelName,
    cv::OutputArray image,
    cv::OutputArray data,
    cv::OutputArray mask)
{
  *selectedViewType =
      DataViewType_PointCloud;

  if( image.needed() ) {
    cv::Mat(points_).copyTo(image);
  }

  if( data.needed() ) {

    if( channelName == "DISTANCES" ) {

      cv::Mat1f distances(points_.size(), 1);

      for ( int i = 0, n = points_.size(); i < n; ++i ) {
        distances[i][0] = std::sqrt(points_[i][0] * points_[i][0] +
            points_[i][1] * points_[i][1] +
            points_[i][2] * points_[i][2]);
      }

      data.move(distances);
    }
    else if( channelName == "X" ) {
      cv::extractChannel(cv::Mat(points_), data, 0);
    }
    else if( channelName == "Y" ) {
      cv::extractChannel(cv::Mat(points_), data, 1);
    }
    else if( channelName == "Z" ) {
      cv::extractChannel(cv::Mat(points_), data, 2);
    }
    else  {
      cv::Mat(colors_).copyTo(data);
    }
  }

  if( mask.needed() ) {
    mask.release();
  }

  return true;
}
