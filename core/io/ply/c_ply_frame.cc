/*
 * c_ply_frame.cc
 *
 *  Created on: Jan 24, 2024
 *      Author: amyznikov
 */

#include "c_ply_frame.h"

c_ply_frame::c_ply_frame()
{
  _display_types.emplace(DisplayType_TextFile);
  _display_types.emplace(DisplayType_PointCloud);

  add_image_display("COLORS",
      "Point Color",
      -1, -1);

  add_image_display("DISTANCES",
      "3-channel 2D Image with distances to points",
      -1, -1);

  add_image_display("X",
      "X coordinate",
      -1, -1);

  add_image_display("Y",
      "Y coordinate",
      -1, -1);

  add_image_display("Z",
      "Z coordinate",
      -1, -1);
}

std::string c_ply_frame::get_filename()
{
  return filename_;
}

bool c_ply_frame::get_point_cloud(const std::string & display_name,
      cv::OutputArray output_points,
      cv::OutputArray output_colors,
      cv::OutputArray output_mask,
      std::vector<uint64_t> * output_pids)
{

  if ( output_pids ) {
    output_pids->clear();
  }

  if( output_points.needed() ) {
    cv::Mat(points_).copyTo(output_points);
  }

  if( output_colors.needed() ) {

    if( display_name == "DISTANCES" ) {

      cv::Mat1f distances(points_.size(), 1);

      for ( int i = 0, n = points_.size(); i < n; ++i ) {
        distances[i][0] = std::sqrt(points_[i][0] * points_[i][0] +
            points_[i][1] * points_[i][1] +
            points_[i][2] * points_[i][2]);
      }

      output_colors.move(distances);
    }
    else if( display_name == "X" ) {
      cv::extractChannel(cv::Mat(points_), output_colors, 0);
    }
    else if( display_name == "Y" ) {
      cv::extractChannel(cv::Mat(points_), output_colors, 1);
    }
    else if( display_name == "Z" ) {
      cv::extractChannel(cv::Mat(points_), output_colors, 2);
    }
    else  {
      cv::Mat(colors_).copyTo(output_colors);
    }
  }

  if( output_mask.needed() ) {
    output_mask.release();
  }

  return true;
}
