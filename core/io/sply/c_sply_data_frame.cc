/*
 * c_sply_data_frame.cc
 *
 *  Created on: Jul 24, 2024
 *      Author: amyznikov
 */

#include "c_sply_data_frame.h"

c_sply_data_frame::c_sply_data_frame()
{
  display_types_.emplace(DisplayType_PointCloud);

  add_display_channel("COLOR",
      "Point Color",
      -1, -1);

  add_display_channel("DISTANCE",
      "Distance to each point",
      -1, -1);

  add_display_channel("DEPTH",
      "3D Point depth",
      0, 300);

  add_display_channel("HEIGHT",
      "3D Point height",
      -5, 15);

//  add_display_channel("AZIMUTH",
//      "3D Point azimuth angle",
//      0, 2 * M_PI);

//  add_display_channel("ELEVATION",
//      "3D Point elevation angle",
//      -25 * M_PI, 25 * M_PI);

//  add_display_channel("GSLOPE",
//      "Local surface horizontal slope",
//      -M_PI, +M_PI);

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


bool c_sply_data_frame::get_point_cloud(const std::string & display_name,
    cv::OutputArray output_points,
    cv::OutputArray output_colors,
    cv::OutputArray output_mask)
{
  if( output_points.needed() ) {
    cv::Mat(points_).copyTo(output_points);
  }

  if( output_colors.needed() ) {

    if( display_name == "DISTANCE" ) {

      cv::Mat1f distances(points_.size(), 1);

      for ( int i = 0, n = points_.size(); i < n; ++i ) {
        distances[i][0] = std::sqrt(points_[i][0] * points_[i][0] +
            points_[i][1] * points_[i][1] +
            points_[i][2] * points_[i][2]);
      }

      output_colors.move(distances);
    }
    else if( display_name == "DEPTH" ) {

      cv::Mat1f depths(points_.size(), 1);

      for ( int i = 0, n = points_.size(); i < n; ++i ) {
        depths[i][0] = std::sqrt(points_[i][0] * points_[i][0] +
            points_[i][1] * points_[i][1] +
            points_[i][2] * points_[i][2]);
      }

      output_colors.move(depths);
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

