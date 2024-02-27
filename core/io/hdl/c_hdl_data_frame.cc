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

  add_display_channel("DEPTH",
      "3D Point depth",
      0, 300);


  add_display_channel("INTENSITY",
      "3D Point intensity",
      0, 1);

  add_display_channel("DISTANCES",
      "3D Point distance",
      0, 300);

  add_display_channel("HEIGHT",
      "3D Point height",
      -5, 15);

  add_display_channel("AZIMUTH",
      "3D Point azimuth angle",
      0, 2 * M_PI);

  add_display_channel("ELEVATION",
      "3D Point elevation angle",
      -25 * M_PI, 25 * M_PI);

  add_display_channel("LASER_ID",
      "3D Point laser_id",
      0, 128);

  add_display_channel("LASER_RING",
      "3D Point laser ring",
      0, 128);

  add_display_channel("DATABLOCK",
      "3D Point Lidar Data block id",
      0, 12);

  add_display_channel("TIMESTAMP",
      "3D Point time stamp relative to start of rotation",
      0, 100);


//  add_display_channel("SELECTION_MASK",
//      "1- or 3-channel binary 2D Image representing current selection mask",
//      0, 255);

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

    if ( channelName == "INTENSITY" ) {

      for ( int i = 0, n = current_frame_->points.size(); i < n; ++i ) {
        colors[i][0] = current_frame_->points[i].intensity;
      }

    }
    else if( channelName == "DISTANCES" ) {

      for ( int i = 0, n = current_frame_->points.size(); i < n; ++i ) {
        colors[i][0] = current_frame_->points[i].distance;
      }

    }
    else if( channelName == "HEIGHT" ) {

      const cv::Mat3f pts =
          image.getMatRef();

      for ( int i = 0, n = pts.rows; i < n; ++i ) {
        colors[i][0] = pts[i][0][2];
      }

    }
    else if( channelName == "AZIMUTH" ) {

      for ( int i = 0, n = current_frame_->points.size(); i < n; ++i ) {
        colors[i][0] = current_frame_->points[i].azimuth;
      }

    }
    else if( channelName == "ELEVATION" ) {

      for ( int i = 0, n = current_frame_->points.size(); i < n; ++i ) {
        colors[i][0] = current_frame_->points[i].elevation;
      }

    }
    else if( channelName == "LASER_ID" ) {

      for ( int i = 0, n = current_frame_->points.size(); i < n; ++i ) {
        colors[i][0] = current_frame_->points[i].laser_id;
      }

    }
    else if( channelName == "LASER_RING" ) {

      for ( int i = 0, n = current_frame_->points.size(); i < n; ++i ) {
        colors[i][0] = current_frame_->points[i].laser_ring;
      }

    }
    else if( channelName == "DATABLOCK" ) {

      for ( int i = 0, n = current_frame_->points.size(); i < n; ++i ) {
        colors[i][0] = current_frame_->points[i].datablock;
      }

    }
    else if( channelName == "TIMESTAMP" ) {

      double tsmin = DBL_MAX;
      for( int i = 0, n = current_frame_->points.size(); i < n; ++i ) {
        const double ts = current_frame_->points[i].timestamp;
        if( ts < tsmin ) {
          tsmin = ts;
        }
      }

      for( int i = 0, n = current_frame_->points.size(); i < n; ++i ) {
        colors[i][0] = current_frame_->points[i].timestamp - tsmin;
      }
    }

//    else if( channelName == "SELECTION_MASK" ) {
//
//      if( selection_mask_.size() != colors.size() ) {
//
//        colors.setTo(cv::Scalar::all(255));
//
//      }
//
//    }
    else { // DEPTH

      for ( int i = 0, n = current_frame_->points.size(); i < n; ++i ) {
        colors[i][0] = compute_depth(current_frame_->points[i]);
      }
    }

  }

  return true;
}


