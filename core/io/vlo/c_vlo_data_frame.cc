/*
 * c_vlo_data_frame.cc
 *
 *  Created on: Dec 15, 2023
 *      Author: amyznikov
 */

#include "c_vlo_data_frame.h"
#include <functional>
#include <core/proc/reduce_channels.h>
#include <core/debug.h>

namespace {

static constexpr float distance_scale = 0.01; // [cm] -> [m]

template<class PixelType>
bool get_vlo_point_cloud_(const c_vlo_scan & scan,
    cv::InputArray _colors_image,
    cv::OutputArray output_points,
    cv::OutputArray output_colors,
    cv::InputArray selection_mask = cv::noArray())
{

  if ( _colors_image.empty() ) {
    return false;
  }


  std::vector<cv::Vec3f> points;
  std::vector<PixelType> colors;

  const int cn =
      _colors_image.channels();

  const cv::Mat_<PixelType> colors_image =
      _colors_image.getMat();

  points.reserve(scan.size.area());
  colors.reserve(scan.size.area());

  vlo_points_callback(scan, selection_mask,
      [&](int l, int s, int e) {

        const PixelType * srcp =
            colors_image[l];

        points.emplace_back(scan.clouds[e][l][s] * distance_scale);
        colors.emplace_back(cn == 1 ? srcp[s * cn] : srcp[s * cn + e]);
      });



  cv::Mat(colors).copyTo(output_colors);
  cv::Mat(points).copyTo(output_points);

  return true;
}


bool get_vlo_point_cloud(const c_vlo_scan & scan,
    cv::InputArray colors_image,
    cv::OutputArray points,
    cv::OutputArray colors,
    cv::InputArray selection_mask = cv::noArray())
{
  switch (colors_image.depth()) {
    case CV_8U:
      return get_vlo_point_cloud_<uint8_t>(scan, colors_image, points, colors, selection_mask);
    case CV_8S:
      return get_vlo_point_cloud_<int8_t>(scan, colors_image, points, colors, selection_mask);
    case CV_16U:
      return get_vlo_point_cloud_<uint16_t>(scan, colors_image, points, colors, selection_mask);
    case CV_16S:
      return get_vlo_point_cloud_<int16_t>(scan, colors_image, points, colors, selection_mask);
    case CV_32S:
      return get_vlo_point_cloud_<int32_t>(scan, colors_image, points, colors, selection_mask);
    case CV_32F:
      return get_vlo_point_cloud_<float>(scan, colors_image, points, colors, selection_mask);
    case CV_64F:
      return get_vlo_point_cloud_<double>(scan, colors_image, points, colors, selection_mask);
  }
  return false;
}

//
//static bool dup_channels(const cv::Mat & src, cv::Mat & dst, int cn)
//{
//  std::vector<cv::Mat> channels(cn);
//
//  for( int i = 0; i < cn; ++i ) {
//    channels[i] = src;
//  }
//
//  cv::merge(channels, dst);
//  return true;
//}

}

c_vlo_data_frame::c_vlo_data_frame()
{
  setup_default_channels();
}

void c_vlo_data_frame::setup_default_channels()
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

  add_display_channel("WIDTH",
      "3-channel 2D Image with pixel intensities measured as WIDTH",
      0, 100);

  add_display_channel("ECHO",
      "Echo index colored as R, G, B",
      0, 255);

  add_display_channel("SELECTION_MASK",
      "1- or 3-channel binary 2D Image representing current selection mask",
      0, 255);

  viewTypes_.clear();
  viewTypes_.emplace(DataViewType_Image);
  viewTypes_.emplace(DataViewType_PointCloud);

}

bool c_vlo_data_frame::get_data(DataViewType * viewType,
    const std::string & channelName,
    cv::OutputArray output_image,
    cv::OutputArray output_colors,
    cv::OutputArray output_mask)
{
  switch (*viewType) {
    case DataViewType_PointCloud:
      case DataViewType_Image:
      break;
    default:
      *viewType = DataViewType_Image;
      break;
  }

  if( output_image.needed() ) {

    /////////////////////////////////////////////////////////////////////////////////////////////
    if( channelName == "SELECTION_MASK" ) {

      switch (*viewType) {
        case DataViewType_Image:
          selection_mask_.copyTo(output_image);
          break;

        case DataViewType_PointCloud:
          if( selection_mask_.empty() ) {
            output_image.release();
          }
          else {

            std::vector<cv::Vec3f> points;
            std::vector<cv::Vec3b> colors;

            points.reserve(current_scan_.size.area());
            colors.reserve(current_scan_.size.area());

            vlo_points_callback(current_scan_, selection_mask_,
                [&](int l, int s, int e) {
                  points.emplace_back(current_scan_.clouds[e][l][s] * distance_scale);
                  colors.emplace_back(255 * (e == 0), 255 * (e == 1), 255 * (e == 2));
                });

            cv::Mat(points).copyTo(output_image);
            cv::Mat(colors).copyTo(output_colors);
          }
          break;
      }

    }
    else if( channelName == "ECHO" ) {
      switch (*viewType) {
        case DataViewType_Image: {

          cv::Mat3b image(current_scan_.size, cv::Vec3b(0, 0, 0));

          vlo_points_callback(current_scan_, selection_mask_,
              [&](int l, int s, int e) {
                image[l][s][e] = 255;
              });

          output_image.move(image);

          break;
        }

        case DataViewType_PointCloud: {

            std::vector<cv::Vec3f> points;
            std::vector<cv::Vec3b> colors;

            points.reserve(current_scan_.size.area());
            colors.reserve(current_scan_.size.area());

            vlo_points_callback(current_scan_, selection_mask_,
                [&](int l, int s, int e) {
                  points.emplace_back(current_scan_.clouds[e][l][s] * distance_scale);
                  colors.emplace_back(255 * (e == 0), 255 * (e == 1), 255 * (e == 2));
                });

            cv::Mat(points).copyTo(output_image);
            cv::Mat(colors).copyTo(output_colors);
          }
          break;
      }
    }
    else {

      cv::Mat image, data, mask;

      if( !channelName.empty() ) {

        const auto pos =
            displayChannels_.find(channelName);

        if( pos != displayChannels_.end() ) {
          image = pos->second.image;
          data = pos->second.data;
        }
      }

      if( image.empty() ) {

        VLO_DATA_CHANNEL vloChannel;

        if( channelName == "AMBIENT" ) {
          vloChannel = VLO_DATA_CHANNEL_AMBIENT;
        }
        else if( channelName == "DISTANCES" ) {
          vloChannel = VLO_DATA_CHANNEL_DISTANCES;
        }
        else if( channelName == "DEPTH" ) {
          vloChannel = VLO_DATA_CHANNEL_DEPTH;
        }
        else if( channelName == "HEIGHT" ) {
          vloChannel = VLO_DATA_CHANNEL_HEIGHT;
        }
        else if( channelName == "AREA" ) {
          vloChannel = VLO_DATA_CHANNEL_AREA;
        }
        else if( channelName == "PEAK" ) {
          vloChannel = VLO_DATA_CHANNEL_PEAK;
        }
        else if( channelName == "WIDTH" ) {
          vloChannel = VLO_DATA_CHANNEL_WIDTH;
        }
        else if( channelName.empty() ) {
          vloChannel = VLO_DATA_CHANNEL_AMBIENT;
        }
        else {
          CF_ERROR("No such display channel : '%s'", channelName.c_str());
          return false;
        }

        /////////////////////////////////////////////////////////////////////////////////////////////


        if( *viewType == DataViewType_Image ) {
          image =
              get_vlo_image(current_scan_,
                  vloChannel,
                  selection_mask_);
        }
        else if( *viewType == DataViewType_PointCloud ) {

          get_vlo_point_cloud(current_scan_,
              get_vlo_image(current_scan_,
                  vloChannel,
                  selection_mask_),
              image,
              data,
              selection_mask_);

        }
      }

      if( *viewType == DataViewType_Image ) {
        output_image.move(image);
      }
      else if( *viewType == DataViewType_PointCloud ) {

        if ( image.size() == data.size() ) {
          output_image.move(image);
          output_colors.move(data);
        }
        else if (data.empty() && image.size() == current_scan_.size ) {

          get_vlo_point_cloud(current_scan_,
              image,
              output_image,
              output_colors,
              selection_mask_);

        }
        else {
          output_image.release();
          output_colors.release();
        }
      }

    }

  }

  return true;
}


void c_vlo_data_frame::cleanup()
{
  selection_mask_.release();
  setup_default_channels();
}



