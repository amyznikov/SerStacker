/*
 * c_vlo_frame.cc
 *
 *  Created on: Dec 15, 2023
 *      Author: amyznikov
 */

#include "c_vlo_frame.h"
#include <functional>
#include <core/proc/reduce_channels.h>
#include <core/ssprintf.h>
#include <core/debug.h>


template<>
const c_enum_member* members_of<c_vlo_frame::SELECTION_MASK_MODE>()
{
  static constexpr c_enum_member members[] = {
      { c_vlo_frame::SELECTION_MASK_REPLACE, "REPLACE", "REPLACE" },
      { c_vlo_frame::SELECTION_MASK_AND, "AND", "AND" },
      { c_vlo_frame::SELECTION_MASK_OR, "OR", "OR" },
      { c_vlo_frame::SELECTION_MASK_XOR, "XOR", "XOR" },
      { c_vlo_frame::SELECTION_MASK_REPLACE }
  };

  return members;
}



namespace {

template<class ScanType, class PixelType>
bool get_vlo_point_cloud_(const ScanType & scan,
    cv::InputArray _colors_image,
    cv::OutputArray output_points,
    cv::OutputArray output_colors,
    cv::InputArray selection_mask = cv::noArray())
{

  std::vector<cv::Vec3f> points;
  std::vector<PixelType> colors;

  const int cn =
      _colors_image.channels();

  const cv::Mat_<PixelType> colors_image =
      _colors_image.getMat();

  points.reserve(scan.NUM_LAYERS * scan.NUM_SLOTS);
  colors.reserve(scan.NUM_LAYERS * scan.NUM_SLOTS);

  get_vlo_points3d(scan, selection_mask,
      [&](int l, int s, int e, double x, double y, double z, const auto & echo) {

        const PixelType * srcp =
            colors_image[l];

        points.emplace_back((float)x, (float)y, (float)z);
        colors.emplace_back(cn == 1 ? srcp[s * cn] : srcp[s * cn + e]);
      });


  cv::Mat(colors).copyTo(output_colors);
  cv::Mat(points).copyTo(output_points);

  return true;
}


template<class ScanType>
bool get_vlo_point_cloud(const ScanType & scan,
    cv::InputArray colors_image,
    cv::OutputArray points,
    cv::OutputArray colors,
    cv::InputArray selection_mask = cv::noArray())
{
  switch (colors_image.depth()) {
    case CV_8U:
      return get_vlo_point_cloud_<ScanType, uint8_t>(scan, colors_image, points, colors, selection_mask);
    case CV_8S:
      return get_vlo_point_cloud_<ScanType, int8_t>(scan, colors_image, points, colors, selection_mask);
    case CV_16U:
      return get_vlo_point_cloud_<ScanType, uint16_t>(scan, colors_image, points, colors, selection_mask);
    case CV_16S:
      return get_vlo_point_cloud_<ScanType, int16_t>(scan, colors_image, points, colors, selection_mask);
    case CV_32S:
      return get_vlo_point_cloud_<ScanType, int32_t>(scan, colors_image, points, colors, selection_mask);
    case CV_32F:
      return get_vlo_point_cloud_<ScanType, float>(scan, colors_image, points, colors, selection_mask);
    case CV_64F:
      return get_vlo_point_cloud_<ScanType, double>(scan, colors_image, points, colors, selection_mask);
  }
  return false;
}

bool get_vlo_point_cloud(const c_vlo_scan & scan,
    cv::InputArray colors_image,
    cv::OutputArray points,
    cv::OutputArray colors,
    cv::InputArray selection_mask = cv::noArray())
{
  switch (scan.version) {
    case VLO_VERSION_1:
      return get_vlo_point_cloud(scan.scan1, colors_image, points, colors, selection_mask);
    case VLO_VERSION_3:
      return get_vlo_point_cloud(scan.scan3, colors_image, points, colors, selection_mask);
    case VLO_VERSION_5:
      return get_vlo_point_cloud(scan.scan5, colors_image, points, colors, selection_mask);
    case VLO_VERSION_6_SLM:
      return get_vlo_point_cloud(scan.scan6_slm, colors_image, points, colors, selection_mask);
  }

  return false;
}

static bool dup_channels(const cv::Mat & src, cv::Mat & dst, int cn)
{
  std::vector<cv::Mat> channels(cn);

  for( int i = 0; i < cn; ++i ) {
    channels[i] = src;
  }

  cv::merge(channels, dst);
  return true;
}

}

c_vlo_frame::c_vlo_frame()
{
  add_display_channel(AMBIENT, "AMBIENT",
      "AMBIENT",
      0,
      65535);

  add_display_channel(DISTANCES, "DISTANCES",
      "3-channel 2D Image with distances to points",
      0, 30000);

  add_display_channel(DEPTH, "DEPTH",
      "3-channel 2D Image with depths (horizontal distances) to points",
      0, 30000);

  add_display_channel(HEIGHT, "HEIGHT",
      "3-channel 2D Image with height (vertical 3D coordinate) of points",
      -100, 1000);

  add_display_channel(AREA, "AREA",
      "3-channel 2D Image with pixel intensities measured as AREA",
      0, 10000);

  add_display_channel(PEAK, "PEAK",
      "3-channel 2D Image with pixel intensities measured as PEAK",
      0, 135);

  add_display_channel(WIDTH, "WIDTH",
      "3-channel 2D Image with pixel intensities measured as WIDTH",
      0, 100);

  add_display_channel(SELECTION_MASK, "SELECTION_MASK",
      "1- or 3-channel binary 2D Image representing current selection mask",
      0, 255);

  viewTypes_.emplace(DataViewType_Image);
  viewTypes_.emplace(DataViewType_PointCloud);
}

bool c_vlo_frame::get_display_data(DataViewType * viewType, int displayId,
    cv::OutputArray output_image,
    cv::OutputArray output_colors3d,
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

    switch (displayId) {

      /////////////////////////////////////////////////////////////////////////////////////////////
      case SELECTION_MASK:

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

              points.reserve(vlo_scan_size(current_scan_).area());
              colors.reserve(vlo_scan_size(current_scan_).area());

              get_vlo_points3d(current_scan_, selection_mask_,
                  [&points, &colors](int l, int s, int e, double x, double y, double z, const auto & echo) {
                    points.emplace_back((float)x, (float)y, (float)z);
                    colors.emplace_back(255 * (e == 0), 255 * (e == 1), 255 * (e == 2));
                  });

              cv::Mat(points).copyTo(output_image);
              cv::Mat(colors).copyTo(output_colors3d);
            }
            break;
        }

        break;

        /////////////////////////////////////////////////////////////////////////////////////////////
      case AMBIENT:
        case DISTANCES:
        case DEPTH:
        case HEIGHT:
        case AREA:
        case PEAK:
        case WIDTH: {

        cv::Mat image =
            get_vlo_image(current_scan_,
                (VLO_DATA_CHANNEL) displayId,
                selection_mask_);

        if( *viewType == DataViewType_Image ) {
          output_image.move(image);
        }
        else if( *viewType == DataViewType_PointCloud ) {

          get_vlo_point_cloud(current_scan_, image,
              output_image,
              output_colors3d,
              selection_mask_);

        }

        break;
      }

      default:
        break;
    }

  }

  return true;
}

bool c_vlo_frame::get_image(int id, cv::OutputArray output_image, cv::OutputArray output_mask)
{
  DataViewType viewType =
      DataViewType_Image;

  bool fOk =
      get_display_data(&viewType, id,
          output_image,
          cv::noArray(),
          output_mask);

  if ( !fOk ) {
    CF_ERROR("get_display_data() fails");
  }

  if ( output_mask.needed() ) {

    if ( selection_mask_.empty() ) {
      output_mask.release();
    }
    else if ( selection_mask_.channels() == 1 ) {
      selection_mask_.copyTo(output_mask);
    }
    else {
      reduce_color_channels(selection_mask_, output_mask,
          cv::REDUCE_MAX);
    }

  }

  return true;
}

bool c_vlo_frame::get_point_cloud(int id, cv::OutputArray output_points, cv::OutputArray output_colors)
{
  DataViewType viewType =
      DataViewType_PointCloud;

  bool fOk =
      get_display_data(&viewType, id,
          output_points,
          output_colors,
          cv::noArray());

  if ( !fOk ) {
    CF_ERROR("get_display_data() fails");
  }

  return true;
}

void c_vlo_frame::update_selection(cv::InputArray mask,
    SELECTION_MASK_MODE mode)
{
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

void c_vlo_frame::cleanup()
{
  selection_mask_.release();
}



