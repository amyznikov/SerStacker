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
  add_data_item("AMBIENT", AMBIENT_IMAGE,
      c_cloudview_data_item::Type::image,
      "1-channel 2D Ambient Image");

  add_data_item("DISTANCES", DISTANCES_IMAGE,
      c_cloudview_data_item::Type::image,
      "3-channel 2D Image with distances to points");

  add_data_item("DEPTH", DEPTH_IMAGE,
      c_cloudview_data_item::Type::image,
      "3-channel 2D Image with depths (horizontal distances) to points");

  add_data_item("AREA", ECHO_AREA_IMAGE,
      c_cloudview_data_item::Type::image,
      "3-channel 2D Image with pixel intensities measured as AREA");

  add_data_item("PEAK", ECHO_PEAK_IMAGE,
      c_cloudview_data_item::Type::image,
      "3-channel 2D Image with pixel intensities measured as PEAK");

  add_data_item("WIDTH", ECHO_WIDTH_IMAGE,
      c_cloudview_data_item::Type::image,
      "3-channel 2D Image with pixel intensities measured as WIDTH");

  add_data_item("DOUBLED_ECHO_DISTANCE", DOUBLED_ECHO_DISTANCES_IMAGE,
      c_cloudview_data_item::Type::image,
      "3-channel 2D Image with DISTANCE values for doubled echos");

  add_data_item("DOUBLED_ECHO_PEAK", DOUBLED_ECHO_PEAKS_IMAGE,
      c_cloudview_data_item::Type::image,
      "3-channel 2D Image with PEAK values for doubled echos");

  add_data_item("DOUBLED_ECHO_AREA", DOUBLED_ECHO_AREAS_IMAGE,
      c_cloudview_data_item::Type::image,
      "3-channel 2D Image with AREA values for doubled echos");

  add_data_item("GHOSTS_MASK",GHOSTS_MASK_IMAGE,
      c_cloudview_data_item::Type::image,
      "3-channel binary 2D Image with Reflectors and Ghosts labeled as 255");

  add_data_item("CLOUD", CLOUD3D,
      c_cloudview_data_item::Type::point_cloud_3d,
      "Unstructured Point cloud");

  add_data_item("STRUCTURED_CLOUD", STRUCTURED_CLOUD3D,
      c_cloudview_data_item::Type::image,
      "Structured Point cloud");
}

bool c_vlo_data_frame::get_image(int id, cv::OutputArray image, cv::OutputArray mask)
{
  switch (id) {
    case AMBIENT_IMAGE:
      image.assign(c_vlo_file::get_image(current_scan_, c_vlo_file::DATA_CHANNEL_AMBIENT));
      mask.release();
      return true;

    case DISTANCES_IMAGE:
      image.assign(c_vlo_file::get_image(current_scan_, c_vlo_file::DATA_CHANNEL_DISTANCES));
      mask.release();
      return true;

    case DEPTH_IMAGE:
      image.assign(c_vlo_file::get_image(current_scan_, c_vlo_file::DATA_CHANNEL_DEPTH));
      mask.release();
      return true;

    case ECHO_AREA_IMAGE:
      image.assign(c_vlo_file::get_image(current_scan_, c_vlo_file::DATA_CHANNEL_AREA));
      mask.release();
      return true;

    case ECHO_PEAK_IMAGE:
      image.assign(c_vlo_file::get_image(current_scan_, c_vlo_file::DATA_CHANNEL_PEAK));
      mask.release();
      return true;

    case ECHO_WIDTH_IMAGE:
      image.assign(c_vlo_file::get_image(current_scan_, c_vlo_file::DATA_CHANNEL_WIDTH));
      mask.release();
      return true;

    case DOUBLED_ECHO_DISTANCES_IMAGE:
      image.assign(c_vlo_file::get_image(current_scan_, c_vlo_file::DATA_CHANNEL_DOUBLED_ECHO_DISTANCES));
      mask.release();
      return true;

    case DOUBLED_ECHO_PEAKS_IMAGE:
      image.assign(c_vlo_file::get_image(current_scan_, c_vlo_file::DATA_CHANNEL_DOUBLED_ECHO_PEAKS));
      mask.release();
      return true;

    case DOUBLED_ECHO_AREAS_IMAGE:
      image.assign(c_vlo_file::get_image(current_scan_, c_vlo_file::DATA_CHANNEL_DOUBLED_ECHO_AREAS));
      mask.release();
      return true;

    case GHOSTS_MASK_IMAGE:
      image.assign(c_vlo_file::get_image(current_scan_, c_vlo_file::DATA_CHANNEL_GHOSTS_MASK));
      mask.release();
      return true;
  }

  return false;
}

bool c_vlo_data_frame::get_point_cloud(int id, cv::OutputArray points, cv::OutputArray colors)
{
  switch (id) {
    case CLOUD3D:
      c_vlo_file::get_cloud3d(current_scan_, c_vlo_file::DATA_CHANNEL_AMBIENT, points, colors);
      return true;
  }

  return false;
}


} /* namespace cloudview */
