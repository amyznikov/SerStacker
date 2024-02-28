/*
 * c_hdl_range_image.h
 *
 *  Created on: Feb 28, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_hdl_range_image_h__
#define __c_hdl_range_image_h__

#include "c_hdl_frame.h"
#include "c_hdl_specification.h"

/**
 * Utility to build HDL range images
 */
class c_hdl_range_image
{
public:
  /* default c'tor */
  c_hdl_range_image();

  /* c'tor with lidar specifcation */
  c_hdl_range_image(const c_hdl_specification * lidar);

  /* c'tor with lidar specifcation and start azimith */
  c_hdl_range_image(const c_hdl_specification * lidar, double start_azimuth);

  /* c'tor with lidar specifcation, azimuthal resolution and start azimith */
  c_hdl_range_image(const c_hdl_specification * lidar, double azimuthal_resolution, double start_azimuth);

  /** set current HDL parameters */
  void set_lidar_specifcation(const c_hdl_specification * lidar);

  /** get current HDL parameters */
  const c_hdl_specification * lidar_specifcation() const;

  /** set desired azimuthal resolution in [radians/px] */
  void set_azimuthal_resolution(double radians_per_pixel);

  /** get current azimuthal resolution in [radians/px] */
  double azimuthal_resolution() const;

  /** set start azimuth (the azimuth of very first image column) in radians. */
  void set_start_azimuth(double radians);

  /** get current start azimuth (the azimuth of very first image column) in radians. */
  double start_azimuth() const;

  /** project given HDL point onto range image */
  bool project(const c_hdl_point & p, int * r, int * c) const;

  /** project given HDL point onto range image, skip laser_ring checking for faster efficiency */
  bool projectncr(const c_hdl_point & p, int * r, int * c) const;

  /** get azimuth for given range image row, in radians */
  double azimuth(int r) const;

  /** get azimuth for given range image column, in radians */
  double elevation(int c) const;

  /** build range image where each pixel is the distance to HDL point  */
  bool build_distances(const std::vector<c_hdl_point> & points,
      /* out*/ cv::Mat1f & distances,
      /* out, opt */ cv::Mat1b * mask = nullptr,
      const std::vector<uint8_t> * filter = nullptr) const;

  /** build range image where each pixel is the horizontal distance (depth) to HDL point  */
  bool build_depths(const std::vector<c_hdl_point> & points,
      /* out*/ cv::Mat1f & depths,
      /* out, opt */ cv::Mat1b * mask = nullptr,
      const std::vector<uint8_t> * filter = nullptr) const;

  /** build range image where each pixel is the horizontal distance (depth) to HDL point
   * scaled to 8 bit */
  bool build_depths(const std::vector<c_hdl_point> & points, double max_distance,
      /* out */ cv::Mat1b & depths,
      /* in, opt */ const std::vector<uint8_t> * filter = nullptr) const;

  /** build range image where each pixel is the height of HDL point  */
  bool build_heights(const std::vector<c_hdl_point> & points,
      /* out*/ cv::Mat1f & heights,
      /* out, opt */ cv::Mat1b * mask = nullptr,
      const std::vector<uint8_t> * filter = nullptr) const;

  /** build range image where each pixel is the intensity of HDL point  */
  bool build_intensity(const std::vector<c_hdl_point> & points,
      /* out*/ cv::Mat1f & intensity,
      /* out, opt */ cv::Mat1b * mask = nullptr,
      const std::vector<uint8_t> * filter = nullptr) const;

  /** build range image where each pixel is the elevation of HDL point in radians */
  bool build_elevations(const std::vector<c_hdl_point> & points,
      /* out*/ cv::Mat1f & elevations,
      /* out, opt */ cv::Mat1b * mask = nullptr,
      const std::vector<uint8_t> * filter = nullptr) const;

  /** build range image where each pixel is the azimuth of HDL point in radians */
  bool build_azimuths(const std::vector<c_hdl_point> & points,
      /* out*/ cv::Mat1f & azimuths,
      /* out, opt */ cv::Mat1b * mask = nullptr,
      const std::vector<uint8_t> * filter = nullptr) const;

  /** build range image where each pixel is the laser_id of HDL point */
  bool build_lazerids(const std::vector<c_hdl_point> & points,
      /* out*/ cv::Mat1b & lazerids,
      /* out, opt */ cv::Mat1b * mask = nullptr,
      const std::vector<uint8_t> * filter = nullptr) const;

  /** build range image where each pixel is the laser_ring of HDL point */
  bool build_lazer_rings(const std::vector<c_hdl_point> & points,
      /* out*/ cv::Mat1b & rings,
      /* out, opt */ cv::Mat1b * mask = nullptr,
      const std::vector<uint8_t> * filter = nullptr) const;

  /** build range image where each pixel is the data block index of HDL point */
  bool build_datablocks(const std::vector<c_hdl_point> & points,
      /* out*/ cv::Mat1b & datablocks,
      /* out, opt */ cv::Mat1b * mask = nullptr,
      const std::vector<uint8_t> * filter = nullptr) const;

  /** build range image where each pixel is the packet index of HDL point */
  bool build_pkts(const std::vector<c_hdl_point> & points,
      /* out*/ cv::Mat1i & pkts,
      /* out, opt */ cv::Mat1b * mask = nullptr,
      const std::vector<uint8_t> * filter = nullptr) const;

  /** build range image where each pixel is the timestamp of HDL point in seconds */
  bool build_timestamps(const std::vector<c_hdl_point> & points,
      /* out*/ cv::Mat1f & timestamps,
      /* out, opt */ cv::Mat1b * mask = nullptr,
      const std::vector<uint8_t> * filter = nullptr) const;

  /** build range image where each pixel counts the number of aliased points */
  bool build_aliasing(const std::vector<c_hdl_point> & points,
      /* out*/ cv::Mat1b & counter,
      const std::vector<uint8_t> * filter = nullptr) const;

  /** build range image where each pixel is the cartesian coordintes (x,y,z) of HDL point  */
  bool build_cartesizan(const std::vector<c_hdl_point> & points,
      /* out*/ cv::Mat3f & cartesian,
      /* out, opt */ cv::Mat1b * mask = nullptr);

  /** build range image where each pixel is the Cartesian coordinates and intensity (x,y,z, w) of HDL point  */
  bool build_cartesizan(const std::vector<c_hdl_point> & points,
      /* out*/ cv::Mat4f & cartesian,
      /* out, opt */ cv::Mat1b * mask = nullptr);

  /** build range image where each pixel is the local surface slope to the horizontal plane (in radians) */
  bool build_gslopes(const std::vector<c_hdl_point> & points,
      /* out*/ cv::Mat1f & slopes,
      /* out, opt */ cv::Mat1b * mask = nullptr,
      /* out, opt */ cv::Mat1f * distances = nullptr,
      /* out, opt */ cv::Mat1f * heights = nullptr) const;

  /** build range image where each pixel is the local surface slope to the horizontal plane (in radians) */
  bool depth2gslopes(const cv::Mat1f & depths,
      /* out*/ cv::Mat1f & slopes,
      /* out, opt */ cv::Mat1f * output_also_heights = nullptr) const;

  /** build range image required for c_hdl_ground_filter */
  bool build_image_for_ground_detection(const std::vector<c_hdl_point> & points, double sensor_height,
      /* out */ cv::Mat3f & dhs,
      /* out */ cv::Mat1b & mask,
      /* in, opt */ const std::vector<uint8_t> * filter = nullptr,
      /* out, opt */ std::vector<cv::Point> * projected_coords = nullptr);

  /** build range images required for c_hdl_ground_filter */
  bool build_images_for_ground_detection(const std::vector<c_hdl_point> & points, double sensor_height,
      /* out */ cv::Mat1f & depths,
      /* out */ cv::Mat1f & heights,
      /* out */ cv::Mat1f & slopes,
      /* out */ cv::Mat1b & mask,
      /* in, opt */ const std::vector<uint8_t> * filter = nullptr,
      /* out, opt */ std::vector<cv::Point> * projected_coords = nullptr);


  /** build range image where each pixel is the local surface normal vector */
  bool build_normals(const std::vector<c_hdl_point> & points,
      /* out*/ cv::Mat3f & surface_normals,
      /* out, opt */ cv::Mat1b * mask = nullptr);


  /* fill small holes in depth map */
  static void median_inpaint(cv::InputOutputArray image,
      int kradius = 1);

  /** get current range image image in pixels */
  const cv::Size & size() const
  {
    return image_size_;
  }

  /** get current range image width in pixels */
  int width() const
  {
    return image_size_.width;
  }

  /** get current range image height in pixels */
  int height() const
  {
    return image_size_.height;
  }

  const std::vector<float> & sin_elevation_table() const
  {
    return sin_elevation_table_;
  }

  const std::vector<float> & cos_elevation_table() const
  {
    return cos_elevation_table_;
  }

  const std::vector<float> & tan_elevation_table() const
  {
    return tan_elevation_table_;
  }

  float cos_elevation_table(int laser_ring) const
  {
    return cos_elevation_table_[laser_ring];
  }

  float sin_elevation_table(int laser_ring) const
  {
    return sin_elevation_table_[laser_ring];
  }

  float tan_elevation_table(int laser_ring) const
  {
    return tan_elevation_table_[laser_ring];
  }

protected:
  void update_image_size();

  bool create_output_images(cv::OutputArray output_range_image,
      /* out, opt */ cv::Mat1b * mask = nullptr,
      int dtype = -1) const;

protected:
  const c_hdl_specification * hdl_ = nullptr;
  double azimuthal_resolution_ = 0.2101 * CV_PI / 180;
  double start_azimuth_ = 0;
  cv::Size image_size_;
  std::vector<float> sin_elevation_table_;
  std::vector<float> cos_elevation_table_;
  std::vector<float> tan_elevation_table_;
};



#endif /* __c_hdl_range_image_h__ */
