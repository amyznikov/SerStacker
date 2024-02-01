/*
 * vlo_depth_segmentation.h
 *
 *  Created on: Nov 16, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __vlo_h__
#define __vlo_h__

#include <core/io/c_vlo_file.h>

struct c_vlo_depth_segmentation_options
{
  double min_distance = 1; // [m]
  double max_distance = 300; // [m]
  double vlo_walk_error = 1; // [m]
  double min_slope = -0.3; // [tan = m / pix]
  double max_slope = +0.3; // [tan = m / pix]
  double min_height = 0.4; // [m]
  int counts_threshold = 500;
  int min_segment_size = 15;
};



bool vlo_depth_segmentation(const cv::Mat3f clouds[3],
    cv::Mat4f & output_histogram, cv::Mat3w & output_segments,
    const c_vlo_depth_segmentation_options & opts);


/**
 * Single point along vertical VLO segment
 **/
struct c_vlo_segment_point
{
  int y; // image row (vlo slot) index
  float intensity; // intensity of a point, max over all echos owned by given segment id
};

/**
 * Sequence of VLO segment points
 **/
struct c_vlo_segment_point_sequence
{
  std::vector<c_vlo_segment_point> points;
};

int extract_vlo_point_sequences(int x, const cv::Mat3w & segments_image, const cv::Mat3f & intensity_image,
    std::vector<c_vlo_segment_point_sequence> & sequences);


/**
 * Chunk (Sub-Sequence) of VLO segment points sequence
 **/
struct c_vlo_segment_chunk
{
  std::vector<c_vlo_segment_point> points;

  double minintensity = 0;
  double minpos = 0;

  double maxintensity = 0;
  double maxpos = 0;

  bool saturated = false;

};

/**
 * VLO segment point sequence splitted into classified chunks
 */
struct c_vlo_segment {
  std::vector<c_vlo_segment_chunk> chunks;
  uint16_t seg_id;
};

int extract_vlo_segments(int x, const cv::Mat3w & segments_image, const cv::Mat3f & intensity_image,
    std::vector<c_vlo_segment> & vlo_segments,
    double intensity_saturation_level);


struct c_vlo_points_range
{
  int start, end;
};

struct c_vlo_refector :
    c_vlo_points_range
{
  float mean_intensity;
};

int search_vlo_reflectors(const std::vector<c_vlo_segment_point> & point_sequence,
    double min_saturation_level, int max_hole_size,
    std::vector<c_vlo_refector> & reflectors);


class c_vlo_gaussian_blur
{
public:
  c_vlo_gaussian_blur(double sigma, int kradius = 0);

  void setup(double sigma, int kradius = 0);

  void apply(std::vector<c_vlo_segment_point> & points);

protected:
  std::vector<float> gc_;
};

#endif /* __vlo_h__ */
