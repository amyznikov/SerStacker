/*
 * vlo.cc
 *
 *  Created on: Nov 16, 2023
 *      Author: amyznikov
 */
#include "vlo.h"
#include <core/proc/c_line_estimate.h>
#include <core/debug.h>

template<class T>
bool vlo_depth_segmentation_(cv::InputArray _distances, cv::OutputArray output_segments,
    const c_vlo_depth_segmentation_options & opts)
{

  const int src_rows =
      _distances.rows();

  const int src_cols =
      _distances.cols();

  const int src_channels =
      _distances.channels();

  const cv::Mat_<T> distances =
      _distances.getMat();

  const double distance_step =
      opts.vlo_walk_error > 0 ?
          opts.vlo_walk_error : 100; // [cm]

  const int num_bins =
      (int) ((opts.max_distance - opts.min_distance) / distance_step) + 1;

  cv::Mat3w segment_ids(src_rows, src_cols,
      cv::Vec3w::all(0));

  std::vector<float> bin_distances[2] = {
      std::vector<float> (num_bins),
      std::vector<float> (num_bins),
  };

  std::vector<int> bin_counts[2] = {
      std::vector<int> (num_bins),
      std::vector<int> (num_bins),
  };

  int segment_id = 0;

  for( int x = 0; x < src_cols; ++x ) {


    // Reset histograms
    for( int j = 0; j < 2; ++j ) {

      memset(bin_distances[j].data(), 0,
          num_bins * sizeof(bin_distances[j][0]));

      memset(bin_counts[j].data(), 0,
          num_bins * sizeof(bin_counts[j][0]));
    }


    // Build histograms

    for( int y = 0; y < src_rows; ++y ) {

      for( int e = 0; e < src_channels; ++e ) {

        const T &distance =
            distances[y][x * src_channels + e];

        if( distance ) {

          const int bin0 =
              (int) ((distance - opts.min_distance) / distance_step);

          if( bin0 >= 0 && bin0 < num_bins ) {

            const int bin1 =
                (int) ((distance - opts.min_distance - 0.5 * distance_step) / distance_step);

            if( bin1 >= 0 && bin1 < num_bins ) {

              bin_counts[0][bin0]++;
              bin_counts[1][bin1]++;

              bin_distances[0][bin0] += distance;
              bin_distances[1][bin1] += distance;
            }
          }
        }
      }
    }

    // search for histogram peaks and mark segments

    for( int i = 1; i < num_bins - 1; ++i ) {

      const std::vector<int> * counts = nullptr;
      const std::vector<float> * dists = nullptr;

      // check if this bin contains appropriate histogram peak
      for( int j = 0; j < 2; ++j ) {

        const int count =
            bin_counts[j][i];

        if( count > opts.min_pts ) {
          if( bin_counts[j][i - 1] < 3 * count / 4 && bin_counts[j][i + 1] < 3 * count / 4 ) {
            counts = &bin_counts[j];
            dists = &bin_distances[j];
            break;
          }
        }
      }

      if ( !counts ) {
        continue;
      }

      //////////////////////////////////////////////////////////////
      // estimate slope

      c_line_estimate line;

      const double mean_distance =
          (*dists)[i] / (*counts)[i];

      const double min_distance =
          mean_distance - 1.41 * distance_step;

      const double max_distance =
          mean_distance + 1.41 * distance_step;

      for( int y = 0; y < src_rows; ++y ) {
        for( int e = 0; e < src_channels; ++e ) {

          const T &distance =
              distances[y][x * src_channels + e];

          if( distance > min_distance && distance < max_distance ) {
            line.update(y, distance);
          }
        }
      }

      //////////////////////////////////////////////////////////////
      // label segment if vertical enough

      const double slope =
          line.slope();

      if ( slope >= opts.min_slope && slope <= opts.max_slope ) {

        ++segment_id;

        for( int y = 0; y < src_rows; ++y ) {

          for( int e = 0; e < src_channels; ++e ) {

            const T &distance =
                distances[y][x * src_channels + e];

            if( distance > min_distance && distance < max_distance ) {
              segment_ids[y][x][e] = segment_id;
            }
          }
        }
      }

      //////////////////////////////////////////////////////////////
    }
  }

  output_segments.move(segment_ids);

  return true;
}


bool vlo_depth_segmentation(cv::InputArray distances, cv::OutputArray segments,
    const c_vlo_depth_segmentation_options & opts)
{
  switch (distances.depth()) {
    case CV_8U:
      return vlo_depth_segmentation_<uint8_t>(distances, segments, opts);
    case CV_8S:
      return vlo_depth_segmentation_<int8_t>(distances, segments, opts);
    case CV_16U:
      return vlo_depth_segmentation_<uint16_t>(distances, segments, opts);
    case CV_16S:
      return vlo_depth_segmentation_<int16_t>(distances, segments, opts);
    case CV_32S:
      return vlo_depth_segmentation_<int32_t>(distances, segments, opts);
    case CV_32F:
      return vlo_depth_segmentation_<float>(distances, segments, opts);
    case CV_64F:
      return vlo_depth_segmentation_<double>(distances, segments, opts);
  }

  CF_ERROR("Unsupported image depth %d", distances.depth());
  return false;
}


