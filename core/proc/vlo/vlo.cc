/*
 * vlo.cc
 *
 *  Created on: Nov 16, 2023
 *      Author: amyznikov
 */
#include "vlo.h"
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

  const int num_bins =
      (int) ((opts.max_distance - opts.min_distance) / opts.distance_step) + 1;

  cv::Mat3w segment_ids(src_rows, src_cols,
      cv::Vec3w::all(0));

  std::vector<int> bin_counts(num_bins);
  std::vector<float> bin_distances(num_bins);

  int segment_id = 0;

  for( int x = 0; x < src_cols; ++x ) {

    memset(bin_counts.data(), 0, num_bins * sizeof(bin_counts[0]));
    memset(bin_distances.data(), 0, num_bins * sizeof(bin_distances[0]));

    for( int y = 0; y < src_rows; ++y ) {

      for( int e = 0; e < src_channels; ++e ) {

        const T & distance =
            distances[y][x * src_channels + e];

        if ( distance ) {

          const int bin_index =
              (int) ((distance - opts.min_distance) / opts.distance_step);

          if ( bin_index >= 0 && bin_index < num_bins ) {
            bin_counts[bin_index] ++;
            bin_distances[bin_index] += distance;
          }
        }
      }
    }



    for ( int i = 0; i < num_bins;  ) {

      if ( bin_counts[i] < opts.min_pts ) {
        ++i;
        continue;
      }

      int range_start = i, range_end = i;
      double mean_distance = bin_distances[range_end];
      int total_points = bin_counts[range_end];

      while (range_end < num_bins && bin_counts[range_end + 1] > opts.min_pts / 2) {
        ++range_end;
        mean_distance += bin_distances[range_end];
        total_points += bin_counts[range_end];
      }


      //////////////////////////////////////////////////////////////
      // gather segment

      if ( total_points >= opts.min_pts ) {

        mean_distance /= total_points;

        const double min_distance =
            mean_distance - 0.7 * opts.distance_step;

        const double max_distance =
            mean_distance + 0.7 * opts.distance_step;

        //////////////////////////////////////////////////////////////
        // estimate slope

        struct {
          double x = 0, y = 0, xy = 0, x2 = 0;
          int n = 0;
        } se;


        for( int y = 0; y < src_rows; ++y ) {
          for( int e = 0; e < src_channels; ++e ) {

            const T &distance =
                distances[y][x * src_channels + e];

            if( distance > min_distance && distance < max_distance ) {
              se.x += y;
              se.y += distance;
              se.xy += y * distance;
              se.x2 += y * y;
              se.n += 1;
            }
          }
        }

        //////////////////////////////////////////////////////////////
        // label segment if vertical enough

        const double slope =
            (se.n * se.xy - se.x * se.y) / (se.n * se.x2 - se.x * se.x);

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

      i = range_end + 1;
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
