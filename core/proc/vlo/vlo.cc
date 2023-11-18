/*
 * vlo.cc
 *
 *  Created on: Nov 16, 2023
 *      Author: amyznikov
 */
#include "vlo.h"
#include <core/proc/c_line_estimate.h>
#include <core/ssprintf.h>
#include <core/debug.h>

template<>
const c_enum_member* members_of<vlo_depth_segmentation_output_type>()
{
  static constexpr c_enum_member members[] = {
      { vlo_depth_segmentation_output_segments, "segments", "" },
      { vlo_depth_segmentation_output_segment_distances, "segment_distances", "" },
      { vlo_depth_segmentation_output_counts0, "counts0", "" },
      { vlo_depth_segmentation_output_counts1, "counts1", "" },
      { vlo_depth_segmentation_output_segments }
  };

  return members;
}

template<class T>
bool vlo_depth_segmentation_(cv::InputArray _distances, cv::OutputArray output_data,
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

  const vlo_depth_segmentation_output_type output_type =
      opts.output_type;

  cv::Mat3w segment_ids;
  cv::Mat1w segment_counts;
  cv::Mat3f segment_distances;

  switch (output_type) {
    case vlo_depth_segmentation_output_segments:
      segment_ids = cv::Mat3w(src_rows, src_cols,
          cv::Vec3w::all(0));
      break;

    case vlo_depth_segmentation_output_segment_distances:
      segment_distances = cv::Mat3f(src_rows, src_cols,
          cv::Vec3f::all(0));
      break;

    case vlo_depth_segmentation_output_counts0:
      segment_counts = cv::Mat1w(num_bins, src_cols,
          (uint16_t) 0);
      break;

    case vlo_depth_segmentation_output_counts1:
      segment_counts = cv::Mat1w(num_bins, src_cols,
          (uint16_t) 0);
      break;

    default:
      CF_ERROR("Invalid output type %d (%s) requested",
          output_type, toString(output_type));
      return false;
  }

  std::vector<float> bin_distances[2] = {
      std::vector<float>(num_bins),
      std::vector<float>(num_bins),
  };

  std::vector<int> bin_counts[2] = {
      std::vector<int>(num_bins),
      std::vector<int>(num_bins),
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

    // Collect output debug data if requested
    if( output_type == vlo_depth_segmentation_output_counts0 ) {
      for( int i = 0; i < num_bins; ++i ) {
        segment_counts[i][x] = bin_counts[0][i];
      }
    }
    else if( output_type == vlo_depth_segmentation_output_counts1 ) {
      for( int i = 0; i < num_bins; ++i ) {
        segment_counts[i][x] = bin_counts[1][i];
      }
    }
    else if ( output_type == vlo_depth_segmentation_output_segments ||
        output_type == vlo_depth_segmentation_output_segment_distances ) {

      // Search for histogram peaks and mark segments

      for( int i = 1; i < num_bins - 1; ++i ) {

        const std::vector<int> *counts = nullptr;
        const std::vector<float> *dists = nullptr;

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

        if( !counts ) {
          continue;
        }

        const double mean_distance =
            (*dists)[i] / (*counts)[i];

        //////////////////////////////////////////////////////////////
        // estimate slope

        c_line_estimate line;


        const double min_distance =
            mean_distance - 1.5 * distance_step;

        const double max_distance =
            mean_distance + 1.5 * distance_step;

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

        if( slope >= opts.min_slope && slope <= opts.max_slope ) {

          ++segment_id;

          for( int y = 0; y < src_rows; ++y ) {

            for( int e = 0; e < src_channels; ++e ) {

              const T &distance =
                  distances[y][x * src_channels + e];

              if( distance > min_distance && distance < max_distance ) {
                switch (output_type) {
                  case vlo_depth_segmentation_output_segments:
                    segment_ids[y][x][e] = segment_id;
                    break;
                  case vlo_depth_segmentation_output_segment_distances:
                    segment_distances[y][x][e] = distance;
                    break;
                  default:
                    break;
                }
              }
            }
          }
        }

        //////////////////////////////////////////////////////////////
      }
    }
  }

  switch (output_type) {
    case vlo_depth_segmentation_output_segments:
      output_data.move(segment_ids);
      break;

    case vlo_depth_segmentation_output_segment_distances:
      output_data.move(segment_distances);
      break;

    case vlo_depth_segmentation_output_counts0:
      output_data.move(segment_counts);
      break;

    case vlo_depth_segmentation_output_counts1:
      output_data.move(segment_counts);
      break;

    default:
      CF_ERROR("Invalid output type %d (%s) requested",
          output_type, toString(output_type));
      return false;
  }

  return true;
}


bool vlo_depth_segmentation(cv::InputArray distances, cv::OutputArray output_data,
    const c_vlo_depth_segmentation_options & opts)
{
  switch (distances.depth()) {
    case CV_8U:
      return vlo_depth_segmentation_<uint8_t>(distances, output_data, opts);
    case CV_8S:
      return vlo_depth_segmentation_<int8_t>(distances, output_data, opts);
    case CV_16U:
      return vlo_depth_segmentation_<uint16_t>(distances, output_data, opts);
    case CV_16S:
      return vlo_depth_segmentation_<int16_t>(distances, output_data, opts);
    case CV_32S:
      return vlo_depth_segmentation_<int32_t>(distances, output_data, opts);
    case CV_32F:
      return vlo_depth_segmentation_<float>(distances, output_data, opts);
    case CV_64F:
      return vlo_depth_segmentation_<double>(distances, output_data, opts);
  }

  CF_ERROR("Unsupported image depth %d", distances.depth());
  return false;
}


