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
const c_enum_member * members_of<c_vlo_depth_segmentation_options::segmentatin_types>()
{
  static constexpr c_enum_member membres[] = {
      {c_vlo_depth_segmentation_options::segmentation_local, "local", "Segment IDs are local for each vertical scan line"},
      {c_vlo_depth_segmentation_options::segmentation_global, "global", "Segment IDs are global for whole frame"},
      {c_vlo_depth_segmentation_options::segmentation_local},
  };

  return membres;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

bool vlo_depth_segmentation(const cv::Mat3f clouds[3],
    cv::Mat4f & histogram, cv::Mat3w & output_segments,
    const c_vlo_depth_segmentation_options & opts)
{
  /*
   * Check input arguments
   * */

  if( clouds[0].empty() || clouds[1].size() != clouds[0].size() || clouds[2].size() != clouds[0].size() ) {
    CF_ERROR("Invalid point clouds sizes specified");
    return false;
  }

  const int src_rows =
      clouds[0].rows;

  const int src_cols =
      clouds[0].cols;

  const double distance_step =
      opts.vlo_walk_error > 0 ?
          opts.vlo_walk_error : 100; // [cm]

  const int num_bins =
      (int) ((opts.max_distance - opts.min_distance) / distance_step) + 1;
  /*
   * Build histogram
   *
   * The 3D cloud axes are assumed as:
   *     x -> forward
   *     y -> left
   *     z -> up
   * */

  histogram.create(num_bins, src_cols);
  histogram.setTo(cv::Vec4f::all(0));

  output_segments.create(src_rows, src_cols);
  output_segments.setTo(cv::Scalar::all(0));

  for( int x = 0; x < src_cols; ++x ) {

    for( int y = 0; y < src_rows; ++y ) {

      for( int e = 0; e < 3; ++e ) {

        const float & depth =
            clouds[e][y][x][0];

        if( depth > 0 ) {

          const int b0 =
              (int) ((depth - opts.min_distance) / distance_step);

          if( b0 >= 0 && b0 < num_bins ) {
            histogram[b0][x][0] += 1;
            histogram[b0][x][1] += depth;
          }

          const int b1 =
              (int) ((depth - opts.min_distance - 0.5 * distance_step) / distance_step);

          if( b1 >= 0 && b1 < num_bins ) {
            histogram[b1][x][2] += 1;
            histogram[b1][x][3] += depth;
          }
        }
      }
    }
  }

  /*
   * Normalize histogram.
   * */

  for( int y = 0; y < histogram.rows; ++y ) {
    for( int x = 0; x < histogram.cols; ++x ) {

      cv::Vec4f & H =
          histogram[y][x];

      float & c0 = H[0];
      float & d0 = H[1];
      if( c0 ) {
        d0 /= c0;
        c0 *= (y + 1);
      }

      float & c1 = H[2];
      float & d1 = H[3];
      if( c1 ) {
        d1 /= c1;
        c1 *= (y + 1);
      }
    }
  }

  /*
   * Analyze and cluster histogram.
   * */

  const float ratio_threshold =
      1.3f;

  const float disph =
      opts.min_height * opts.min_height;

  int cluster_id = 0;

  for( int x = 0; x < histogram.cols; ++x ) {

    float dmin, dmax;

    if ( opts.segmentation_type == c_vlo_depth_segmentation_options::segmentation_local ) {
      cluster_id = 0;
    }

    for( int y = 1; y < histogram.rows - 1; ++y ) {

      const float & cp0 =
          histogram[y - 1][x][0];

      const float & cc0 =
          histogram[y][x][0];

      const float & cn0 =
          histogram[y + 1][x][0];

      const float & cp1 =
          histogram[y - 1][x][2];

      const float & cc1 =
          histogram[y][x][2];

      const float & cn1 =
          histogram[y + 1][x][2];

      const bool c0_extreme =
          cc0 > opts.counts_threshold &&
              cc0 > ratio_threshold * std::max(cp0, cn0);

      const bool c1_extreme =
          cc1 > opts.counts_threshold &&
              cc1 > ratio_threshold * std::max(cp1, cn1);

      if( c0_extreme && c1_extreme ) {

        if( cc0 > cc1 ) {
          dmin = histogram[y][x][1] - distance_step;
          dmax = histogram[y][x][1] + distance_step;
        }
        else {
          dmin = histogram[y][x][3] - distance_step;
          dmax = histogram[y][x][3] + distance_step;
        }

      }
      else if( c0_extreme ) {
        dmin = histogram[y][x][1] - distance_step;
        dmax = histogram[y][x][1] + distance_step;
      }
      else if( c1_extreme ) {
        dmin = histogram[y][x][3] - distance_step;
        dmax = histogram[y][x][3] + distance_step;
      }
      else {
        continue;
      }



      //////////////////////////////////////////////////////////////
      // estimate vertical slope of segment

      c_line_estimate line;

      double h = 0, sh = 0;
      int npts = 0;

      for( int y = 0; y < src_rows; ++y ) {
        for( int e = 0; e < 3; ++e ) {

          const float & depth =
              clouds[e][y][x][0];

          if( depth > dmin && depth < dmax ) {

            line.update(y, depth);

            const float & height =
                clouds[e][y][x][2];

            h += height;
            sh +=  height * height;
            ++npts;
          }
        }
      }

      //////////////////////////////////////////////////////////////
      // label segment if vertical enough

      if ( npts < opts.min_segment_size ) {
        continue;
      }

      h /= npts; // compute mean
      sh = sh / npts - h * h; // compute variation (dispersion)
      if( ( sh < disph) ) {
        continue;
      }

      const double slope =
          line.slope();

      if( slope < opts.min_slope || slope > opts.max_slope ) {
        continue;
      }

      ++cluster_id;

      for( int y = 0; y < src_rows; ++y ) {

        for( int e = 0; e < 3; ++e ) {

          const float & depth =
              clouds[e][y][x][0];

          if( depth > dmin && depth < dmax ) {
            output_segments[y][x][e] = cluster_id;
          }
        }
      }

      //////////////////////////////////////////////////////////////
    }
  }

  return true;
}


