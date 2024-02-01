/*
 * vlo_bloom_detection.cc
 *
 *  Created on: Jan 30, 2024
 *      Author: amyznikov
 */

#include "vlo_bloom_detection.h"
#include <core/proc/c_line_estimate.h>
#include <core/ssprintf.h>
#include <core/debug.h>


template<>
const c_enum_member* members_of<VLO_BLOOM_INTENSITY_MEASURE>()
{
  static const c_enum_member members[] = {
      { VLO_BLOOM_INTENSITY_PEAK, "PEAK", "PEAK" },
      { VLO_BLOOM_INTENSITY_AREA, "AREA", "AREA" },
      { VLO_BLOOM_INTENSITY_PEAK }
  };

  return members;
}

namespace {

struct c_wall_segment
{
  std::vector<cv::Point> pts;
  double d = 0;
  double dd = 0;
  int nhr = 0;
};

bool extract_vertical_walls(const cv::Mat3f & D, const cv::Mat3b & R, int s, std::vector<c_wall_segment> & segments,
    const c_vlo_bloom_detection_options & opts)
{
  //  const int src_rows = D.rows;
  //  const int src_cols = D.cols;

  const double min_distance = opts.min_distance >= 0 ? opts.min_distance : 100; // [cm]
  const double max_distance = opts.max_distance > 0 ? opts.max_distance : 30000; // [cm]
  const double distance_step = opts.distance_tolerance > 0 ? opts.distance_tolerance : 100; // [cm]
  const int num_bins = (int) ((max_distance - min_distance) / distance_step) + 1;

  segments.clear();

  /*
   * Build histogram of distances
   */

  cv::Mat4f histogram(num_bins, 1, cv::Vec4f::all(0));

  for( int y = 0; y < D.rows; ++y ) {

    for( int e = 0; e < 3; ++e ) {

      const float & d =
          D[y][s][e];

      if( d > 0 ) {

        const int b0 =
            (int) ((d - min_distance) / distance_step);

        if( b0 >= 0 && b0 < num_bins ) {
          histogram[b0][0][0] += 1;
          histogram[b0][0][1] += d;
        }

        const int b1 =
            (int) ((d - min_distance - 0.5 * distance_step) / distance_step);

        if( b1 >= 0 && b1 < num_bins ) {
          histogram[b1][0][2] += 1;
          histogram[b1][0][3] += d;
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

      float & c0 = H[0];  // counter
      float & d0 = H[1];  // distance
      if( c0 ) {
        d0 /= c0;         // compute averaged distance
        c0 *= (y + 1);    // normalize counter by distance
      }

      float & c1 = H[2];  // counter
      float & d1 = H[3];  // distance
      if( c1 ) {
        d1 /= c1;         // compute averaged distance
        c1 *= (y + 1);    // normalize counter by distance
      }
    }
  }

  /*
   * Search local maximums over depth histogram
   * */

  const float ratio_threshold =
      1.3f; // Relative height of a local maximum

  const float hdisp =
      opts.min_segment_height * opts.min_segment_height;

  const double max_slope =
      std::tan(opts.max_segment_slope * CV_PI / 180);

  for( int x = 0; x < histogram.cols; ++x ) {

    float dmin, dmax; // depth bounds

    for( int y = 1; y < histogram.rows - 1; ++y ) {

      const float & cp0 = // previous point from first histogram
          histogram[y - 1][x][0];

      const float & cc0 = // current point from first histogram
          histogram[y][x][0];

      const float & cn0 = // next point from first histogram
          histogram[y + 1][x][0];

      const float & cp1 = // previous point from second histogram
          histogram[y - 1][x][2];

      const float & cc1 = // current point from second histogram
          histogram[y][x][2];

      const float & cn1 = // next point from second histogram
          histogram[y + 1][x][2];

      const bool c0_extreme = // check if current point from first histogram is local maximum
          cc0 > opts.counts_threshold &&
              cc0 > ratio_threshold * std::max(cp0, cn0);

      const bool c1_extreme = // check if current point from second histogram is local maximum
          cc1 > opts.counts_threshold &&
              cc1 > ratio_threshold * std::max(cp1, cn1);

      if( c0_extreme && c1_extreme ) {
        // if current point is local maximum on both histograms the select max of them

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
        // if current point is local maximum on first histograms
        dmin = histogram[y][x][1] - distance_step;
        dmax = histogram[y][x][1] + distance_step;
      }

      else if( c1_extreme ) {
        // if current point is local maximum on second histograms
        dmin = histogram[y][x][3] - distance_step;
        dmax = histogram[y][x][3] + distance_step;
      }

      else { // not an appropriate local maximum
        continue;
      }

      //////////////////////////////////////////////////////////////
      // estimate slope and vertical dispersion of the wall

      c_line_estimate line;

      double h = 0, sh = 0;

      for( int y = 0; y < D.rows; ++y ) {
        for( int e = 0; e < 3; ++e ) {

          const float & d =
              D[y][s][e];

          if( d > dmin && d < dmax ) {

            line.update(y, d);

            h += y;
            sh += y * y;
          }
        }
      }

      //////////////////////////////////////////////////////////////
      // label segment if vertical enough
      //CF_DEBUG("H line.pts()=%d / %d", line.pts(), opts.min_segment_size);

      if ( line.pts() < opts.min_segment_size ) {
        continue;
      }


      h /= line.pts(); // compute mean height
      sh = sh / line.pts() - h * h; // compute variation (dispersion) of heights
      if( ( sh < hdisp) ) { // too short vertically
        continue;
      }

      const double slope = std::abs(line.slope());
      if( slope > max_slope ) { // not enough vertical
        continue;
      }


      // label points

      c_wall_segment wall;

      for( int y = 0; y < D.rows; ++y ) {

        for( int e = 0; e < 3; ++e ) {

          const float & d =
              D[y][s][e];

          if( d > dmin && d < dmax ) {

            wall.pts.emplace_back(y, e);

            wall.d += d;
            wall.dd += d * d;

            if ( R[y][s][e] ) {
              ++wall.nhr;
            }
          }
        }
      }

      if( wall.nhr ) {
        wall.d /= wall.pts.size();
        wall.dd = sqrt(wall.dd / wall.pts.size() - wall.d * wall.d);
        segments.emplace_back(wall);
      }

      //////////////////////////////////////////////////////////////
    }
  }


  return true;
}


static bool extract_intensity_image(const c_vlo_scan & scan, VLO_BLOOM_INTENSITY_MEASURE intensity_measure,
    cv::Mat3f & output_intensity_image)
{
  VLO_DATA_CHANNEL intensity_channel;

  if ( intensity_measure == VLO_BLOOM_INTENSITY_AREA || scan.version == VLO_VERSION_6_SLM )  {
    intensity_channel = VLO_DATA_CHANNEL_AREA;
  }
  else {
    intensity_channel = VLO_DATA_CHANNEL_PEAK;
  }

  get_vlo_image(scan, intensity_channel).convertTo(output_intensity_image,
      output_intensity_image.depth());

  return true;
}

}


bool vlo_bloom_detection(const c_vlo_scan & scan,
    const c_vlo_bloom_detection_options & opts,
    cv::Mat & output_bloom_mask,
    cv::Mat & output_reflectors_mask)
{
  cv::Mat3f I, D;
  cv::Mat3b R, B;
  std::vector<c_wall_segment> segments;

  /*
   * Create empty output bloom mask B
   * */
  B.create(vlo_scan_size(scan));
  B.setTo(0);
  output_bloom_mask = B;


  /*
   *  Extract intensity image I and threshold it to get high-reflectors mask R
   * */

  extract_intensity_image(scan, opts.intensity_measure, I);
  cv::compare(I, cv::Scalar::all(opts.intensity_saturation_level), R, cv::CMP_GE);
  output_reflectors_mask = R;


  /*
   * Get point distances D
   * */
  get_vlo_image(scan, VLO_DATA_CHANNEL_DISTANCES).convertTo(D, D.depth());


  /*
   * Scan pints by columns (slot index),
   * for each of reflectors extract corresponding vertical wall and
   * analyze vertical intensity profile
   * */

  for( int s = 0; s < R.cols; ++s ) {

    // Check if this vertical column (slot index s) has at least one reflector
    bool have_reflectors = false;

    for( int l = 0; l < R.rows && !have_reflectors; ++l ) {
      for( int e = 0; e < 3; ++e ) {
        if ( R[l][s][e] ) {
          have_reflectors = true;
          break;
        }
      }
    }
    if ( !have_reflectors ) {
      continue;
    }


    // If there are least one reflective point on this column then
    // extract all vertical walls having at least one reflective point
    extract_vertical_walls(D, R, s, segments, opts);
    if( segments.empty() ) {
      continue; // strange ?
    }


    // Analyze each of extracted vertical wall
    for ( const c_wall_segment & seg : segments ) {


      for ( const cv::Point & p : seg.pts ) {

        const int & l = p.x; // image row (layer index)
        const int & e = p.y; // image channel (echo index)
        B[l][s][e] = 255;

      }
    }
  }


  return true;
}
