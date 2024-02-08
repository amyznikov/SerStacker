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

static const cv::Mat3w & get_intensity_image(const c_vlo_scan & scan,
    const c_vlo_bloom_detection_options & opts)
{
  if ( opts.intensity_measure == VLO_BLOOM_INTENSITY_PEAK && !scan.peak.empty() ) {
    return scan.peak;
  }
  return scan.area;
}


static void create_histogram_of_distances(const c_vlo_scan & scan, const cv::Mat3b & R,
    const c_vlo_bloom_detection_options & opts,
    cv::Mat4f & output_histogram)
{
  INSTRUMENT_REGION("");

  const auto & D = scan.distances;
  const int num_layers = scan.size.height;
  const int num_slots = scan.size.width;
  const double min_distance = opts.min_distance >= 0 ? opts.min_distance : 100; // [cm]
  const double max_distance = opts.max_distance > 0 ? opts.max_distance : 30000; // [cm]
  const double distance_step = opts.distance_tolerance > 0 ? opts.distance_tolerance : 100; // [cm]
  const int num_bins = (int) ((max_distance - min_distance) / distance_step) + 1;

  /*
   * Build histogram of distances
   */

  output_histogram.create(num_bins, num_slots);
  output_histogram.setTo(cv::Vec4f::all(0));

  for( int s = 0; s < num_slots; ++s ) {

    // check if this slot (column) contains reflectors
    bool have_reflectors = false;
    for( int l = 0; l < num_layers; ++l ) {
      for( int e = 0; e < 3; ++e ) {
        if( R[l][s][e] ) {
          have_reflectors = true;
          break;
        }
      }
      if( have_reflectors ) {
        break;
      }
    }
    if( !have_reflectors ) {
      // ignore this column as it has no reflectors
      continue;
    }

    // Count points and distribute by distance bins
    for( int l = 0; l < num_layers; ++l ) {
      for( int e = 0; e < 3; ++e ) {

        const auto & distance =
            D[l][s][e];

        if( distance > 0 ) {

          const int b0 =
              (int) ((distance - min_distance) / distance_step);

          if( b0 >= 0 && b0 < num_bins ) {
            output_histogram[b0][s][0] += 1;
            output_histogram[b0][s][1] += distance;
          }

          const int b1 =
              (int) ((distance - min_distance - 0.5 * distance_step) / distance_step);

          if( b1 >= 0 && b1 < num_bins ) {
            output_histogram[b1][s][2] += 1;
            output_histogram[b1][s][3] += distance;
          }
        }

      }
    }

    // normalize histogram on this slot (column)
    for( int y = 0; y < output_histogram.rows; ++y ) {

        auto & H =
            output_histogram[y][s];

        auto & c0 = H[0];  // counter
        auto & d0 = H[1];  // distance
        if( c0 ) {
          d0 /= c0;         // compute averaged distance
          c0 *= (y + 1);    // normalize counter by distance
        }

        auto & c1 = H[2];  // counter
        auto & d1 = H[3];  // distance
        if( c1 ) {
          d1 /= c1;         // compute averaged distance
          c1 *= (y + 1);    // normalize counter by distance
        }
    }
  }
}

#if 1

struct c_wall_segment
{
  std::vector<cv::Point> pts;
  int nrp = 0;
};

struct c_distance_segment
{
  float dmin, dmax;
  float h, sh;
  c_line_estimate<float> line;
  c_wall_segment wall;
};

static inline void segment_distances(const c_vlo_scan & scan, const cv::Mat4f & H, const cv::Mat3b & R, int s,
    const c_vlo_bloom_detection_options & opts,
    std::vector<c_distance_segment> & dsegs,
    std::vector<const c_wall_segment*> & output_segments)
{
 // INSTRUMENT_REGION("");

  const auto & D = scan.distances;
  const double distance_step = opts.distance_tolerance > 0 ? opts.distance_tolerance : 100; // [cm]

  if ( (int)dsegs.size() < H.rows - 2 ) {
    dsegs.resize(H.rows - 2);
  }

  int dsegs_count = 0;

  //c_wall_segment wall;

  output_segments.clear();

  /*
   * Search local maximums over depth histogram
   * */

  const float ratio_threshold =
      1.3f; // Relative height of a local maximum

  const float hdisp =
      opts.min_segment_height * opts.min_segment_height;

  const float max_slope =
      std::tan(opts.max_segment_slope * CV_PI / 180);

  for( int y = 1; y < H.rows - 1; ++y ) {

    float dmin, dmax; // depth bounds

    const auto & cp0 = // previous point from first histogram
        H[y - 1][s][0];

    const auto & cc0 = // current point from first histogram
        H[y][s][0];

    const auto & cn0 = // next point from first histogram
        H[y + 1][s][0];

    const auto & cp1 = // previous point from second histogram
        H[y - 1][s][2];

    const auto & cc1 = // current point from second histogram
        H[y][s][2];

    const auto & cn1 = // next point from second histogram
        H[y + 1][s][2];

    const bool c0_extreme = // check if current point from first histogram is local maximum
        cc0 > opts.counts_threshold &&
            cc0 > ratio_threshold * std::max(cp0, cn0);

    const bool c1_extreme = // check if current point from second histogram is local maximum
        cc1 > opts.counts_threshold &&
            cc1 > ratio_threshold * std::max(cp1, cn1);

    if( c0_extreme && c1_extreme ) {
      // if current point is local maximum on both histograms the select max of them

      if( cc0 > cc1 ) {
        dmin = H[y][s][1] - distance_step;
        dmax = H[y][s][1] + distance_step;
      }
      else {
        dmin = H[y][s][3] - distance_step;
        dmax = H[y][s][3] + distance_step;
      }

    }

    else if( c0_extreme ) {
      // if current point is local maximum on first histograms
      dmin = H[y][s][1] - distance_step;
      dmax = H[y][s][1] + distance_step;
    }

    else if( c1_extreme ) {
      // if current point is local maximum on second histograms
      dmin = H[y][s][3] - distance_step;
      dmax = H[y][s][3] + distance_step;
    }

    else { // not an appropriate local maximum
      continue;
    }

    c_distance_segment & dseg =
        dsegs[dsegs_count++];

    // The data must become sorted by the 'dmax' by the definition of this distance histogram
    dseg.dmin = dmin;
    dseg.dmax = dmax;
    dseg.h = 0;
    dseg.sh = 0;
    dseg.line.reset();
    dseg.wall.pts.clear();
    dseg.wall.nrp = 0;
  }


  //////////////////////////////////////////////////////////////
  // estimate slope and vertical dispersion of the walls
  if( dsegs_count > 0 ) {

    if ( true ) {
     //  INSTRUMENT_REGION("comppts");

      for( int l = 0; l < D.rows; ++l ) {
        for( int e = 0; e < 3; ++e ) {

          const auto & point_distance =
              D[l][s][e];

          if( point_distance ) {

            const auto ii =
                std::lower_bound(dsegs.begin(), dsegs.begin() + dsegs_count,
                    point_distance,
                    [](const c_distance_segment & dseg, const auto & distance) {
                      return dseg.dmax < distance;
                    });

            if( ii == dsegs.end() || point_distance < ii->dmin ) {
              continue;
            }

            ii->wall.pts.emplace_back(l, e);

            if( R[l][s][e] ) {
              ++ii->wall.nrp;
            }

            ii->h += l;
            ii->sh += l * l;

            const auto & point_height =
                scan.clouds[e][l][s][2]; // Z-coordinate of the point

            ii->line.update(point_height,
                point_distance);
          }
        }
      }
    }

    if( true ) {
      // INSTRUMENT_REGION("checkpts");

      for( int i = 0; i < dsegs_count; ++i ) {

        const c_distance_segment & dseg =
            dsegs[i];

        const int npts =
            dseg.line.pts();

        if( !dseg.wall.nrp || npts < opts.min_segment_size || std::abs(dseg.line.slope()) > max_slope ) {
          continue;
        }

        const auto h = dseg.h / npts; // mean height
        const auto sh = dseg.sh / npts - h * h; // compute variation (dispersion) of heights
        if( (sh < hdisp) ) { // too short vertically
          continue;
        }

        output_segments.emplace_back(&dseg.wall);
      }
    }
  }


}
#else

static inline void segment_distances(const c_vlo_scan & scan, const cv::Mat4f & H, const cv::Mat3b & R, int s,
    const c_vlo_bloom_detection_options & opts,
    std::vector<c_wall_segment> & output_segments)
{
  INSTRUMENT_REGION("");

  const auto & D = scan.distances;
  const double distance_step = opts.distance_tolerance > 0 ? opts.distance_tolerance : 100; // [cm]

  c_wall_segment wall;

  output_segments.clear();

  /*
   * Search local maximums over depth histogram
   * */

  const float ratio_threshold =
      1.3f; // Relative height of a local maximum

  const float hdisp =
      opts.min_segment_height * opts.min_segment_height;

  const float max_slope =
      std::tan(opts.max_segment_slope * CV_PI / 180);

  c_line_estimate<float> line;

  float dmin, dmax; // depth bounds
  float h, sh;

  for( int y = 1; y < H.rows - 1; ++y ) {

    const auto & cp0 = // previous point from first histogram
        H[y - 1][s][0];

    const auto & cc0 = // current point from first histogram
        H[y][s][0];

    const auto & cn0 = // next point from first histogram
        H[y + 1][s][0];

    const auto & cp1 = // previous point from second histogram
        H[y - 1][s][2];

    const auto & cc1 = // current point from second histogram
        H[y][s][2];

    const auto & cn1 = // next point from second histogram
        H[y + 1][s][2];

    const bool c0_extreme = // check if current point from first histogram is local maximum
        cc0 > opts.counts_threshold &&
            cc0 > ratio_threshold * std::max(cp0, cn0);

    const bool c1_extreme = // check if current point from second histogram is local maximum
        cc1 > opts.counts_threshold &&
            cc1 > ratio_threshold * std::max(cp1, cn1);

    if( c0_extreme && c1_extreme ) {
      // if current point is local maximum on both histograms the select max of them

      if( cc0 > cc1 ) {
        dmin = H[y][s][1] - distance_step;
        dmax = H[y][s][1] + distance_step;
      }
      else {
        dmin = H[y][s][3] - distance_step;
        dmax = H[y][s][3] + distance_step;
      }

    }

    else if( c0_extreme ) {
      // if current point is local maximum on first histograms
      dmin = H[y][s][1] - distance_step;
      dmax = H[y][s][1] + distance_step;
    }

    else if( c1_extreme ) {
      // if current point is local maximum on second histograms
      dmin = H[y][s][3] - distance_step;
      dmax = H[y][s][3] + distance_step;
    }

    else { // not an appropriate local maximum
      continue;
    }


    //////////////////////////////////////////////////////////////
    // estimate slope and vertical dispersion of the wall
    {
      INSTRUMENT_REGION("checkpts");

    line.reset();

    wall.pts.clear();
    wall.nrp = 0;

    h = 0;
    sh = 0;

    for( int l = 0; l < D.rows; ++l ) {
      for( int e = 0; e < 3; ++e ) {

        const auto & point_distance =
            D[l][s][e];

        if( point_distance > dmin && point_distance < dmax ) {

          wall.pts.emplace_back(l, e);

          if( R[l][s][e] ) {
            ++wall.nrp;
          }

          const auto & point_height =
              scan.clouds[e][l][s][2]; // Z-coordinate of the point

          line.update(point_height,
              point_distance);

          h += l;
          sh += l * l;
        }
      }
    }

    if( !wall.nrp ) {
      continue;
    }

    //////////////////////////////////////////////////////////////
    // label segment if vertical enough
    //CF_DEBUG("H line.pts()=%d / %d", line.pts(), opts.min_segment_size);
    if( line.pts() < opts.min_segment_size ) {
      continue;
    }

    const auto slope = std::abs(line.slope());
    if( slope > max_slope ) { // not enough vertical
      continue;
    }

    h /= line.pts(); // compute mean height
    sh = sh / line.pts() - h * h; // compute variation (dispersion) of heights
    if( (sh < hdisp) ) { // too short vertically
      continue;
    }


    output_segments.emplace_back(wall);

    }
    //////////////////////////////////////////////////////////////
  }

}

#endif
}


bool vlo_bloom_detection(const c_vlo_scan & scan,
    const c_vlo_bloom_detection_options & opts,
    cv::Mat & output_bloom_mask,
    cv::Mat & output_reflectors_mask)
{

  INSTRUMENT_REGION("");


  // Get intensity image
  const cv::Mat3w & I =
      get_intensity_image(scan,
          opts);

  /*
   * Create output masks
   * */

  output_bloom_mask.create(scan.size, CV_8UC3);
  output_bloom_mask.setTo(0);

  cv::compare(I, cv::Scalar::all(opts.intensity_saturation_level),
      output_reflectors_mask,
      cv::CMP_GE);

  cv::Mat3b B =
      output_bloom_mask;

  cv::Mat3b R =
      output_reflectors_mask;


  /*
   * Scan pints by columns (slot index),
   * for each of reflectors extract corresponding vertical wall and
   * analyze vertical intensity profile
   * */

  const float IT =
      opts.intensity_saturation_level -
          opts.intensity_tolerance;

  cv::Mat4f H;

  std::vector<c_distance_segment> dsegs;
  std::vector<const c_wall_segment*> wsegs;

  create_histogram_of_distances(scan, R, opts, H);

  for( int s = 0; s < R.cols; ++s ) {

    segment_distances(scan, H, R, s, opts, dsegs, wsegs);
    if( wsegs.empty() ) {
      continue;
    }

    // Analyze each of extracted vertical wall
    for ( const c_wall_segment * w : wsegs ) {

      int rstart = -1, rend = -1;

      for ( int p = 0, np = w->pts.size(); p < np; ++p ) {

        const cv::Point & sp = w->pts[p];
        const int & l = sp.x; // image row (layer index)
        const int & e = sp.y; // image channel (echo index)

        if( I[l][s][e] >= IT ) {

          if( rstart < 0 ) {
            rstart = rend = p;
          }
          else {
            rend = p;
          }
        }
      }

      if( rend >= rstart ) {
        for( int p = 0, np = w->pts.size(); p < np; ++p ) {
          if( p < rstart || p > rend ) {
            const cv::Point & sp = w->pts[p];
            const int & l = sp.x; // image row (layer index)
            const int & e = sp.y; // image channel (echo index)
            B[l][s][e] = 255;
          }
        }
      }
    }
  }

  return true;
}

#if 0

static inline void segment_distances(const c_vlo_scan & scan, const cv::Mat4f & H, const cv::Mat3b & R, int s,
    const c_vlo_bloom_detection_options & opts,
    std::vector<c_wall_segment> & output_segments)
{
  INSTRUMENT_REGION("");

  const auto & D = scan.distances;
  const double distance_step = opts.distance_tolerance > 0 ? opts.distance_tolerance : 100; // [cm]

  c_wall_segment wall;

  output_segments.clear();

  /*
   * Search local maximums over depth histogram
   * */

  const float ratio_threshold =
      1.3f; // Relative height of a local maximum

  const float hdisp =
      opts.min_segment_height * opts.min_segment_height;

  const double max_slope =
      std::tan(opts.max_segment_slope * CV_PI / 180);

  c_line_estimate line;

  float dmin, dmax; // depth bounds
  float h, sh;

  for( int y = 1; y < H.rows - 1; ++y ) {

    const auto & cp0 = // previous point from first histogram
        H[y - 1][s][0];

    const auto & cc0 = // current point from first histogram
        H[y][s][0];

    const auto & cn0 = // next point from first histogram
        H[y + 1][s][0];

    const auto & cp1 = // previous point from second histogram
        H[y - 1][s][2];

    const auto & cc1 = // current point from second histogram
        H[y][s][2];

    const auto & cn1 = // next point from second histogram
        H[y + 1][s][2];

    const bool c0_extreme = // check if current point from first histogram is local maximum
        cc0 > opts.counts_threshold &&
            cc0 > ratio_threshold * std::max(cp0, cn0);

    const bool c1_extreme = // check if current point from second histogram is local maximum
        cc1 > opts.counts_threshold &&
            cc1 > ratio_threshold * std::max(cp1, cn1);

    if( c0_extreme && c1_extreme ) {
      // if current point is local maximum on both histograms the select max of them

      if( cc0 > cc1 ) {
        dmin = H[y][s][1] - distance_step;
        dmax = H[y][s][1] + distance_step;
      }
      else {
        dmin = H[y][s][3] - distance_step;
        dmax = H[y][s][3] + distance_step;
      }

    }

    else if( c0_extreme ) {
      // if current point is local maximum on first histograms
      dmin = H[y][s][1] - distance_step;
      dmax = H[y][s][1] + distance_step;
    }

    else if( c1_extreme ) {
      // if current point is local maximum on second histograms
      dmin = H[y][s][3] - distance_step;
      dmax = H[y][s][3] + distance_step;
    }

    else { // not an appropriate local maximum
      continue;
    }


    //////////////////////////////////////////////////////////////
    // estimate slope and vertical dispersion of the wall

    line.reset();

    h = 0;
    sh = 0;

    for( int l = 0; l < D.rows; ++l ) {
      for( int e = 0; e < 3; ++e ) {

        const auto & point_distance =
            D[l][s][e];

        if( point_distance > dmin && point_distance < dmax ) {

          const auto & point_height =
              scan.clouds[e][l][s][2]; // Z-coordinate of the point

          line.update(point_height, point_distance);

          h += l;
          sh += l * l;
        }
      }
    }

    //////////////////////////////////////////////////////////////
    // label segment if vertical enough
    //CF_DEBUG("H line.pts()=%d / %d", line.pts(), opts.min_segment_size);
    if( line.pts() < opts.min_segment_size ) {
      continue;
    }

    h /= line.pts(); // compute mean height
    sh = sh / line.pts() - h * h; // compute variation (dispersion) of heights
    if( (sh < hdisp) ) { // too short vertically
      continue;
    }

    const double slope = std::abs(line.slope());
    if( slope > max_slope ) { // not enough vertical
      continue;
    }

    // label points


    {
      // INSTRUMENT_REGION("labelpts");
    wall.pts.clear();
    wall.nrp = 0;

    for( int l = 0; l < D.rows; ++l ) {

      for( int e = 0; e < 3; ++e ) {

        const auto & d =
            D[l][s][e];

        if( d > dmin && d < dmax ) {

          wall.pts.emplace_back(l, e);

          if( R[l][s][e] ) {
            ++wall.nrp;
          }
        }
      }
    }


    if( wall.nrp ) {
     output_segments.emplace_back(wall);
    }

    }
    //////////////////////////////////////////////////////////////
  }

}
#endif


#if 0


bool segment_distances(const c_vlo_scan & scan,const cv::Mat3b & R, int s, cv::Mat4f & histogram,
    std::vector<c_wall_segment> & segments,
    const c_vlo_bloom_detection_options & opts)
{

//  INSTRUMENT_REGION("");

  const auto & D = scan.distances;
  const double min_distance = opts.min_distance >= 0 ? opts.min_distance : 100; // [cm]
  const double max_distance = opts.max_distance > 0 ? opts.max_distance : 30000; // [cm]
  const double distance_step = opts.distance_tolerance > 0 ? opts.distance_tolerance : 100; // [cm]
  const int num_bins = (int) ((max_distance - min_distance) / distance_step) + 1;

  segments.clear();

  /*
   * Build histogram of distances
   */


  histogram.create(num_bins, 1);
  histogram.setTo(cv::Vec4f::all(0));

  for( int y = 0; y < D.rows; ++y ) {

    for( int e = 0; e < 3; ++e ) {

      const auto & d =
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

      for( int l = 0; l < D.rows; ++l ) {
        for( int e = 0; e < 3; ++e ) {

          const auto & d =
              D[l][s][e];

          if( d > dmin && d < dmax ) {

            line.update(l, d);

            h += l;
            sh += l * l;
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

          const auto & d =
              D[y][s][e];

          if( d > dmin && d < dmax ) {

            wall.pts.emplace_back(y, e);

            if ( R[y][s][e] ) {
              ++wall.nrp;
            }
          }
        }
      }

      if( wall.nrp ) {
        segments.emplace_back(wall);
      }

      //////////////////////////////////////////////////////////////
    }
  }


  return true;
}


bool vlo_bloom_detection(const c_vlo_scan & scan,
    const c_vlo_bloom_detection_options & opts,
    cv::Mat & output_bloom_mask,
    cv::Mat & output_reflectors_mask)
{

  INSTRUMENT_REGION("");

  std::vector<c_wall_segment> segments;

  // Get intensity image
  const cv::Mat3w & I =
      get_intensity_image(scan,
          opts);

  /*
   * Create output masks
   * */

  output_bloom_mask.create(scan.size, CV_8UC3);
  output_bloom_mask.setTo(0);

  cv::compare(I, cv::Scalar::all(opts.intensity_saturation_level),
      output_reflectors_mask,
      cv::CMP_GE);

  cv::Mat3b B =
      output_bloom_mask;

  cv::Mat3b R =
      output_reflectors_mask;


  /*
   * Scan pints by columns (slot index),
   * for each of reflectors extract corresponding vertical wall and
   * analyze vertical intensity profile
   * */

  const float IT =
      opts.intensity_saturation_level -
          opts.intensity_tolerance;

  cv::Mat4f H;

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
    segment_distances(scan, R, s, H, segments, opts);
    if( segments.empty() ) {
      continue; // strange ?
    }


    // Analyze each of extracted vertical wall
    for ( const c_wall_segment & w : segments ) {

      int rstart = -1, rend = -1;

      for ( int p = 0, np = w.pts.size(); p < np; ++p ) {

        const cv::Point & sp = w.pts[p];
        const int & l = sp.x; // image row (layer index)
        const int & e = sp.y; // image channel (echo index)

        if( I[l][s][e] >= IT ) {

          if( rstart < 0 ) {
            rstart = rend = p;
          }
          else {
            rend = p;
          }
        }
      }

      if( rend >= rstart ) {
        for( int p = 0, np = w.pts.size(); p < np; ++p ) {
          if( p < rstart || p > rend ) {
            const cv::Point & sp = w.pts[p];
            const int & l = sp.x; // image row (layer index)
            const int & e = sp.y; // image channel (echo index)
            B[l][s][e] = 255;
          }
        }
      }
    }
  }


  return true;
}

#endif

