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

  /*
   * Build histogram of distances
   *
   * The 3D cloud axes are assumed as:
   *     x -> forward
   *     y -> left
   *     z -> up
   *
   *  Each pixel in each of clouds[] is presented as cv::Vec3f :
   *   clouds[e][y][x][0] -> X
   *   clouds[e][y][x][1] -> Y
   *   clouds[e][y][x][2] -> Z
   *
   * */

  const double distance_step =
      opts.vlo_walk_error > 0 ?
          opts.vlo_walk_error : 1; // [m]

  const int num_bins =
      (int) ((opts.max_distance - opts.min_distance) / distance_step) + 1;

  histogram.create(num_bins, src_cols);
  histogram.setTo(cv::Vec4f::all(0));

  output_segments.create(src_rows, src_cols);
  output_segments.setTo(cv::Scalar::all(0));

  for( int x = 0; x < src_cols; ++x ) {

    for( int y = 0; y < src_rows; ++y ) {

      for( int e = 0; e < 3; ++e ) {

        const float & depth = // depth is assumed to be forward-looking X-coordinate of a point in cloud
            clouds[e][y][x][0];

        if( depth > 0 ) {

          const int b0 =
              (int) ((depth - opts.min_distance) /
                  distance_step);

          if( b0 >= 0 && b0 < num_bins ) {
            histogram[b0][x][0] += 1;
            histogram[b0][x][1] += depth;
          }

          const int b1 =
              (int) ((depth - opts.min_distance - 0.5 * distance_step) /
                  distance_step);

          if( b1 >= 0 && b1 < num_bins ) {
            histogram[b1][x][2] += 1;
            histogram[b1][x][3] += depth;
          }
        }
      }
    }
  }

  /*
   * Normalize histograms.
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

  const float hdisp = // minimally acceptable vertical dispersion of a wall
      opts.min_height * opts.min_height;

  for( int x = 0; x < histogram.cols; ++x ) {

    float dmin, dmax; // depth bounds

    int seg_id = 0;

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
          }
        }
      }

      //////////////////////////////////////////////////////////////
      // label segment if vertical enough

      if ( line.pts() < opts.min_segment_size ) {
        continue;
      }

      h /= line.pts(); // compute mean height
      sh = sh / line.pts() - h * h; // compute variation (dispersion) of heights
      if( ( sh < hdisp) ) { // too short vertically
        continue;
      }

      const double slope = line.slope();
      if( slope < opts.min_slope || slope > opts.max_slope ) { // not enough vertical
        continue;
      }


      // label points

      ++seg_id;

      for( int y = 0; y < src_rows; ++y ) {

        for( int e = 0; e < 3; ++e ) {

          const float & depth =
              clouds[e][y][x][0];

          if( depth > dmin && depth < dmax ) {
            output_segments[y][x][e] = seg_id;
          }
        }
      }

      //////////////////////////////////////////////////////////////
    }
  }

  return true;
}


int extract_vlo_point_sequences(int x, const cv::Mat3w & segments_image, const cv::Mat3f & intensity_image,
    std::vector<c_vlo_segment_point_sequence> & sequences)
{
  sequences.clear();
  sequences.reserve(16);

  for( int y = 0; y < segments_image.rows; ++y ) {
    for( int e = 0; e < 3; ++e ) {

      const uint16_t seg_id =
          segments_image[y][x][e];

      if ( seg_id ) {

        if ( sequences.size() < seg_id ) {
          sequences.resize(seg_id);
        }

        c_vlo_segment_point_sequence & sequence =
            sequences[seg_id - 1];

        const float & point_intensity =
            intensity_image[y][x][e];

        if( sequence.points.empty() || sequence.points.back().y != y ) {
          sequence.points.emplace_back(c_vlo_segment_point { y, point_intensity });
        }
        else if( point_intensity > sequence.points.back().intensity ) {
          sequence.points.back().intensity = point_intensity;
        }

      }
    }
  }

  return (int)(sequences.size());
}


int extract_vlo_segments(int x, const cv::Mat3w & segments_image, const cv::Mat3f & intensity_image,
    std::vector<c_vlo_segment> & vlo_segments,
    double intensity_saturation_level)
{
  // Group point sequences by their segment id's

  std::vector<c_vlo_segment_point_sequence> point_sequences; // the size is number of unique segment id's over slot
  point_sequences.reserve(16);

  for( int y = 0; y < segments_image.rows; ++y ) {
    for( int e = 0; e < 3; ++e ) {

      const uint16_t seg_id =
          segments_image[y][x][e];

      if ( seg_id ) {

        if ( point_sequences.size() < seg_id ) {
          point_sequences.resize(seg_id);
        }

        c_vlo_segment_point_sequence & sequence =
            point_sequences[seg_id - 1];

        const float & point_intensity =
            intensity_image[y][x][e];

        if( sequence.points.empty() || sequence.points.back().y != y ) {
          sequence.points.emplace_back(c_vlo_segment_point { y, point_intensity });
        }
        else if( point_intensity > sequence.points.back().intensity ) {
          sequence.points.back().intensity = point_intensity;
        }

      }
    }
  }

  // Split each point sequence into continuous chunks based on point intensities
  vlo_segments.clear();
  vlo_segments.resize(point_sequences.size());


  for( uint16_t seg_id = 1; seg_id <= point_sequences.size(); ++seg_id ) {

    const std::vector<c_vlo_segment_point>  & sequence =
        point_sequences[seg_id - 1].points;

    c_vlo_segment & segment =
        vlo_segments[seg_id - 1];

    segment.seg_id =
        seg_id;

    for( int p = 0; p < sequence.size(); ) {

      // collect saturated chunk if exists
      if( sequence[p].intensity >= intensity_saturation_level ) {

        segment.chunks.emplace_back();

        c_vlo_segment_chunk & chunk =
            segment.chunks.back();

        chunk.saturated = true;

        while (p < sequence.size() && sequence[p].intensity >= intensity_saturation_level) {
          chunk.points.emplace_back(sequence[p++]);
        }
      }

      // collect unsaturated chunk if exists

      if( p + 1 >= sequence.size() ) {
        break;
      }

      segment.chunks.emplace_back();

      c_vlo_segment_chunk & chunk =
          segment.chunks.back();

      while (p < sequence.size() && sequence[p].intensity < intensity_saturation_level) {
        chunk.points.emplace_back(sequence[p++]);
      }
    }
  }

  return vlo_segments.size();
}


int search_vlo_reflectors(const std::vector<c_vlo_segment_point> & point_sequence,
    double min_saturation_level, int max_hole_size,
    std::vector<c_vlo_refector> & reflectors)
{
  reflectors.clear();

  const int sequence_size =
      point_sequence.size();

  for( int j = 0; j < sequence_size; ++j ) {

    // skip unsaturated
    while (j + 1 < sequence_size && point_sequence[j + 1].intensity < min_saturation_level) {
      ++j;
    }

    if( j >= sequence_size - 1 ) {
      break;
    }

    //  track saturated

    reflectors.emplace_back();
    c_vlo_points_range & r =
        reflectors.back();

    r.start = j;
    while (j + 1 < sequence_size && point_sequence[j + 1].intensity >= min_saturation_level) {
      ++j;
    }
    r.end = j;

    if( j >= sequence_size - 1 ) {
      break;
    }
  }

  if ( max_hole_size > 0 && reflectors.size() > 1 ) {
    // join consecutive reflectors with small hole in between

    for( size_t i = 0; i < reflectors.size() - 1; ) {

      c_vlo_points_range & rc =
          reflectors[i];

      c_vlo_points_range & rn =
          reflectors[i + 1];

      if( point_sequence[rc.end].y + max_hole_size > point_sequence[rn.start].y ) {
        ++i;
      }
      else {
        rc.end = rn.end;
        reflectors.erase(reflectors.begin() + i + 1);
      }
    }
  }

  if ( !reflectors.empty() ) {

    for( c_vlo_refector & rc : reflectors ) {

      rc.mean_intensity = 0;

      for ( int i = rc.start; i <= rc.end; ++i ) {
        rc.mean_intensity += point_sequence[i].intensity;
      }

      rc.mean_intensity /= (rc.end - rc.start + 1);
    }
  }

  return reflectors.size();
}


c_vlo_gaussian_blur::c_vlo_gaussian_blur(double sigma, int kradius)
{
  setup(sigma, kradius);
}

void c_vlo_gaussian_blur::setup(double sigma, int kradius)
{
  if( !(sigma > 0) && kradius < 1 ) {
    kradius = std::max(1, ((int) (3 * (sigma = 3))));
  }
  else if( !(sigma > 0) ) {
    sigma = (double) (kradius) / 3;
  }
  else if( kradius < 1 ) {
    kradius = std::max(1, ((int) (3 * (sigma))));
  }

  gc_.resize(kradius);

  for( int k = 0; k < kradius; ++k ) {
    const double x = (k + 1) / sigma;
    gc_[k] = std::exp(-0.5 * x * x);
  }

}

void c_vlo_gaussian_blur::apply(std::vector<c_vlo_segment_point> & points)
{
  const int kradius = gc_.size() / 2;

  std::vector<c_vlo_segment_point> filtered_points(points.size());

  for( int i = 0, n = points.size(); i < n; ++i ) {

    float s = points[i].intensity;
    float w = 1;

    for( int k = i - 1; k >= 0; --k ) {
      const int dist = points[i].y - points[k].y;
      if( dist >= kradius ) {
        break;
      }
      s += points[k].intensity * gc_[dist];
      w += gc_[dist];
    }

    for( int k = i + 1, nk = points.size(); k < nk; ++k ) {
      const int dist = points[k].y - points[i].y;
      if( dist >= kradius ) {
        break;
      }
      s += points[k].intensity * gc_[dist];
      w += gc_[dist];
    }

    filtered_points[i].y = points[i].y;
    filtered_points[i].intensity = s / w;
  }

  std::copy(filtered_points.begin(), filtered_points.end(),
      points.begin());

}


