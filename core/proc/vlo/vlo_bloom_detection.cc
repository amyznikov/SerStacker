/*
 * vlo_bloom_detection.cc
 *
 *  Created on: Jan 30, 2024
 *      Author: amyznikov
 */

#include "vlo_bloom_detection.h"
#include <core/proc/c_line_estimate.h>
#include <core/proc/c_quad_estimate.h>
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


  CF_DEBUG("INITIALIZED: members=%p", (void*)members);
  CF_DEBUG("members[0]: value=%d name='%s' comm='%s'", members[0].value, members[0].name.c_str(), members[0].comment.c_str());
  CF_DEBUG("members[1]: value=%d name='%s' comm='%s'", members[1].value, members[1].name.c_str(), members[1].comment.c_str());
  CF_DEBUG("members[2]: value=%d name='%s' comm='%s'", members[2].value, members[2].name.c_str(), members[2].comment.c_str());

  return members;
}

namespace {

static bool get_intensity_image(const c_vlo_scan & scan,
    const c_vlo_bloom_detection_options & opts,
    cv::Mat3w & intensity)
{
  switch (opts.intensity_measure) {

    case VLO_BLOOM_INTENSITY_PEAK:
      if( scan.peak.empty() ) {
        CF_ERROR("Scan has no peak values");
        return false;
      }

      intensity = scan.peak;
      break;

    case VLO_BLOOM_INTENSITY_AREA:
      if( scan.area.empty() ) {
        CF_ERROR("Scan has no area values");
        return false;
      }

      intensity = scan.area;
      break;

    default:
      CF_DEBUG("invalid intensity_measure=%d requested",
          opts.intensity_measure);
      return false;
  }

  return true;
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

    // check if this slot (column) has reflectors
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


struct c_point
{
  int l; // image row (layer index)
  int e;// image channel (echo index)

  c_point(int _l, int _e) :
    l(_l), e(_e)
  {
  }
};

struct c_distance_segment
{
  float dmin, dmax;
  float h, sh;
  c_line_estimate<float> line;

  std::vector<c_point> pts;
  int nrp = 0;

  //c_bloom_segment wall;
};

static inline void segment_distances(const c_vlo_scan & scan, const cv::Mat4f & H, cv::Mat3b & R, int s,
    const c_vlo_bloom_detection_options & opts,
    std::vector<c_distance_segment> & dsegs,
    std::vector<const c_distance_segment*> & output_segments)
{
 // INSTRUMENT_REGION("");

  const auto & D =
      scan.distances;

  const double distance_step =
      opts.distance_tolerance > 0 ?
          opts.distance_tolerance :
          100; // [cm]

  int dsegs_count = 0;

  if ( (int)dsegs.size() < H.rows - 2 ) {
    dsegs.resize(H.rows - 2);
  }


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

    // The data must become sorted by the 'dmax' by the definition of the distance histogram
    dseg.dmin = dmin;
    dseg.dmax = dmax;
    dseg.h = 0;
    dseg.sh = 0;
    dseg.line.reset();
    dseg.pts.clear();
    dseg.nrp = 0;
  }


  //////////////////////////////////////////////////////////////
  // estimate slope and vertical dispersion of the walls
  if( dsegs_count < 1 ) {

    if( opts.rreset ) {
      for( int l = 0; l < R.rows; ++l ) {
        for( int e = 0; e < 3; ++e ) {
          R[l][s][e] = 0;
        }
      }
    }

  }
  else {

    for( int l = 0; l < D.rows; ++l ) {
      for( int e = 0; e < 3; ++e ) {

        const auto & point_distance =
            D[l][s][e];

        if( point_distance ) {

          const auto beg =
              dsegs.begin();

          const auto end =
              dsegs.begin() + dsegs_count;

          const auto ii =
              std::lower_bound(beg, end,
                  point_distance,
                  [](const c_distance_segment & dseg, const auto & distance) {
                    return dseg.dmax < distance;
                  });

          if( ii == end || point_distance < ii->dmin ) {

            if( opts.rreset ) {
              R[l][s][e] = 0;
            }

            continue;
          }

          ii->pts.emplace_back(l, e);

          if( R[l][s][e] ) {
            ++ii->nrp;
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

    for( int i = 0; i < dsegs_count; ++i ) {

      const c_distance_segment & dseg =
          dsegs[i];

      if( !dseg.nrp ) {
        continue;
      }

      const int npts =
          dseg.line.pts();

      if( npts < opts.min_segment_size || std::abs(dseg.line.slope()) > max_slope ) {

        if( opts.rreset ) {
          for( const auto & p : dseg.pts ) {
            R[p.l][s][p.e] = 0;
          }
        }

        continue;
      }

      const auto h = dseg.h / npts; // mean height
      const auto sh = dseg.sh / npts - h * h; // compute variation (dispersion) of heights
      if( (sh < hdisp) ) { // too short vertically

        if( opts.rreset ) {
          for( const auto & p : dseg.pts ) {
            R[p.l][s][p.e] = 0;
          }
        }

        continue;
      }

      output_segments.emplace_back(&dseg);
    }

  }
}

} // namespace


/**
 * Begin is first point inside of reflector.
 * End is first point outside of reflector.
 */
static bool find_reflector(const std::vector<c_point> & pts, const cv::Mat3b & R, int p, int s,
    int * pbeg, int * pend)
{
  const int npts =
      pts.size();

  // search for begin of reflector
  for( ; p < npts; ++p ) {
    if( R[pts[p].l][s][pts[p].e] ) {
      *pbeg = p;
      break;
    }
  }

  if( p >= npts ) {
    return false;
  }

  // search for the end of reflector
  for( ++p; p < npts; ++p ) {
    if( !R[pts[p].l][s][pts[p].e] ) {
      break;
    }
  }

  *pend = p;

//  for( p = npts-1; p > *pbeg; --p ) {
//    if( R[pts[p].l][s][pts[p].e] ) {
//      ++p;
//      break;
//    }
//  }
//
//  *pend = p;

  return true;
}


static void analyze_profile(const std::vector<c_point> & pts, const cv::Mat3w & I, const cv::Mat3b & R,
    cv::Mat3b & B, cv::Mat3f & Q,
    const c_vlo_bloom_detection_options & opts,
    int rbeg, int rend, int s)
{

  struct c_profile_point
  {
    int l, e, I;
  };

  struct c_itensity_profle
  {
    std::vector<c_profile_point> pts;
    std::vector<c_profile_point> mpts;
    std::vector<c_profile_point> epts;

    int min_value, max_value;
  };

  static const auto create_profile =
      [](const c_vlo_bloom_detection_options & opts, c_itensity_profle & P,
          int beg, int end, int inc, int s,
          const std::vector<c_point> & pts,
          const cv::Mat3w & I) {

        const uint16_t T =
            (opts.intensity_measure == VLO_BLOOM_INTENSITY_PEAK) ? (inc > 0 ? 25 : 50) :
                (inc > 0 ? 500 : 900);

        P.min_value = P.max_value =
            I[pts[beg].l][s][pts[beg].e];

        for( int p = beg; p != end; p += inc ) {

          const int & l = pts[p].l;
          const int & e = pts[p].e;

          P.pts.emplace_back((c_profile_point){l, e, I[l][s][e]});

          //if ( I[l][s][e] > 300 /*&& std::abs(l - pts[beg].l) < 100*/ )
          {
            if( I[l][s][e] < P.min_value ) {
              P.min_value = I[l][s][e];
              P.mpts.emplace_back((c_profile_point) {l, e, I[l][s][e]});
              if( std::abs(l - pts[beg].l) < 100 && I[l][s][e] > T ) {
                P.epts.emplace_back((c_profile_point) {l, e, I[l][s][e]});
              }

            }
          }
        }

      };

  const int npts =
      pts.size();

  c_itensity_profle PP[2];

  if ( rbeg > 5 ) {
    create_profile(opts, PP[0], rbeg - 1, -1, -1, s, pts, I);
  }

  if( rend < npts - 5 ) {
    create_profile(opts, PP[1], rend, npts, +1, s, pts, I);
  }

  // analyze profiles
  for ( int i = 0; i < 2; ++i ) {

    c_itensity_profle & P =
        PP[i];

    if( P.epts.size() > 5 && ((P.epts.back().l < 15) || (P.epts.back().I < opts.bloom_min_intensity)) ) {

      const double BLT =
          opts.bloom_intensity_tolerance;

      c_line_estimate<float> estimate;

      for( const auto & p : P.epts ) {
        estimate.update(p.l, log(p.I));
      }


#if 1
      if ( true ) {
        for( int pass = 0; pass < 3 && P.epts.size() > 5; ++pass ) {

          const float a0 = estimate.a0();
          const float a1 = estimate.a1();
          //const float a2 = estimate.a2();

          int ipmax = -1;
          float dfmax = BLT;

          for( int ip = 1; ip < P.epts.size(); ++ip ) {

            const auto & p =
                P.epts[ip];

            const float fm =
                p.I;

            const float fp =
                exp(a0 + a1 * p.l /*+ a2 * p.l * p.l*/ );

            const float df =
                fm - fp;

            if ( df > dfmax ) {
              dfmax = df;
              ipmax = ip;
            }
          }

          if ( ipmax < 0 ) {
            break;
          }

          estimate.remove(P.epts[ipmax].l, log(P.epts[ipmax].I));
          P.epts.erase(P.epts.begin() + ipmax);
        }
      }
#endif

//      estimate.update(P.mpts.front().l, log(P.mpts.front().I));
//      estimate.update(P.mpts.back().l, log(P.mpts.back().I));
//      for( const auto & p : P.mpts ) {
//        estimate.update(p.l, log(p.I));
//      }

      const float a0 = estimate.a0();
      const float a1 = estimate.a1();
      const float slope = std::abs(a1);

      if( slope >= opts.min_bloom_slope && slope <= opts.max_bloom_slope ) {

//          for( const auto & p : P.epts ) {
//            if( !R[p.l][s][p.e] ) {
//
//              const double fp = exp(a0 + a1 * p.l /*+ a2 * p.l * p.l*/);
//
//              if( !Q[p.l][s][p.e] || fp > Q[p.l][s][p.e] ) {
//                Q[p.l][s][p.e] = fp;
//              }
//            }
//          }

        for( const auto & p : P.pts ) {
          if( !R[p.l][s][p.e] ) {

            const double fp = exp(a0 + a1 * p.l /*+ a2 * p.l * p.l*/);

            if( !Q[p.l][s][p.e] || fp > Q[p.l][s][p.e] ) {
              Q[p.l][s][p.e] = fp;
            }
          }
        }

        //

//        for( const auto & p : P.mpts ) {
//
//          const double fm = p.I;
//          const double fp = exp(a0 + a1 * p.l /*+ a2 * p.l * p.l*/);
//          Q[p.l][s][p.e] = fp; // std::abs(a1);
//          B[p.l][s][p.e] = 255;
//        }



      }

    }

  }

}

static void detect_saturated_pixels(const cv::Mat3w & I, const cv::Mat3w & D,
    const c_vlo_bloom_detection_options & opts,
    cv::Mat3b & output_mask)
{
  //  cv::compare(I, cv::Scalar::all(opts.intensity_saturation_level),
  //      output_reflectors_mask,
  //      cv::CMP_GE);

  output_mask.create(I.size());

  for ( int l = 0; l < I.rows; ++l ) {
    for ( int s = 0; s < I.cols; ++s ) {
      for ( int e = 0; e < 3; ++e ) {

        if ( D[l][s][e] < 100 ) {
          output_mask[l][s][e] = 0;
        }
        else {

          const double D020 = 2000;
          const double D100 = 10000;
          const double I020 = opts.intensity_saturation_level_020m;
          const double I100 = opts.intensity_saturation_level_100m;
          const double d = D[l][s][e];
          const double T = I020 + (d - D020) * (I100 - I020) / (D100 - D020);

          if ( I[l][s][e] >= T ) {
            output_mask[l][s][e] = 255;
          }
          else {
            output_mask[l][s][e] = 0;
          }
        }
      }
    }
  }

}

bool vlo_bloom_detection(const c_vlo_scan & scan,
    const c_vlo_bloom_detection_options & opts,
    cv::Mat & output_bloom_mask,
    cv::Mat & output_reflectors_mask,
    cv::Mat & output_debug_image)
{

  INSTRUMENT_REGION("");


  /*
   * Create point intensity image
   * */

  cv::Mat3w I;

  if( !get_intensity_image(scan, opts, I) ) {
    CF_DEBUG("get_intensity_image() fails");
    return false;
  }


  /*
   * Create output masks
   * */

  cv::Mat3b R;
  detect_saturated_pixels(I, scan.distances, opts, R);
  output_reflectors_mask = R;

  cv::Mat3b B(scan.size, cv::Vec3b::all(0));
  output_bloom_mask = B;


  /*
   * Scan points by columns (slot index),
   * for each of reflectors extract corresponding vertical wall and
   * analyze vertical intensity profile
   * */

  cv::Mat4f H;


  std::vector<c_distance_segment> dsegs;
  std::vector<const c_distance_segment*> selected_segments;

  create_histogram_of_distances(scan, R, opts, H);

  cv::Mat3f Q =
      cv::Mat3f::zeros(B.size());

  struct c_retro_reflector
  {
    int beg, end;
  };
  std::vector<c_retro_reflector> reflectors;

  for( int s = 0; s < R.cols; ++s ) {

    // Select and analyze each of appropriate vertical segment

    segment_distances(scan, H, R, s, opts,  dsegs,
        selected_segments);

    for( const c_distance_segment * ss : selected_segments ) {

      reflectors.clear();

      const auto & pts = ss->pts;
      const int npts = pts.size();

      for( int p = 0; p < npts; ) {

        int rbeg, rend;

        if( !find_reflector(pts, R, p, s, &rbeg, &rend) ) {
          break;
        }

        if( !reflectors.empty() ) {

          auto & reflector =
              reflectors.back();

          if( pts[rbeg].l < pts[reflector.end].l + opts.max_reflector_hole_size ) {

            for( int pp = reflector.end; pp < rbeg; ++pp ) {
              const auto & pt = pts[pp];
              R[pt.l][s][pt.e] = 255;
            }

            p = reflector.end = rend; // fuse two consecutive reflectors into single one
            continue;
          }
        }

        reflectors.emplace_back((c_retro_reflector){rbeg, rend});
        p = rend;
      }

//      int rbeg, rend;
//
//      if( find_reflector(ss->pts, R, 0, s, &rbeg, &rend) ) {
//        analyze_profile(ss->pts, I, B, Q, opts, rbeg, rend, s);
//      }


      /* analyze intensity profile for each of extractor high-reflective objects */
      for( const auto & reflector : reflectors ) {
        analyze_profile(ss->pts, I, R, B, Q, opts, reflector.beg, reflector.end, s);
      }

      for( int l = 0; l < R.rows; ++l ) {
        for( int e = 0; e < 3; ++e ) {
          if( !R[l][s][e] && Q[l][s][e] && Q[l][s][e] < opts.intensity_saturation_level_100m ) {
            if ( I[l][s][e] < Q[l][s][e] + opts.intensity_tolerance ) {
              B[l][s][e] = 255;
            }
          }
        }
      }
    }

  }

  output_debug_image = Q;

  return true;
}

