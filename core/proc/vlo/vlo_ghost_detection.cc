/*
 * vlo_ghost_detection.cc
 *
 *  Created on: Dec 19, 2023
 *      Author: amyznikov
 */

#include "vlo_ghost_detection.h"
#include <core/debug.h>

namespace {

template<class DistanceType, class IntensityType>
static inline bool is_ghost(const DistanceType & DR, const DistanceType & DG,
    const IntensityType & IR, const IntensityType & IG,
    const c_vlo_ghost_detection_options & opts)
{
  static constexpr double Dmin = 200;

  const double & saturation_level =
      opts.saturation_level;

  const double & systematic_correction =
      opts.doubled_distanse_systematic_correction;

  const double & depth_tolerance =
      opts.doubled_distanse_depth_tolerance;

  // const double intensity_ratio_slope = 0.7 / 6000;

  const double KK = 3;

  if( DG && DR > Dmin && IR >= saturation_level && IG <= IR ) { // * (1. - intensity_ratio_slope * IR)
    if( (DG > KK * DR) || std::abs(DG - 2.0 * DR + systematic_correction) < depth_tolerance ) {
      return true;
    }
  }

  return false;
}

}

bool vlo_ghost_detection(const c_vlo_scan & scan,
    const c_vlo_ghost_detection_options & opts,
    cv::Mat & output_ghsosts_mask)
{
  output_ghsosts_mask.create(scan.size, CV_8UC3);
  output_ghsosts_mask.setTo(cv::Scalar::all(0));

  cv::Mat3b image =
      output_ghsosts_mask;

  if( !scan.peak.empty() ) {

    vlo_pixels_callback(scan, cv::noArray(),
        [&](int l, int s) {

        const auto & D = scan.distances[l][s];
        const auto &I = scan.peak[l][s];

        static constexpr double Dmin = 100;
        const double & saturation_level = opts.saturation_level;
        const double & depth_tolerance = opts.doubled_distanse_depth_tolerance;

        for ( int e = 0; e < 2; ++e ) {
          if ( I[e] >= saturation_level ) {
            for ( int ee = e + 1; ee < 3; ++ee ) {
              if ( opts.drop_noise_behind_reflector && (D[ee] > D[e] + depth_tolerance && I[ee] < saturation_level) ) {
                image[l][s][ee] = 255;
              }
              else if( is_ghost(D[e], D[ee], I[e], I[ee], opts) ) {
                image[l][s][ee] = 255;
              }
            }
          }
        }

      });

  }
  else if( !scan.area.empty() ) {

    vlo_pixels_callback(scan, cv::noArray(),
        [&](int l, int s) {

        const auto & D = scan.distances[l][s];
        const auto &I = scan.area[l][s];

        static constexpr double Dmin = 100;
        const double & saturation_level = opts.saturation_level;
        const double & depth_tolerance = opts.doubled_distanse_depth_tolerance;

        for ( int e = 0; e < 2; ++e ) {
          if ( I[e] >= saturation_level ) {
            for ( int ee = e + 1; ee < 3; ++ee ) {
              if ( opts.drop_noise_behind_reflector && (D[ee] > D[e] + depth_tolerance && I[ee] < saturation_level) ) {
                image[l][s][ee] = 255;
              }
              else if( is_ghost(D[e], D[ee], I[e], I[ee], opts) ) {
                image[l][s][ee] = 255;
              }
            }
          }
        }

      });
  }

  return true;
}


//bool vlo_ghost_detection(const c_vlo_scan & scan, const c_vlo_ghost_detection_options & opts,
//    cv::Mat & output_ghsosts_mask)
//{
//  switch (scan.version) {
//    case VLO_VERSION_1:
//      return vlo_ghost_detection_(scan.scan1, opts, output_ghsosts_mask);
//    case VLO_VERSION_3:
//      return vlo_ghost_detection_(scan.scan3, opts, output_ghsosts_mask);
//    case VLO_VERSION_5:
//      return vlo_ghost_detection_(scan.scan5, opts, output_ghsosts_mask);
//    case VLO_VERSION_6_SLM:
//      return vlo_ghost_detection_(scan.scan6_slm, opts, output_ghsosts_mask);
//    default:
//      break;
//  }
//
//  CF_ERROR("Unsupported scan version %d", scan.version);
//  return false;
//}
