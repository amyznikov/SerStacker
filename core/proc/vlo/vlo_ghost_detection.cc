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

///**
// * Prefer 'peak' intensity measure if available
// *  */
//template<class _Cb>
//static inline void check_ghosts_by_peak_intensity(const c_vlo_scan & scan,
//    int l, int s,
//    const c_vlo_ghost_detection_options & opts,
//    const _Cb & callback)
//{
//}
//
///**
// * Fallback to 'area' intensity measure if 'peak' is not available
// *  */
//template<class _Cb> std::enable_if_t<!has_peak_member(EchoType()),
//void> static inline get_ghosts(const EchoType echos[3],
//    const c_vlo_ghost_detection_options & opts,
//    const _Cb & callback)
//{
//  const auto &D0 = echos[0].dist;
//  const auto &D1 = echos[1].dist;
//  const auto &D2 = echos[2].dist;
//
//  const auto &I0 = echos[0].area;
//  const auto &I1 = echos[1].area;
//  const auto &I2 = echos[2].area;
//
//  if( is_ghost(D0, D1, I0, I1, opts) ) {
//    callback(0, 1);
//  }
//
//  if( is_ghost(D0, D2, I0, I2, opts) ) {
//    callback(0, 2);
//  }
//
//  if( is_ghost(D1, D2, I1, I2, opts) ) {
//    callback(1, 2);
//  }
//
//}
}

bool vlo_ghost_detection(const c_vlo_scan & scan,
    const c_vlo_ghost_detection_options & opts,
    cv::Mat & output_ghsosts_mask)
{
  output_ghsosts_mask.create(scan.size, CV_8UC3);
  output_ghsosts_mask.setTo(cv::Scalar::all(0));

  cv::Mat3b image =
      output_ghsosts_mask;

  if ( !scan.peak.empty() )  {

    vlo_pixels_callback(scan, cv::noArray(),
        [&](int l, int s) {

          const auto &D0 = scan.distance[l][s][0];
          const auto &D1 = scan.distance[l][s][1];
          const auto &D2 = scan.distance[l][s][2];

          const auto &I0 = scan.peak[l][s][0];
          const auto &I1 = scan.peak[l][s][1];
          const auto &I2 = scan.peak[l][s][2];

          if( is_ghost(D0, D1, I0, I1, opts) ) {
            image[l][s][1] = 255;
          }

          if( is_ghost(D0, D2, I0, I2, opts) ) {
            image[l][s][2] = 255;
          }

          if( is_ghost(D1, D2, I1, I2, opts) ) {
            image[l][s][2] = 255;
          }

        });
  }
  else if( !scan.area.empty() ) {

    vlo_pixels_callback(scan, cv::noArray(),
        [&](int l, int s) {

          const auto &D0 = scan.distance[l][s][0];
          const auto &D1 = scan.distance[l][s][1];
          const auto &D2 = scan.distance[l][s][2];

          const auto &I0 = scan.area[l][s][0];
          const auto &I1 = scan.area[l][s][1];
          const auto &I2 = scan.area[l][s][2];

          if( is_ghost(D0, D1, I0, I1, opts) ) {
            image[l][s][1] = 255;
          }

          if( is_ghost(D0, D2, I0, I2, opts) ) {
            image[l][s][2] = 255;
          }

          if( is_ghost(D1, D2, I1, I2, opts) ) {
            image[l][s][2] = 255;
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
