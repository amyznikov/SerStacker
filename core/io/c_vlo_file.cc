/*
 * c_vlo_file.cc
 *
 *  Created on: Oct 21, 2023
 *      Author: amyznikov
 */

#include "c_vlo_file.h"
#include <fcntl.h>
#include <limits>
#include <core/ssprintf.h>
#include <core/debug.h>


#ifdef _MSC_VER
# pragma warning (disable:4996)
#endif

#if ! __DEBUG_H_INCLUDED__
#include <stdio.h>

#define CF_DEBUG(...) \
    fprintf(stderr, "%s(): %d ", __func__, __LINE__), \
    fprintf(stderr, __VA_ARGS__), \
    fprintf(stderr, "\n"), \
    fflush(stderr)

#define CF_ERROR CF_DEBUG
#endif

#ifdef __ssprintf_h__

template<>
const c_enum_member* members_of<c_vlo_file::DATA_CHANNEL>()
{
  static constexpr c_enum_member members[] = {
      { c_vlo_file::DATA_CHANNEL_AMBIENT, "AMBIENT", "" },
      { c_vlo_file::DATA_CHANNEL_DISTANCES, "DISTANCES", "" },
      { c_vlo_file::DATA_CHANNEL_DEPTH, "DEPTH", "" },
      { c_vlo_file::DATA_CHANNEL_AREA, "ECHO_AREA", "" },
      { c_vlo_file::DATA_CHANNEL_PEAK, "ECHO_PEAK", "" },
      { c_vlo_file::DATA_CHANNEL_WIDTH, "ECHO_WIDTH", "" },
      { c_vlo_file::DATA_CHANNEL_AREA_MUL_DIST, "AREA_MUL_DIST", "" },
      { c_vlo_file::DATA_CHANNEL_PEAK_MUL_DIST, "PEAK_MUL_DIST", "" },
      { c_vlo_file::DATA_CHANNEL_AREA_MUL_SQRT_DIST, "AREA_MUL_SQRT_DIST", "" },
      { c_vlo_file::DATA_CHANNEL_PEAK_MUL_SQRT_DIST, "PEAK_MUL_SQRT_DIST", "" },
      { c_vlo_file::DATA_CHANNEL_DOUBLED_ECHO_PEAKS, "DOUBLED_ECHO_PEAKS", "Peak values for double echos" },
      { c_vlo_file::DATA_CHANNEL_DOUBLED_ECHO_AREAS, "DOUBLED_ECHO_AREAS", "Area values for double echos"},
      { c_vlo_file::DATA_CHANNEL_DOUBLED_ECHO_DISTANCES, "DOUBLED_ECHO_DISTANCES", "Distance values for double echos"},
      { c_vlo_file::DATA_CHANNEL_DIST_TO_MAX_PEAK, "DIST_TO_MAX_PEAK", "" },
      { c_vlo_file::DATA_CHANNEL_GHOSTS_MASK, "GHOSTS_MASK", "" },

      { c_vlo_file::DATA_CHANNEL_AMBIENT },
  };

  return members;
}

#endif // __ssprintf_h__

namespace {
///////////////////////////////////////////////////////////////////////////////////////////////////

/*
 * This is simple compile-time check if struct EchoType has field named 'peak'
 * */
template <class EchoType>
static inline constexpr auto has_peak_member(const EchoType & e) ->
  decltype(EchoType::peak, bool())
{
  return true;
}
static inline constexpr bool has_peak_member(...)
{
  return false;
}


/**
 * Sort echoes by distance appropriate for scans of type 1, 3, and 5
 * */

template<class ScanType>
std::enable_if_t<(c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_1 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_3 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_5),
void> sort_echos_by_distance(ScanType & scan)
{
//  force if ( scan.config.echoOrdering != VLO_ECHO_ORDER_DISTANCE_NEAR_TO_FAR )
  {

    typedef typename ScanType::Echo echo_type;
    typedef decltype(echo_type::dist) distance_type;

    constexpr auto min_distance = scan.MIN_DISTANCE;
    constexpr auto max_distance = scan.MAX_DISTANCE;

    echo_type echos[scan.NUM_ECHOS];
    int cne = 0; // count non-empty echos

    for( int s = 0; s < scan.NUM_SLOTS; ++s ) {

      auto &slot = scan.slot[s];

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {

        memset(echos, 0, sizeof(echos));
        cne = 0;

        for( int e = 0; e < scan.NUM_ECHOS; ++e ) {

          const auto & distance  =
              slot.echo[l][e].dist;

          const auto & area =
              slot.echo[l][e].area;

          if ( distance >= min_distance && distance <= max_distance && area > 0 && area < 30000 ) {
            echos[cne++] = slot.echo[l][e];
          }
        }

        if( cne > 1 ) {

          std::sort(echos, echos + cne,
              [](const auto & prev, const auto & next) {
                return prev.dist < next.dist;
              });
        }

        memcpy(slot.echo[l], echos, sizeof(echos));
      }
    }

    scan.config.echoOrdering = VLO_ECHO_ORDER_DISTANCE_NEAR_TO_FAR;
  }
}

/**
 * Sort echoes by distance appropriate for scans of type 6
 * */

void sort_echos_by_distance(c_vlo_scan6_base & scan)
{
  typedef c_vlo_scan6_base ScanType;

  // force if ( scan.config.echoOrdering != VLO_ECHO_ORDER_DISTANCE_NEAR_TO_FAR )
  {

    typedef struct ScanType::Echo echo_type;
    typedef decltype(ScanType::Echo::dist) dist_type;
    constexpr auto max_dist_value = std::numeric_limits<dist_type>::max() - 2;

    echo_type echos[scan.NUM_ECHOS];
    int cne = 0; // count non-empty echos

    for( int s = 0; s < scan.NUM_SLOTS; ++s ) {

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {

        memset(echos, 0, sizeof(echos));
        cne = 0;

        for( int e = 0; e < scan.NUM_ECHOS; ++e ) {

          const auto & distance  =
              scan.echo[s][l][e].dist;

          const auto & area =
              scan.echo[s][l][e].area;

          if( distance > 0 && distance < max_dist_value && area > 0 && area < 30000 ) {
            echos[cne++] = scan.echo[s][l][e];
          }
        }

        if( cne > 1 ) {

          std::sort(echos, echos + cne,
              [](const auto & prev, const auto & next) {
                return prev.dist < next.dist;
              });
        }

        memcpy(scan.echo[s][l], echos, sizeof(echos));
      }
    }

    scan.config.echoOrdering = VLO_ECHO_ORDER_DISTANCE_NEAR_TO_FAR;
  }
}

/**
 * Sort echoes by distance appropriate for scans of type SLIM
 * */
void sort_echos_by_distance(c_vlo_scan6_slm & scan)
{
  // force if ( scan.config.echoOrdering != VLO_ECHO_ORDER_DISTANCE_NEAR_TO_FAR )
  {

    typedef c_vlo_scan6_slm ScanType;
    typedef struct ScanType::Echo echo_type;
    typedef decltype(ScanType::Echo::dist) dist_type;

    constexpr auto min_distance = scan.MIN_DISTANCE;
    constexpr auto max_distance = scan.MAX_DISTANCE;
    constexpr auto min_area = scan.MIN_AREA;
    constexpr auto max_area = scan.MAX_AREA;

    echo_type echos[scan.NUM_ECHOS];
    int cne = 0; // count non-empty echos

    for( int s = 0; s < scan.NUM_SLOTS; ++s ) {

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {

        cne = 0;
        memset(echos, 0, sizeof(echos));

        for( int e = 0; e < scan.NUM_ECHOS; ++e ) {

          const auto & distance  =
              scan.echo[s][l][e].dist;

          if ( distance >= min_distance && distance <= max_distance ) {

            const auto & area =
                scan.echo[s][l][e].area;

            if ( area > 0 && area < 30000 ) { // area >= min_area && area <= max_area
              echos[cne++] = scan.echo[s][l][e];
            }
          }
        }

        if( cne > 1 ) {

          std::sort(echos, echos + cne,
              [](const auto & prev, const auto & next) {
                return prev.dist < next.dist;
              });
        }

        memcpy(scan.echo[s][l], echos, sizeof(echos));
      }
    }

    scan.config.echoOrdering = VLO_ECHO_ORDER_DISTANCE_NEAR_TO_FAR;
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////

template<class ScanType, class Fn>
std::enable_if_t<(c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_1 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_3 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_5),
bool> get_points3d(const ScanType & scan, const Fn & fn)
{
  const double firstVertAngle =
      0.5 * 0.05 * scan.NUM_LAYERS;

  const double tick2radian =
      0.00000008381903173490870551553291862726 * CV_PI / 180;

  const double yawCorrection = 0;

  for( int s = 0; s < scan.NUM_SLOTS; ++s ) {

    const auto &slot =
        scan.slot[s];

    const double horizontalAngle =
        slot.angleTicks * tick2radian + yawCorrection;

    for( int l = 0; l < scan.NUM_LAYERS; ++l ) {

      const double verticalAngle =
          firstVertAngle * CV_PI / 180 - 0.05 * l * CV_PI / 180;

      const double cos_vert = std::cos(verticalAngle);
      const double sin_vert = std::sin(verticalAngle);
      const double cos_hor_cos_vert = std::cos(horizontalAngle) * cos_vert;
      const double sin_hor_cos_vert = std::sin(horizontalAngle) * cos_vert;

      for( int e = 0; e < std::min(3, (int)scan.NUM_ECHOS); ++e ) {

        const auto &echo =
            slot.echo[l][e];

        const auto & distance =
            echo.dist;

        if( distance ) {

          const double scale = 0.01;
          const double x = scale * distance * cos_hor_cos_vert;
          const double y = scale * distance * sin_hor_cos_vert;
          const double z = scale * distance * sin_vert;

          fn(s, l, e, x, y, z);
        }
      }
    }
  }

  return true;
}

template<class Fn>
bool get_points3d(const c_vlo_scan6_slm & scan, const Fn & fn)
{
  typedef c_vlo_scan6_slm ScanType;
  typedef decltype(ScanType::Echo::dist) distance_type;
  typedef decltype(ScanType::Echo::area) area_type;

  for( int s = 0; s < scan.NUM_SLOTS; ++s ) {

    const double sin_azimuth =
        std::sin(scan.horizontalAngles[s]);

    const double cos_azimuth =
        std::cos(scan.horizontalAngles[s]);

    for( int l = 0; l < std::min(scan.START_BAD_LAYERS, scan.NUM_LAYERS); ++l ) {

      const double inclination =
          CV_PI / 2 - scan.verticalAngles[l];

      const double sin_inclination =
          std::sin(inclination);

      const double cos_inclination =
          std::cos(inclination);

      for( int e = 0; e < scan.NUM_ECHOS; ++e ) {

        const auto & dist =
            scan.echo[s][l][e].dist;

        if( !dist ) {
          continue;
        }

        const auto & area =
            scan.echo[s][l][e].area;

        if( !area ) {
          continue;
        }

        // mirrorSide = scan.mirrorSide;
        const double distance = 0.01 * dist;
        const double x = distance * sin_inclination * cos_azimuth;
        const double y = -distance * sin_inclination * sin_azimuth;
        const double z = -distance * cos_inclination;

        fn(s, scan.NUM_LAYERS - l - 1, e, x, y, z);

      }
    }
  }


  return true;
}



template<class ScanType, class Fn>
std::enable_if_t<(c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_1 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_3 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_5),
bool> get_points2d(const ScanType & scan, const cv::Mat3b & emask, const Fn & fn)
{
  for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
    for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
      for( int e = 0; e < 3; ++e ) {
        if( scan.slot[s].echo[l][e].dist && (emask.empty() || !emask[l][s][e]) ) {
          fn(s, l, e, scan.slot[s].echo[l][e]);
        }
      }
    }
  }

  return true;
}

template<class Fn>
bool get_points2d(const c_vlo_scan6_slm & scan, const cv::Mat3b & emask, const Fn & fn)
{
  for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
    for( int l = 0; l < std::min(scan.START_BAD_LAYERS, scan.NUM_LAYERS); ++l ) {
      for( int e = 0; e < 3; ++e ) {
        if( scan.echo[s][l][e].dist && scan.echo[s][l][e].area && (emask.empty() || !emask[l][s][e]) ) {
          fn(s, scan.NUM_LAYERS - l - 1, e, scan.echo[s][l][e]);
        }
      }
    }
  }
  return true;
}


template<class ScanType, class Fn>
std::enable_if_t<(c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_1 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_3 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_5),
bool> get_echos(const ScanType & scan, const Fn & fn)
{
  for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
    for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
      fn(s, l, scan.slot[s].echo[l]);
    }
  }

  return true;
}

template<class Fn>
bool get_echos(const c_vlo_scan6_slm & scan, const Fn & fn)
{
  for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
    for( int l = 0; l < std::min(scan.START_BAD_LAYERS, scan.NUM_LAYERS); ++l ) {
      fn(s, scan.NUM_LAYERS - l - 1, scan.echo[s][l]);
    }
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////


template<class DistanceType, class IntensityType>
static inline bool is_ghost(const DistanceType & DR, const DistanceType & DG,
    const IntensityType & IR, const IntensityType & IG,
    const c_vlo_processing_options * opts)
{
  static constexpr double Dmin = 200;

  const double & saturation_level =
      opts->ghost_options.saturation_level;

  const double & systematic_correction =
      opts->ghost_options.doubled_distanse_systematic_correction;

  const double & depth_tolerance =
      opts->ghost_options.doubled_distanse_depth_tolerance;

 // const double intensity_ratio_slope = 0.7 / 6000;

  const double KK = 3;

  if( DG && DR > Dmin && IR >= saturation_level && IG <= IR  ) { // * (1. - intensity_ratio_slope * IR)
    if( (DG > KK * DR) || std::abs(DG - 2.0 * DR + systematic_correction) < depth_tolerance ) {
      return true;
    }
  }

  return false;
}

/**
 * Prefer 'peak' intensity measure if available
 *  */
template<class EchoType, class _Cb> std::enable_if_t<has_peak_member(EchoType()),
void> static inline get_ghosts(const EchoType echos[3],
    const c_vlo_processing_options * opts,
    const _Cb & callback)
{
//  if( !opts ) {
//    return;
//  }

  const auto &D0 = echos[0].dist;
  const auto &D1 = echos[1].dist;
  const auto &D2 = echos[2].dist;

  const auto &I0 = echos[0].peak;
  const auto &I1 = echos[1].peak;
  const auto &I2 = echos[2].peak;

  if( is_ghost(D0, D1, I0, I1, opts) ) {
    callback(0, 1);
  }

  if( is_ghost(D0, D2, I0, I2, opts) ) {
    callback(0, 2);
  }

  if( is_ghost(D1, D2, I1, I2, opts) ) {
    callback(1, 2);
  }

}

/**
 * Fallback to 'area' intensity measure if 'peak' is not available
 *  */
template<class EchoType, class _Cb> std::enable_if_t<!has_peak_member(EchoType()),
void> static inline get_ghosts(const EchoType echos[3],
    const c_vlo_processing_options * opts,
    const _Cb & callback)
{
//  if( !opts ) {
//    return;
//  }

  const auto &D0 = echos[0].dist;
  const auto &D1 = echos[1].dist;
  const auto &D2 = echos[2].dist;

  const auto &I0 = echos[0].area;
  const auto &I1 = echos[1].area;
  const auto &I2 = echos[2].area;

  if( is_ghost(D0, D1, I0, I1, opts) ) {
    callback(0, 1);
  }

  if( is_ghost(D0, D2, I0, I2, opts) ) {
    callback(0, 2);
  }

  if( is_ghost(D1, D2, I1, I2, opts) ) {
    callback(1, 2);
  }

}


///////////////////////////////////////////////////////////////////////////////////////////////////

template<class ScanType>
std::enable_if_t<(c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_1 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_3 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_5),
cv::Mat> get_image(const ScanType & scan, c_vlo_file::DATA_CHANNEL channel, cv::InputArray exclude_mask,
    const c_vlo_processing_options * opts)
{
  cv::Mat3b emask;

  if( !exclude_mask.empty() ) {
    if( exclude_mask.type() == CV_8UC3 ) {
      if( exclude_mask.rows() == scan.NUM_LAYERS && exclude_mask.cols() == scan.NUM_SLOTS ) {
        emask = exclude_mask.getMat();
      }
    }
  }

  typedef decltype(ScanType::Echo::dist) distance_type;

  switch (channel) {

    case c_vlo_file::DATA_CHANNEL_AMBIENT: {

      typedef typename std::remove_reference_t<decltype(ScanType::Slot::ambient[0])> value_type;
      static constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;

      cv::Mat_<value_type> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
          (value_type) 0);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {

          const auto & value =
              scan.slot[s].ambient[l];

          if ( value < max_value ) {
            image[l][s] = value;
          }
        }
      }

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_DISTANCES: {

      cv::Mat_<cv::Vec<distance_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
          cv::Vec<distance_type, 3>::all(0));

      get_points2d(scan, emask,
          [&](int s, int l, int e, const auto & echo) {
            image[l][s][e] = echo.dist;
          });

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_DEPTH: {

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS,
          cv::Vec3f::all(0));

      get_points3d(scan,
          [&](int s, int l, int e, double x, double y, double z) {
            image[l][s][e] = (float)(100 * x);
          });

      return image;
    }



    case c_vlo_file::DATA_CHANNEL_AREA: {

      typedef decltype(ScanType::Echo::area) value_type;
      static constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;

      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
          cv::Vec<value_type, 3>::all(0));

      get_points2d(scan, emask,
          [&](int s, int l, int e, const auto & echo) {
            image[l][s][e] = echo.area;
          });

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_PEAK: {

      typedef decltype(ScanType::Echo::peak) value_type;
      static constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;

      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
          cv::Vec<value_type, 3>::all(0));

      get_points2d(scan, emask,
          [&](int s, int l, int e, const auto & echo) {
            image[l][s][e] = echo.peak;
          });

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_WIDTH: {
      typedef decltype(ScanType::Echo::width) value_type;
      static constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;

      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
          cv::Vec<value_type, 3>::all(0));

      get_points2d(scan, emask,
          [&](int s, int l, int e, const auto & echo) {
            image[l][s][e] = echo.width;
          });

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_AREA_MUL_DIST: {

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS,
          cv::Vec3f::all(0));

      get_points2d(scan, emask,
          [&](int s, int l, int e, const auto & echo) {
            image[l][s][e] = (double)echo.area * ((double) echo.dist);
          });

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_PEAK_MUL_DIST: {

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS,
          cv::Vec3f::all(0));

      get_points2d(scan, emask,
          [&](int s, int l, int e, const auto & echo) {
            image[l][s][e] = (double)echo.peak * ((double) echo.dist);
          });

      return image;
    }

    //
    case c_vlo_file::DATA_CHANNEL_AREA_MUL_SQRT_DIST: {

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS,
          cv::Vec3f::all(0));

      get_points2d(scan, emask,
          [&](int s, int l, int e, const auto & echo) {
            image[l][s][e] = (double)echo.area * sqrt(0.01 * echo.dist);
          });

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_PEAK_MUL_SQRT_DIST: {

      typedef decltype(ScanType::Echo::peak) value_type;

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS,
          cv::Vec3f::all(0));

      get_points2d(scan, emask,
          [&](int s, int l, int e, const auto & echo) {
            image[l][s][e] = (double)echo.peak * sqrt(0.01 * echo.dist);
          });

      return image;
    }



    case c_vlo_file::DATA_CHANNEL_DOUBLED_ECHO_PEAKS: {

      typedef typename ScanType::Echo echo_type;
      typedef decltype(ScanType::Echo::peak) value_type;

      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
          cv::Vec<value_type, 3>::all(0));

      get_echos(scan,
          [&](int s, int l, const echo_type echos[3]) {

        get_ghosts(echos, opts, [&](int e0, int e1) {
                  image[l][s][e0] = echos[e0].peak;
                  image[l][s][e1] = echos[e1].peak;
                });
          });

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_DOUBLED_ECHO_AREAS: {

      typedef typename ScanType::Echo echo_type;
      typedef decltype(ScanType::Echo::area) value_type;

      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
          cv::Vec<value_type, 3>::all(0));

      get_echos(scan,
          [&](int s, int l, const echo_type echos[3]) {

        get_ghosts(echos, opts, [&](int e0, int e1) {
                  image[l][s][e0] = echos[e0].area;
                  image[l][s][e1] = echos[e1].area;
                });
          });

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_DOUBLED_ECHO_DISTANCES: {

      typedef typename ScanType::Echo echo_type;
      typedef decltype(ScanType::Echo::dist) value_type;

      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
          cv::Vec<value_type, 3>::all(0));

      get_echos(scan,
          [&](int s, int l, const echo_type echos[3]) {

        get_ghosts(echos, opts, [&](int e0, int e1) {
                  image[l][s][e0] = echos[e0].dist;
                  image[l][s][e1] = echos[e1].dist;
                });
          });

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_GHOSTS_MASK: {
      typedef typename ScanType::Echo echo_type;
      typedef decltype(ScanType::Echo::dist) value_type;

      cv::Mat3b image(scan.NUM_LAYERS, scan.NUM_SLOTS,
          cv::Vec3b::all(0));

      get_echos(scan,
          [&](int s, int l, const echo_type echos[3]) {

        get_ghosts(echos, opts, [&](int e0, int e1) {
                  image[l][s][e1] = 255;
                });
          });

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_DIST_TO_MAX_PEAK: {

      typedef decltype(ScanType::Echo::peak) value_type;

      cv::Mat_<distance_type> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
          (distance_type) 0);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {

          distance_type max_distance;
          value_type max_value = 0;

          for( int e = 0; e < 3; ++e ) {

            const auto &distance = scan.slot[s].echo[l][e].dist;
            const auto &value = scan.slot[s].echo[l][e].peak;

            if( distance && value > max_value ) {
              max_value = value;
              max_distance = distance;
            }
          }

          if( max_distance ) {
            image[l][s] = max_distance;
          }
        }
      }

      return image;
    }
  }


  return cv::Mat();
}

cv::Mat get_image(const c_vlo_scan6_slm & scan, c_vlo_file::DATA_CHANNEL channel, cv::InputArray exclude_mask,
    const c_vlo_processing_options * opts)
{
  typedef c_vlo_scan6_slm ScanType;

  cv::Mat3b emask;

  if( !exclude_mask.empty() ) {
    if( exclude_mask.type() == CV_8UC3 ) {
      if( exclude_mask.rows() == scan.NUM_LAYERS && exclude_mask.cols() == scan.NUM_SLOTS ) {
        emask = exclude_mask.getMat();
      }
    }
  }

  typedef decltype(ScanType::Echo::dist) distance_type;

  switch (channel) {

    case c_vlo_file::DATA_CHANNEL_AMBIENT:
      case c_vlo_file::DATA_CHANNEL_PEAK:
      case c_vlo_file::DATA_CHANNEL_AREA: {

      typedef decltype(ScanType::Echo::area) value_type;

      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
          cv::Vec<value_type, 3>::all(0));

      get_points2d(scan, emask,
          [&](int s, int l, int e, const auto & echo) {
            image[l][s][e] = echo.area;
          });

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_DISTANCES: {

      cv::Mat_<cv::Vec<distance_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
          cv::Vec<distance_type, 3>::all(0));

      get_points2d(scan, emask,
          [&](int s, int l, int e, const auto & echo) {
            image[l][s][e] = echo.dist;
          });

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_DEPTH: {

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS,
          cv::Vec3f::all(0));

      get_points3d(scan,
          [&](int s, int l, int e, double x, double y, double z) {
            image[l][s][e] = (float)(100 * x);
          });

      return image;
    }


    case c_vlo_file::DATA_CHANNEL_PEAK_MUL_DIST:
      case c_vlo_file::DATA_CHANNEL_AREA_MUL_DIST: {

      typedef decltype(ScanType::Echo::area) value_type;

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS,
          cv::Vec3f::all(0));

      get_points2d(scan, emask,
          [&](int s, int l, int e, const auto & echo) {
            image[l][s][e] = ((double) echo.area) * ((double) echo.dist);
          });

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_PEAK_MUL_SQRT_DIST:
      case c_vlo_file::DATA_CHANNEL_AREA_MUL_SQRT_DIST: {

      typedef decltype(ScanType::Echo::area) value_type;

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS,
          cv::Vec3f::all(0));

      get_points2d(scan, emask,
          [&](int s, int l, int e, const auto & echo) {
            image[l][s][e] = ((double) echo.area) * sqrt(0.01 * echo.dist);
          });

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_DOUBLED_ECHO_AREAS:
    case c_vlo_file::DATA_CHANNEL_DOUBLED_ECHO_PEAKS: {

      typedef typename ScanType::Echo echo_type;
      typedef decltype(ScanType::Echo::area) value_type;

      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
          cv::Vec<value_type, 3>::all(0));

      get_echos(scan,
          [&](int s, int l, const echo_type echos[3]) {

        get_ghosts(echos, opts, [&](int e0, int e1) {
                  image[l][s][e0] = echos[e0].area;
                  image[l][s][e1] = echos[e1].area;
                });
          });

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_DOUBLED_ECHO_DISTANCES: {

      typedef typename ScanType::Echo echo_type;
      typedef decltype(ScanType::Echo::dist) value_type;

      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
          cv::Vec<value_type, 3>::all(0));

      get_echos(scan,
          [&](int s, int l, const echo_type echos[3]) {

        get_ghosts(echos, opts, [&](int e0, int e1) {
                  image[l][s][e0] = echos[e0].dist;
                  image[l][s][e1] = echos[e1].dist;
                });
          });

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_GHOSTS_MASK: {
      typedef typename ScanType::Echo echo_type;
      typedef decltype(ScanType::Echo::dist) value_type;

      cv::Mat3b image(scan.NUM_LAYERS, scan.NUM_SLOTS,
          cv::Vec3b::all(0));

      get_echos(scan,
          [&](int s, int l, const echo_type echos[3]) {

        get_ghosts(echos, opts, [&](int e0, int e1) {
                  image[l][s][e1] = 255;
                });
          });

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_DIST_TO_MAX_PEAK: {

      typedef decltype(ScanType::Echo::area) value_type;

      cv::Mat_<distance_type> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
          (distance_type) 0);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {

          distance_type max_distance;
          value_type max_value = 0;

          for( int e = 0; e < 3; ++e ) {

            const auto &distance = scan.echo[s][l][e].dist;
            const auto &value = scan.echo[s][l][e].area;

            if( distance && value > max_value ) {
              max_value = value;
              max_distance = distance;
            }
          }

          if( max_distance ) {
            image[scan.NUM_LAYERS - l - 1][s] = max_distance;
          }
        }
      }

      return image;
    }
  }

  return cv::Mat();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

template<class ScanType>
std::enable_if_t<(c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_1 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_3 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_5),
bool> get_ray_inclinations_table(const ScanType & scan, cv::Mat1f & table)
{
  constexpr double tick2radian =
      0.00000008381903173490870551553291862726 * CV_PI / 180;

  constexpr double yawCorrection = 0;

  table.create(scan.NUM_SLOTS, 1);

  for( int s = 0; s < scan.NUM_SLOTS; ++s ) {

    table[s][0] = (float) (scan.slot[s].angleTicks * tick2radian + yawCorrection);
  }

  return true;
}

template<class ScanType>
std::enable_if_t<(c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_1 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_3 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_5),
bool> get_ray_azimuths_table(const ScanType & scan, cv::Mat1f & table)
{
  const double firstVertAngle =
      0.5 * 0.05 * scan.NUM_LAYERS;

  table.create(scan.NUM_LAYERS, 1);

  for( int l = 0; l < scan.NUM_LAYERS; ++l ) {

    table[l][0] = (float)(firstVertAngle * CV_PI / 180 - 0.05 * l * CV_PI / 180);
  }

  return true;
}

bool get_ray_inclinations_table(const c_vlo_scan6_slm & scan, cv::Mat1f & table)
{
  table.create(scan.NUM_SLOTS, 1);

  for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
    table[s][0] = scan.horizontalAngles[s];
  }

  return true;
}

bool get_ray_azimuths_table(const c_vlo_scan6_slm & scan, cv::Mat1f & table)
{
  table.create(scan.NUM_LAYERS, 1);

  for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
    table[scan.NUM_LAYERS - l - 1][0] = (float) (CV_PI / 2 - scan.verticalAngles[l]);
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////


template<class ScanType>
bool get_cloud3d(const ScanType & scan, c_vlo_reader::DATA_CHANNEL intensity_channel,
    cv::OutputArray points, cv::OutputArray colors,
    cv::InputArray exclude_mask,
    const c_vlo_processing_options * opts)
{

  cv::Mat3b emask;

  if( !exclude_mask.empty() ) {
    if( exclude_mask.type() == CV_8UC3 ) {
      if( exclude_mask.rows() == scan.NUM_LAYERS && exclude_mask.cols() == scan.NUM_SLOTS ) {
        emask = exclude_mask.getMat();
      }
    }
  }

  cv::Mat intensityImage =
      get_image(scan, intensity_channel,
          cv::noArray(),
          opts);

  if( intensityImage.empty() ) {
    CF_ERROR("get_image(intensity_channel) fails");
    return false;
  }

  if( intensityImage.channels() == 1 ) {
    cv::cvtColor(intensityImage, intensityImage,
        cv::COLOR_GRAY2BGR);
  }

  if( intensityImage.depth() != CV_32F ) {
    intensityImage.convertTo(intensityImage, CV_32F);
  }

  const cv::Mat3f intensity =
      intensityImage;

  std::vector<cv::Vec3f> output_points;
  std::vector<cv::Vec3f> output_colors;

  output_points.reserve(scan.NUM_SLOTS * scan.NUM_LAYERS);
  output_colors.reserve(scan.NUM_SLOTS * scan.NUM_LAYERS);

  get_points3d(scan,
      [&](int s, int l, int e, double x, double y, double z) {

        if ( emask.empty() || !emask[l][s][e]) {

          const float gray_level =
              intensity[l][s][e];

          output_points.emplace_back((float)x, (float)y, (float)z);
          output_colors.emplace_back(gray_level, gray_level, gray_level);
        }
      });


  cv::Mat(output_points).copyTo(points);
  cv::Mat(output_colors).copyTo(colors);

  return true;
}

template<class ScanType>
bool get_clouds3d(const ScanType & scan, cv::Mat3f clouds[3],
    const c_vlo_processing_options * opts)
{
  for( int i = 0; i < 3; ++i ) {
    clouds[i] = cv::Mat3f(scan.NUM_LAYERS, scan.NUM_SLOTS,
        cv::Vec3f::all(0));
  }

  get_points3d(scan,
      [&](int s, int l, int e, double x, double y, double z) {
        clouds[e][l][s][0] = (float) x;
        clouds[e][l][s][1] = (float) y;
        clouds[e][l][s][2] = (float) z;
      });

  return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////
} // namespace


//////////////////////////////////////////

c_vlo_file::c_vlo_file()
{
}

c_vlo_file::c_vlo_file(const std::string & filename) :
  filename_(filename)
{
}

const std::string & c_vlo_file::filename() const
{
  return filename_;
}

VLO_VERSION c_vlo_file::version() const
{
  return version_;
}

bool c_vlo_file::sort_echos_by_distance(c_vlo_scan & scan)
{
  switch (scan.version) {
    case VLO_VERSION_1:
      ::sort_echos_by_distance(scan.scan1);
      return true;
    case VLO_VERSION_3:
      ::sort_echos_by_distance(scan.scan3);
      return true;
    case VLO_VERSION_5:
      ::sort_echos_by_distance(scan.scan5);
      return true;
    case VLO_VERSION_6_SLM:
      ::sort_echos_by_distance(scan.scan6_slm);
      return true;
  }
  return false;
}

cv::Mat c_vlo_file::get_image(const c_vlo_scan & scan, DATA_CHANNEL channel, cv::InputArray exclude_mask,
    const c_vlo_processing_options * opts)
{
  switch (scan.version) {
    case VLO_VERSION_1:
      return ::get_image(scan.scan1, channel, exclude_mask, opts);
    case VLO_VERSION_3:
      return ::get_image(scan.scan3, channel, exclude_mask, opts);
    case VLO_VERSION_5:
      return ::get_image(scan.scan5, channel, exclude_mask, opts);
    case VLO_VERSION_6_SLM:
      return ::get_image(scan.scan6_slm, channel, exclude_mask, opts);
  }
  return cv::Mat();
}

bool c_vlo_file::get_cloud3d(const c_vlo_scan & scan, DATA_CHANNEL intensity_channel,
    cv::OutputArray points, cv::OutputArray colors,
    cv::InputArray exclude_mask,
    const c_vlo_processing_options * opts)
{
  switch (scan.version) {
    case VLO_VERSION_1:
      return ::get_cloud3d(scan.scan1, intensity_channel, points, colors, exclude_mask, opts);
    case VLO_VERSION_3:
      return ::get_cloud3d(scan.scan3, intensity_channel, points, colors, exclude_mask, opts);
    case VLO_VERSION_5:
      return ::get_cloud3d(scan.scan5, intensity_channel, points, colors, exclude_mask, opts);
    case VLO_VERSION_6_SLM:
      return ::get_cloud3d(scan.scan6_slm, intensity_channel, points, colors, exclude_mask, opts);
  }
  CF_DEBUG("Unsupported scan version %d specified", scan.version);
  return false;
}

bool c_vlo_file::get_clouds3d(const c_vlo_scan & scan, cv::Mat3f clouds[3],
    const c_vlo_processing_options * opts)
{
  switch (scan.version) {
    case VLO_VERSION_1:
      return ::get_clouds3d(scan.scan1, clouds, opts);
    case VLO_VERSION_3:
      return ::get_clouds3d(scan.scan3, clouds, opts);
    case VLO_VERSION_5:
      return ::get_clouds3d(scan.scan5, clouds, opts);
    case VLO_VERSION_6_SLM:
      return ::get_clouds3d(scan.scan6_slm, clouds, opts);
  }
  CF_DEBUG("Unsupported scan version %d specified", scan.version);
  return false;
}


bool c_vlo_file::get_ray_inclinations_table(const c_vlo_scan & scan, cv::Mat1f & table)
{
  switch (scan.version) {
    case VLO_VERSION_1:
      return ::get_ray_inclinations_table(scan.scan1, table);
    case VLO_VERSION_3:
      return ::get_ray_inclinations_table(scan.scan3, table);
    case VLO_VERSION_5:
      return ::get_ray_inclinations_table(scan.scan5, table);
    case VLO_VERSION_6_SLM:
      return ::get_ray_inclinations_table(scan.scan6_slm, table);
  }
  CF_DEBUG("Unsupported scan version %d specified", scan.version);
  return false;

}

bool c_vlo_file::get_ray_azimuths_table(const c_vlo_scan & scan, cv::Mat1f & table)
{
  switch (scan.version) {
    case VLO_VERSION_1:
      return ::get_ray_azimuths_table(scan.scan1, table);
    case VLO_VERSION_3:
      return ::get_ray_azimuths_table(scan.scan3, table);
    case VLO_VERSION_5:
      return ::get_ray_azimuths_table(scan.scan5, table);
    case VLO_VERSION_6_SLM:
      return ::get_ray_azimuths_table(scan.scan6_slm, table);
  }
  CF_DEBUG("Unsupported scan version %d specified", scan.version);
  return false;

}

//////////////////////////////////////////

c_vlo_reader::c_vlo_reader()
{
}

c_vlo_reader::c_vlo_reader(const std::string & filename) :
    base(filename)
{
  if ( !filename_.empty() && !open() ) {
    CF_ERROR("open('%s') fails: %s", filename_.c_str(),
        strerror(errno));
  }
}

c_vlo_reader::~c_vlo_reader()
{
  close();
}

c_vlo_processing_options * c_vlo_reader::processing_options()
{
  return &processing_options_;
}

bool c_vlo_reader::open(const std::string & filename)
{
  close();

  if( !filename.empty() ) {
    filename_ = filename;
  }

  if( filename_.empty() ) {
    CF_ERROR("c_vlo_reader::open() : no filename specified");
    return false;
  }

  bool fOk = false;
  uint16_t format_version = 0;

  frame_size_ = -1;
  version_ = VLO_VERSION_UNKNOWN;

  ////////////

  if( ifhd_.open(filename) && ifhd_.select_stream("ScaLa 3-PointCloud") ) {

    const size_t payload_size =
        ifhd_.current_payload_size();

    if( !ifhd_.read_payload(&format_version, sizeof(format_version)) == sizeof(format_version) ) {
      CF_ERROR("ifhd_.read_payload(format_version) fails. payload_size=%zu", payload_size);
    }
    else {

      static constexpr uint32_t scanV1Extra = 336;
      static constexpr uint32_t scanV3Extra = 1328;
      static constexpr uint32_t scanV5Extra = 1120;
      static constexpr uint32_t scanV6SLMExtra = 1;

//        CF_DEBUG("\n"
//            "format_version = %u\n"
//            "payload_size=%zu \n"
//            "sizeof(c_vlo_scan6_slm)=%zu\n"
//            "sizeof(c_vlo_scan6_imx449)=%zu\n"
//            "sizeof(c_vlo_scan6_imx479)=%zu\n"
//            "sizeof(c_vlo_scan6_base)=%zu\n"
//            "sizeof(c_vlo_scan5)=%zu\n"
//            "sizeof(c_vlo_scan3)=%zu\n"
//            "sizeof(c_vlo_scan1)=%zu\n"
//            "\n",
//            format_version,
//            payload_size,
//            sizeof(c_vlo_scan6_slm),
//            sizeof(c_vlo_scan6_imx449),
//            sizeof(c_vlo_scan6_imx449),
//            sizeof(c_vlo_scan6_imx479),
//            sizeof(c_vlo_scan6_base),
//            sizeof(c_vlo_scan5),
//            sizeof(c_vlo_scan3),
//            sizeof(c_vlo_scan1));

      if( format_version == 18 && payload_size == sizeof(c_vlo_scan1) + scanV1Extra ) {
        version_ = VLO_VERSION_1;
        frame_size_ = sizeof(c_vlo_scan1);
      }
      else if( format_version == 18 && payload_size == sizeof(c_vlo_scan3) + scanV3Extra ) {
        version_ = VLO_VERSION_3;
        frame_size_ = sizeof(c_vlo_scan3);
      }
      else if( format_version == 5 && payload_size == sizeof(c_vlo_scan5) + scanV5Extra ) {
        version_ = VLO_VERSION_5;
        frame_size_ = sizeof(c_vlo_scan5);
      }
      else if( format_version == 5 && payload_size == sizeof(c_vlo_scan5) ) {
        version_ = VLO_VERSION_5;
        frame_size_ = sizeof(c_vlo_scan5);
      }
      else if( format_version == 6 && payload_size == sizeof(c_vlo_scan6_slm) + scanV6SLMExtra ) {
        version_ = VLO_VERSION_6_SLM;
        frame_size_ = sizeof(c_vlo_scan6_slm);
      }

      CF_DEBUG("format_version: %d -> %d frame_size_ = %zd",
          format_version, version_, frame_size_);

      if( frame_size_ > 0 ) {
        ifhd_.seek(0);
        num_frames_ = ifhd_.num_frames();
        fOk = true;
      }
    }
  }

  if ( !fOk ) {
    ifhd_.close();
  }


  ////////////

  if ( !ifhd_.is_open() ) {

#if _WIN32 || _WIN64
  constexpr int openflags = O_RDONLY | O_BINARY;
#else
    constexpr int openflags = O_RDONLY | O_NOATIME;
#endif
    ssize_t file_size = 0;

    const char *fname = filename_.c_str();

    if( !fd_.open(fname, openflags)) {
      CF_ERROR("fd_.open('%s') fails: %s", fname, strerror(errno));
      goto end;
    }

    if( fd_.read(&format_version, sizeof(format_version)) != sizeof(format_version) ) {
      CF_ERROR("read('%s', format_version) fails: %s", fname,
          strerror(errno));
      goto end;
    }

    //    CF_DEBUG("vlo struct sizes: scan1=%zu scan3=%zu scan=%zu",
    //        sizeof(c_vlo_scan1),
    //        sizeof(c_vlo_scan3),
    //        sizeof(c_vlo_scan5));

    CF_DEBUG("vlo format version %u in '%s'",
        format_version, fname);

    switch (format_version) {
      case 1:
        version_ = VLO_VERSION_1;
        frame_size_ = sizeof(c_vlo_scan1);
        break;
      case 3:
        version_ = VLO_VERSION_3;
        frame_size_ = sizeof(c_vlo_scan3);
        break;
      case 5:
        version_ = VLO_VERSION_5;
        frame_size_ = sizeof(c_vlo_scan5);
        break;
      case 18: {

        uint16_t data11, data12, data31, data32;
        bool isv1, isv3;
        long offset;

        if( fd_.readfrom(offset = 1 * sizeof(c_vlo_scan1), &data11, sizeof(data11)) != sizeof(data11) ) {
          CF_ERROR("readfrom('%s', offset=%ld, data11) fails: %s", fname, offset,
              strerror(errno));
          goto end;
        }
        if( fd_.readfrom(offset = 2 * sizeof(c_vlo_scan1), &data12, sizeof(data12)) != sizeof(data12) ) {
          CF_ERROR("readfrom('%s', offset=%ld, data12) fails: %s", fname, offset,
              strerror(errno));
          goto end;
        }
        if( fd_.readfrom(offset = 1 * sizeof(c_vlo_scan3), &data31, sizeof(data31)) != sizeof(data31) ) {
          CF_ERROR("readfrom('%s', offset=%ld, data31) fails: %s", fname, offset,
              strerror(errno));
          goto end;
        }
        if( fd_.readfrom(offset = 2 * sizeof(c_vlo_scan3), &data32, sizeof(data32)) != sizeof(data32) ) {
          CF_ERROR("readfrom('%s', offset=%ld, data32) fails: %s", fname, offset,
              strerror(errno));
          goto end;
        }

        isv1 = data11 == 18 && data12 == 18;
        isv3 = data31 == 18 && data32 == 18;

        if( isv1 && !isv3 ) {
          version_ = VLO_VERSION_1;
          frame_size_ = sizeof(c_vlo_scan1);
        }
        else if( isv3 && !isv1 ) {
          version_ = VLO_VERSION_3;
          frame_size_ = sizeof(c_vlo_scan3);
        }
        else {
          CF_ERROR("Can not determine exact format version for buggy vlo file '%s' : '%u' ",
              fname, format_version);
          errno = ENOMSG;
          goto end;
        }

        CF_DEBUG("vlo version %u selected for '%s'",
            version_, fname);

        break;
      }
      default:
        CF_ERROR("Not supported format version in vlo file '%s' : '%u' ", fname,
            format_version);
        errno = ENOMSG;
        goto end;
    }

    fd_.seek(0, SEEK_SET);

    if( (file_size = fd_.size()) < 0 ) {
      CF_ERROR("fd_.size('%s') fails: %s", fname,
          strerror(errno));
      goto end;
    }

    num_frames_ = file_size / frame_size();

    if( num_frames_ * frame_size() != file_size ) { // temporary ignore this error
      CF_ERROR("vlo file '%s': file size = %zd bytes not match to expected number of frames %zd in", fname,
          file_size, num_frames_);
    }


    fOk = true;
  }

end:
  if( !fOk ) {
    close();
  }

  ////////////

  return fOk;
}

void c_vlo_reader::close()
{
  ifhd_.close();
  fd_.close();
}

bool c_vlo_reader::is_open() const
{
  return fd_.is_open() || ifhd_.is_open();
}

/// @brief get frame size in bytes
ssize_t c_vlo_reader::frame_size() const
{
  return frame_size_;
}

/// @brief get number of frames in this file
ssize_t c_vlo_reader::num_frames() const
{
  return ifhd_.is_open() ?
      ifhd_.num_frames() :
      num_frames_;
}

bool c_vlo_reader::seek(int32_t frame_index)
{
  if( ifhd_.is_open() ) {
    return ifhd_.seek(frame_index);
  }

  if( fd_.is_open() ) {

    const ssize_t seekpos =
        frame_index * frame_size();

    const ssize_t pos =
        fd_.seek(seekpos, SEEK_SET);

    return pos ==  seekpos;
  }

  errno = EBADF;
  return false;
}

int32_t c_vlo_reader::curpos()
{
  if( ifhd_.is_open() ) {
    return ifhd_.curpos();
  }

  if( fd_.is_open() ) {
    return fd_.whence() / frame_size();
  }

  errno = EBADF;
  return -1;
}

template<class ScanType> std::enable_if_t<(c_vlo_scan_type_traits<ScanType>::VERSION > 0),
    bool> c_vlo_reader::read(ScanType * scan)
{
  if( this->version_ != c_vlo_scan_type_traits<ScanType>::VERSION ) {
    errno = EINVAL;
    return false;
  }

  if( fd_.is_open() ) {
    if( fd_.read(scan, sizeof(*scan)) == sizeof(*scan) ) {
      ::sort_echos_by_distance(*scan);
      return true;
    }
    return false;
  }

  if( ifhd_.is_open() ) {
    if( ifhd_.read_payload(scan, sizeof(*scan)) == sizeof(*scan) ) {
      ::sort_echos_by_distance(*scan);
      return true;
    }
    return false;
  }

  errno = EBADF;
  return false;
}


bool c_vlo_reader::read(c_vlo_scan * scan)
{
  switch (scan->version = this->version_) {
    case VLO_VERSION_1:
      return read(&scan->scan1);
    case VLO_VERSION_3:
      return read(&scan->scan3);
    case VLO_VERSION_5:
      return read(&scan->scan5);
    case VLO_VERSION_6_IMX449:
      return read(&scan->scan6_imx449);
    case VLO_VERSION_6_IMX479:
      return read(&scan->scan6_imx479);
    case VLO_VERSION_6_SLM:
      return read(&scan->scan6_slm);
  }
  errno = EBADF;
  return false;
}

bool c_vlo_reader::read(cv::Mat * image, c_vlo_file::DATA_CHANNEL channel)
{
  std::unique_ptr<c_vlo_scan> scan(new c_vlo_scan());
  if( read(scan.get()) ) {

    const cv::Mat3b exclude_mask =
        processing_options_.enable_ghost_filter ?
            get_image(*scan, DATA_CHANNEL_GHOSTS_MASK, cv::noArray(), &processing_options_) :
            cv::Mat3b();

    return !(*image = get_image(*scan, channel, exclude_mask, &processing_options_)).empty();
  }
  return false;
}

bool c_vlo_reader::read_cloud3d(cv::OutputArray points, cv::OutputArray colors, c_vlo_file::DATA_CHANNEL colors_channel)
{
  std::unique_ptr<c_vlo_scan> scan(new c_vlo_scan());
  if ( read(scan.get()) ) {

    const cv::Mat3b exclude_mask =
        processing_options_.enable_ghost_filter ?
            get_image(*scan, DATA_CHANNEL_GHOSTS_MASK, cv::noArray(), &processing_options_) :
            cv::Mat3b();

    return get_cloud3d(*scan, colors_channel, points, colors, exclude_mask, &processing_options_);
  }

  return false;
}

