/*
 * c_vlo_scan.cc
 *
 *  Created on: Dec 17, 2023
 *      Author: amyznikov
 */
#include "c_vlo_scan.h"
#include <core/ssprintf.h>
#include <core/debug.h>

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
const c_enum_member* members_of<VLO_DATA_CHANNEL>()
{
  static constexpr c_enum_member members[] = {
      { VLO_DATA_CHANNEL_AMBIENT, "AMBIENT", "" },
      { VLO_DATA_CHANNEL_DISTANCES, "DISTANCES", "" },
      { VLO_DATA_CHANNEL_DEPTH, "DEPTH", "" },
      { VLO_DATA_CHANNEL_HEIGHT, "HEIGHT", "" },
      { VLO_DATA_CHANNEL_AREA, "ECHO_AREA", "" },
      { VLO_DATA_CHANNEL_PEAK, "ECHO_PEAK", "" },
      { VLO_DATA_CHANNEL_WIDTH, "ECHO_WIDTH", "" },
//      { VLO_DATA_CHANNEL_AREA_MUL_DIST, "AREA_MUL_DIST", "" },
//      { VLO_DATA_CHANNEL_PEAK_MUL_DIST, "PEAK_MUL_DIST", "" },
//      { VLO_DATA_CHANNEL_AREA_MUL_SQRT_DIST, "AREA_MUL_SQRT_DIST", "" },
//      { VLO_DATA_CHANNEL_PEAK_MUL_SQRT_DIST, "PEAK_MUL_SQRT_DIST", "" },
//      { VLO_DATA_CHANNEL_DOUBLED_ECHO_PEAKS, "DOUBLED_ECHO_PEAKS", "Peak values for double echos" },
//      { VLO_DATA_CHANNEL_DOUBLED_ECHO_AREAS, "DOUBLED_ECHO_AREAS", "Area values for double echos"},
//      { VLO_DATA_CHANNEL_DOUBLED_ECHO_DISTANCES, "DOUBLED_ECHO_DISTANCES", "Distance values for double echos"},
//      { VLO_DATA_CHANNEL_DIST_TO_MAX_PEAK, "DIST_TO_MAX_PEAK", "" },
//      { VLO_DATA_CHANNEL_GHOSTS_MASK, "GHOSTS_MASK", "" },
//      { VLO_DATA_CHANNEL_LOW_INTENSITY_VALUE, "LOW_INTENSITY_VALUE", "" },
//      { VLO_DATA_CHANNEL_LOW_INTENSITY_MASK, "LOW_INTENSITY_MASK", "" },
      { VLO_DATA_CHANNEL_AMBIENT },
  };

  return members;
}

#endif // __ssprintf_h__


namespace {


///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Sort VLO echoes by distance appropriate for scans of type 1, 3, and 5
 * */
template<class ScanType>
std::enable_if_t<(c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_1 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_3 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_5),
void> sort_vlo_echos_by_distance(ScanType & scan)
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
 * Sort VLO echoes by distance appropriate for scans of type 6
 * */
void sort_vlo_echos_by_distance(c_vlo_scan6_base & scan)
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
 * Sort VLO echoes by distance appropriate for scans of type SLIM
 * */
void sort_vlo_echos_by_distance(c_vlo_scan6_slm & scan)
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


template<class ScanType>
std::enable_if_t<(c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_1 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_3 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_5),
bool> inline get_vlo_ray_inclinations_table(const ScanType & scan, cv::Mat1f & table)
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
bool> inline get_vlo_ray_azimuths_table(const ScanType & scan, cv::Mat1f & table)
{
  const double firstVertAngle =
      0.5 * 0.05 * scan.NUM_LAYERS;

  table.create(scan.NUM_LAYERS, 1);

  for( int l = 0; l < scan.NUM_LAYERS; ++l ) {

    table[l][0] = (float)(firstVertAngle * CV_PI / 180 - 0.05 * l * CV_PI / 180);
  }

  return true;
}

inline bool get_vlo_ray_inclinations_table(const c_vlo_scan6_slm & scan, cv::Mat1f & table)
{
  table.create(scan.NUM_SLOTS, 1);

  for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
    table[s][0] = scan.horizontalAngles[s];
  }

  return true;
}

inline bool get_vlo_ray_azimuths_table(const c_vlo_scan6_slm & scan, cv::Mat1f & table)
{
  table.create(scan.NUM_LAYERS, 1);

  for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
    table[scan.NUM_LAYERS - l - 1][0] = (float) (CV_PI / 2 - scan.verticalAngles[l]);
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

//
//
///**
// * Still not sure if this routine can be useful
// */
//template<class ScanType>
//std::enable_if_t<(c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_1 ||
//    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_3 ||
//    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_5),
//cv::Mat> get_vlo_ambient_image(const ScanType & scan)
//{
//  typedef typename std::remove_reference_t<decltype(ScanType::Slot::ambient[0])> value_type;
//  static constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;
//
//  cv::Mat_<value_type> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//      (value_type) 0);
//
//  for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
//    for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
//
//      const auto & value =
//          scan.slot[s].ambient[l];
//
//      if ( value < max_value ) {
//        image[l][s] = value;
//      }
//    }
//  }
//
//  return image;
//}
//
//
///**
// * Still not sure if this routine can be useful
// */
//inline cv::Mat get_vlo_ambient_image(const c_vlo_scan6_slm & scan)
//{
//  typedef c_vlo_scan6_slm ScanType;
//  typedef decltype(ScanType::Echo::area) value_type;
//
//  cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//      cv::Vec<value_type, 3>::all(0));
//
//  for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
//    for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
//      for( int e = 0; e < 3; ++e ) {
//
//        const auto & echo =
//            scan.echo[s][l][e];
//
//        if( echo.dist && echo.area < 30000 ) {
//          image[scan.NUM_LAYERS - l - 1][s][e] = echo.area;
//        }
//      }
//    }
//  }
//
//  return image;
//}

///////////////////////////////////////////////////////////////////////////////////////////////////
}

/**
 * Sort VLO echoes by distance with version tag auto-selection
 */
bool sort_vlo_echos_by_distance(c_vlo_scan & scan)
{
  switch (scan.version) {
    case VLO_VERSION_1:
      sort_vlo_echos_by_distance(scan.scan1);
      return true;
    case VLO_VERSION_3:
      sort_vlo_echos_by_distance(scan.scan3);
      return true;
    case VLO_VERSION_5:
      sort_vlo_echos_by_distance(scan.scan5);
      return true;
    case VLO_VERSION_6_SLM:
      sort_vlo_echos_by_distance(scan.scan6_slm);
      return true;
  }
  return false;
}

bool get_vlo_ray_inclinations_table(const c_vlo_scan & scan, cv::Mat1f & table)
{
  switch (scan.version) {
    case VLO_VERSION_1:
      return get_vlo_ray_inclinations_table(scan.scan1, table);
    case VLO_VERSION_3:
      return get_vlo_ray_inclinations_table(scan.scan3, table);
    case VLO_VERSION_5:
      return get_vlo_ray_inclinations_table(scan.scan5, table);
    case VLO_VERSION_6_SLM:
      return get_vlo_ray_inclinations_table(scan.scan6_slm, table);
  }
  return false;
}

bool get_vlo_ray_azimuths_table(const c_vlo_scan & scan, cv::Mat1f & table)
{
  switch (scan.version) {
    case VLO_VERSION_1:
      return get_vlo_ray_azimuths_table(scan.scan1, table);
    case VLO_VERSION_3:
      return get_vlo_ray_azimuths_table(scan.scan3, table);
    case VLO_VERSION_5:
      return get_vlo_ray_azimuths_table(scan.scan5, table);
    case VLO_VERSION_6_SLM:
      return get_vlo_ray_azimuths_table(scan.scan6_slm, table);
  }
  return false;
}


//cv::Mat get_vlo_ambient_image(const c_vlo_scan & scan)
//{
//  switch (scan.version) {
//    case VLO_VERSION_1:
//      return get_vlo_ambient_image(scan.scan1);
//    case VLO_VERSION_3:
//      return get_vlo_ambient_image(scan.scan3);
//    case VLO_VERSION_5:
//      return get_vlo_ambient_image(scan.scan5);
//    case VLO_VERSION_6_SLM:
//      return get_vlo_ambient_image(scan.scan6_slm);
//    default:
//      break;
//  }
//  return cv::Mat();
//}


///////////////////////////////////////////////////////////////////////////////////////////////////

