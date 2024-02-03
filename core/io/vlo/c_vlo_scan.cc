/*
 * c_vlo_scan.cc
 *
 *  Created on: Dec 17, 2023
 *      Author: amyznikov
 */
#include "c_vlo_scan.h"
#include <core/proc/reduce_channels.h>

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
  static const c_enum_member members[] = {
      { VLO_DATA_CHANNEL_AMBIENT, "AMBIENT", "" },
      { VLO_DATA_CHANNEL_DISTANCES, "DISTANCES", "" },
      { VLO_DATA_CHANNEL_DEPTH, "DEPTH", "" },
      { VLO_DATA_CHANNEL_HEIGHT, "HEIGHT", "" },
      { VLO_DATA_CHANNEL_AREA, "ECHO_AREA", "" },
      { VLO_DATA_CHANNEL_PEAK, "ECHO_PEAK", "" },
      { VLO_DATA_CHANNEL_WIDTH, "ECHO_WIDTH", "" },
      { VLO_DATA_CHANNEL_AMBIENT },
  };

  return members;
}

#endif // __ssprintf_h__


namespace {

///////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//template<class ScanType>
//std::enable_if_t<(c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_1 ||
//    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_3 ||
//    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_5),
//bool> inline get_vlo_ray_inclinations_table(const ScanType & scan, cv::Mat1f & table)
//{
//  constexpr double tick2radian =
//      0.00000008381903173490870551553291862726 * CV_PI / 180;
//
//  constexpr double yawCorrection = 0;
//
//  table.create(scan.NUM_SLOTS, 1);
//
//  for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
//
//    table[s][0] = (float) (scan.slot[s].angleTicks * tick2radian + yawCorrection);
//  }
//
//  return true;
//}
//
//template<class ScanType>
//std::enable_if_t<(c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_1 ||
//    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_3 ||
//    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_5),
//bool> inline get_vlo_ray_azimuths_table(const ScanType & scan, cv::Mat1f & table)
//{
//  const double firstVertAngle =
//      0.5 * 0.05 * scan.NUM_LAYERS;
//
//  table.create(scan.NUM_LAYERS, 1);
//
//  for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
//
//    table[l][0] = (float)(firstVertAngle * CV_PI / 180 - 0.05 * l * CV_PI / 180);
//  }
//
//  return true;
//}
//
//inline bool get_vlo_ray_inclinations_table(const c_vlo_scan6_slm & scan, cv::Mat1f & table)
//{
//  table.create(scan.NUM_SLOTS, 1);
//
//  for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
//    table[s][0] = scan.horizontalAngles[s];
//  }
//
//  return true;
//}
//
//inline bool get_vlo_ray_azimuths_table(const c_vlo_scan6_slm & scan, cv::Mat1f & table)
//{
//  table.create(scan.NUM_LAYERS, 1);
//
//  for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
//    table[scan.NUM_LAYERS - l - 1][0] = (float) (CV_PI / 2 - scan.verticalAngles[l]);
//  }
//
//  return true;
//}

///////////////////////////////////////////////////////////////////////////////////////////////////

}

cv::Mat get_vlo_image(const c_vlo_scan & scan, VLO_DATA_CHANNEL channel,
    cv::InputArray selection)
{
  cv::Mat image;

  switch (channel) {
    case VLO_DATA_CHANNEL_AMBIENT:
      if( selection.empty() ) {
        image = scan.ambient.empty() ? (cv::Mat) scan.distances : (cv::Mat) scan.ambient;
      }
      else if( !scan.ambient.empty() ) {

        if ( selection.channels() == 1 ) {
          scan.ambient.copyTo(image, selection);
        }
        else {
          cv::Mat m;
          reduce_color_channels(selection, m, cv::REDUCE_MAX);
          scan.ambient.copyTo(image, m);
        }
      }
      else if( !scan.distances.empty() ) {

        if ( selection.channels() == 1 ) {
          scan.distances.copyTo(image, selection);
        }
        else {
          cv::Mat m;
          reduce_color_channels(selection, m, cv::REDUCE_MAX);
          scan.distances.copyTo(image, m);
        }
      }
      break;

    case VLO_DATA_CHANNEL_DISTANCES:
      if( selection.empty() ) {
        image = scan.distances;
      }
      else if( !scan.distances.empty() ) {
        scan.distances.copyTo(image, selection);
      }
      break;

    case VLO_DATA_CHANNEL_AREA:
      if( selection.empty() ) {
        image = scan.area;
      }
      else if( !scan.area.empty() ) {
        scan.area.copyTo(image, selection);
      }
      break;

    case VLO_DATA_CHANNEL_PEAK:
      if( selection.empty() ) {
        image = scan.peak;
      }
      else if( !scan.peak.empty() ) {
        scan.peak.copyTo(image, selection);
      }
      break;

    case VLO_DATA_CHANNEL_WIDTH:
      if( selection.empty() ) {
        image = scan.width;
      }
      else if( !scan.width.empty() ) {
        scan.width.copyTo(image, selection);
      }
      break;

    case VLO_DATA_CHANNEL_DEPTH: {

      cv::Mat3f depths(scan.size, cv::Vec3f(0, 0, 0));

      vlo_points_callback(scan, selection,
          [&](int l, int s, int e) {
            depths[l][s][e] = scan.clouds[e][l][s][0];
          });

      image = std::move(depths);

      break;
    }

    case VLO_DATA_CHANNEL_HEIGHT: {

      cv::Mat3f heights(scan.size, cv::Vec3f(0, 0, 0));

      vlo_points_callback(scan, selection,
          [&](int l, int s, int e) {
            heights[l][s][e] = scan.clouds[e][l][s][2];
          });

      image = std::move(heights);

      break;
    }

    default:
      break;
  }

  return image;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

