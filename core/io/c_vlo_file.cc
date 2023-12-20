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

#if ! __DEBUG_H_INCLUDED__
#include <stdio.h>

#define CF_DEBUG(...) \
    fprintf(stderr, "%s(): %d ", __func__, __LINE__), \
    fprintf(stderr, __VA_ARGS__), \
    fprintf(stderr, "\n"), \
    fflush(stderr)

#define CF_ERROR CF_DEBUG
#endif

namespace {
///////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//

//// DATA_CHANNEL_LOW_INTENSITY_VALUE
//template<class EchoType>
//static inline double low_intensity_value(const EchoType & echo, const c_vlo_processing_options * opts)
//{
//  const auto & distance = echo.dist;
//
//  const auto & area = echo.area;
//
//  if( area <= opts->low_intensity_filter.u ) {
//    return -1;
//  }
//
//  const double v =
//      log(area - opts->low_intensity_filter.u) +
//          opts->low_intensity_filter.v * distance;
//
//  return v;
//}
//
//
//
//template<class EchoType>
//static inline bool is_low_intensity(const EchoType & echo, const c_vlo_processing_options * opts)
//{
//  const auto & distance = echo.dist;
//
//  const auto & area = echo.area;
//
//  if( area <= opts->low_intensity_filter.u ) {
//    return true;
//  }
//
//  const double v =
//      low_intensity_value(echo, opts);
//
//  return v < opts->low_intensity_filter.low_intensity_level;
//}

///////////////////////////////////////////////////////////////////////////////////////////////////
//
//template<class ScanType>
//std::enable_if_t<(c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_1 ||
//    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_3 ||
//    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_5),
//cv::Mat> get_image(const ScanType & scan, VLO_DATA_CHANNEL channel, cv::InputArray exclude_mask,
//    const c_vlo_processing_options * opts)
//{
//  cv::Mat3b emask;
//
//  if( !exclude_mask.empty() ) {
//    if( exclude_mask.type() == CV_8UC3 ) {
//      if( exclude_mask.rows() == scan.NUM_LAYERS && exclude_mask.cols() == scan.NUM_SLOTS ) {
//        emask = exclude_mask.getMat();
//      }
//    }
//  }
//
//  typedef decltype(ScanType::Echo::dist)
//      distance_type;
//
//  switch (channel) {
//
//    case VLO_DATA_CHANNEL_AMBIENT: {
//
//      typedef typename std::remove_reference_t<decltype(ScanType::Slot::ambient[0])> value_type;
//      static constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;
//
//      cv::Mat_<value_type> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          (value_type) 0);
//
//      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
//        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
//
//          const auto & value =
//              scan.slot[s].ambient[l];
//
//          if ( value < max_value ) {
//            image[l][s] = value;
//          }
//        }
//      }
//
//      return image;
//    }
//
//    case VLO_DATA_CHANNEL_DISTANCES: {
//
//      cv::Mat_<cv::Vec<distance_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          cv::Vec<distance_type, 3>::all(0));
//
//      get_vlo_points2d(scan,
//          [&](int l, int s, int e, const auto & echo) {
//            if ( emask.empty() || !emask[l][s][e] ) {
//              image[l][s][e] = echo.dist;
//            }
//          });
//
//      return image;
//    }
//
//    case VLO_DATA_CHANNEL_DEPTH: {
//
//      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          cv::Vec3f::all(0));
//
//      get_vlo_points3d(scan,
//          [&](int l, int s, int e, double x, double y, double z, const auto & echo) {
//            image[l][s][e] = (float)(100 * x);
//          });
//
//      return image;
//    }
//
//
//
//    case VLO_DATA_CHANNEL_AREA: {
//
//      typedef decltype(ScanType::Echo::area) value_type;
//      static constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;
//
//      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          cv::Vec<value_type, 3>::all(0));
//
//      get_vlo_points2d(scan,
//          [&](int l, int s, int e, const auto & echo) {
//            if ( emask.empty() || !emask[l][s][e] ) {
//              image[l][s][e] = echo.area;
//            }
//          });
//
//      return image;
//    }
//
//    case VLO_DATA_CHANNEL_PEAK: {
//
//      typedef decltype(ScanType::Echo::peak) value_type;
//      static constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;
//
//      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          cv::Vec<value_type, 3>::all(0));
//
//      get_vlo_points2d(scan,
//          [&](int l, int s, int e, const auto & echo) {
//            if ( emask.empty() || !emask[l][s][e] ) {
//              image[l][s][e] = echo.peak;
//            }
//          });
//
//      return image;
//    }
//
//    case VLO_DATA_CHANNEL_WIDTH: {
//      typedef decltype(ScanType::Echo::width) value_type;
//      static constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;
//
//      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          cv::Vec<value_type, 3>::all(0));
//
//      get_vlo_points2d(scan,
//          [&](int l, int s, int e, const auto & echo) {
//            if ( emask.empty() || !emask[l][s][e] ) {
//              image[l][s][e] = echo.width;
//            }
//          });
//
//      return image;
//    }
//
//    case VLO_DATA_CHANNEL_AREA_MUL_DIST: {
//
//      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          cv::Vec3f::all(0));
//
//      get_vlo_points2d(scan,
//          [&](int s, int l, int e, const auto & echo) {
//            if ( emask.empty() || !emask[l][s][e] ) {
//              image[l][s][e] = (double)echo.area * ((double) echo.dist);
//            }
//          });
//
//      return image;
//    }
//
//    case VLO_DATA_CHANNEL_PEAK_MUL_DIST: {
//
//      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          cv::Vec3f::all(0));
//
//      get_vlo_points2d(scan,
//          [&](int l, int s, int e, const auto & echo) {
//            if ( emask.empty() || !emask[l][s][e] ) {
//              image[l][s][e] = (double)echo.peak * ((double) echo.dist);
//            }
//          });
//
//      return image;
//    }
//
//    //
//    case VLO_DATA_CHANNEL_AREA_MUL_SQRT_DIST: {
//
//      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          cv::Vec3f::all(0));
//
//      get_vlo_points2d(scan,
//          [&](int l, int s, int e, const auto & echo) {
//            if ( emask.empty() || !emask[l][s][e] ) {
//              image[l][s][e] = (double)echo.area * std::log(0.01 * echo.dist);
//            }
//          });
//
//      return image;
//    }
//
//    case VLO_DATA_CHANNEL_PEAK_MUL_SQRT_DIST: {
//
//      typedef decltype(ScanType::Echo::peak) value_type;
//
//      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          cv::Vec3f::all(0));
//
//      get_vlo_points2d(scan,
//          [&](int l, int s, int e, const auto & echo) {
//            if ( emask.empty() || !emask[l][s][e] ) {
//              image[l][s][e] = (double)echo.peak * sqrt(0.01 * echo.dist);
//            }
//          });
//
//      return image;
//    }
//
//
//
//    case VLO_DATA_CHANNEL_DOUBLED_ECHO_PEAKS: {
//
//      typedef typename ScanType::Echo echo_type;
//      typedef decltype(ScanType::Echo::peak) value_type;
//
//      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          cv::Vec<value_type, 3>::all(0));
//
//      get_vlo_echos(scan,
//          [&](int s, int l, const echo_type echos[3]) {
//
//            get_ghosts(echos, opts, [&](int e0, int e1) {
//                  image[l][s][e0] = echos[e0].peak;
//                  image[l][s][e1] = echos[e1].peak;
//                });
//          });
//
//      return image;
//    }
//
//    case VLO_DATA_CHANNEL_DOUBLED_ECHO_AREAS: {
//
//      typedef typename ScanType::Echo echo_type;
//      typedef decltype(ScanType::Echo::area) value_type;
//
//      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          cv::Vec<value_type, 3>::all(0));
//
//      get_vlo_echos(scan,
//          [&](int s, int l, const echo_type echos[3]) {
//
//            get_ghosts(echos, opts, [&](int e0, int e1) {
//                  image[l][s][e0] = echos[e0].area;
//                  image[l][s][e1] = echos[e1].area;
//                });
//          });
//
//      return image;
//    }
//
//    case VLO_DATA_CHANNEL_DOUBLED_ECHO_DISTANCES: {
//
//      typedef typename ScanType::Echo echo_type;
//      typedef decltype(ScanType::Echo::dist) value_type;
//
//      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          cv::Vec<value_type, 3>::all(0));
//
//      get_vlo_echos(scan,
//          [&](int s, int l, const echo_type echos[3]) {
//
//            get_ghosts(echos, opts, [&](int e0, int e1) {
//                  image[l][s][e0] = echos[e0].dist;
//                  image[l][s][e1] = echos[e1].dist;
//                });
//          });
//
//      return image;
//    }
//
//    case VLO_DATA_CHANNEL_GHOSTS_MASK: {
//      typedef typename ScanType::Echo echo_type;
//      typedef decltype(ScanType::Echo::dist) value_type;
//
//      cv::Mat3b image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          cv::Vec3b::all(0));
//
//      if ( opts ) {
//
//        get_vlo_echos(scan,
//            [&](int s, int l, const echo_type echos[3]) {
//
//            get_ghosts(echos, opts, [&](int e0, int e1) {
//                    image[l][s][e1] = 255;
//                  });
//            });
//      }
//
//      return image;
//    }
//
//    case VLO_DATA_CHANNEL_LOW_INTENSITY_VALUE: {
//
//      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          cv::Vec3f::all(0));
//
//      get_vlo_points2d(scan,
//          [&](int l, int s, int e, const auto & echo) {
//            if ( emask.empty() || !emask[l][s][e] ) {
//              image[l][s][e] = low_intensity_value(echo, opts);
//            }
//          });
//
//      return image;
//    }
//
//    case VLO_DATA_CHANNEL_LOW_INTENSITY_MASK : {
//
//      typedef typename ScanType::Echo echo_type;
//      typedef decltype(ScanType::Echo::dist) value_type;
//
//      cv::Mat3b image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          cv::Vec3b::all(0));
//
//      get_vlo_points2d(scan,
//          [&](int l, int s, int e, const auto & echo) {
//            if ( emask.empty() || !emask[l][s][e] ) {
//              if ( is_low_intensity(echo, opts) ) {
//                image[l][s][e] = 255;
//              }
//            }
//          });
//
//      return image;
//    }
//
//
//    case VLO_DATA_CHANNEL_DIST_TO_MAX_PEAK: {
//
//      typedef decltype(ScanType::Echo::peak) value_type;
//
//      cv::Mat_<distance_type> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          (distance_type) 0);
//
//      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
//        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
//
//          distance_type max_distance;
//          value_type max_value = 0;
//
//          for( int e = 0; e < 3; ++e ) {
//
//            const auto &distance = scan.slot[s].echo[l][e].dist;
//            const auto &value = scan.slot[s].echo[l][e].peak;
//
//            if( distance && value > max_value ) {
//              max_value = value;
//              max_distance = distance;
//            }
//          }
//
//          if( max_distance ) {
//            image[l][s] = max_distance;
//          }
//        }
//      }
//
//      return image;
//    }
//  }
//
//
//  return cv::Mat();
//}
//
//cv::Mat get_image(const c_vlo_scan6_slm & scan, VLO_DATA_CHANNEL channel, cv::InputArray exclude_mask,
//    const c_vlo_processing_options * opts)
//{
//  typedef c_vlo_scan6_slm ScanType;
//
//  cv::Mat3b emask;
//
//  if( !exclude_mask.empty() ) {
//    if( exclude_mask.type() == CV_8UC3 ) {
//      if( exclude_mask.rows() == scan.NUM_LAYERS && exclude_mask.cols() == scan.NUM_SLOTS ) {
//        emask = exclude_mask.getMat();
//      }
//    }
//  }
//
//  typedef decltype(ScanType::Echo::dist) distance_type;
//
//  switch (channel) {
//
//    case VLO_DATA_CHANNEL_AMBIENT:
//      case VLO_DATA_CHANNEL_PEAK:
//      case VLO_DATA_CHANNEL_AREA: {
//
//      typedef decltype(ScanType::Echo::area) value_type;
//
//      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          cv::Vec<value_type, 3>::all(0));
//
//      get_vlo_points2d(scan,
//          [&](int l, int s, int e, const auto & echo) {
//            if ( emask.empty() || !emask[l][s][e] ) {
//              image[l][s][e] = echo.area;
//            }
//          });
//
//      return image;
//    }
//
//    case VLO_DATA_CHANNEL_DISTANCES: {
//
//      cv::Mat_<cv::Vec<distance_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          cv::Vec<distance_type, 3>::all(0));
//
//      get_vlo_points2d(scan,
//          [&](int l, int s, int e, const auto & echo) {
//            if ( emask.empty() || !emask[l][s][e] ) {
//              image[l][s][e] = echo.dist;
//            }
//          });
//
//      return image;
//    }
//
//    case VLO_DATA_CHANNEL_DEPTH: {
//
//      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          cv::Vec3f::all(0));
//
//      get_vlo_points3d(scan,
//          [&](int l, int s, int e, double x, double y, double z, const auto & echo) {
//            if ( emask.empty() || !emask[l][s][e] ) {
//              image[l][s][e] = (float)(100 * x);
//            }
//          });
//
//      return image;
//    }
//
//
//    case VLO_DATA_CHANNEL_PEAK_MUL_DIST:
//      case VLO_DATA_CHANNEL_AREA_MUL_DIST: {
//
//      typedef decltype(ScanType::Echo::area) value_type;
//
//      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          cv::Vec3f::all(0));
//
//      get_vlo_points2d(scan,
//          [&](int l, int s, int e, const auto & echo) {
//            if ( emask.empty() || !emask[l][s][e] ) {
//              image[l][s][e] = ((double) echo.area) * ((double) echo.dist);
//            }
//          });
//
//      return image;
//    }
//
//    case VLO_DATA_CHANNEL_PEAK_MUL_SQRT_DIST:
//      case VLO_DATA_CHANNEL_AREA_MUL_SQRT_DIST: {
//
//      typedef decltype(ScanType::Echo::area) value_type;
//
//      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          cv::Vec3f::all(0));
//
//      get_vlo_points2d(scan,
//          [&](int l, int s, int e, const auto & echo) {
//            if ( emask.empty() || !emask[l][s][e] ) {
//              image[l][s][e] = (double)echo.area * std::log(0.01 * echo.dist);
//            }
//          });
//
//      return image;
//    }
//
//    case VLO_DATA_CHANNEL_DOUBLED_ECHO_AREAS:
//    case VLO_DATA_CHANNEL_DOUBLED_ECHO_PEAKS: {
//
//      typedef typename ScanType::Echo echo_type;
//      typedef decltype(ScanType::Echo::area) value_type;
//
//      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          cv::Vec<value_type, 3>::all(0));
//
//      get_vlo_echos(scan,
//          [&](int s, int l, const echo_type echos[3]) {
//
//            get_ghosts(echos, opts, [&](int e0, int e1) {
//                  image[l][s][e0] = echos[e0].area;
//                  image[l][s][e1] = echos[e1].area;
//                });
//          });
//
//      return image;
//    }
//
//    case VLO_DATA_CHANNEL_DOUBLED_ECHO_DISTANCES: {
//
//      typedef typename ScanType::Echo echo_type;
//      typedef decltype(ScanType::Echo::dist) value_type;
//
//      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          cv::Vec<value_type, 3>::all(0));
//
//      get_vlo_echos(scan,
//          [&](int s, int l, const echo_type echos[3]) {
//
//            get_ghosts(echos, opts, [&](int e0, int e1) {
//                  image[l][s][e0] = echos[e0].dist;
//                  image[l][s][e1] = echos[e1].dist;
//                });
//          });
//
//      return image;
//    }
//
//    case VLO_DATA_CHANNEL_GHOSTS_MASK: {
//      typedef typename ScanType::Echo echo_type;
//      typedef decltype(ScanType::Echo::dist) value_type;
//
//      cv::Mat3b image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          cv::Vec3b::all(0));
//
//      if ( opts ) {
//
//        get_vlo_echos(scan,
//            [&](int s, int l, const echo_type echos[3]) {
//
//            get_ghosts(echos, opts, [&](int e0, int e1) {
//                    image[l][s][e1] = 255;
//                  });
//            });
//      }
//
//      return image;
//    }
//
//    case VLO_DATA_CHANNEL_LOW_INTENSITY_VALUE: {
//
//      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          cv::Vec3f::all(0));
//
//      get_vlo_points2d(scan,
//          [&](int l, int s, int e, const auto & echo) {
//            if ( emask.empty() || !emask[l][s][e] ) {
//              image[l][s][e] = low_intensity_value(echo, opts);
//            }
//          });
//
//      return image;
//    }
//
//    case VLO_DATA_CHANNEL_LOW_INTENSITY_MASK : {
//
//      typedef typename ScanType::Echo echo_type;
//      typedef decltype(ScanType::Echo::dist) value_type;
//
//      cv::Mat3b image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          cv::Vec3b::all(0));
//
//      get_vlo_points2d(scan,
//          [&](int l, int s, int e, const auto & echo) {
//            if ( emask.empty() || !emask[l][s][e] ) {
//              if ( is_low_intensity(echo, opts) ) {
//                image[l][s][e] = 255;
//              }
//            }
//          });
//
//      return image;
//    }
//
//    case VLO_DATA_CHANNEL_DIST_TO_MAX_PEAK: {
//
//      typedef decltype(ScanType::Echo::area) value_type;
//
//      cv::Mat_<distance_type> image(scan.NUM_LAYERS, scan.NUM_SLOTS,
//          (distance_type) 0);
//
//      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
//        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
//
//          distance_type max_distance;
//          value_type max_value = 0;
//
//          for( int e = 0; e < 3; ++e ) {
//
//            const auto &distance = scan.echo[s][l][e].dist;
//            const auto &value = scan.echo[s][l][e].area;
//
//            if( distance && value > max_value ) {
//              max_value = value;
//              max_distance = distance;
//            }
//          }
//
//          if( max_distance ) {
//            image[scan.NUM_LAYERS - l - 1][s] = max_distance;
//          }
//        }
//      }
//
//      return image;
//    }
//  }
//
//  return cv::Mat();
//}

///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////

//
//template<class ScanType>
//bool get_cloud3d(const ScanType & scan, VLO_DATA_CHANNEL intensity_channel,
//    cv::OutputArray points, cv::OutputArray colors,
//    cv::InputArray exclude_mask,
//    const c_vlo_processing_options * opts)
//{
//
//  cv::Mat3b emask;
//
//  if( !exclude_mask.empty() ) {
//    if( exclude_mask.type() == CV_8UC3 ) {
//      if( exclude_mask.rows() == scan.NUM_LAYERS && exclude_mask.cols() == scan.NUM_SLOTS ) {
//        emask = exclude_mask.getMat();
//      }
//    }
//  }
//
//  cv::Mat intensityImage =
//      get_image(scan, intensity_channel,
//          cv::noArray(),
//          opts);
//
//  if( intensityImage.empty() ) {
//    CF_ERROR("get_image(intensity_channel) fails");
//    return false;
//  }
//
//  if( intensityImage.channels() == 1 ) {
//    cv::cvtColor(intensityImage, intensityImage,
//        cv::COLOR_GRAY2BGR);
//  }
//
//  if( intensityImage.depth() != CV_32F ) {
//    intensityImage.convertTo(intensityImage, CV_32F);
//  }
//
//  const cv::Mat3f intensity =
//      intensityImage;
//
//  std::vector<cv::Vec3f> output_points;
//  std::vector<cv::Vec3f> output_colors;
//
//  output_points.reserve(scan.NUM_SLOTS * scan.NUM_LAYERS);
//  output_colors.reserve(scan.NUM_SLOTS * scan.NUM_LAYERS);
//
//  get_vlo_points3d(scan,
//      [&](int l, int s, int e, double x, double y, double z, const auto & echo) {
//
//        if ( emask.empty() || !emask[l][s][e]) {
//
//          const float gray_level =
//              intensity[l][s][e];
//
//          output_points.emplace_back((float)x, (float)y, (float)z);
//          output_colors.emplace_back(gray_level, gray_level, gray_level);
//        }
//      });
//
//
//  cv::Mat(output_points).copyTo(points);
//  cv::Mat(output_colors).copyTo(colors);
//
//  return true;
//}
//
//template<class ScanType>
//bool get_clouds3d(const ScanType & scan, cv::Mat3f clouds[3],
//    const c_vlo_processing_options * opts)
//{
//  for( int i = 0; i < 3; ++i ) {
//    clouds[i] = cv::Mat3f(scan.NUM_LAYERS, scan.NUM_SLOTS,
//        cv::Vec3f::all(0));
//  }
//
//  get_vlo_points3d(scan,
//      [&](int l, int s, int e, double x, double y, double z, const auto & echo) {
//        clouds[e][l][s][0] = (float) x;
//        clouds[e][l][s][1] = (float) y;
//        clouds[e][l][s][2] = (float) z;
//      });
//
//  return true;
//}
//

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


cv::Mat c_vlo_file::get_thumbnail_image(const c_vlo_scan & scan)
{
  return get_vlo_image(scan, VLO_DATA_CHANNEL_AMBIENT);
}
//
//cv::Mat c_vlo_file::get_image(const c_vlo_scan & scan, VLO_DATA_CHANNEL channel, cv::InputArray exclude_mask,
//    const c_vlo_processing_options * opts)
//{
//  switch (scan.version) {
//    case VLO_VERSION_1:
//      return ::get_image(scan.scan1, channel, exclude_mask, opts);
//    case VLO_VERSION_3:
//      return ::get_image(scan.scan3, channel, exclude_mask, opts);
//    case VLO_VERSION_5:
//      return ::get_image(scan.scan5, channel, exclude_mask, opts);
//    case VLO_VERSION_6_SLM:
//      return ::get_image(scan.scan6_slm, channel, exclude_mask, opts);
//  }
//  return cv::Mat();
//}
//
//bool c_vlo_file::get_cloud3d(const c_vlo_scan & scan, VLO_DATA_CHANNEL intensity_channel,
//    cv::OutputArray points, cv::OutputArray colors,
//    cv::InputArray exclude_mask,
//    const c_vlo_processing_options * opts)
//{
//  switch (scan.version) {
//    case VLO_VERSION_1:
//      return ::get_cloud3d(scan.scan1, intensity_channel, points, colors, exclude_mask, opts);
//    case VLO_VERSION_3:
//      return ::get_cloud3d(scan.scan3, intensity_channel, points, colors, exclude_mask, opts);
//    case VLO_VERSION_5:
//      return ::get_cloud3d(scan.scan5, intensity_channel, points, colors, exclude_mask, opts);
//    case VLO_VERSION_6_SLM:
//      return ::get_cloud3d(scan.scan6_slm, intensity_channel, points, colors, exclude_mask, opts);
//  }
//  CF_ERROR("Unsupported scan version %d specified", scan.version);
//  return false;
//}
//
//bool c_vlo_file::get_clouds3d(const c_vlo_scan & scan, cv::Mat3f clouds[3],
//    const c_vlo_processing_options * opts)
//{
//  switch (scan.version) {
//    case VLO_VERSION_1:
//      return ::get_clouds3d(scan.scan1, clouds, opts);
//    case VLO_VERSION_3:
//      return ::get_clouds3d(scan.scan3, clouds, opts);
//    case VLO_VERSION_5:
//      return ::get_clouds3d(scan.scan5, clouds, opts);
//    case VLO_VERSION_6_SLM:
//      return ::get_clouds3d(scan.scan6_slm, clouds, opts);
//  }
//  CF_ERROR("Unsupported scan version %d specified", scan.version);
//  return false;
//}
//


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
    return fd_.read(scan, sizeof(*scan)) == sizeof(*scan);
  }

  if( ifhd_.is_open() ) {
    return ifhd_.read_payload(scan, sizeof(*scan)) == sizeof(*scan);
  }

  errno = EBADF;
  return false;
}


bool c_vlo_reader::read(c_vlo_scan * scan)
{
  bool fOk = false;

  switch (scan->version = this->version_) {
    case VLO_VERSION_1:
      fOk = read(&scan->scan1);
      break;
    case VLO_VERSION_3:
      fOk = read(&scan->scan3);
      break;
    case VLO_VERSION_5:
      fOk = read(&scan->scan5);
      break;
    case VLO_VERSION_6_IMX449:
      fOk = read(&scan->scan6_imx449);
      break;
    case VLO_VERSION_6_IMX479:
      fOk = read(&scan->scan6_imx479);
      break;
    case VLO_VERSION_6_SLM:
      fOk = read(&scan->scan6_slm);
      break;
    default:
      errno = EBADF;
      return false;
  }

  if ( fOk ) {
    sort_vlo_echos_by_distance(*scan);
  }

  return fOk;
}

//
//template<class ScanType>
//static void compute_exclude_mask_(const ScanType & scan, const c_vlo_processing_options * opts,
//    cv::Mat3b & output_mask)
//{
//  if( opts ) {
//
//    if( opts->ghost_filter.enabled ) {
//      output_mask =
//          get_image(scan,
//              VLO_DATA_CHANNEL_GHOSTS_MASK,
//              cv::noArray(),
//              opts);
//    }
//
//    if( opts->low_intensity_filter.enabled ) {
//
//      if( output_mask.empty() ) {
//
//        output_mask =
//            get_image(scan,
//                VLO_DATA_CHANNEL_LOW_INTENSITY_MASK,
//                cv::noArray(),
//                opts);
//      }
//      else {
//        cv::bitwise_or(output_mask,
//            get_image(scan,
//                VLO_DATA_CHANNEL_LOW_INTENSITY_MASK,
//                cv::noArray(),
//                opts),
//            output_mask);
//      }
//    }
//  }
//}
//
//static void compute_exclude_mask(const c_vlo_scan & scan, const c_vlo_processing_options * opts,
//    cv::Mat3b & output_mask)
//{
//  switch (scan.version) {
//    case VLO_VERSION_1:
//      return compute_exclude_mask_(scan.scan1, opts, output_mask);
//    case VLO_VERSION_3:
//      return compute_exclude_mask_(scan.scan3, opts, output_mask);
//    case VLO_VERSION_5:
//      return compute_exclude_mask_(scan.scan5, opts, output_mask);
////    case VLO_VERSION_6_IMX449:
////      return compute_exclude_mask_(scan.scan6_imx449, opts, output_mask);
////    case VLO_VERSION_6_IMX479:
////      return compute_exclude_mask_(scan.scan6_imx479, opts, output_mask);
//    case VLO_VERSION_6_SLM:
//      return compute_exclude_mask_(scan.scan6_slm, opts, output_mask);
//  }
//}
//
//bool c_vlo_reader::read(cv::Mat * image, VLO_DATA_CHANNEL channel)
//{
//  std::unique_ptr<c_vlo_scan> scan(new c_vlo_scan());
//  if( read(scan.get()) ) {
//    return !(*image = get_vlo_image(*scan, channel)).empty();
//  }
//  return false;
//}
//
//bool c_vlo_reader::read_cloud3d(cv::OutputArray points, cv::OutputArray colors, VLO_DATA_CHANNEL colors_channel)
//{
//  std::unique_ptr<c_vlo_scan> scan(new c_vlo_scan());
//  if ( read(scan.get()) ) {
//
//    cv::Mat3b exclude_mask;
//
//    compute_exclude_mask(*scan, &processing_options_,
//        exclude_mask);
//
//    return get_cloud3d(*scan, colors_channel,
//        points, colors,
//        exclude_mask,
//        &processing_options_);
//  }
//
//  return false;
//}

