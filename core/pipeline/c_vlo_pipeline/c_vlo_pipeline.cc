/*
 * c_vlo_pipeline.cc
 *
 *  Created on: Oct 26, 2023
 *      Author: amyznikov
 */

#include "c_vlo_pipeline.h"
#include <core/readdir.h>
#include <core/ssprintf.h>
#include <core/proc/autoclip.h>
#include <core/proc/colormap.h>
#include <core/mtf/pixinsight-mtf.h>
#include <core/io/save_ply.h>
#include <type_traits>
#include <chrono>
#include <thread>
#include <core/debug.h>

//////////////////////////////
template<>
const c_enum_member* members_of<VLO_INTENSITY_CHANNEL>()
{
  static constexpr c_enum_member members[] = {
      { VLO_INTENSITY_PEAK, "PEAK", "PEAK" },
      { VLO_INTENSITY_AREA, "AREA", "AREA" },
      { VLO_INTENSITY_PEAK },
  };

  return members;
}

//////////////////////////////
//
//template<class ScanType>
//static bool get_cloud3d_(const ScanType & scan,
//    std::vector<cv::Point3f> & output_points,
//    std::vector<cv::Vec3b> & output_colors,
//    c_vlo_reader::DATA_CHANNEL intensity_channel)
//{
//  cv::Mat intensityImage =
//      c_vlo_file::get_image(scan,
//          intensity_channel);
//
//  if( !intensityImage.empty() ) {
//
//    autoclip(intensityImage, cv::noArray(),
//        0.01, 99.99,
//        0, 255);
//
//    if( intensityImage.depth() != CV_8U ) {
//      intensityImage.convertTo(intensityImage, CV_8U);
//    }
//
//    if( false ) {
//
//      static cv::Mat3b lut;
//      if( lut.empty() ) {
//
//        cv::Mat1b gray(1, 256);
//        for( int i = 0; i < 256; ++i ) {
//          gray[0][i] = i;
//        }
//
//        apply_colormap(gray,
//            lut,
//            COLORMAP_TURBO);
//      }
//    }
//
//    if ( intensityImage.channels() == 1 ) {
//      cv::cvtColor(intensityImage, intensityImage,
//          cv::COLOR_GRAY2BGR);
//    }
//  }
//
//  const cv::Mat3b intensity =
//      intensityImage;
//
//  const float firstVertAngle =
//      0.5 * 0.05 * scan.NUM_LAYERS;
//
//  const float tick2deg =
//      (float) (0.00000008381903173490870551553291862726);
//
//  const float yawCorrection = 0;
//
//
//  output_points.clear();
//  output_colors.clear();
//  output_points.reserve(scan.NUM_POINTS);
//  output_colors.reserve(scan.NUM_POINTS);
//
//  for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
//
//    const auto &slot =
//        scan.slot[s];
//
//    const float horizontalAngle =
//        slot.angleTicks * tick2deg * CV_PI / 180 + yawCorrection;
//
//    for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
//
//      const float verticalAngle =
//          firstVertAngle * CV_PI / 180 - 0.05 * l * CV_PI / 180;
//
//      const float cos_vert = cos(verticalAngle);
//      const float sin_vert = sin(verticalAngle);
//      const float cos_hor_cos_vert = cos(horizontalAngle) * cos_vert;
//      const float sin_hor_cos_vert = sin(horizontalAngle) * cos_vert;
//
//      for( int e = 0; e < 2/*scan.NUM_ECHOS*/; ++e ) {
//
//        const auto &echo =
//            slot.echo[l][e];
//
//        const uint16_t distance =
//            echo.dist;
//
//        if( distance > 0 && distance < 65534 ) {
//
//          const float scale = 0.01;
//          const float x = scale * distance * cos_hor_cos_vert;
//          const float y = scale * distance * sin_hor_cos_vert;
//          const float z = scale * distance * sin_vert;
//          output_points.emplace_back(x, y, z);
//
//          const uint8_t color =
//              intensity[l][s][e];
//
//          output_colors.emplace_back(color, color, color);
//        }
//      }
//    }
//  }
//
//
//  return true;
//}
//
//static bool get_cloud3d(const c_vlo_scan & scan,
//    std::vector<cv::Point3f> & output_points,
//    std::vector<cv::Vec3b> & output_colors,
//    c_vlo_reader::DATA_CHANNEL intensity_channel)
//{
//  switch (scan.version) {
//    case VLO_VERSION_1:
//      return get_cloud3d_(scan.scan1, output_points, output_colors, intensity_channel);
//    case VLO_VERSION_3:
//      return get_cloud3d_(scan.scan3, output_points, output_colors, intensity_channel);
//    case VLO_VERSION_5:
//      return get_cloud3d_(scan.scan5, output_points, output_colors, intensity_channel);
//  }
//  CF_DEBUG("Unsupported scan version %d specified", scan.version);
//  return false;
//}
//

template<class ScanType>
static bool run_reflectors_detection_(const ScanType & scan,
    VLO_INTENSITY_CHANNEL intensity_channel,
    bool enable_doubled_echo,
    bool enable_auto_threshold,
    THRESHOLD_TYPE auto_threshold_type,
    double auto_threshold_value,
    double auto_clip_min,
    double auto_clip_max,
    cv::Mat1b * output_mask)
{
  typedef decltype(ScanType::Echo::area) area_type;
  constexpr auto max_area_value = std::numeric_limits<area_type>::max() - 2;

  typedef decltype(ScanType::Echo::peak) peak_type;
  constexpr auto max_peak_value = std::numeric_limits<peak_type>::max() - 2;

  typedef decltype(ScanType::Echo::dist) dist_type;
  constexpr auto max_dist_value = std::numeric_limits<dist_type>::max() - 2;

  cv::Mat1b intensity_mask;
  cv::Mat1b doubled_echo_mask;

  if( enable_auto_threshold ) {

    const int max_intensity_value =
        intensity_channel == VLO_INTENSITY_AREA ?
            max_area_value :
            max_peak_value;

    cv::Mat1w intensity_image(scan.NUM_LAYERS, scan.NUM_SLOTS, (uint16_t) 0);


    for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
      for( int s = 0; s < scan.NUM_SLOTS; ++s ) {

        for ( int e = 0; e < 2; ++e ) {

          const auto &distance =
              scan.slot[s].echo[l][e].dist;

          if( distance < 600 ) {
            continue;
          }

          if( distance < max_dist_value ) {

            const int intensity =
                intensity_channel == VLO_INTENSITY_AREA ?
                    scan.slot[s].echo[l][e].area :
                    scan.slot[s].echo[l][e].peak;

            if( intensity > 0 && intensity < max_intensity_value ) {
              intensity_image[l][s] = intensity;
            }
          }

          break;
        }
      }
    }

    if ( auto_clip_max > auto_clip_min ) {
      autoclip(intensity_image, cv::noArray(), auto_clip_min, auto_clip_max, -1, -1);
    }

    cv::compare(intensity_image,
        get_threshold_value(intensity_image, cv::noArray(),
            auto_threshold_type, auto_threshold_value),
        intensity_mask,
        cv::CMP_GE);
  }


  if( enable_doubled_echo ) {

    doubled_echo_mask.create(scan.NUM_LAYERS, scan.NUM_SLOTS);
    doubled_echo_mask.setTo(0);

    for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
      for( int s = 0; s < scan.NUM_SLOTS; ++s ) {

        for( int e = 0; e < 2; ++e ) {

          const auto &dist0 =
              scan.slot[s].echo[l][e].dist;

          if( dist0 < 600 ) {
            continue;
          }

          if( dist0 > 0 && dist0 < max_dist_value ) {

            const auto &dist1 =
                scan.slot[s].echo[l][e + 1].dist;

            if( dist1 > 0 && dist1 < max_dist_value ) {

              static constexpr dist_type wall_distance = 30000;

              if ( dist1 >= wall_distance ) {
                doubled_echo_mask[l][s] =  dist0 > 0.45 * wall_distance ? 255 : 0;
              }
              else if( std::abs((double) dist1 / (double) dist0 - 2) < 0.04 ) {
                doubled_echo_mask[l][s] = 255;
              }
            }

          }
        }
      }
    }
  }

  if ( !intensity_mask.empty() && enable_doubled_echo ) {
    cv::bitwise_and(intensity_mask, doubled_echo_mask, *output_mask);
  }
  else if ( !intensity_mask.empty() ) {
    *output_mask = std::move(intensity_mask);
  }
  else if ( enable_doubled_echo ) {
    *output_mask = std::move(doubled_echo_mask);
  }

  return true;
}

static bool run_reflectors_detection(const c_vlo_scan & scan,
    VLO_INTENSITY_CHANNEL intensity_channel,
    bool enable_doubled_echo,
    bool enable_auto_threshold,
    THRESHOLD_TYPE auto_threshold_type,
    double auto_threshold_value,
    double auto_clip_min,
    double auto_clip_max,
    cv::Mat1b * output_mask)
{
  switch (scan.version) {
    case VLO_VERSION_1:

      return run_reflectors_detection_(scan.scan1, intensity_channel,
          enable_doubled_echo,
          enable_auto_threshold, auto_threshold_type, auto_threshold_value,
          auto_clip_min, auto_clip_max,
          output_mask);

    case VLO_VERSION_3:
      return run_reflectors_detection_(scan.scan3, intensity_channel,
          enable_doubled_echo,
          enable_auto_threshold, auto_threshold_type, auto_threshold_value,
          auto_clip_min, auto_clip_max,
          output_mask);

    case VLO_VERSION_5:
      return run_reflectors_detection_(scan.scan5, intensity_channel,
          enable_doubled_echo,
          enable_auto_threshold, auto_threshold_type, auto_threshold_value,
          auto_clip_min, auto_clip_max,
          output_mask);
  }
  CF_DEBUG("Unsupported scan version %d specified", scan.version);
  return false;
}


template<class ScanType>
static int run_double_echo_detection_(const ScanType & scan,
    cv::Mat3b & output_mask)
{
  typedef decltype(ScanType::Echo::area) area_type;
  constexpr auto max_area_value = std::numeric_limits<area_type>::max() - 2;

  typedef decltype(ScanType::Echo::peak) peak_type;
  constexpr auto max_peak_value = std::numeric_limits<peak_type>::max() - 2;

  typedef decltype(ScanType::Echo::dist) distance_type;
  constexpr auto max_distance_value = std::numeric_limits<distance_type>::max() - 2;

  static constexpr distance_type wall_distance = 30000;

  output_mask.create(scan.NUM_LAYERS, scan.NUM_SLOTS);
  output_mask.setTo(0);

  int N = 0;

  for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
    for( int s = 0; s < scan.NUM_SLOTS; ++s ) {

      for( int e = 0; e < 2; ++e ) {

        const auto &dist0 =
            scan.slot[s].echo[l][e].dist;

        if( dist0 < 600 ) {
          continue;
        }

        if( dist0 > 0 && dist0 < max_distance_value ) {

          const auto &dist1 =
              scan.slot[s].echo[l][e + 1].dist;

          if( dist1 > 0 && dist1 < max_distance_value ) {

            if( dist1 >= wall_distance ) {
              if ( dist0 > 0.45 * wall_distance ) {
                output_mask[l][s][e] = 255;
                ++N;
              }
            }
            else if( std::abs((double) dist1 / (double) dist0 - 2) < 0.05 ) {
              output_mask[l][s][e] = 255;
              ++N;
            }
          }
        }
      }
    }
  }

  return N;
}


template<class ScanType>
static int run_high_intensity_detection_(const ScanType & scan,
    double high_intensity_threshold,
    cv::Mat3b & output_mask)
{
  typedef decltype(ScanType::Echo::area) area_type;
  constexpr auto max_area_value = std::numeric_limits<area_type>::max() - 2;

  typedef decltype(ScanType::Echo::peak) peak_type;
  constexpr auto max_peak_value = std::numeric_limits<peak_type>::max() - 2;

  typedef decltype(ScanType::Echo::dist) distance_type;
  constexpr auto max_distance_value = std::numeric_limits<distance_type>::max() - 2;

  output_mask.create(scan.NUM_LAYERS, scan.NUM_SLOTS);
  output_mask.setTo(0);

  int N = 0;

  for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
    for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
      for( int e = 0; e < 2; ++e ) {

        const auto & dist =
            scan.slot[s].echo[l][e].dist;

        if( dist < 600 || dist > max_distance_value ) {
          continue;
        }

        if ( scan.slot[s].echo[l][e].peak >= high_intensity_threshold ) {
          output_mask[l][s][e] = 255;
          ++N;
        }
      }
    }
  }

  return N;
}



template<class ScanType>
static bool run_blom_detection_(const ScanType & scan,
    double high_intensity_threshold,
    double blom_slope_min,
    double blom_slope_max,
    double walk_error,
    double double_echo_distance,
    cv::Mat3b & reflectors_mask,
    cv::Mat1f & reflectors_image,
    cv::Mat1w & reflectors_distances,
    cv::Mat1f & blom_slopes_image,
    cv::Mat1f & blom_intensity_image,
    cv::Mat3b & bloming_mask
    )
{
  // 1 Double echo mask
  // 2 High intensity mask
  // 3 Pattern check

  cv::Mat3b double_echo_mask;
  cv::Mat3b high_intensity_mask;
//  cv::Mat1b reflector_echo_index;

  int reflectors_per_slots[scan.NUM_SLOTS] = {0};
  int total_reflectors = 0;


  bloming_mask.release();

  if( run_double_echo_detection_(scan, double_echo_mask) < 0 ) {
    CF_ERROR("run_double_echo_detection() fails");
    return false;
  }

  if ( run_high_intensity_detection_(scan, high_intensity_threshold, high_intensity_mask) < 0 ) {
    CF_ERROR("run_high_intensity_detection_() fails");
    return false;
  }

//  double_echo_distance > 0 &&

//  if ( !double_echo_mask.empty() ) {
//    cv::bitwise_and(double_echo_mask, high_intensity_mask,
//        reflectors_mask);
//  }
//  else {
//    reflectors_mask = high_intensity_mask;
//  }

  reflectors_mask.create(high_intensity_mask.size());
  reflectors_mask.setTo(0);

  for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
    for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
      for( int e = 0; e < 2; ++e ) {

        if ( high_intensity_mask[l][s][e] ) {

          const auto & distance =
              scan.slot[s].echo[l][e].dist;

          if ( distance > double_echo_distance ) {
            reflectors_mask[l][s][e] = 255;
          }
          else if ( double_echo_mask[l][s][e] ) {
            reflectors_mask[l][s][e] = 255;
          }
        }
      }
    }
  }

//  cv::dilate(reflectors_mask, reflectors_mask,
//      cv::Mat1b(1, 3, 255),
//      cv::Point(-1, -1),
//      1,
//      cv::BORDER_CONSTANT);


  if ( false ) {
    cv::Mat1b channels[3];
    cv::Mat1b and_mask;

    cv::split(reflectors_mask, channels);

    cv::bitwise_and(channels[0], channels[1], and_mask);

    int nnz = cv::countNonZero(and_mask);
    if ( nnz > 0 ) {
      CF_ERROR("NNZ=%d", nnz);
    }
  }


  /////////////////////

  reflectors_image.create(scan.NUM_LAYERS, scan.NUM_SLOTS);
  reflectors_image.setTo(0);

  reflectors_distances.create(scan.NUM_LAYERS, scan.NUM_SLOTS);
  reflectors_distances.setTo(0);

  blom_slopes_image.create(scan.NUM_LAYERS, scan.NUM_SLOTS);
  blom_slopes_image.setTo(0);

//  reflector_echo_index.create(scan.NUM_LAYERS, scan.NUM_SLOTS);
//  reflector_echo_index.setTo(0);

  blom_intensity_image.create(scan.NUM_LAYERS, scan.NUM_SLOTS);
  blom_intensity_image.setTo(0);

  for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
    for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
      for( int e = 0; e < 2; ++e ) {
        if ( reflectors_mask[l][s][e] ) {

          reflectors_image[l][s] =
              scan.slot[s].echo[l][e].peak;

//          reflector_echo_index[l][s] = e + 1;

          reflectors_distances[l][s] =
              scan.slot[s].echo[l][e].dist;

          ++reflectors_per_slots[s];
          ++total_reflectors;
        }
      }
    }
  }

  bloming_mask.create(scan.NUM_LAYERS, scan.NUM_SLOTS);
  bloming_mask.setTo(0);

  if( total_reflectors < 1 ) {
    return true;
  }

  ////////////////////////////

  for ( int s = 0; s < scan.NUM_SLOTS; ++s ) {
    if ( reflectors_per_slots[s] > 0 ) {
      for ( int l = 0; l < scan.NUM_LAYERS; ++l ) {

        if ( !reflectors_distances[l][s] ) {
          continue;
        }

        // reflector starts at this pixel

        ////////////////////////////////////////////////////////////
        // Scan the line to find the end pixel of this reflector
        // Compute mean distance and mean intensity of the reflector
        int reflector_start_l = l;
        int reflector_end_l = l;
        float mean_reflector_distance = reflectors_distances[reflector_end_l][s];
        float mean_reflector_intensity = reflectors_image[reflector_end_l][s];

        while (reflector_end_l + 1 < scan.NUM_LAYERS && reflectors_distances[reflector_end_l + 1][s] > 0) {
          ++reflector_end_l;
          mean_reflector_distance += reflectors_distances[reflector_end_l][s];
          mean_reflector_intensity += reflectors_image[reflector_end_l][s];
        }
        mean_reflector_distance /= (reflector_end_l - reflector_start_l + 1);
        mean_reflector_intensity /= (reflector_end_l - reflector_start_l + 1);

        ////////////////////////////////////////////////////////////
        // Search for potential blooming up
        if ( true ) {

          for( int ll = reflector_start_l - 1; ll >= 0; --ll ) {

            for( int e = 0; e < 2; ++e ) {

              const float distance =
                  scan.slot[s].echo[ll][e].dist;

              if( distance < 600 || distance > 30000 ) {
                continue;
              }

              if( std::abs(distance - mean_reflector_distance) < walk_error ) {

                const float intensity =
                    scan.slot[s].echo[ll][e].peak;

                  const float slope =
                      (mean_reflector_intensity - intensity) / (reflector_start_l - ll);

                  blom_slopes_image[ll][s] = slope;
                  blom_intensity_image[ll][s] = intensity;

                  if ( slope >= blom_slope_min && slope <= blom_slope_max ) {

                    if ( bloming_mask.empty() ) {
                      bloming_mask.create(scan.NUM_LAYERS, scan.NUM_SLOTS);
                      bloming_mask.setTo(0);
                    }

                    bloming_mask[ll][s][e] = 255;
                  }

                // break;
              }
            }
          }
        }

        ////////////////////////////////////////////////////////////
        // Search for potential booming down
        if ( true ) {

          for( int ll = reflector_end_l + 1; ll < scan.NUM_LAYERS; ++ll ) {

            for( int e = 0; e < 2; ++e ) {

              const float distance =
                  scan.slot[s].echo[ll][e].dist;

              if( distance < 600 || distance > 30000 ) {
                continue;
              }

              if( std::abs(distance - mean_reflector_distance) < walk_error ) {

                const float intensity =
                    scan.slot[s].echo[ll][e].peak;

                const float slope =
                    (mean_reflector_intensity - intensity) / (ll - reflector_end_l);

                blom_slopes_image[ll][s] = slope;
                blom_intensity_image[ll][s] = intensity;

                if ( slope >= blom_slope_min && slope <= blom_slope_max ) {

                  if ( bloming_mask.empty() ) {
                    bloming_mask.create(scan.NUM_LAYERS, scan.NUM_SLOTS);
                    bloming_mask.setTo(0);
                  }

                  bloming_mask[ll][s][e] = 255;
                }

               // break;
              }
            }
          }
        }

        ////////////////////////////////////////////////////////////



        // search next reflector
        l = reflector_end_l;
      }
    }
  }


  ////////////////////////////

  return true;
}


static bool run_blom_detection(const c_vlo_scan & scan,
    double high_intensity_threshold,
    double blom_slope_min,
    double blom_slope_max,
    double walk_error,
    double double_echo_distance,
    cv::Mat3b & reflectors_mask,
    cv::Mat1f & reflectors_image,
    cv::Mat1w & reflectors_distances,
    cv::Mat1f & blom_slopes_image,
    cv::Mat1f & blom_intensity_image,
    cv::Mat3b & bloming_mask)
{
  switch (scan.version) {
    case VLO_VERSION_1:
      return run_blom_detection_(scan.scan1,
          high_intensity_threshold,
          blom_slope_min,
          blom_slope_max,
          walk_error,
          double_echo_distance,
          reflectors_mask,
          reflectors_image,
          reflectors_distances,
          blom_slopes_image,
          blom_intensity_image,
          bloming_mask);

    case VLO_VERSION_3:
      return run_blom_detection_(scan.scan3,
          high_intensity_threshold,
          blom_slope_min,
          blom_slope_max,
          walk_error,
          double_echo_distance,
          reflectors_mask,
          reflectors_image,
          reflectors_distances,
          blom_slopes_image,
          blom_intensity_image,
          bloming_mask);

    case VLO_VERSION_5:
      return run_blom_detection_(scan.scan5,
          high_intensity_threshold,
          blom_slope_min,
          blom_slope_max,
          walk_error,
          double_echo_distance,
          reflectors_mask,
          reflectors_image,
          reflectors_distances,
          blom_slopes_image,
          blom_intensity_image,
          bloming_mask);
  }
  CF_DEBUG("Unsupported scan version %d specified", scan.version);
  return false;
}


template<class ScanType>
static bool update_vlo_lookup_table_statistics_(const ScanType & scan,
    c_vlo_lookup_table_statistics & statistics)
{

  typedef typename ScanType::Echo echo_type;

  typedef decltype(ScanType::Echo::area) area_type;
  constexpr auto max_area_value = std::numeric_limits<area_type>::max() - 2;

  typedef decltype(ScanType::Echo::peak) peak_type;
  constexpr auto max_peak_value = std::numeric_limits<peak_type>::max() - 2;

  typedef decltype(ScanType::Echo::dist) dist_type;
  constexpr auto max_dist_value = std::numeric_limits<dist_type>::max() - 2;

  constexpr double max_distance_meters  = 500;
  constexpr double bin_size_in_meters = 1.0;
  constexpr int num_bins = (int) (max_distance_meters / bin_size_in_meters) + 1;


  if ( statistics.empty() ) { // Initialize new statistics table;
    statistics.resize(num_bins);
  }

  for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
    for( int s = 0; s < scan.NUM_SLOTS; ++s ) {

      if ( s >= 296 && s <= 325 ) {
        continue;
      }


      echo_type echos[3] = {0};
      int num_echos = 0;

      for( int e = 0; e < scan.NUM_ECHOS; ++e ) {

        const auto & echo =
            scan.slot[s].echo[l][e];

        const auto &distance =
            echo.dist;

        if( distance <= 0 || distance > max_dist_value ) {
          continue;
        }

        const auto &peak =
            echo.peak;

        if( peak <= 0 || peak > max_peak_value ) {
          continue;
        }

        const auto &area =
            echo.area;

        if( area <= 0 || peak > max_area_value ) {
          continue;
        }


        // all data are valid
        echos[num_echos++] = echo;
      }


      if ( num_echos > 0 ) {

        if ( num_echos > 1 ) {

          std::sort(echos, echos + num_echos,
              [](const auto & prev, const auto & next) {
                return next.peak < prev.peak;
              });

        }

        const auto & echo =
            echos[0];

        const int bin_index =
            std::min(num_bins-1, (int)( echo.dist * 0.01 / bin_size_in_meters ));


        c_vlo_lookup_table_item & item =
            statistics[bin_index];

        item.distance += echo.dist * 0.01;
        item.area += echo.area;
        item.peak += echo.peak;
        item.area2 += ((double)echo.area) * ((double)echo.area);
        item.peak2 += ((double)echo.peak) * ((double)echo.peak);
        item.num_measurements += 1;
      }
    }
  }

  return true;
}

static bool update_vlo_lookup_table_statistics(const c_vlo_scan & scan,
    c_vlo_lookup_table_statistics & statistics)
{
  switch (scan.version) {
    case VLO_VERSION_1:
      return update_vlo_lookup_table_statistics_(scan.scan1, statistics);
    case VLO_VERSION_3:
      return update_vlo_lookup_table_statistics_(scan.scan3, statistics);
    case VLO_VERSION_5:
      return update_vlo_lookup_table_statistics_(scan.scan5, statistics);
  }
  CF_DEBUG("Unsupported scan version %d specified", scan.version);
  return false;
}

static bool save_vlo_lookup_table_statistics(const std::string & filename,
    const c_vlo_lookup_table_statistics & table)
{
  FILE * fp = nullptr;


  if ( !create_path(get_parent_directory(filename)) ) {
    CF_ERROR("create_path() for '%s' fails: %s", filename.c_str(), strerror(errno));
    return false;
  }



  if( !(fp = fopen(filename.c_str(), "w")) ) {
    CF_ERROR("FATAL: can not write file '%s' : %s ", filename.c_str(), strerror(errno));
    return false;
  }



  fprintf(fp, "BIN\tDIST\tAREA_MEAN\tAREA_STDEV\tPEAK_MEAN\tPEAK_STDEV\tNPTS\n");

  for ( int i = 0, n = table.size(); i < n; ++i ) {

    const c_vlo_lookup_table_item & item =
        table[i];

    if ( item.num_measurements < 1 ) {
      continue;
    }

    const double N = item.num_measurements;
    const double mean_distance = item.distance / N;
    const double mean_peak = item.peak / N;
    const double mean_area = item.area / N;
    const double stdev_peak = sqrt(item.peak2 / N - mean_peak * mean_peak);
    const double stdev_area = sqrt(item.area2 / N - mean_area * mean_area);

    fprintf(fp, "%5d\t%6.1f"
        "\t%12f\t%12f"
        "\t%7f\t%7f"
        "\t%15g"
        "\n",
        i,
        mean_distance,
        mean_area, stdev_area,
        mean_peak, stdev_peak,
        N
        );

  }


  fclose(fp);

  return true;
}

//////////////////////////////


c_vlo_pipeline::c_vlo_pipeline(const std::string & name, const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
}

bool c_vlo_pipeline::serialize(c_config_setting settings, bool save)
{
  c_config_setting section;

  if ( !base::serialize(settings, save)) {
    return false;
  }

  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    serialize_base_input_options(section, save, input_options_);
    SERIALIZE_OPTION(section, save, input_options_, sort_echos_by_distance);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "processing_options")) ) {
    SERIALIZE_OPTION(section, save, processing_options_, enable_reflectors_detection);
    SERIALIZE_OPTION(section, save, processing_options_, enable_double_echo_detection);
    SERIALIZE_OPTION(section, save, processing_options_, enable_auto_threshold);
    SERIALIZE_OPTION(section, save, processing_options_, auto_threshold_type);
    SERIALIZE_OPTION(section, save, processing_options_, auto_threshold_value);
    SERIALIZE_OPTION(section, save, processing_options_, auto_clip_min);
    SERIALIZE_OPTION(section, save, processing_options_, auto_clip_max);

    SERIALIZE_OPTION(section, save, processing_options_, enable_blom_detection);
    SERIALIZE_OPTION(section, save, processing_options_, high_intensity_threshold);
    SERIALIZE_OPTION(section, save, processing_options_, blom_slope_min);
    SERIALIZE_OPTION(section, save, processing_options_, blom_slope_max);
    SERIALIZE_OPTION(section, save, processing_options_, walk_error);
    SERIALIZE_OPTION(section, save, processing_options_, double_echo_distance);

    SERIALIZE_OPTION(section, save, processing_options_, enable_blom_detection2);
    SERIALIZE_OPTION(section, save, processing_options_.depth_segmentation_, min_distance);
    SERIALIZE_OPTION(section, save, processing_options_.depth_segmentation_, max_distance);
    SERIALIZE_OPTION(section, save, processing_options_.depth_segmentation_, vlo_walk_error);
    SERIALIZE_OPTION(section, save, processing_options_.depth_segmentation_, min_slope);
    SERIALIZE_OPTION(section, save, processing_options_.depth_segmentation_, max_slope);
    SERIALIZE_OPTION(section, save, processing_options_.depth_segmentation_, counts_threshold);
    SERIALIZE_OPTION(section, save, processing_options_.depth_segmentation_, min_segment_size);
    SERIALIZE_OPTION(section, save, processing_options_.depth_segmentation_, min_height);
    SERIALIZE_OPTION(section, save, processing_options_.depth_segmentation_, segmentation_type);

    SERIALIZE_OPTION(section, save, processing_options_, enable_gather_lookup_table_statistics);
    SERIALIZE_OPTION(section, save, processing_options_, vlo_lookup_table_statistics_filename);

    SERIALIZE_OPTION(section, save, processing_options_, vlo_intensity_channel);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    // SERIALIZE_OPTION(section, save, output_options_, output_directory);
    SERIALIZE_OPTION(section, save, output_options_, save_progress_video);
    SERIALIZE_OPTION(section, save, output_options_, progress_video_filename);
    SERIALIZE_OPTION(section, save, output_options_, save_cloud3d_ply);
    SERIALIZE_OPTION(section, save, output_options_, cloud3d_filename);
    SERIALIZE_OPTION(section, save, output_options_, cloud3d_intensity_channel);
    SERIALIZE_OPTION(section, save, output_options_, save_bloom2_segments);
    SERIALIZE_OPTION(section, save, output_options_, bloom2_segments_file_options);
    SERIALIZE_OPTION(section, save, output_options_, save_bloom2_intensity_profiles);
    SERIALIZE_OPTION(section, save, output_options_, bloom2_intensity_profiles_options);

  }

  return true;
}


const std::vector<c_image_processing_pipeline_ctrl> & c_vlo_pipeline::get_controls()
{
  static std::vector<c_image_processing_pipeline_ctrl> ctrls;

  if( ctrls.empty() ) {

    PIPELINE_CTL_GROUP(ctrls, "Input options", "");
      POPULATE_PIPELINE_INPUT_OPTIONS(ctrls)
      PIPELINE_CTL(ctrls, input_options_.sort_echos_by_distance, "sort_echos_by_distance", "");
    PIPELINE_CTL_END_GROUP(ctrls);


    PIPELINE_CTL_GROUP(ctrls, "Processing options", "");

      PIPELINE_CTL_GROUP(ctrls, "VLO lookup table statistics", "");
        PIPELINE_CTL(ctrls, processing_options_.enable_gather_lookup_table_statistics,
            "enable_gather_lookup_table_statistics", "");
        PIPELINE_CTLC(ctrls, processing_options_.vlo_intensity_channel, "vlo_intensity_channel", "",
            _this->processing_options_.enable_gather_lookup_table_statistics);
        PIPELINE_CTLC(ctrls, processing_options_.vlo_lookup_table_statistics_filename, "vlo_statistics_filename", "",
            _this->processing_options_.enable_gather_lookup_table_statistics);
      PIPELINE_CTL_END_GROUP(ctrls);




      PIPELINE_CTL_GROUP(ctrls, "Reflectors Detection", "");
        PIPELINE_CTL(ctrls, processing_options_.enable_reflectors_detection, "enable_reflectors_detection", "");
        PIPELINE_CTLC(ctrls, processing_options_.enable_double_echo_detection, "enable_double_echo_detection", "",
            _this->processing_options_.enable_reflectors_detection);

        PIPELINE_CTLC(ctrls, processing_options_.enable_auto_threshold, "enable_auto_threshold", "",
            _this->processing_options_.enable_reflectors_detection);
        PIPELINE_CTLC(ctrls, processing_options_.auto_threshold_type, "threshold_type", "",
            _this->processing_options_.enable_reflectors_detection && _this->processing_options_.enable_auto_threshold);

        PIPELINE_CTLC(ctrls, processing_options_.auto_threshold_value, "threshold_value", "",
            _this->processing_options_.enable_reflectors_detection &&
            _this->processing_options_.enable_auto_threshold &&
            _this->processing_options_.auto_threshold_type == THRESHOLD_TYPE_VALUE);

        PIPELINE_CTLC(ctrls, processing_options_.auto_clip_min, "auto_clip_min", "",
            _this->processing_options_.enable_reflectors_detection &&
            _this->processing_options_.enable_auto_threshold);

        PIPELINE_CTLC(ctrls, processing_options_.auto_clip_max, "auto_clip_max", "",
            _this->processing_options_.enable_reflectors_detection &&
            _this->processing_options_.enable_auto_threshold);
      PIPELINE_CTL_END_GROUP(ctrls);


      PIPELINE_CTL_GROUP(ctrls, "Blom Detection", "");
        PIPELINE_CTL(ctrls, processing_options_.enable_blom_detection, "enable_bloom_detection", "");
        PIPELINE_CTLC(ctrls, processing_options_.high_intensity_threshold, "high_intensity_threshold", "",
            _this->processing_options_.enable_blom_detection);
        PIPELINE_CTLC(ctrls, processing_options_.blom_slope_min, "blom_slope_min", "",
            _this->processing_options_.enable_blom_detection);
        PIPELINE_CTLC(ctrls, processing_options_.blom_slope_max, "blom_slope_max", "",
            _this->processing_options_.enable_blom_detection);
        PIPELINE_CTLC(ctrls, processing_options_.walk_error, "walk_error", "",
            _this->processing_options_.enable_blom_detection);
        PIPELINE_CTLC(ctrls, processing_options_.double_echo_distance, "double_echo_distance", "",
            _this->processing_options_.enable_blom_detection);
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_GROUP(ctrls, "Blom Detection2", "");
        PIPELINE_CTL(ctrls, processing_options_.enable_blom_detection2, "enable_bloom_detection2", "");

        PIPELINE_CTL_GROUP(ctrls, "Depth segmentation", "");
          PIPELINE_CTL(ctrls, processing_options_.depth_segmentation_.min_distance, "min_distance", "");
          PIPELINE_CTL(ctrls, processing_options_.depth_segmentation_.max_distance, "max_distance", "");
          PIPELINE_CTL(ctrls, processing_options_.depth_segmentation_.vlo_walk_error, "walk_error", "");
          PIPELINE_CTL(ctrls, processing_options_.depth_segmentation_.counts_threshold, "counts_threshold", "");
          PIPELINE_CTL(ctrls, processing_options_.depth_segmentation_.min_segment_size, "min_segment_size", "");
          PIPELINE_CTL(ctrls, processing_options_.depth_segmentation_.min_slope, "min_slope", "");
          PIPELINE_CTL(ctrls, processing_options_.depth_segmentation_.max_slope, "max_slope", "");
          PIPELINE_CTL(ctrls, processing_options_.depth_segmentation_.min_height, "min_height", "");
          PIPELINE_CTL(ctrls, processing_options_.depth_segmentation_.segmentation_type, "segmentation_type", "");

          PIPELINE_CTL_END_GROUP(ctrls);
        PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Output options", "");
      // PIPELINE_CTL_BROWSE_FOR_DIRECTORY(ctrls, output_options_.output_directory, "output_directory", "");

      PIPELINE_CTL(ctrls, output_options_.save_progress_video, "save_progress_video", "");
      PIPELINE_CTL(ctrls, output_options_.progress_video_filename, "progress_video_filename", "");

      PIPELINE_CTL(ctrls, output_options_.save_cloud3d_ply, "save cloud3d ply", "");
      PIPELINE_CTLC(ctrls, output_options_.cloud3d_filename, "cloud3d_filename", "", _this->output_options_.save_cloud3d_ply);
      PIPELINE_CTLC(ctrls, output_options_.cloud3d_intensity_channel, "cloud3d_intensity_channel", "", _this->output_options_.save_cloud3d_ply);

      PIPELINE_CTL(ctrls, output_options_.save_bloom2_segments, "save_bloom2_segments", "");
      PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, output_options_.bloom2_segments_file_options,
          _this->output_options_.save_bloom2_segments);


      PIPELINE_CTL(ctrls, output_options_.save_bloom2_intensity_profiles, "save_bloom2_intensity_profiles", "");
      PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, output_options_.bloom2_intensity_profiles_options,
          _this->output_options_.save_bloom2_intensity_profiles);

    PIPELINE_CTL_END_GROUP(ctrls);
  }

  return ctrls;
}

bool c_vlo_pipeline::copyParameters(const base::sptr & dst) const
{
  if ( !base::copyParameters(dst) ) {
    CF_ERROR("c_vlo_pipeline::base::copyParameters() fails");
    return false;
  }

  this_class::sptr p =
      std::dynamic_pointer_cast<this_class>(dst);

  if( !p ) {
    CF_ERROR("std::dynamic_pointer_cast<this_class=%s>(dst) fails",
        get_class_name().c_str());
    return false;
  }

  p->input_options_ = this->input_options_;
  p->processing_options_ = this->processing_options_;
  p->output_options_ = this->output_options_;

  return true;
}

bool c_vlo_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  lock_guard lock(mutex());

  cv::Mat image =
      c_vlo_file::get_image(current_scan_,
          c_vlo_reader::DATA_CHANNEL_AMBIENT);

  if( !image.empty() ) {
    autoclip(image, cv::noArray(),
        0.5, 99.5,
        0, 255);
    if( image.depth() != CV_8U ) {
      image.convertTo(image, CV_8U);
    }
  }

  display_frame.move(image);
  display_mask.release();

  return true;
}

bool c_vlo_pipeline::initialize_pipeline()
{
  if ( !base::initialize_pipeline() ) {
   CF_ERROR("c_camera_calibration_pipeline: base::initialize() fails");
   return false;
 }

  output_path_ =
      create_output_path(output_options_.output_directory);

  const int num_sources =
      input_sequence_->sources().size();

  for( int i = 0; i < num_sources; ++i ) {

    const c_input_source::sptr &source =
        input_sequence_->source(i);

    if (source->enabled() ) {

      c_vlo_input_source *vlo =
          dynamic_cast<c_vlo_input_source*>(source.get());

      if( !vlo ) {
        CF_ERROR("ERROR: input source %d is not a VLO source", i);
        return false;
      }
    }
  }


  vlo_lookup_table_statistics_.clear();

  return true;
}

void c_vlo_pipeline::cleanup_pipeline()
{
  if ( input_sequence_ ) {
    input_sequence_->close();
  }

  if ( progress_writer_.is_open() ) {
    CF_DEBUG("Closing '%s'", progress_writer_.filename().c_str());
    progress_writer_.close();
  }

  if ( reflectors_writer_.is_open() ) {
    CF_DEBUG("Closing '%s'", reflectors_writer_.filename().c_str());
    reflectors_writer_.close();
  }

  if ( blom_reflectors_mask_writer_.is_open() ) {
    CF_DEBUG("Closing '%s'", blom_reflectors_mask_writer_.filename().c_str());
    blom_reflectors_mask_writer_.close();
  }

  if ( blom_reflectors_writer_.is_open() ) {
    CF_DEBUG("Closing '%s'", blom_reflectors_writer_.filename().c_str());
    blom_reflectors_writer_.close();
  }

  if ( blom_distances_writer_.is_open() ) {
    CF_DEBUG("Closing '%s'", blom_distances_writer_.filename().c_str());
    blom_distances_writer_.close();
  }

  if ( blom_slopes_writer_.is_open() ) {
    CF_DEBUG("Closing '%s'", blom_slopes_writer_.filename().c_str());
    blom_slopes_writer_.close();
  }

  if ( blom_intensity_writer_.is_open() ) {
    CF_DEBUG("Closing '%s'", blom_intensity_writer_.filename().c_str());
    blom_intensity_writer_.close();
  }

  if ( blom_mask_writer_.is_open() ) {
    CF_DEBUG("Closing '%s'", blom_mask_writer_.filename().c_str());
    blom_mask_writer_.close();
  }

  if ( blom_display_writer_.is_open() ) {
    CF_DEBUG("Closing '%s'", blom_display_writer_.filename().c_str());
    blom_display_writer_.close();
  }

  if ( bloom2_segments_writer_.is_open() ) {
    CF_DEBUG("Closing '%s'", bloom2_segments_writer_.filename().c_str());
    bloom2_segments_writer_.close();
  }

  if ( bloom2_intensity_writer_.is_open() ) {
    CF_DEBUG("Closing '%s'", bloom2_intensity_writer_.filename().c_str());
    bloom2_intensity_writer_.close();
  }

  if ( blom2_display_writer_.is_open() ) {
    CF_DEBUG("Closing '%s'", blom2_display_writer_.filename().c_str());
    blom2_display_writer_.close();
  }

  if ( !vlo_lookup_table_statistics_.empty() ) {

    const std::string filename =
        generate_output_filename(processing_options_.vlo_lookup_table_statistics_filename,
            "vlo-statistics",
            ".txt");

    CF_DEBUG("SAVING '%s'", filename.c_str());

    if ( !save_vlo_lookup_table_statistics(filename, vlo_lookup_table_statistics_) ) {
      CF_ERROR("save_vlo_lookup_table_statistics('%s') fails", filename.c_str());
    }
  }

}

bool c_vlo_pipeline::run_pipeline()
{
  if( !input_sequence_->open() ) {
    CF_ERROR("input_sequence_->open() fails");
    return false;
  }

  const bool is_live_sequence =
      input_sequence_->is_live();

  if( is_live_sequence ) {
    total_frames_ = INT_MAX;
  }
  else {

    const int start_pos =
        std::max(input_options_.start_frame_index, 0);

    const int end_pos =
        input_options_.max_input_frames < 1 ?
            input_sequence_->size() :
            std::min(input_sequence_->size(),
                input_options_.start_frame_index + input_options_.max_input_frames);

    total_frames_ = end_pos - start_pos;

    if( total_frames_ < 1 ) {
      CF_ERROR("INPUT ERROR: Number of frames to process = %d is less than 1\n"
          "start_pos=%d end_pos=%d input_sequence_->size()=%d max_input_frames=%d is_live_sequence=%d",
          total_frames_,
          start_pos,
          end_pos,
          input_sequence_->size(),
          input_options_.max_input_frames,
          is_live_sequence);
      return false;
    }

    if( !input_sequence_->seek(start_pos) ) {
      CF_ERROR("ERROR: input_sequence_->seek(start_pos=%d) fails", start_pos);
      return false;
    }
  }

  set_status_msg("RUNNING ...");
  CF_DEBUG("total_frames_=%d", total_frames_);

  processed_frames_ = 0;
  accumulated_frames_ = 0;

  for( ; processed_frames_ < total_frames_; ++processed_frames_, on_frame_processed() ) {

    if( canceled() ) {
      break;
    }

    c_vlo_input_source *vlo =
        dynamic_cast<c_vlo_input_source*>(input_sequence_->current_source().get());
    if( !vlo ) {
      CF_ERROR("dynamic_cast<c_vlo_input_source*>(current_source) fails");
      return false;
    }

    if( true ) {
      lock_guard lock(mutex());
      if( !vlo->read(&current_scan_) ) {
        CF_ERROR("vlo->read(scan) fails: %s", strerror(errno));
        return false;
      }
    }

    if( canceled() ) {
      break;
    }

    if( !process_current_frame() ) {
      CF_ERROR("process_current_frame() fails");
      return false;
    }

    ++accumulated_frames_;

    if( !is_live_sequence ) {
      // give chance to GUI thread to call get_display_image()
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    const int read_pos =
        input_sequence_->current_pos() + 1;
    if( !input_sequence_->seek(read_pos) ) {
      CF_ERROR("input_sequence_->seek(read_pos=%d) fails: %s",
          read_pos, strerror(errno));
      break;
    }

  }

  return true;
}


bool c_vlo_pipeline::process_current_frame()
{
  if( input_options_.sort_echos_by_distance && !c_vlo_file::sort_echos_by_distance(current_scan_) ) {
    CF_ERROR("c_vlo_file::sort_echos_by_distance() fails");
    return false;
  }

  if ( !run_reflectors_detection() ) {
    CF_ERROR("run_reflectors_detection() fails");
    return false;
  }

  if ( !run_blom_detection() ) {
    CF_ERROR("run_bloom_detection() fails");
    return false;
  }

  if ( !run_blom_detection2() ) {
    CF_ERROR("run_bloom_detection2() fails");
    return false;
  }

  if ( !update_vlo_lookup_table_statistics() ) {
    CF_ERROR("update_vlo_lookup_table_statistics() fails");
    return false;
  }

  if( !save_progress_video() ) {
    CF_ERROR("save_progress_video() fails");
    return false;
  }

  if( !save_cloud3d_ply() ) {
    CF_ERROR("save_cloud3d_ply() fails");
    return false;
  }

  return true;
}

bool c_vlo_pipeline::run_reflectors_detection()
{
  if( !processing_options_.enable_reflectors_detection ) {
    return true; // silently ignore
  }

  if( !processing_options_.enable_auto_threshold && !processing_options_.enable_double_echo_detection ) {
    CF_DEBUG("Reflector detection requires at least one of yen_threshold or double_echo to be set");
    return false;
  }

  bool fOk =
      ::run_reflectors_detection(current_scan_,
          processing_options_.vlo_intensity_channel,
          processing_options_.enable_double_echo_detection,
          processing_options_.enable_auto_threshold,
          processing_options_.auto_threshold_type,
          processing_options_.auto_threshold_value,
          processing_options_.auto_clip_min,
          processing_options_.auto_clip_max,
          &current_reflection_mask_);

  if ( !fOk ) {
    CF_DEBUG("::run_reflectors_detection() fails");
    return false;
  }

  cv::Mat image =
      c_vlo_file::get_image(current_scan_,
          c_vlo_reader::DATA_CHANNEL_DISTANCES);

  if( !image.empty() ) {

    autoclip(image, cv::noArray(),
        0.5, 99.5,
        0, 255);

    if( image.depth() != CV_8U ) {
      image.convertTo(image,
      CV_8U);
    }
  }


  cv::Mat mask;
  cv::cvtColor(current_reflection_mask_, mask, cv::COLOR_GRAY2BGR);
  cv::addWeighted(image, 0.25, mask, 0.75, 0, image);


  if( !reflectors_writer_.is_open() ) {

    std::string output_filename =
        generate_output_filename("",
            "_reflectors",
            ".avi");

    if( !reflectors_writer_.open(output_filename) ) {
      CF_ERROR("reflectors_writer_.open(%s) fails", output_filename.c_str());
      return false;
    }
  }

  if( !reflectors_writer_.write(image, cv::noArray()) ) {
    CF_ERROR("reflectors_writer_.write(%s) fails", reflectors_writer_.filename().c_str());
    return false;
  }


  return true;
}

bool c_vlo_pipeline::run_blom_detection()
{

  if( !processing_options_.enable_blom_detection ) {
    return true; // silently ignore
  }

  static const std::string ffmpeg_options =
      "-r 10 -c rawvideo -pix_fmt bgr24 -f avi";

  cv::Mat3b reflectors_mask;
  cv::Mat1f reflectors_image;
  cv::Mat1w reflectors_distances;
  cv::Mat1f blom_slopes_image;
  cv::Mat1f blom_intensity_image;
  cv::Mat3b blom_mask;
  cv::Mat display_image;

  bool fOK =
      ::run_blom_detection(current_scan_,
          processing_options_.high_intensity_threshold,
          processing_options_.blom_slope_min,
          processing_options_.blom_slope_max,
          processing_options_.walk_error,
          processing_options_.double_echo_distance,
          reflectors_mask,
          reflectors_image,
          reflectors_distances,
          blom_slopes_image,
          blom_intensity_image,
          blom_mask);

  if ( !fOK)  {
      CF_ERROR("::run_bloom_detection() fails");
      return false;
  }

  /////////////////////////////

  if( !blom_reflectors_mask_writer_.is_open() ) {

    std::string output_filename =
        generate_output_filename("blom_reflecors/.avi",
            "reflectors_mask",
            ".avi");

    if( !blom_reflectors_mask_writer_.open(output_filename, ffmpeg_options) ) {
      CF_ERROR("blom_writer_.open(%s) fails", output_filename.c_str());
      return false;
    }
  }

  if( !blom_reflectors_mask_writer_.write(reflectors_mask, cv::noArray()) ) {
    CF_ERROR("blom_writer_.write(%s) fails", blom_reflectors_mask_writer_.filename().c_str());
    return false;
  }

  /////////////////////////////

  if( !blom_reflectors_writer_.is_open() ) {

    std::string output_filename =
        generate_output_filename("blom_reflecors/.ser",
            "blom_reflectors",
            ".ser");

    if( !blom_reflectors_writer_.open(output_filename) ) {
      CF_ERROR("blom_reflectors_writer_.open(%s) fails", output_filename.c_str());
      return false;
    }
  }

  if( !blom_reflectors_writer_.write(reflectors_image, cv::noArray()) ) {
    CF_ERROR("blom_reflectors_writer_.write(%s) fails", blom_reflectors_writer_.filename().c_str());
    return false;
  }

  /////////////////////////////

  if( !blom_distances_writer_.is_open() ) {

    std::string output_filename =
        generate_output_filename("blom_reflecors/.ser",
            "blom_distances",
            ".ser");

    if( !blom_distances_writer_.open(output_filename) ) {
      CF_ERROR("blom_distances_writer_.open(%s) fails", output_filename.c_str());
      return false;
    }
  }


  if( !blom_distances_writer_.write(reflectors_distances, cv::noArray()) ) {
    CF_ERROR("blom_distances_writer_.write(%s) fails", blom_distances_writer_.filename().c_str());
    return false;
  }

  /////////////////////////////

  if( !blom_slopes_writer_.is_open() ) {

    std::string output_filename =
        generate_output_filename("blom_reflecors/.ser",
            "blom_slopes",
            ".ser");

    if( !blom_slopes_writer_.open(output_filename) ) {
      CF_ERROR("blom_slopes_writer_.open(%s) fails", output_filename.c_str());
      return false;
    }
  }

  if( !blom_slopes_writer_.write(blom_slopes_image, cv::noArray()) ) {
    CF_ERROR("blom_slopes_writer_.write(%s) fails", blom_slopes_writer_.filename().c_str());
    return false;
  }

  /////////////////////////////


  if( !blom_intensity_writer_.is_open() ) {

    std::string output_filename =
        generate_output_filename("blom_reflecors/.ser",
            "blom_intensity",
            ".ser");

    if( !blom_intensity_writer_.open(output_filename) ) {
      CF_ERROR("blom_intensity_writer_.open(%s) fails", output_filename.c_str());
      return false;
    }
  }

  if( !blom_intensity_writer_.write(blom_intensity_image, cv::noArray()) ) {
    CF_ERROR("blom_intensity_writer_.write(%s) fails", blom_intensity_writer_.filename().c_str());
    return false;
  }

  /////////////////////////////


  if( !blom_mask_writer_.is_open() ) {

    std::string output_filename =
        generate_output_filename("blom_reflecors/.avi",
            "blom_mask",
            ".avi");

    if( !blom_mask_writer_.open(output_filename, ffmpeg_options) ) {
      CF_ERROR("blom_mask_writer_.open(%s) fails", output_filename.c_str());
      return false;
    }
  }

  if( !blom_mask_writer_.write(blom_mask, cv::noArray()) ) {
    CF_ERROR("blom_mask_writer_.write(%s) fails", blom_mask_writer_.filename().c_str());
    return false;
  }

  /////////////////////////////


  if ( false ) {

    display_image.create(2 * reflectors_mask.rows, reflectors_mask.cols, CV_16UC3);

    c_vlo_file::get_image(current_scan_, c_vlo_file::DATA_CHANNEL_DISTANCES).
        copyTo(display_image(cv::Rect(0, 0, reflectors_mask.cols, reflectors_mask.rows)));

    c_vlo_file::get_image(current_scan_, c_vlo_file::DATA_CHANNEL_DISTANCES, blom_mask).
        copyTo(display_image(cv::Rect(0, reflectors_mask.rows, reflectors_mask.cols, reflectors_mask.rows)));


    cv::line(display_image, cv::Point(0, reflectors_mask.rows),
        cv::Point(reflectors_mask.cols, reflectors_mask.rows),
        cv::Scalar::all(30000),
        1);

  }

  if ( true ) {

    cv::Mat channels[3];

    // single frame size
    const cv::Size size =
        reflectors_mask.size();


    // total display size

    display_image.create(3 * size.height, 2 * size.width, CV_16UC1);

    const cv::Rect roi[6] = {

        cv::Rect(0, 0, size.width, size.height),
        cv::Rect(0, size.height, size.width, size.height),
        cv::Rect(0, 2 * size.height, size.width, size.height),

        cv::Rect(size.width, 0, size.width, size.height),
        cv::Rect(size.width, size.height, size.width, size.height),
        cv::Rect(size.width, 2 * size.height, size.width, size.height),

    };

    cv::split(c_vlo_file::get_image(current_scan_,
        c_vlo_file::DATA_CHANNEL_DISTANCES),
        channels);

    channels[0].copyTo(display_image(roi[0]));
    channels[1].copyTo(display_image(roi[1]));
    channels[2].copyTo(display_image(roi[2]));

    cv::split(c_vlo_file::get_image(current_scan_,
        c_vlo_file::DATA_CHANNEL_DISTANCES,
        blom_mask),
        channels);

    channels[0].copyTo(display_image(roi[3]));
    channels[1].copyTo(display_image(roi[4]));
    channels[2].copyTo(display_image(roi[5]));


    autoclip(display_image, cv::noArray(),
        0.1, 99.9,
        0, 255);

    if( display_image.depth() != CV_8U ) {
      display_image.convertTo(display_image, CV_8U);
    }

    apply_mtf_pixinsight(display_image, display_image,
        0, 255,
        0, 255,
        0, 1,
        0.15);


    cv::cvtColor(display_image, display_image,
        cv::COLOR_GRAY2BGR);


    //
    cv::split(blom_mask, channels);
    for ( int i = 0; i < 3;++i ) {

      display_image(roi[i]).setTo(CV_RGB(255, 255, 0),
          channels[i]);
    }

    //
    cv::split(reflectors_mask, channels);
    for( int i = 0; i < 3; ++i ) {
      display_image(roi[i]).setTo(CV_RGB(245, 64, 41),
          channels[i]);
    }

    for ( int i = 0; i < 3;++i ) {
      cv::putText(display_image(roi[i]),
          ssprintf("Echo%d", i),
          cv::Point(16, 32),
          cv::FONT_HERSHEY_DUPLEX,
          1,
          CV_RGB(238, 198, 31),
          1,
          cv::LINE_AA,
          false);
    }


    //
    for ( int i = 1; i < 3;++i ) {
      cv::line(display_image,
          cv::Point(0, i * size.height),
          cv::Point(display_image.cols, i * size.height),
          cv::Scalar::all(255),
          1);
    }
    cv::line(display_image,
        cv::Point(size.width, 0),
        cv::Point(size.width, display_image.rows),
        cv::Scalar::all(255),
        1);
    //
  }


  if( !blom_display_writer_.is_open() ) {

    std::string output_filename =
        generate_output_filename("blom_reflecors/.avi",
            "blom_display",
            ".avi");

    if( !blom_display_writer_.open(output_filename, ffmpeg_options) ) {
      CF_ERROR("blom_display_writer_.open(%s) fails",
          output_filename.c_str());
      return false;
    }
  }

  if( !blom_display_writer_.write(display_image, cv::noArray()) ) {
    CF_ERROR("blom_display_writer_.write(%s) fails",
        blom_display_writer_.filename().c_str());
    return false;
  }


  /////////////
  // Save clouds
  if ( false ) {

    cv::Mat3f clouds[3];

    std::vector<cv::Vec3d> original_points;
    std::vector<cv::Vec3b>  original_colors;

    std::vector<cv::Vec3d> filtered_points;
    std::vector<cv::Vec3b>  filtered_colors;

    std::string filename;

    c_vlo_file::get_clouds3d(current_scan_, clouds);

    const cv::Vec3b echo_colors[3] = {
        cv::Vec3b(255, 0, 0),
        cv::Vec3b(0, 255, 0),
        cv::Vec3b(0, 0, 255),
    };

    for ( int e = 0; e < 3; ++e ) {

      const cv::Mat3f & cloud =
          clouds[e];

      for ( int y = 0; y < cloud.rows; ++y ) {
        for ( int x = 0; x < cloud.cols; ++x ) {

          const cv::Vec3f & p =
              cloud[y][x];

          if( p[0] || p[1] || p[2] ) {

            original_points.emplace_back(p);
            original_colors.emplace_back(echo_colors[e]);


            if ( !blom_mask[y][x][e] ) {
              filtered_points.emplace_back(p);
              filtered_colors.emplace_back(echo_colors[e]);
            }
          }
        }
      }
    }


    filename =
        ssprintf("%s/bloom-cloud3d/%s/cloud3d.%03d.original.ply",
            output_path_.c_str(),
            input_sequence_->name().c_str(),
            input_sequence_->current_pos() - 1);

    if( !save_ply(original_points, original_colors, filename) ) {
      CF_ERROR("save_ply('%s' fails)", filename.c_str());
      return false;
    }

    filename =
        ssprintf("%s/bloom-cloud3d/%s/cloud3d.%03d.filtered.ply",
            output_path_.c_str(),
            input_sequence_->name().c_str(),
            input_sequence_->current_pos() - 1);

    if( !save_ply(filtered_points, filtered_colors, filename) ) {
      CF_ERROR("save_ply('%s' fails)", filename.c_str());
      return false;
    }

  }


  return true;
}


bool c_vlo_pipeline::run_blom_detection2()
{
  if( !processing_options_.enable_blom_detection2 ) {
    return true; // silently ignore
  }

  cv::Mat3f clouds[3];
  cv::Mat4f histogram;
  cv::Mat3w segments;

  bool fOk =
      c_vlo_file::get_clouds3d(current_scan_,
          clouds);

  if ( !fOk ) {
    CF_ERROR("c_vlo_file::get_clouds3d() fails");
    return false;
  }


  fOk =
      vlo_depth_segmentation(clouds,
          histogram,
          segments,
          processing_options_.depth_segmentation_);

  if( !fOk ) {
    CF_ERROR("vlo_depth_segmentation() fails");
    return false;
  }

  if ( output_options_.save_bloom2_segments ) {

    if( !bloom2_segments_writer_.is_open() ) {

      std::string filename =
          generate_output_filename(output_options_.bloom2_segments_file_options.output_filename,
              "segments",
              ".ser");

      const bool fOk =
          bloom2_segments_writer_.open(filename,
              output_options_.bloom2_segments_file_options.ffmpeg_opts,
              output_options_.bloom2_segments_file_options.output_image_processor,
              output_options_.bloom2_segments_file_options.output_pixel_depth,
              output_options_.bloom2_segments_file_options.save_frame_mapping);

      if( !fOk ) {
        CF_ERROR("bloom2_segments_writer_.open(%s) fails",
            bloom2_segments_writer_.filename().c_str());
        return false;
      }
    }

    if ( !bloom2_segments_writer_.write(segments, cv::noArray()) ) {
      CF_ERROR("bloom2_segments_writer_.write(%s) fails",
          bloom2_segments_writer_.filename().c_str());
      return false;
    }
  }

  if ( output_options_.save_bloom2_intensity_profiles ) {

    cv::Mat3f intensity_image;

    c_vlo_file::get_image(current_scan_,
        c_vlo_file::DATA_CHANNEL_ECHO_PEAK).convertTo(intensity_image,
            CV_32F);

    cv::Mat1f intensity(clouds[0].size(), 0.f);

    const int segment_id = 1;

    for( int y = 0; y < segments.rows; ++y ) {
      for( int x = 0; x < segments.cols; ++x ) {
        for( int e = 0; e < 3; ++e ) {

          if( segments[y][x][e] == segment_id ) {

            intensity[y][x] = intensity_image[y][x][e];

          }
        }

      }
    }


    if( !bloom2_intensity_writer_.is_open() ) {

      std::string filename =
          generate_output_filename(output_options_.bloom2_intensity_profiles_options.output_filename,
              "sintensity",
              ".ser");

      const bool fOk =
          bloom2_intensity_writer_.open(filename,
              output_options_.bloom2_intensity_profiles_options.ffmpeg_opts,
              output_options_.bloom2_intensity_profiles_options.output_image_processor,
              output_options_.bloom2_intensity_profiles_options.output_pixel_depth,
              output_options_.bloom2_intensity_profiles_options.save_frame_mapping);

      if( !fOk ) {
        CF_ERROR("bloom2_intensity_writer_.open(%s) fails",
            bloom2_intensity_writer_.filename().c_str());
        return false;
      }
    }

    if ( !bloom2_intensity_writer_.write(intensity, cv::noArray()) ) {
      CF_ERROR("bloom2_intensity_writer_.write(%s) fails",
          bloom2_segments_writer_.filename().c_str());
      return false;
    }
  }


  if( true ) {
    // segments & counts

    std::vector<cv::Mat1f> histogram_channels;
    cv::split(histogram, histogram_channels);


    const cv::Size display_size =
        cv::Size(std::max(std::max(clouds[0].cols, segments.cols), histogram_channels[0].cols),
            clouds[0].rows + segments.rows + histogram_channels[0].rows);

    const cv::Rect roi[3] = {
        cv::Rect(0, 0, clouds[0].cols, clouds[0].rows), // depths
        cv::Rect(0, clouds[0].rows, segments.cols, segments.rows), // segments
        cv::Rect(0, clouds[0].rows + segments.rows, histogram_channels[0].cols, histogram_channels[0].rows), // counts
    };

    cv::Mat3f display_image(display_size,
        cv::Vec3w::all(0));

    c_vlo_file::get_image(current_scan_,
        c_vlo_file::DATA_CHANNEL_DEPTH).convertTo(display_image(roi[0]),
        display_image.depth());

    segments.convertTo(display_image(roi[1]),
        display_image.depth());

    std::vector<cv::Mat1f> counts_channels( {
        histogram_channels[0],
        histogram_channels[2],
        cv::Mat1f(histogram_channels[0].size(), 0.f)
    });

    cv::merge(counts_channels, display_image(roi[2]));


    if( !blom2_display_writer_.is_open() ) {

      std::string filename =
          generate_output_filename("",
              "blom2_display",
              ".ser");

      if( !blom2_display_writer_.open(filename) ) {
        CF_ERROR("blom2_display_writer_.open(%s) fails",
            filename.c_str());
        return false;
      }
    }

    if( !blom2_display_writer_.write(display_image, cv::noArray()) ) {
      CF_ERROR("blom2_display_writer_.write('%s') fails",
          blom2_display_writer_.filename().c_str());
      return false;
    }
  }

  return true;
}


bool c_vlo_pipeline::update_vlo_lookup_table_statistics()
{
  if ( !processing_options_.enable_gather_lookup_table_statistics ) {
    return true; // silently ignore
  }

  // update here
  if ( !::update_vlo_lookup_table_statistics(current_scan_, vlo_lookup_table_statistics_) ) {
    CF_ERROR("update_vlo_lookup_table_statistics() fails");
    return false;
  }

  // success
  return true;
}


bool c_vlo_pipeline::save_progress_video()
{
  if( !output_options_.save_progress_video ) {
    return true; // silently ignore
  }


  cv::Mat display_image, display_mask;

  if ( !get_display_image(display_image, display_mask) ) {
    CF_ERROR("get_display_image() fails");
    return false;
  }

  if( !progress_writer_.is_open() ) {

    std::string output_filename =
        generate_output_filename(output_options_.progress_video_filename,
            "_progress",
            ".avi");

    if( !progress_writer_.open(output_filename) ) {
      CF_ERROR("progress_writer_.open(%s) fails", output_filename.c_str());
      return false;
    }
  }

  if( !progress_writer_.write(display_image, display_mask) ) {
    CF_ERROR("progress_writer_.write(%s) fails", progress_writer_.filename().c_str());
    return false;
  }

  return true;
}

bool c_vlo_pipeline::save_cloud3d_ply()
{
  if( !output_options_.save_cloud3d_ply) {
    return true; // silently ignore
  }

  std::vector<cv::Point3f> points;
  std::vector<cv::Vec3b> colors;

  bool fOk =
      c_vlo_file::get_cloud3d(current_scan_,
          output_options_.cloud3d_intensity_channel,
          points,
          colors);

  if( !fOk ) {
    CF_ERROR("get_cloud3d() fails");
    return false;
  }

  const std::string filename =
      ssprintf("%s/cloud3d/%s/cloud3d.%03d.ply",
          output_path_.c_str(),
          input_sequence_->name().c_str(),
          input_sequence_->current_pos() - 1);

  if ( !save_ply(points, colors, filename) ) {
    CF_ERROR("save_ply('%s') fails", filename.c_str());
    return false;
  }

  return true;
}

