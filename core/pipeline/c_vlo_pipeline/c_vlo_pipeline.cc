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


template<class ScanType>
static bool get_cloud3d(const ScanType & scan,
    std::vector<cv::Point3f> & output_points,
    std::vector<cv::Vec3b> & output_colors)
{

  cv::Mat ambientImage =
      c_vlo_file::get_image(scan,
          c_vlo_reader::DATA_CHANNEL_AMBIENT);

  if( !ambientImage.empty() ) {
    autoclip(ambientImage, cv::noArray(),
        0.5, 99.5,
        0, 255);
    if( ambientImage.depth() != CV_8U ) {
      ambientImage.convertTo(ambientImage, CV_8U);
    }
  }

  output_points.clear();
  output_colors.clear();
  output_points.reserve(scan.NUM_POINTS);
  output_colors.reserve(scan.NUM_POINTS);

  const float firstVertAngle =
      0.5 * 0.05 * scan.NUM_LAYERS;

  const float tick2deg =
      (float) (0.00000008381903173490870551553291862726);

  const float yawCorrection = 0;

  const cv::Mat1b ambient =
      ambientImage;

  if( false ) {

    static cv::Mat3b lut;
    if( lut.empty() ) {

      cv::Mat1b gray(1, 256);
      for( int i = 0; i < 256; ++i ) {
        gray[0][i] = i;
      }

      apply_colormap(gray,
          lut,
          COLORMAP_TURBO);
    }
  }


  for( int s = 0; s < scan.NUM_SLOTS; ++s ) {

    const auto &slot =
        scan.slot[s];

    const float horizontalAngle =
        slot.angleTicks * tick2deg * CV_PI / 180 + yawCorrection;

    for( int l = 0; l < scan.NUM_LAYERS; ++l ) {

      const float verticalAngle =
          firstVertAngle * CV_PI / 180 - 0.05 * l * CV_PI / 180;

      const float cos_vert = cos(verticalAngle);
      const float sin_vert = sin(verticalAngle);
      const float cos_hor_cos_vert = cos(horizontalAngle) * cos_vert;
      const float sin_hor_cos_vert = sin(horizontalAngle) * cos_vert;

      for( int e = 0; e < 2/*scan.NUM_ECHOS*/; ++e ) {

        const auto &echo =
            slot.echo[l][e];

        const uint16_t distance =
            echo.dist;

        if( distance > 0 && distance < 65534 ) {

          const float scale = 0.01;
          const float x = scale * distance * cos_hor_cos_vert;
          const float y = scale * distance * sin_hor_cos_vert;
          const float z = scale * distance * sin_vert;
          output_points.emplace_back(x, y, z);

          const uint8_t color =
              ambient[l][s];

          output_colors.emplace_back(cv::Vec3b(color, color, color));
        }
      }
    }
  }


  return true;
}

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
  typedef decltype(ScanType::echo::area) area_type;
  constexpr auto max_area_value = std::numeric_limits<area_type>::max() - 2;

  typedef decltype(ScanType::echo::peak) peak_type;
  constexpr auto max_peak_value = std::numeric_limits<peak_type>::max() - 2;

  typedef decltype(ScanType::echo::dist) dist_type;
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
static bool run_reflectors_detection2_(const ScanType & scan,
    double peak_line_a,
    double peak_line_b,
    double peak_line_bias,
    double peak_line_scale,
    cv::Mat1b * output_mask)
{

  static const int lookup_table[] = {
      143 ,
      143 ,
      143 ,
      143 ,
      143 ,
      143 ,
      140 ,
      137 ,
      137 ,
      136 ,
      136 ,
      135 ,
      133 ,
      133 ,
      136 ,
      137 ,
      137 ,
      137 ,
      136 ,
      134 ,
      132 ,
      130 ,
      128 ,
      126 ,
      125 ,
      123 ,
      120 ,
      119 ,
      116 ,
      114 ,
      112 ,
      110 ,
      109 ,
      107 ,
      103 ,
      103 ,
      102 ,
      101 ,
      100 ,
      100 ,
      99  ,
      97  ,
      96  ,
      95  ,
      94  ,
      93  ,
      91  ,
      90  ,
      89  ,
      89  ,
      87  ,
      85  ,
      84  ,
      83  ,
      82  ,
      81  ,
      80  ,
      79  ,
      78  ,
      77  ,
      76  ,
      76  ,
      74  ,
      74  ,
      73  ,
      72  ,
      71  ,
      70  ,
      70  ,
      69  ,
      68  ,
      68  ,
      68  ,
      66  ,
      66  ,
      65  ,
      65  ,
      64  ,
      63  ,
      59  ,
      58  ,
      58  ,
      58  ,
      59  ,
      60  ,
      62  ,
      62  ,
      63  ,
      63  ,
      63  ,
      63  ,
      63  ,
      63  ,
      63  ,
      63  ,
      63  ,
      63  ,
      63  ,
      62  ,
      62  ,
      61  ,
      61  ,
      61  ,
      60  ,
      60  ,
      59  ,
      59  ,
      59  ,
      58  ,
      58  ,
      58  ,
      58  ,
      58  ,
      58  ,
      57  ,
      57  ,
      57  ,
      56  ,
      56  ,
      55  ,
      55  ,
      55  ,
      55  ,
      55  ,
      55  ,
      54  ,
      54  ,
      54  ,
      54  ,
      54  ,
      54  ,
      54  ,
      54  ,
      53  ,
      53  ,
      53  ,
      53  ,
      52  ,
      52  ,
      52  ,
      52  ,
      52  ,
      52  ,
      52  ,
      51  ,
      51  ,
      51  ,
      51  ,
      51  ,
      51  ,
      51  ,
      51  ,
      51  ,
      51  ,
      51  ,
      50  ,
      50  ,
      49  ,
      49  ,
      49  ,
      50  ,
      50  ,
      50  ,
      49  ,
      49  ,
      49  ,
      49  ,
      49  ,
      49  ,
      49  ,
      49  ,
      49  ,
      48  ,
      48  ,
      48  ,
      48  ,
      48  ,
      48  ,
      48  ,
      48  ,
      48  ,
      48  ,
      48  ,
      48  ,
      48  ,
      48  ,
      48  ,
      48  ,
      48  ,
      47  ,
      47  ,
      47  ,
      47  ,
      47  ,
      47  ,
      47  ,
      47  ,
      47  ,
      47  ,
      47  ,
      46  ,
      46  ,
      46  ,
      46  ,
      46  ,
      45  ,
      45  ,
      44  ,
      44  ,
      42  ,
      42  ,
      41  ,
      41  ,
      41  ,
      41  ,
      39  ,
      39  ,
      39  ,
      38  ,
      38  ,
      38  ,
      38  ,
      38  ,
      38  ,
      37  ,
      37  ,
      37  ,
      36  ,
      36  ,
      36  ,
      36  ,
      36  ,
      36  ,
      36  ,
      36  ,
      36  ,
      37  ,
      37  ,
      36  ,
      37  ,
      36  ,
      36  ,
      36  ,
      37  ,
      37  ,
      37  ,
      37  ,
      37  ,
      37  ,
      37  ,
      37  ,
      37  ,
      37  ,
      37  ,
      38  ,
      38  ,
      39  ,
      39  ,
      39  ,
      39  ,
      39  ,
      39  ,
      38  ,
      38  ,
      38  ,
      38  ,
      38  ,
      38  ,
      38  ,
      38  ,
      39  ,
      39  ,
      39  ,
      39  ,
      39  ,
      40  ,
      40  ,
      40  ,
      40  ,
      40  ,
      40  ,
      40  ,
      40  ,
      40  ,
      40  ,
      40  ,
      40  ,
      40  ,
      40  ,
      40  ,
      40  ,
      40  ,
      41  ,
      41  ,
      41  ,
      41  ,
      41  ,
      41  ,
      41  ,
      41  ,
      41  ,
      41  ,
      41  ,
      41  ,
      41  ,
      41  ,
  };


  typedef typename ScanType::echo echo_type;

  typedef decltype(ScanType::echo::area) area_type;
  constexpr auto max_area_value = std::numeric_limits<area_type>::max() - 2;

  typedef decltype(ScanType::echo::peak) peak_type;
  constexpr auto max_peak_value = std::numeric_limits<peak_type>::max() - 2;

  typedef decltype(ScanType::echo::dist) dist_type;
  constexpr auto max_dist_value = std::numeric_limits<dist_type>::max() - 2;

  output_mask->create(scan.NUM_LAYERS, scan.NUM_SLOTS);
  output_mask->setTo(0);

  for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
    for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
      for( int e = 0; e < 1/*scan.NUM_ECHOS*/; ++e ) {

        const auto & echo =
            scan.slot[s].echo[l][e];

        if( echo.dist <= 0 || echo.dist > max_dist_value ) {
          continue;
        }

        if( echo.peak <= 0 || echo.peak > max_peak_value ) {
          continue;
        }

        // all data are valid
        const int num_bins = sizeof(lookup_table)/sizeof(lookup_table[0]);
        const double distance = echo.dist * 0.01;
        const int peak = echo.peak;

        const int threshold =
            (int) (lookup_table[std::min(num_bins - 1, (int) distance)] * peak_line_scale + peak_line_bias);

        //const double L = (peak_line_a / sqrt(distance) + peak_line_b) * peak_line_scale + peak_line_bias;
        //const bool is_reflector = peak > L;

        const bool is_reflector = peak > threshold;

        if ( is_reflector ) {
          (*output_mask)[l][s] = 255;
        }
      }
    }
  }

  return true;
}



static bool run_reflectors_detection2(const c_vlo_scan & scan,
    double peak_line_a,
    double peak_line_b,
    double peak_line_bias,
    double peak_line_scale,
    cv::Mat1b * output_mask)
{
  switch (scan.version) {
    case VLO_VERSION_1:
      return run_reflectors_detection2_(scan.scan1, peak_line_a, peak_line_b, peak_line_bias, peak_line_scale, output_mask);
    case VLO_VERSION_3:
      return run_reflectors_detection2_(scan.scan3, peak_line_a, peak_line_b, peak_line_bias, peak_line_scale, output_mask);
    case VLO_VERSION_5:
      return run_reflectors_detection2_(scan.scan5, peak_line_a, peak_line_b, peak_line_bias, peak_line_scale, output_mask);
  }
  CF_DEBUG("Unsupported scan version %d specified", scan.version);
  return false;
}

template<class ScanType>
static bool update_vlo_lookup_table_statistics_(const ScanType & scan,
    c_vlo_lookup_table_statistics & statistics)
{

  typedef typename ScanType::echo echo_type;

  typedef decltype(ScanType::echo::area) area_type;
  constexpr auto max_area_value = std::numeric_limits<area_type>::max() - 2;

  typedef decltype(ScanType::echo::peak) peak_type;
  constexpr auto max_peak_value = std::numeric_limits<peak_type>::max() - 2;

  typedef decltype(ScanType::echo::dist) dist_type;
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

    SERIALIZE_OPTION(section, save, processing_options_, enable_reflectors_detection2);
    SERIALIZE_OPTION(section, save, processing_options_, peak_line_a);
    SERIALIZE_OPTION(section, save, processing_options_, peak_line_b);
    SERIALIZE_OPTION(section, save, processing_options_, peak_line_bias);
    SERIALIZE_OPTION(section, save, processing_options_, peak_line_scale);

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

    PIPELINE_CTL_GROUP(ctrls, "Reflectors Detection2", "");
      PIPELINE_CTL(ctrls, processing_options_.enable_reflectors_detection2, "enable_reflectors_detection2", "");
      PIPELINE_CTLC(ctrls, processing_options_.peak_line_a, "peak_line_a", "",
          _this->processing_options_.enable_reflectors_detection2);
      PIPELINE_CTLC(ctrls, processing_options_.peak_line_b, "peak_line_b", "",
          _this->processing_options_.enable_reflectors_detection2);
      PIPELINE_CTLC(ctrls, processing_options_.peak_line_bias, "peak_line_bias", "",
          _this->processing_options_.enable_reflectors_detection2);
      PIPELINE_CTLC(ctrls, processing_options_.peak_line_scale, "peak_line_scale", "",
          _this->processing_options_.enable_reflectors_detection2);
    PIPELINE_CTL_END_GROUP(ctrls);


    PIPELINE_CTL(ctrls, processing_options_.vlo_intensity_channel, "vlo_intensity_channel", "");

    PIPELINE_CTL(ctrls, processing_options_.enable_gather_lookup_table_statistics, "enable_gather_lookup_table_statistics", "");
    PIPELINE_CTLC(ctrls, processing_options_.vlo_lookup_table_statistics_filename, "vlo_statistics_filename", "",
        _this->processing_options_.enable_gather_lookup_table_statistics);

    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Output options", "");
      // PIPELINE_CTL_BROWSE_FOR_DIRECTORY(ctrls, output_options_.output_directory, "output_directory", "");

      PIPELINE_CTL(ctrls, output_options_.save_progress_video, "save_progress_video", "");
      PIPELINE_CTL(ctrls, output_options_.progress_video_filename, "progress_video_filename", "");

      PIPELINE_CTL(ctrls, output_options_.save_cloud3d_ply, "save cloud3d ply", "");
      PIPELINE_CTL(ctrls, output_options_.cloud3d_filename, "cloud3d_filename", "");

    PIPELINE_CTL_END_GROUP(ctrls);
  }

  return ctrls;
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

//  if ( output_options_.save_processed_frames ) {
//    output_file_name_ =
//        generate_output_filename(output_options_.processed_frames_filename,
//            "processed",
//            ".avi");
//  }

  const int num_sources =
      input_sequence_->sources().size();

  for( int i = 0; i < num_sources; ++i ) {

    const c_input_source::sptr &source =
        input_sequence_->source(i);

    c_vlo_input_source *vlo =
        dynamic_cast<c_vlo_input_source*>(source.get());

    if( !vlo ) {
      CF_ERROR("ERROR: input source %d is not a VLO source", i);
      return false;
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

  if ( reflectors2_writer_.is_open() ) {
    CF_DEBUG("Closing '%s'", reflectors2_writer_.filename().c_str());
    reflectors2_writer_.close();
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

    const int read_pos =
        input_sequence_->current_pos() + 1;

    if( !input_sequence_->seek(read_pos) ) {
      CF_ERROR("input_sequence_->seek(read_pos=%d) fails: %s",
          read_pos, strerror(errno));
      return false;
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

  if ( !run_reflectors_detection2() ) {
    CF_ERROR("run_reflectors_detection2() fails");
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

    bool fOk =
        reflectors_writer_.open(output_filename);

    if( !fOk ) {
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

bool c_vlo_pipeline::run_reflectors_detection2()
{
  if( !processing_options_.enable_reflectors_detection2 ) {
    return true; // silently ignore
  }

  bool fOk =
      ::run_reflectors_detection2(current_scan_,
          processing_options_.peak_line_a,
          processing_options_.peak_line_b,
          processing_options_.peak_line_bias,
          processing_options_.peak_line_scale,
          &current_reflection2_mask_);

  if ( !fOk ) {
    CF_DEBUG("::run_reflectors_detection2() fails");
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
  cv::cvtColor(current_reflection2_mask_, mask, cv::COLOR_GRAY2BGR);
  cv::addWeighted(image, 0.3, mask, 0.7, 0, image);


  if( !reflectors2_writer_.is_open() ) {

    std::string output_filename =
        generate_output_filename("masks2/masks2.avi",
            "_reflectors2",
            ".masks2.avi");

    bool fOk =
        reflectors2_writer_.open(output_filename);

    if( !fOk ) {
      CF_ERROR("reflectors2_writer_.open(%s) fails", output_filename.c_str());
      return false;
    }
  }

  if( !reflectors2_writer_.write(image, cv::noArray()) ) {
    CF_ERROR("reflectors2_writer_.write(%s) fails", reflectors2_writer_.filename().c_str());
    return false;
  }

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

    bool fOk =
        progress_writer_.open(output_filename);

    if( !fOk ) {
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

  std::vector<cv::Point3f> cloud3d;
  std::vector<cv::Vec3b> colors;

  switch (current_scan_.version) {
    case VLO_VERSION_1:
      if( !get_cloud3d(current_scan_.scan1, cloud3d, colors) ) {
        CF_ERROR("get_cloud3d(scan1) fails");
      }
      break;
    case VLO_VERSION_3:
      if( !get_cloud3d(current_scan_.scan3, cloud3d, colors) ) {
        CF_ERROR("get_cloud3d(scan1) fails");
      }
      break;
    case VLO_VERSION_5:
      if( !get_cloud3d(current_scan_.scan5, cloud3d, colors) ) {
        CF_ERROR("get_cloud3d(scan1) fails");
      }
      break;
  }

  const std::string filename =
      ssprintf("%s/cloud3d/%s/cloud3d.%03d.ply",
          output_path_.c_str(),
          input_sequence_->name().c_str(),
          input_sequence_->current_pos() - 1);

  if ( !save_ply(cloud3d, colors, filename) ) {
    CF_ERROR("save_ply('%s') fails", filename.c_str());
    return false;
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
