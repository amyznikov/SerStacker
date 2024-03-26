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


/**
 * Sort VLO echoes by distance appropriate for scans of type 1, 3, and 5
 * */
template<class ScanType>
std::enable_if_t<(c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_1 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_3 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_5),
void> sort_vlo_echos_by_distance(ScanType & scan)
{
  INSTRUMENT_REGION("");
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
void sort_vlo_echos_by_distance(c_vlo_scan6_slim_imx479 & scan)
{
  INSTRUMENT_REGION("");

  typedef c_vlo_scan6_slim_imx479 ScanType;

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
  INSTRUMENT_REGION("");

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



/**
 * Sort VLO echoes by distance appropriate for scans of type SLIM
 * */
void sort_vlo_echos_by_distance(c_vlo_scan_cruise & scan)
{
  INSTRUMENT_REGION("");

  // force if ( scan.config.echoOrdering != VLO_ECHO_ORDER_DISTANCE_NEAR_TO_FAR )
  {

    typedef c_vlo_scan_cruise ScanType;
    typedef struct ScanType::Echo echo_type;
    typedef decltype(ScanType::Echo::dist) dist_type;

    constexpr auto min_distance = 0;//scan.MIN_DISTANCE;
    constexpr auto max_distance = 60000;// 65535 - 2;//scan.MAX_DISTANCE;
    constexpr auto min_area = 0;//scan.MIN_AREA;
    constexpr auto max_area = 30000; // 65535-2;// scan.MAX_AREA;

    echo_type echos[scan.NUM_ECHOS];
    int cne = 0; // count non-empty echos

    for( int s = 0; s < scan.NUM_SLOTS; ++s ) {

      auto & slot =
          scan.slot[s];

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {

        cne = 0;
        memset(echos, 0, sizeof(echos));

        for( int e = 0; e < scan.NUM_ECHOS; ++e ) {

          const auto & distance  =
              slot.echo[l][e].dist;

          if ( distance >= min_distance && distance <= max_distance ) {

            const auto & area =
                slot.echo[l][e].area;

            if ( area > 0 && area < max_area ) { // area >= min_area && area <= max_area
              echos[cne++] = slot.echo[l][e];
            }
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

    // scan.config.echoOrdering = VLO_ECHO_ORDER_DISTANCE_NEAR_TO_FAR;
  }
}


/**
 * Sort VLO echoes by distance appropriate for scans of type 1, 3, and 5
 * || c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_CRUISE
 * */
template<class ScanType>
std::enable_if_t<(c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_1 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_3 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_5 ),
void> convert(ScanType & src, c_vlo_scan * dst)
{
  INSTRUMENT_REGION("");

  sort_vlo_echos_by_distance(src);

  dst->size.width = src.NUM_SLOTS;
  dst->size.height = src.NUM_LAYERS;

  dst->ambient.create(dst->size);
  dst->distances.create(dst->size);
  dst->area.create(dst->size);
  dst->peak.create(dst->size);
  dst->width.create(dst->size);
  dst->azimuth.create(src.NUM_SLOTS, 1);
  dst->elevation.create(src.NUM_LAYERS, 1);

  for ( int e = 0; e < 3; ++e ) {
    dst->clouds[e].create(dst->size);
  }

  constexpr double tick2radian = 0.00000008381903173490870551553291862726 * CV_PI / 180;
  constexpr double firstVertAngle = 0.5 * 0.05 * src.NUM_LAYERS;
  constexpr double yawCorrection = 0;

  for( int s = 0; s < src.NUM_SLOTS; ++s ) {

    const int ss =
        src.NUM_SLOTS - s - 1;

    dst->azimuth[s][0] =
        (float) (src.slot[s].angleTicks * tick2radian + yawCorrection);
  }

  for( int l = 0; l < src.NUM_LAYERS; ++l ) {
    dst->elevation[l][0] =
        (float)(firstVertAngle * CV_PI / 180 - 0.05 * l * CV_PI / 180);;
  }


  for( int s = 0; s < src.NUM_SLOTS; ++s ) {

    const auto &slot =
        src.slot[s];

    const double horizontalAngle =
        slot.angleTicks * tick2radian + yawCorrection;

    const int ss =
        src.NUM_SLOTS - s - 1;

    for( int l = 0; l < src.NUM_LAYERS; ++l ) {

      const double verticalAngle =
          firstVertAngle * CV_PI / 180 - 0.05 * l * CV_PI / 180;

      const double cos_vert =
          cos(verticalAngle);

      const double sin_vert =
          sin(verticalAngle);

      const double cos_hor_cos_vert =
          cos(horizontalAngle) * cos_vert;

      const double sin_hor_cos_vert =
          sin(horizontalAngle) * cos_vert;

      dst->ambient[l][ss] =
          slot.ambient[l] < 65535 - 2 ? slot.ambient[l] : 0;

      for( int e = 0; e < std::min(3, (int) src.NUM_ECHOS); ++e ) {

        const auto & echo =
            slot.echo[l][e];

        dst->distances[l][ss][e] = echo.dist;
        dst->area[l][ss][e] = echo.area ;
        dst->peak[l][ss][e] = echo.peak;
        dst->width[l][ss][e] = echo.width;

        const auto & distance = echo.dist;
        if( !distance ) {
          dst->clouds[e][l][ss][0] = 0;
          dst->clouds[e][l][ss][1] = 0;
          dst->clouds[e][l][ss][2] = 0;
        }
        else {
          const float x = distance * cos_hor_cos_vert;
          const float y = -distance * sin_hor_cos_vert;
          const float z = distance * sin_vert;

          dst->clouds[e][l][ss][0] = x;
          dst->clouds[e][l][ss][1] = y;
          dst->clouds[e][l][ss][2] = z;
        }
      }
    }
  }
}

static void convert(c_vlo_scan_cruise & src, c_vlo_scan * dst)
{
  INSTRUMENT_REGION("");

  sort_vlo_echos_by_distance(src);


  dst->size.width = src.NUM_SLOTS;
  dst->size.height = src.NUM_LAYERS;

  dst->ambient.create(dst->size);
  dst->distances.create(dst->size);
  dst->area.create(dst->size);
  dst->peak.create(dst->size);
  dst->width.create(dst->size);
  dst->azimuth.create(src.NUM_SLOTS, 1);
  dst->elevation.create(src.NUM_LAYERS, 1);

  for ( int e = 0; e < 3; ++e ) {
    dst->clouds[e].create(dst->size);
  }

  for( int s = 0; s < src.NUM_SLOTS; ++s ) {

    const int ss =
        src.NUM_SLOTS - s - 1;

    dst->azimuth[ss][0] =
        (float) (((src.slot[s].angleTicks) * 0.000000083819031734908705515532918627265) * 0.0174533);
  }

  for( int l = 0; l < src.NUM_LAYERS; ++l ) {
    dst->elevation[l][0] =
        (float) (((25.0 * l / src.NUM_LAYERS - 25.0 / 2.0) + 90) * 0.0174533);
  }

  for( int s = 0; s < src.NUM_SLOTS; ++s ) {

    const auto &slot =
        src.slot[s];

    const double azimuth =
        ((slot.angleTicks) * 0.000000083819031734908705515532918627265) * 0.0174533;

    const int ss =
        src.NUM_SLOTS - s - 1;

    for( int l = 0; l < src.NUM_LAYERS; ++l ) {

      const double inclination =
          ((25.0 / src.NUM_LAYERS * l - 25.0 / 2.0) + 90) * 0.0174533;

      dst->ambient[l][ss] =
          slot.ambient[l] < 65535 - 2 ? slot.ambient[l] : 0;

      for( int e = 0; e < std::min(3, (int) src.NUM_ECHOS); ++e ) {

        const auto & echo =
            slot.echo[l][e];


        dst->distances[l][ss][e] = echo.dist;
        dst->area[l][ss][e] = echo.area ;
        dst->peak[l][ss][e] = echo.peak;
        dst->width[l][ss][e] = echo.width;

        const auto & distance = echo.dist;
        if( !distance ) {
          dst->clouds[e][l][ss][0] = 0;
          dst->clouds[e][l][ss][1] = 0;
          dst->clouds[e][l][ss][2] = 0;
        }
        else {
          const float x = distance * sin(inclination) * cos(azimuth);
          const float y = -distance * sin(inclination) * sin(azimuth);
          const float z = distance * cos(inclination);

          dst->clouds[e][l][ss][0] = x;
          dst->clouds[e][l][ss][1] = y;
          dst->clouds[e][l][ss][2] = z;
        }
      }
    }
  }

}

static void convert(c_vlo_scan6_slm & src, c_vlo_scan * dst)
{
  INSTRUMENT_REGION("");

  sort_vlo_echos_by_distance(src);

  dst->size.width = src.NUM_SLOTS;
  dst->size.height = src.NUM_LAYERS;

  dst->ambient.release();
  dst->distances.create(dst->size);
  dst->area.create(dst->size);
  dst->peak.release();
  dst->width.release();
  dst->azimuth.create(src.NUM_SLOTS, 1);
  dst->elevation.create(src.NUM_LAYERS, 1);

  for ( int e = 0; e < 3; ++e ) {
    dst->clouds[e].create(dst->size);
  }

  for( int s = 0; s < src.NUM_SLOTS; ++s ) {

    const int ss =
        src.NUM_SLOTS - s - 1;

    dst->azimuth[ss][0] =
        src.horizontalAngles[s];
  }

  for( int l = 0; l < src.NUM_LAYERS; ++l ) {

    const int ll =
        src.NUM_LAYERS - l - 1;

    dst->elevation[ll][0] =
        (float) (CV_PI / 2 - src.verticalAngles[l]);
  }

  for( int s = 0; s < src.NUM_SLOTS; ++s ) {

    const double sin_azimuth =
        sin(src.horizontalAngles[s]);

    const double cos_azimuth =
        cos(src.horizontalAngles[s]);

    const int ss =
        src.NUM_SLOTS - s - 1;

    for( int l = 0; l < src.NUM_LAYERS; ++l ) {

      const double inclination =
          CV_PI / 2 - src.verticalAngles[l];

      const double sin_inclination =
          sin(inclination);

      const double cos_inclination =
          cos(inclination);

      const int ll =
          src.NUM_LAYERS - l - 1;

      for( int e = 0; e < std::min(3, (int) src.NUM_ECHOS); ++e ) {

        const auto & echo =
            src.echo[s][l][e];

        dst->distances[ll][ss][e] = echo.dist;
        dst->area[ll][ss][e] = echo.area;

        const auto & distance = echo.dist;
        if ( !distance ) {
          dst->clouds[e][ll][ss][0] = 0;
          dst->clouds[e][ll][ss][1] = 0;
          dst->clouds[e][ll][ss][2] = 0;
        }
        else {

          const float x = distance * sin_inclination * cos_azimuth;
          const float y = -distance * sin_inclination * sin_azimuth;
          const float z = -distance * cos_inclination;

          dst->clouds[e][ll][ss][0] = x;
          dst->clouds[e][ll][ss][1] = y;
          dst->clouds[e][ll][ss][2] = z;
        }
      }
    }
  }
}

static void convert(c_vlo_scan6_slim_imx479 & src, c_vlo_scan * dst)
{
  INSTRUMENT_REGION("");

  sort_vlo_echos_by_distance(src);

  constexpr uint16_t NUM_SLOTS = src.NUM_SLOTS;
  constexpr uint16_t NUM_LAYERS = src.NUM_LAYERS;
  constexpr uint16_t NUM_ECHOS = src.NUM_ECHOS;

  dst->size.width = NUM_SLOTS;
  dst->size.height = NUM_LAYERS;

  dst->ambient.create(dst->size);
  dst->distances.create(dst->size);
  dst->area.create(dst->size);
  dst->peak.create(dst->size);
  dst->width.create(dst->size);
  dst->azimuth.create(src.NUM_SLOTS, 1);
  dst->elevation.create(src.NUM_LAYERS, 1);

  for ( int e = 0; e < 3; ++e ) {
    dst->clouds[e].create(dst->size);
  }


  float verticalAngleSin[NUM_LAYERS];
  float verticalAngleCos[NUM_LAYERS];
  float horizontalAngleSin[NUM_SLOTS];
  float horizontalAngleCos[NUM_SLOTS];

  for( uint16_t s = 0; s < NUM_SLOTS; ++s ) {
    dst->azimuth[s][0] = src.horizontalAngles[s];
    horizontalAngleSin[s] = std::sin(src.horizontalAngles[s]);
    horizontalAngleCos[s] = std::cos(src.horizontalAngles[s]);
  }

  for( uint16_t l = 0; l < NUM_LAYERS; ++l ) {
    dst->elevation[NUM_LAYERS - l - 1][0] = src.verticalAngles[l];
    verticalAngleSin[l] = std::sin(src.verticalAngles[l]);
    verticalAngleCos[l] = std::cos(src.verticalAngles[l]);
  }

  cv::Matx33f projH;
  cv::Matx33f rotMid;
  cv::Matx33f projV;
  cv::Matx33f rotTransform;
  cv::Vec3f radialVec;
  cv::Vec3f result;

  for( uint16_t e = 0U; e < NUM_ECHOS; ++e ) {

    for( uint16_t s = 0; s < NUM_SLOTS; ++s ) {

      projH = cv::Matx33f::eye();
      projH(0U, 0U) = horizontalAngleCos[s];
      projH(0U, 1U) = -horizontalAngleSin[s];
      projH(1U, 0U) = horizontalAngleSin[s];
      projH(1U, 1U) = horizontalAngleCos[s];
      rotMid = projH;

      for( uint16_t l = 0; l < NUM_LAYERS; ++l ) {

        const uint16_t ll =
            NUM_LAYERS - l - 1;

        const auto & echo =
            src.echo[s][l][e];

        const auto & distance =
            echo.dist;

        if( !distance ) {
          dst->ambient[ll][s] = 0;
          dst->distances[ll][s][e] = 0;
          dst->area[ll][s][e] = 0;
          dst->peak[ll][s][e] = 0;
          dst->width[ll][s][e] = 0;

          dst->clouds[e][ll][s][0] = 0;
          dst->clouds[e][ll][s][1] = 0;
          dst->clouds[e][ll][s][2] = 0;
        }
        else {

          projV = cv::Matx33f::eye();
          projV(0U, 0U) = verticalAngleCos[l];
          projV(0U, 2U) = -verticalAngleSin[l];
          projV(2U, 0U) = verticalAngleSin[l];
          projV(2U, 2U) = verticalAngleCos[l];

          radialVec(0U) = distance;
          radialVec(1U) = 0.F;
          radialVec(2U) = 0.F;

          rotTransform = rotMid * projV;

          result = rotTransform * radialVec;


          dst->ambient[ll][s] = src.ambientLight[s][l];
          dst->distances[ll][s][e] = distance;
          dst->area[ll][s][e] = echo.area;
          dst->peak[ll][s][e] = echo.peak;
          dst->width[ll][s][e] = echo.width;

          dst->clouds[e][ll][s][0] = result(0);
          dst->clouds[e][ll][s][1] = result(1);
          dst->clouds[e][ll][s][2] = result(2);
        }
      }
    }
  }

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


cv::Mat c_vlo_file::get_thumbnail_image(const c_vlo_scan & scan)
{
  return get_vlo_image(scan, VLO_DATA_CHANNEL_AMBIENT);
}

//////////////////////////////////////////

c_vlo_reader::c_vlo_reader() :
    this_class("")
{
}

c_vlo_reader::c_vlo_reader(const std::string & filename) :
    base(filename), u_(new U())
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
  if ( !ifhd_.open(filename) ) {
    // CF_ERROR("c_vlo_reader: ifhd_.open('%s') fails", filename.c_str());
  }
  else if( ( ifhd_.select_stream("ScaLa 3-PointCloud") ) ) {

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
//            "ScaLa 3-PointCloud\n"
//            "format_version = %u\n"
//            "payload_size=%zu \n"
//            "sizeof(c_vlo_scan6_slm)=%zu\n"
//            "sizeof(c_vlo_scan_6_slim_imx479)=%zu\n"
//            "sizeof(c_vlo_scan5)=%zu\n"
//            "sizeof(c_vlo_scan3)=%zu\n"
//            "sizeof(c_vlo_scan1)=%zu\n"
//            "sizeof(c_vlo_scan_cruise)=%zu\n"
//            "\n",
//            format_version,
//            payload_size,
//            sizeof(c_vlo_scan6_slm),
//            sizeof(c_vlo_scan6_slim_imx479),
//            sizeof(c_vlo_scan5),
//            sizeof(c_vlo_scan3),
//            sizeof(c_vlo_scan1),
//            sizeof(c_vlo_scan_cruise)
//            );

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
      else if( format_version == 6 && payload_size == sizeof(c_vlo_scan6_slim_imx479) ) {
        version_ = VLO_VERSION_6_SLM_IMX479;
        frame_size_ = sizeof(c_vlo_scan6_slim_imx479);
      }
//      else if (format_version == 6/*VLO_VERSION_CRUISE*/ ) {
//        version_ = VLO_VERSION_CRUISE;
//        frame_size_ = sizeof(c_vlo_scan_cruise);
//      }

      CF_DEBUG("format_version: %d -> %d frame_size_ = %zd",
          format_version, version_, frame_size_);

      if( frame_size_ > 0 ) {
        ifhd_.seek(0);
        num_frames_ = ifhd_.num_frames();
        fOk = true;
      }
    }
  }
  else if ( ifhd_.select_stream("ScaLa 3 Cruise-PointCloud") ) {

    // CF_DEBUG("ScaLa 3 Cruise-PointCloud");

    const size_t payload_size =
        ifhd_.current_payload_size();

    const size_t cruise_size =
        sizeof(c_vlo_scan_cruise);

    if ( payload_size != cruise_size ) {
      CF_ERROR("Bad cruise payload size: %zu bytes. Expected %zu bytes",
          payload_size, cruise_size);
    }
    else {
      version_ = VLO_VERSION_CRUISE;
      frame_size_ = cruise_size;
      num_frames_ = ifhd_.num_frames();
      CF_DEBUG("Cruise: payload_size=%zu/%zu num_frames_=%zd", payload_size, cruise_size, num_frames_);
      fOk = true;
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
  INSTRUMENT_REGION("");

  bool fOk = false;

  switch (scan->version = this->version_) {
    case VLO_VERSION_1:
      if( (fOk = read(&u_->scan1)) ) {
        convert(u_->scan1, scan);
      }
      break;
    case VLO_VERSION_3:
      if( (fOk = read(&u_->scan3)) ) {
        convert(u_->scan3, scan);
      }
      break;
    case VLO_VERSION_5:
      if( (fOk = read(&u_->scan5)) ) {
        convert(u_->scan5, scan);
      }
      break;
    case VLO_VERSION_6_SLM:
      if( (fOk = read(&u_->scan6_slm)) ) {
        convert(u_->scan6_slm, scan);
      }
      break;
    case VLO_VERSION_CRUISE:
      if( (fOk = read(&u_->cruise)) ) {
        convert(u_->cruise, scan);
      }
      break;
    case VLO_VERSION_6_SLM_IMX479:
      if( (fOk = read(&u_->slim_imx479)) ) {
        convert(u_->slim_imx479, scan);
      }
      break;
    default:
      errno = EBADF;
      return false;
  }

  return fOk;
}

