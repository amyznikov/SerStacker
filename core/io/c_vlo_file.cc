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
//#include <core/proc/bswap.h>
#include <core/proc/autoclip.h>
#include <core/debug.h>

template<>
const c_enum_member* members_of<c_vlo_file::DATA_CHANNEL>()
{
  static constexpr c_enum_member members[] = {
      { c_vlo_file::DATA_CHANNEL_AMBIENT, "AMBIENT", "" },
      { c_vlo_file::DATA_CHANNEL_DISTANCES, "DISTANCES", "" },
      { c_vlo_file::DATA_CHANNEL_ECHO_AREA, "ECHO_AREA", "" },
      { c_vlo_file::DATA_CHANNEL_ECHO_PEAK, "ECHO_PEAK", "" },
      { c_vlo_file::DATA_CHANNEL_ECHO_WIDTH, "ECHO_WIDTH", "" },
      { c_vlo_file::DATA_CHANNEL_ECHO_AREA_DIV_WIDTH, "AREA_DIV_WIDTH", "" },
      { c_vlo_file::DATA_CHANNEL_ECHO_PEAK_DIV_WIDTH, "PEAK_DIV_WIDTH", "" },
      { c_vlo_file::DATA_CHANNEL_ECHO_AREA_MUL_DIST, "AREA_MUL_DIST", "" },
      { c_vlo_file::DATA_CHANNEL_ECHO_PEAK_MUL_DIST, "PEAK_MUL_DIST", "" },
      { c_vlo_file::DATA_CHANNEL_ECHO_AREA_MUL_DIST2, "AREA_MUL_DIST2", "" },
      { c_vlo_file::DATA_CHANNEL_ECHO_PEAK_MUL_DIST2, "PEAK_MUL_DIST2", "" },
      { c_vlo_file::DATA_CHANNEL_ECHO_AREA_MUL_SQRT_DIST, "AREA_MUL_SQRT_DIST", "" },
      { c_vlo_file::DATA_CHANNEL_ECHO_PEAK_MUL_SQRT_DIST, "PEAK_MUL_SQRT_DIST", "" },
      { c_vlo_file::DATA_CHANNEL_DOUBLED_ECHO_PEAKS, "DOUBLED_ECHO_PEAKS", "" },
      { c_vlo_file::DATA_CHANNEL_DIST_TO_MAX_PEAK, "DIST_TO_MAX_PEAK", "" },
      { c_vlo_file::DATA_CHANNEL_AMBIENT },
  };

  return members;
}

namespace {

//@brief get current file position
static inline ssize_t whence(int fd)
{
  return ::lseek64(fd, 0, SEEK_CUR);
}

static inline bool readfrom(int fd, ssize_t offset, uint16_t * data)
{
  if( ::lseek64(fd, offset, SEEK_SET) != offset ) {
    return false;
  }

  if( ::read(fd, data, sizeof(*data)) != sizeof(*data) ) {
    return false;
  }

  return true;
}


static void sort_echos_by_distance(c_vlo_scan1 & scan)
{
  typedef c_vlo_scan1 ScanType;

  if ( scan.config.echoOrdering != VLO_ECHO_ORDER_DISTANCE_NEAR_TO_FAR ) {

    typedef struct ScanType::Echo echo_type;
    typedef decltype(echo_type::dist) dist_type;
    constexpr auto max_dist_value = std::numeric_limits<dist_type>::max() - 2;

    echo_type echos[scan.NUM_ECHOS];
    int cnz = 0;

    for( int s = 0; s < scan.NUM_SLOTS; ++s ) {

      auto &slot = scan.slot[s];

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {

        cnz = 0;
        memset(echos, 0, sizeof(echos));

        for( int e = 0; e < scan.NUM_ECHOS; ++e ) {
          if( slot.echo[l][e].dist > 0 && slot.echo[l][e].dist < max_dist_value ) {
            echos[cnz++] = slot.echo[l][e];
          }
        }

        if( cnz > 1 ) {

          std::sort(echos, echos + cnz,
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

static void sort_echos_by_distance(c_vlo_scan3 & scan)
{
  typedef c_vlo_scan3 ScanType;

  if ( scan.config.echoOrdering != VLO_ECHO_ORDER_DISTANCE_NEAR_TO_FAR ) {

    typedef struct ScanType::Echo echo_type;
    typedef decltype(echo_type::dist) dist_type;
    constexpr auto max_dist_value = std::numeric_limits<dist_type>::max() - 2;

    echo_type echos[scan.NUM_ECHOS];
    int cnz = 0;

    for( int s = 0; s < scan.NUM_SLOTS; ++s ) {

      auto &slot = scan.slot[s];

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {

        cnz = 0;
        memset(echos, 0, sizeof(echos));

        for( int e = 0; e < scan.NUM_ECHOS; ++e ) {
          if( slot.echo[l][e].dist > 0 && slot.echo[l][e].dist < max_dist_value ) {
            echos[cnz++] = slot.echo[l][e];
          }
        }

        if( cnz > 1 ) {

          std::sort(echos, echos + cnz,
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


static void sort_echos_by_distance(c_vlo_scan5 & scan)
{
  typedef c_vlo_scan5 ScanType;

  if ( scan.config.echoOrdering != VLO_ECHO_ORDER_DISTANCE_NEAR_TO_FAR ) {

    typedef struct ScanType::Echo echo_type;
    typedef decltype(echo_type::dist) dist_type;
    constexpr auto max_dist_value = std::numeric_limits<dist_type>::max() - 2;

    echo_type echos[scan.NUM_ECHOS];
    int cnz = 0;

    for( int s = 0; s < scan.NUM_SLOTS; ++s ) {

      auto &slot = scan.slot[s];

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {

        cnz = 0;
        memset(echos, 0, sizeof(echos));

        for( int e = 0; e < scan.NUM_ECHOS; ++e ) {
          if( slot.echo[l][e].dist > 0 && slot.echo[l][e].dist < max_dist_value ) {
            echos[cnz++] = slot.echo[l][e];
          }
        }

        if( cnz > 1 ) {

          std::sort(echos, echos + cnz,
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

static void sort_echos_by_distance(c_vlo_scan6_base & scan)
{
  typedef c_vlo_scan6_base ScanType;

  if ( scan.config.echoOrdering != VLO_ECHO_ORDER_DISTANCE_NEAR_TO_FAR ) {

    typedef struct ScanType::Echo echo_type;
    typedef decltype(ScanType::Echo::dist) dist_type;
    constexpr auto max_dist_value = std::numeric_limits<dist_type>::max() - 2;

    echo_type echos[scan.NUM_ECHOS];
    int cnz = 0;

    for( int s = 0; s < scan.NUM_SLOTS; ++s ) {

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {

        cnz = 0;
        memset(echos, 0, sizeof(echos));

        for( int e = 0; e < scan.NUM_ECHOS; ++e ) {
          if( scan.echo[s][l][e].dist > 0 && scan.echo[s][l][e].dist < max_dist_value ) {
            echos[cnz++] = scan.echo[s][l][e];
          }
        }

        if( cnz > 1 ) {

          std::sort(echos, echos + cnz,
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

static void sort_echos_by_distance(c_vlo_scan6_slm & scan)
{
  //  CF_DEBUG("SLM: NumEchos=%u NumLayers=%u NumSlots=%u echoOrdering=%u",
  //      scan.config.maxNumEchos,
  //      scan.config.maxNumLayers,
  //      scan.config.maxNumSlots,
  //      scan.config.echoOrdering);


  if ( scan.config.echoOrdering != VLO_ECHO_ORDER_DISTANCE_NEAR_TO_FAR ) {

    typedef c_vlo_scan6_slm ScanType;
    typedef struct ScanType::Echo echo_type;
    typedef decltype(ScanType::Echo::dist) dist_type;
    constexpr auto max_dist_value = std::numeric_limits<dist_type>::max() - 2;

    echo_type echos[scan.NUM_ECHOS];
    int cnz = 0;

    for( int s = 0; s < scan.NUM_SLOTS; ++s ) {

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {

        cnz = 0;
        memset(echos, 0, sizeof(echos));

        for( int e = 0; e < scan.NUM_ECHOS; ++e ) {
          if( scan.echo[s][l][e].dist > 0 && scan.echo[s][l][e].dist < max_dist_value ) {
            echos[cnz++] = scan.echo[s][l][e];
          }
        }

        if( cnz > 1 ) {

          std::sort(echos, echos + cnz,
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

template<class ScanType>
std::enable_if_t<(c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_1 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_3 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_5),
cv::Mat> get_image(const ScanType & scan, c_vlo_file::DATA_CHANNEL channel, cv::InputArray exclude_mask)
{
  cv::Mat3b emask;

  if( !exclude_mask.empty() ) {
    if( exclude_mask.type() == CV_8UC3 ) {
      if( exclude_mask.rows() == scan.NUM_LAYERS && exclude_mask.cols() == scan.NUM_SLOTS ) {
        emask = exclude_mask.getMat();
      }
    }
  }

  switch (channel) {

    case c_vlo_file::DATA_CHANNEL_AMBIENT: {
      typedef typename std::remove_reference_t<decltype(ScanType::Slot::ambient[0])> value_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;

      cv::Mat_<value_type> image(scan.NUM_LAYERS, scan.NUM_SLOTS);
      image.setTo(0);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          const auto & value = scan.slot[s].ambient[l];
          image[l][s] = value <= max_value ? value : 0;
        }
      }

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_DISTANCES: {

      typedef decltype(ScanType::Echo::dist) value_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;

      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS);
      image.setTo(0);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int e = 0; e < 3; ++e ) {

            const auto & value = scan.slot[s].echo[l][e].dist;

            if ( value > 0 && value < max_value ) {
              if ( emask.empty() || !emask[l][s][e] ) {
                image[l][s][e] = value;
              }
            }

          }
        }
      }

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_ECHO_AREA: {
      typedef decltype(ScanType::Echo::area) value_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;

      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS);
      image.setTo(0);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int e = 0; e < 3; ++e ) {
            const auto & value = scan.slot[s].echo[l][e].area;
            if ( value > 0 && value < max_value ) {
              if ( emask.empty() || !emask[l][s][e] ) {
                image[l][s][e] = value;
              }
            }
          }
        }
      }

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_ECHO_PEAK: {
      typedef decltype(ScanType::Echo::peak) value_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;

      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS);
      image.setTo(0);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int e = 0; e < 3; ++e ) {
            const auto & value = scan.slot[s].echo[l][e].peak;
            if ( value > 0 && value < max_value ) {
              if ( emask.empty() || !emask[l][s][e] ) {
                image[l][s][e] = value;
              }
            }
          }
        }
      }

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_ECHO_WIDTH: {
      typedef decltype(ScanType::Echo::width) value_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;

      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS);
      image.setTo(0);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int e = 0; e < 3; ++e ) {
            const auto & value = scan.slot[s].echo[l][e].width;
            if ( value > 0 && value < max_value ) {
              if ( emask.empty() || !emask[l][s][e] ) {
                image[l][s][e] = value;
              }
            }
          }
        }
      }

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_ECHO_AREA_DIV_WIDTH : {
      typedef decltype(ScanType::Echo::area) value_type;
      typedef decltype(ScanType::Echo::width) width_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;
      constexpr auto max_width = std::numeric_limits<width_type>::max() - 2;

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS);
      image.setTo(0);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int e = 0; e < 3; ++e ) {

            const auto & value = scan.slot[s].echo[l][e].area;
            const auto & width = scan.slot[s].echo[l][e].width;

            if ( value > 0 && value < max_value && width > 0 && width < max_width ) {
              if ( emask.empty() || !emask[l][s][e] ) {
                image[l][s][e] = ((double) value ) / ((double) width);
              }
            }

          }
        }
      }
      return image;
    }

    case c_vlo_file::DATA_CHANNEL_ECHO_PEAK_DIV_WIDTH : {
      typedef decltype(ScanType::Echo::peak) value_type;
      typedef decltype(ScanType::Echo::width) width_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;
      constexpr auto max_width = std::numeric_limits<width_type>::max() - 2;

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS);
      image.setTo(0);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int e = 0; e < 3; ++e ) {

            const auto & value = scan.slot[s].echo[l][e].peak;
            const auto & width = scan.slot[s].echo[l][e].width;

            if ( value > 0 && value < max_value && width > 0 && width < max_width ) {
              if ( emask.empty() || !emask[l][s][e] ) {
                image[l][s][e] = ((double) value ) / ((double) width);
              }
            }
          }
        }
      }
      return image;
    }

    case c_vlo_file::DATA_CHANNEL_ECHO_AREA_MUL_DIST: {

      typedef decltype(ScanType::Echo::area) value_type;
      typedef decltype(ScanType::Echo::dist) distance_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;
      constexpr auto max_distance = std::numeric_limits<distance_type>::max() - 2;

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS);
      image.setTo(0);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int e = 0; e < 3; ++e ) {

            const auto & value = scan.slot[s].echo[l][e].area;
            const auto & distance = scan.slot[s].echo[l][e].dist;

            if ( value > 0 && value < max_value && distance > 0 && distance < max_distance ) {
              if ( emask.empty() || !emask[l][s][e] ) {
                image[l][s][e] = ((double) value ) * ((double) distance);
              }
            }

          }
        }
      }

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_ECHO_PEAK_MUL_DIST: {

      typedef decltype(ScanType::Echo::peak) value_type;
      typedef decltype(ScanType::Echo::dist) distance_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;
      constexpr auto max_distance = std::numeric_limits<distance_type>::max() - 2;

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS);
      image.setTo(0);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int e = 0; e < 3; ++e ) {

            const auto &value = scan.slot[s].echo[l][e].peak;
            const auto &distance = scan.slot[s].echo[l][e].dist;

            if ( value > 0 && value < max_value && distance > 0 && distance < max_distance ) {
              if ( emask.empty() || !emask[l][s][e] ) {
                image[l][s][e] = ((double) value ) * ((double) distance);
              }
            }

          }
        }
      }

      return image;
    }


    case c_vlo_file::DATA_CHANNEL_ECHO_AREA_MUL_DIST2: {

      typedef decltype(ScanType::Echo::area) value_type;
      typedef decltype(ScanType::Echo::dist) distance_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;
      constexpr auto max_distance = std::numeric_limits<distance_type>::max() - 2;

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS);
      image.setTo(0);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int e = 0; e < 3; ++e ) {

            const auto & value = scan.slot[s].echo[l][e].area;
            const auto & distance = scan.slot[s].echo[l][e].dist;

            if ( value > 0 && value < max_value && distance > 0 && distance < max_distance ) {
              if ( emask.empty() || !emask[l][s][e] ) {
                image[l][s][e] = ((double) value ) * ((double) distance) * ((double) distance);
              }
            }

          }
        }
      }

      return image;

    }

    case c_vlo_file::DATA_CHANNEL_ECHO_PEAK_MUL_DIST2: {

      typedef decltype(ScanType::Echo::peak) value_type;
      typedef decltype(ScanType::Echo::dist) distance_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;
      constexpr auto max_distance = std::numeric_limits<distance_type>::max() - 2;

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS);
      image.setTo(0);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int e = 0; e < 3; ++e ) {

            const auto &value = scan.slot[s].echo[l][e].peak;
            const auto &distance = scan.slot[s].echo[l][e].dist;

            if ( value > 0 && value < max_value && distance > 0 && distance < max_distance ) {
              if ( emask.empty() || !emask[l][s][e] ) {
                image[l][s][e] = ((double) value ) * ((double) distance) * ((double) distance);
              }
            }
          }
        }
      }

      return image;
    }

    //
    case c_vlo_file::DATA_CHANNEL_ECHO_AREA_MUL_SQRT_DIST: {

      typedef decltype(ScanType::Echo::area) value_type;
      typedef decltype(ScanType::Echo::dist) distance_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;
      constexpr auto max_distance = std::numeric_limits<distance_type>::max() - 2;

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS);
      image.setTo(0);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int c = 0; c < 3; ++c ) {

            const auto & value = scan.slot[s].echo[l][c].area;
            const auto & distance = scan.slot[s].echo[l][c].dist;

            if ( value > 0 && value < max_value && distance > 0 && distance < max_distance ) {
              image[l][s][c] = ((double) value ) * sqrt(0.01 * distance );
            }
            else {
              image[l][s][c] = 0;
            }
          }
        }
      }

      return image;

    }

    case c_vlo_file::DATA_CHANNEL_ECHO_PEAK_MUL_SQRT_DIST: {

      typedef decltype(ScanType::Echo::peak) value_type;
      typedef decltype(ScanType::Echo::dist) distance_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;
      constexpr auto max_distance = std::numeric_limits<distance_type>::max() - 2;

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS);
      image.setTo(0);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int c = 0; c < 3; ++c ) {

            const auto &value = scan.slot[s].echo[l][c].peak;
            const auto &distance = scan.slot[s].echo[l][c].dist;

            if( value > 0 && value < max_value && distance > 0 && distance < max_distance ) {
              image[l][s][c] = ((double) value) * sqrt(0.01 * distance);
              //image[l][s][c] = ((double) value) * log( 1 + 0.01 * distance);
              //image[l][s][c] = ((double) value) * sqrt(0.01 * distance + 30);
              //image[l][s][c] = ((double) value) * log(10 + 0.01 * distance);
            }
            else {
              image[l][s][c] = 0;
            }
          }
        }
      }

      return image;
    }

//

    case c_vlo_file::DATA_CHANNEL_DOUBLED_ECHO_PEAKS: {

      cv::Mat3b image(scan.NUM_LAYERS, scan.NUM_SLOTS);
      image.setTo(0);

      double min_intens[3] = {255, 255, 255};
      double max_intens[3] = {0, 0, 0};

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {

          const auto &dist0 = scan.slot[s].echo[l][0].dist;
          const auto &dist1 = scan.slot[s].echo[l][1].dist;

          if( dist0 > 0 && dist0 < 65534 && dist1 > 0 && dist1 < 65534 ) {
            if( std::abs((double) dist1 / (double) dist0 - 2.) < 0.04 ) {
              for( int e = 0; e < 3; ++e ) {

                const uint8_t &peak =
                    scan.slot[s].echo[l][e].peak;

                if( peak < 254 ) {
                  image[l][s][e] = peak;
                }
              }
            }
          }
        }
      }

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_DIST_TO_MAX_PEAK: {

      cv::Mat1w image(scan.NUM_LAYERS, scan.NUM_SLOTS);
      image.setTo(0);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {

          int max_echo_index = -1;
          uint8_t max_peak = 0;

          for ( int e = 0; e < 3; ++e ) {

            const auto &dist = scan.slot[s].echo[l][e].dist;

            if ( dist > 0 && dist < 65534 ) {
              if ( scan.slot[s].echo[l][e].peak > max_peak ) {
                max_peak = scan.slot[s].echo[l][e].peak;
                max_echo_index = e;
              }
            }
          }

          if ( max_echo_index >= 0 ) {
            image[l][s] = scan.slot[s].echo[l][max_echo_index].dist;
          }
        }

      }

      return image;
    }
  }


  return cv::Mat();
}

cv::Mat get_image(const c_vlo_scan6_slm & scan, c_vlo_file::DATA_CHANNEL channel, cv::InputArray exclude_mask)
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

  switch (channel) {

    case c_vlo_file::DATA_CHANNEL_AMBIENT:
    case c_vlo_file::DATA_CHANNEL_ECHO_PEAK:
    case c_vlo_file::DATA_CHANNEL_ECHO_AREA: {
      typedef decltype(ScanType::Echo::area) value_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;

      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS);
      image.setTo(0);

      for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
        for( int l = 0; l < std::min(scan.BAD_LAYERS, scan.NUM_LAYERS); ++l ) {
          for( int e = 0; e < 3; ++e ) {
            const auto &value = scan.echo[s][l][e].area;
            if( value > 0 && value < max_value ) {
              if( emask.empty() || !emask[l][s][e] ) {
                image[scan.NUM_LAYERS - l - 1][s][e] = value; //
              }
            }
          }
        }
      }

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_DISTANCES: {

      typedef decltype(ScanType::Echo::dist) value_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;

      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS);
      image.setTo(0);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int e = 0; e < 3; ++e ) {

            const auto & distance = scan.echo[s][l][e].dist;

            if ( distance > 0 ) {
              if ( emask.empty() || !emask[l][s][e] ) {
                image[scan.NUM_LAYERS - l - 1][s][e] = distance;
              }
            }

          }
        }
      }

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_ECHO_PEAK_MUL_DIST:
    case c_vlo_file::DATA_CHANNEL_ECHO_AREA_MUL_DIST: {

      typedef decltype(ScanType::Echo::area) value_type;
      typedef decltype(ScanType::Echo::dist) distance_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS);
      image.setTo(0);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int e = 0; e < 3; ++e ) {

            const auto & value = scan.echo[s][l][e].area;
            const auto & distance = scan.echo[s][l][e].dist;

            if ( value > 0 && value < max_value && distance > 0 ) {
              if ( emask.empty() || !emask[l][s][e] ) {
                image[scan.NUM_LAYERS - l - 1][s][e] = ((double) value ) * ((double) distance);
              }
            }

          }
        }
      }

      return image;
    }

    //
    case c_vlo_file::DATA_CHANNEL_ECHO_PEAK_MUL_SQRT_DIST:
    case c_vlo_file::DATA_CHANNEL_ECHO_AREA_MUL_SQRT_DIST: {

      typedef decltype(ScanType::Echo::area) value_type;
      typedef decltype(ScanType::Echo::dist) distance_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS);
      image.setTo(0);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int c = 0; c < 3; ++c ) {

            const auto & value = scan.echo[s][l][c].area;
            const auto & distance = scan.echo[s][l][c].dist;

            if ( value > 0 && value < max_value && distance > 0 ) {
              image[scan.NUM_LAYERS - l - 1][s][c] = ((double) value ) * sqrt(0.01 * distance );
            }
            else {
              image[scan.NUM_LAYERS - l - 1][s][c] = 0;
            }
          }
        }
      }

      return image;

    }

    case c_vlo_file::DATA_CHANNEL_DOUBLED_ECHO_PEAKS: {

      cv::Mat3b image(scan.NUM_LAYERS, scan.NUM_SLOTS);
      image.setTo(0);

      double min_intens[3] = {255, 255, 255};
      double max_intens[3] = {0, 0, 0};

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {

          const auto &dist0 = scan.echo[s][l][0].dist;
          const auto &dist1 = scan.echo[s][l][1].dist;

          if( dist0 > 0 && dist1 > 0 ) {
            if( std::abs((double) dist1 / (double) dist0 - 2.) < 0.04 ) {
              for( int e = 0; e < 3; ++e ) {

                const uint8_t &area =
                    scan.echo[s][l][e].area;

                //if( area < 254 )
                {
                  image[scan.NUM_LAYERS - l - 1][s][e] = area;
                }
              }
            }
          }
        }
      }

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_DIST_TO_MAX_PEAK: {

      cv::Mat1w image(scan.NUM_LAYERS, scan.NUM_SLOTS);
      image.setTo(0);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {

          int max_echo_index = -1;
          uint8_t max_peak = 0;

          for ( int e = 0; e < 3; ++e ) {

            const auto &dist = scan.echo[s][l][e].dist;

            if ( dist > 0 && dist < 65534 ) {
              if ( scan.echo[s][l][e].area > max_peak ) {
                max_peak = scan.echo[s][l][e].area;
                max_echo_index = e;
              }
            }
          }

          if ( max_echo_index >= 0 ) {
            image[scan.NUM_LAYERS - l - 1][s] =
                scan.echo[s][l][max_echo_index].dist;
          }
        }

      }

      return image;
    }
  }


  return cv::Mat();
}



template<class ScanType>
std::enable_if_t<(c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_1 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_3 ||
    c_vlo_scan_type_traits<ScanType>::VERSION == VLO_VERSION_5),
bool> get_cloud3d(const ScanType & scan, c_vlo_reader::DATA_CHANNEL intensity_channel,
    cv::OutputArray points, cv::OutputArray colors)
{
  cv::Mat intensityImage =
      get_image(scan, intensity_channel,
          cv::noArray());

  if( !intensityImage.empty() ) {

    autoclip(intensityImage, cv::noArray(),
        0.01, 99.99,
        0, 255);

    if( intensityImage.depth() != CV_8U ) {
      intensityImage.convertTo(intensityImage, CV_8U);
    }

    if ( intensityImage.channels() == 1 ) {
      cv::cvtColor(intensityImage, intensityImage,
          cv::COLOR_GRAY2BGR);
    }
  }

  const cv::Mat3b intensity =
      intensityImage;

  const float firstVertAngle =
      0.5 * 0.05 * scan.NUM_LAYERS;

  const float tick2deg =
      (float) (0.00000008381903173490870551553291862726);

  const float yawCorrection = 0;

  std::vector<cv::Vec3f> output_points;
  std::vector<cv::Vec3b> output_colors;

  output_points.reserve(scan.NUM_POINTS);
  output_colors.reserve(scan.NUM_POINTS);

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

      for( int e = 0; e < std::min(3, (int)scan.NUM_ECHOS); ++e ) {

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
              intensity[l][s][e];

          output_colors.emplace_back(color, color, color);
        }
      }
    }
  }

  cv::Mat(output_points).copyTo(points);
  cv::Mat(output_colors).copyTo(colors);


  return true;
}


bool get_cloud3d(const c_vlo_scan6_slm & scan, c_vlo_reader::DATA_CHANNEL intensity_channel,
    cv::OutputArray points, cv::OutputArray colors)
{
  cv::Mat intensityImage =
      get_image(scan, intensity_channel,
          cv::noArray());

  if( !intensityImage.empty() ) {

    autoclip(intensityImage, cv::noArray(),
        0.01, 99.99,
        0, 255);

    if( intensityImage.depth() != CV_8U ) {
      intensityImage.convertTo(intensityImage, CV_8U);
    }

    if ( intensityImage.channels() == 1 ) {
      cv::cvtColor(intensityImage, intensityImage,
          cv::COLOR_GRAY2BGR);
    }
  }

  const cv::Mat3b intensity =
      intensityImage;

  std::vector<cv::Vec3f> output_points;
  std::vector<cv::Vec3b> output_colors;

  output_points.reserve(scan.NUM_SLOTS * scan.NUM_LAYERS);
  output_colors.reserve(scan.NUM_SLOTS * scan.NUM_LAYERS);

  for( int s = 0; s < scan.NUM_SLOTS; ++s ) {

    const float sin_azimuth =
        std::sin(scan.horizontalAngles[s]);

    const float cos_azimuth =
        std::cos(scan.horizontalAngles[s]);

    for( int l = 0; l < std::min(scan.BAD_LAYERS, scan.NUM_LAYERS); ++l ) {

      const float inclination =
          CV_PI / 2 - scan.verticalAngles[l];

      const float sin_inclination =
          std::sin(inclination);

      const float cos_inclination =
          std::cos(inclination);

      for( int e = 0; e < scan.NUM_ECHOS; ++e ) {

        const uint16_t & dist = scan.echo[s][l][e].dist;
        if ( dist < 200 || dist >= 30000 ) {
          continue;
        }

        const uint16_t area = scan.echo[s][l][e].area;

        const float distance = 0.01 * dist;

        // mirrorSide = scan.mirrorSide;
        const float x = distance * sin_inclination * cos_azimuth;
        const float y = -distance * sin_inclination * sin_azimuth;
        const float z = -distance * cos_inclination;

        const uint8_t gray =
            intensity[scan.NUM_LAYERS - l - 1][s][e];

        output_points.emplace_back(x, y, z);
        output_colors.emplace_back(gray, gray, gray);
      }
    }
  }

  cv::Mat(output_points).copyTo(points);
  cv::Mat(output_colors).copyTo(colors);


  return true;
}

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

cv::Mat c_vlo_file::get_image(const c_vlo_scan & scan, DATA_CHANNEL channel, cv::InputArray exclude_mask)
{
  switch (scan.version) {
    case VLO_VERSION_1:
      return ::get_image(scan.scan1, channel, exclude_mask);
    case VLO_VERSION_3:
      return ::get_image(scan.scan3, channel, exclude_mask);
    case VLO_VERSION_5:
      return ::get_image(scan.scan5, channel, exclude_mask);
    case VLO_VERSION_6_SLM:
      return ::get_image(scan.scan6_slm, channel, exclude_mask);
  }
  return cv::Mat();
}

bool c_vlo_file::get_cloud3d(const c_vlo_scan & scan, DATA_CHANNEL intensity_channel,
    cv::OutputArray points, cv::OutputArray colors)
{
  switch (scan.version) {
    case VLO_VERSION_1:
      return ::get_cloud3d(scan.scan1, intensity_channel, points, colors);
    case VLO_VERSION_3:
      return ::get_cloud3d(scan.scan3, intensity_channel, points, colors);
    case VLO_VERSION_5:
      return ::get_cloud3d(scan.scan5, intensity_channel, points, colors);
    case VLO_VERSION_6_SLM:
      return ::get_cloud3d(scan.scan6_slm, intensity_channel, points, colors);
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

  if( ifhd_.open(filename) ) {
    if ( ifhd_.select_stream("ScaLa 3-PointCloud")  ) {

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


        if( format_version == 18  && payload_size == sizeof(c_vlo_scan1) + scanV1Extra ) {
          version_ = VLO_VERSION_1;
          frame_size_ = sizeof(c_vlo_scan1);
        }
        else if( format_version == 18  && payload_size == sizeof(c_vlo_scan3) + scanV3Extra ) {
          version_ = VLO_VERSION_3;
          frame_size_ = sizeof(c_vlo_scan3);
        }
        else if( format_version == 5 && payload_size == sizeof(c_vlo_scan5) + scanV5Extra ) {
          version_ = VLO_VERSION_5;
          frame_size_ = sizeof(c_vlo_scan5);
        }
        else if( format_version == 5  && payload_size == sizeof(c_vlo_scan5) ) {
          version_ = VLO_VERSION_5;
          frame_size_ = sizeof(c_vlo_scan5);
        }
        else if( format_version == 6  && payload_size == sizeof(c_vlo_scan6_slm) + scanV6SLMExtra ) {
          version_ = VLO_VERSION_6_SLM;
          frame_size_ = sizeof(c_vlo_scan6_slm);
        }

        CF_DEBUG("SET: format_version: %d -> %d frame_size_=%zd",
            format_version, version_, frame_size_);

        if( frame_size_ > 0 ) {
          ifhd_.seek(0);
          num_frames_ = ifhd_.num_frames();
          fOk = true;
        }
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

    if( (fd_ = ::open(fname, openflags)) < 0 ) {
      CF_ERROR("open('%s') fails: %s", fname, strerror(errno));
      goto end;
    }

    if( ::read(fd_, &format_version, sizeof(format_version)) != sizeof(format_version) ) {
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

        if( !readfrom(fd_, offset = 1 * sizeof(c_vlo_scan1), &data11) ) {
          CF_ERROR("readfrom('%s', offset=%ld, data11) fails: %s", fname, offset,
              strerror(errno));
          goto end;
        }
        if( !readfrom(fd_, offset = 2 * sizeof(c_vlo_scan1), &data12) ) {
          CF_ERROR("readfrom('%s', offset=%ld, data12) fails: %s", fname, offset,
              strerror(errno));
          goto end;
        }

        if( !readfrom(fd_, offset = 1 * sizeof(c_vlo_scan3), &data31) ) {
          CF_ERROR("readfrom('%s', offset=%ld, data31) fails: %s", fname, offset,
              strerror(errno));
          goto end;
        }
        if( !readfrom(fd_, offset = 2 * sizeof(c_vlo_scan3), &data32) ) {
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

    if( ::lseek64(fd_, 0, SEEK_SET) != 0 ) {
      CF_ERROR("lseek('%s', offset=0, SEEK_SET) fails: %s", fname,
          strerror(errno));
      goto end;
    }

    if( (file_size = ::lseek64(fd_, 0, SEEK_END)) < 0 ) {
      CF_ERROR("lseek('%s', offset=0, SEEK_END) fails: %s", fname,
          strerror(errno));
      goto end;
    }

    if( ::lseek64(fd_, 0, SEEK_SET) != 0 ) {
      CF_ERROR("lseek('%s', offset=0, SEEK_SET) fails: %s", fname,
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
  if( fd_ >= 0 ) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool c_vlo_reader::is_open() const
{
  return fd_ >= 0 || ifhd_.is_open();
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

  if( fd_ >= 0 ) {

    const ssize_t seekpos =
        frame_index * frame_size();

    return ::lseek64(fd_, seekpos, SEEK_SET) == seekpos;
  }

  errno = EBADF;
  return false;
}

int32_t c_vlo_reader::curpos() const
{
  if( ifhd_.is_open() ) {
    return ifhd_.curpos();
  }

  if( fd_ >= 0 ) {
    return whence(fd_) / frame_size();
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

  if( fd_ >= 0 ) {
    if( ::read(fd_, scan, sizeof(*scan)) == sizeof(*scan) ) {
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
    return !(*image = get_image(*scan, channel, cv::noArray())).empty();
  }
  return false;
}

bool c_vlo_reader::read_cloud3d(cv::OutputArray points, cv::OutputArray colors, c_vlo_file::DATA_CHANNEL colors_channel)
{
  std::unique_ptr<c_vlo_scan> scan(new c_vlo_scan());
  if ( read(scan.get()) ) {
    return get_cloud3d(*scan, colors_channel, points, colors);
  }

  return false;
}

