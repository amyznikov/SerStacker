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
#include <core/proc/autoclip.h>
#include <core/debug.h>

//static constexpr uint32_t sizeScanV1 = sizeof(c_vlo_scan1); // 3561744;
//static constexpr uint32_t sizeScanV3 = sizeof(c_vlo_scan3);// 5622704;
//static constexpr uint32_t sizeScanV5 = sizeof(c_vlo_scan5);// 4560080;


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
      { c_vlo_file::DATA_CHANNEL_ECHO_AREA_DIV_DIST, "AREA_DIV_DIST", "" },
      { c_vlo_file::DATA_CHANNEL_ECHO_PEAK_DIV_DIST, "PEAK_DIV_DIST", "" },
      { c_vlo_file::DATA_CHANNEL_ECHO_AREA_DIV_DIST2, "AREA_DIV_DIST2", "" },
      { c_vlo_file::DATA_CHANNEL_ECHO_PEAK_DIV_DIST2, "PEAK_DIV_DIST2", "" },
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

template<class ScanType>
static cv::Mat get_image(const ScanType & scan, c_vlo_file::DATA_CHANNEL channel)
{
  switch (channel) {

    case c_vlo_file::DATA_CHANNEL_AMBIENT: {
      typedef typename std::remove_reference_t<decltype(ScanType::slot::ambient[0])> value_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;

      cv::Mat_<value_type> image(scan.NUM_LAYERS, scan.NUM_SLOTS);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          const auto & value = scan.slot[s].ambient[l];
          image[l][s] = value <= max_value ? value : 0;
        }
      }

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_DISTANCES: {

      typedef decltype(ScanType::echo::dist) value_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;

      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int c = 0; c < 3; ++c ) {
            const auto & value = scan.slot[s].echo[l][c].dist;
            image[l][s][c] = value <= max_value ? value : 0;
          }
        }
      }

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_ECHO_AREA: {
      typedef decltype(ScanType::echo::area) value_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;

      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int c = 0; c < 3; ++c ) {
            const auto & value = scan.slot[s].echo[l][c].area;
            image[l][s][c] = value <= max_value ? value : 0;
          }
        }
      }

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_ECHO_PEAK: {
      typedef decltype(ScanType::echo::peak) value_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;

      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int c = 0; c < 3; ++c ) {
            const auto & value = scan.slot[s].echo[l][c].peak;
            image[l][s][c] = value <= max_value ? value : 0;
          }
        }
      }

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_ECHO_WIDTH: {
      typedef decltype(ScanType::echo::width) value_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;

      cv::Mat_<cv::Vec<value_type, 3>> image(scan.NUM_LAYERS, scan.NUM_SLOTS);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int c = 0; c < 3; ++c ) {
            const auto & value = scan.slot[s].echo[l][c].width;
            image[l][s][c] = value <= max_value ? value : 0;
          }
        }
      }

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_ECHO_AREA_DIV_WIDTH : {
      typedef decltype(ScanType::echo::area) value_type;
      typedef decltype(ScanType::echo::width) width_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;
      constexpr auto max_width = std::numeric_limits<width_type>::max() - 2;

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int c = 0; c < 3; ++c ) {

            const auto & value = scan.slot[s].echo[l][c].area;
            const auto & width = scan.slot[s].echo[l][c].width;

            if ( value > 0 && value < max_value && width > 0 && width < max_width ) {
              image[l][s][c] = ((double) value ) / ((double) width);
            }
            else {
              image[l][s][c] = 0;
            }
          }
        }
      }
      return image;
    }

    case c_vlo_file::DATA_CHANNEL_ECHO_PEAK_DIV_WIDTH : {
      typedef decltype(ScanType::echo::peak) value_type;
      typedef decltype(ScanType::echo::width) width_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;
      constexpr auto max_width = std::numeric_limits<width_type>::max() - 2;

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int c = 0; c < 3; ++c ) {

            const auto & value = scan.slot[s].echo[l][c].peak;
            const auto & width = scan.slot[s].echo[l][c].width;

            if ( value > 0 && value < max_value && width > 0 && width < max_width ) {
              image[l][s][c] = ((double) value ) / ((double) width);
            }
            else {
              image[l][s][c] = 0;
            }
          }
        }
      }
      return image;
    }

    case c_vlo_file::DATA_CHANNEL_ECHO_AREA_DIV_DIST : {
      typedef decltype(ScanType::echo::area) value_type;
      typedef decltype(ScanType::echo::dist) distance_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;
      constexpr auto max_distance = std::numeric_limits<distance_type>::max() - 2;

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int c = 0; c < 3; ++c ) {

            const auto & value = scan.slot[s].echo[l][c].area;
            const auto & distance = scan.slot[s].echo[l][c].dist;

            if ( value > 0 && value < max_value && distance > 0 && distance < max_distance ) {
              image[l][s][c] = ((double) value ) / ((double) distance);
            }
            else {
              image[l][s][c] = 0;
            }
          }
        }
      }

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_ECHO_PEAK_DIV_DIST : {
      typedef decltype(ScanType::echo::peak) value_type;
      typedef decltype(ScanType::echo::dist) distance_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;
      constexpr auto max_distance = std::numeric_limits<distance_type>::max() - 2;

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int c = 0; c < 3; ++c ) {

            const auto & value = scan.slot[s].echo[l][c].peak;
            const auto & distance = scan.slot[s].echo[l][c].dist;

            if ( value > 0 && value < max_value && distance > 0 && distance < max_distance ) {
              image[l][s][c] = ((double) value ) / ((double) distance);
            }
            else {
              image[l][s][c] = 0;
            }
          }
        }
      }

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_ECHO_AREA_DIV_DIST2 : {
      typedef decltype(ScanType::echo::area) value_type;
      typedef decltype(ScanType::echo::dist) distance_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;
      constexpr auto max_distance = std::numeric_limits<distance_type>::max() - 2;

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int c = 0; c < 3; ++c ) {

            const auto & value = scan.slot[s].echo[l][c].area;
            const auto & distance = scan.slot[s].echo[l][c].dist;

            if ( value > 0 && value < max_value && distance > 0 && distance < max_distance ) {
              image[l][s][c] = ((double) value ) / (((double) distance) * (double) distance) ;
            }
            else {
              image[l][s][c] = 0;
            }
          }
        }
      }

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_ECHO_PEAK_DIV_DIST2 : {
      typedef decltype(ScanType::echo::peak) value_type;
      typedef decltype(ScanType::echo::dist) distance_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;
      constexpr auto max_distance = std::numeric_limits<distance_type>::max() - 2;

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int c = 0; c < 3; ++c ) {

            const auto & value = scan.slot[s].echo[l][c].peak;
            const auto & distance = scan.slot[s].echo[l][c].dist;

            if ( value > 0 && value < max_value && distance > 0 && distance < max_distance ) {
              image[l][s][c] = ((double) value ) / (((double) distance) * (double) distance) ;
            }
            else {
              image[l][s][c] = 0;
            }
          }
        }
      }

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_ECHO_AREA_MUL_DIST: {

      typedef decltype(ScanType::echo::area) value_type;
      typedef decltype(ScanType::echo::dist) distance_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;
      constexpr auto max_distance = std::numeric_limits<distance_type>::max() - 2;

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int c = 0; c < 3; ++c ) {

            const auto & value = scan.slot[s].echo[l][c].area;
            const auto & distance = scan.slot[s].echo[l][c].dist;

            if ( value > 0 && value < max_value && distance > 0 && distance < max_distance ) {
              image[l][s][c] = ((double) value ) * ((double) distance);
            }
            else {
              image[l][s][c] = 0;
            }
          }
        }
      }

      return image;
    }

    case c_vlo_file::DATA_CHANNEL_ECHO_PEAK_MUL_DIST: {

      typedef decltype(ScanType::echo::peak) value_type;
      typedef decltype(ScanType::echo::dist) distance_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;
      constexpr auto max_distance = std::numeric_limits<distance_type>::max() - 2;

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int c = 0; c < 3; ++c ) {

            const auto &value = scan.slot[s].echo[l][c].peak;
            const auto &distance = scan.slot[s].echo[l][c].dist;

            if( value > 0 && value < max_value && distance > 0 && distance < max_distance ) {
              image[l][s][c] = ((double) value) * ((double) distance);
            }
            else {
              image[l][s][c] = 0;
            }
          }
        }
      }

      return image;
    }


    case c_vlo_file::DATA_CHANNEL_ECHO_AREA_MUL_DIST2: {

      typedef decltype(ScanType::echo::area) value_type;
      typedef decltype(ScanType::echo::dist) distance_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;
      constexpr auto max_distance = std::numeric_limits<distance_type>::max() - 2;

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int c = 0; c < 3; ++c ) {

            const auto & value = scan.slot[s].echo[l][c].area;
            const auto & distance = scan.slot[s].echo[l][c].dist;

            if ( value > 0 && value < max_value && distance > 0 && distance < max_distance ) {
              image[l][s][c] = ((double) value ) * ((double) distance) * ((double) distance);
            }
            else {
              image[l][s][c] = 0;
            }
          }
        }
      }

      return image;

    }

    case c_vlo_file::DATA_CHANNEL_ECHO_PEAK_MUL_DIST2: {

      typedef decltype(ScanType::echo::peak) value_type;
      typedef decltype(ScanType::echo::dist) distance_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;
      constexpr auto max_distance = std::numeric_limits<distance_type>::max() - 2;

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int c = 0; c < 3; ++c ) {

            const auto &value = scan.slot[s].echo[l][c].peak;
            const auto &distance = scan.slot[s].echo[l][c].dist;

            if( value > 0 && value < max_value && distance > 0 && distance < max_distance ) {
              image[l][s][c] = ((double) value) * ((double) distance) * ((double) distance);
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
    case c_vlo_file::DATA_CHANNEL_ECHO_AREA_MUL_SQRT_DIST: {

      typedef decltype(ScanType::echo::area) value_type;
      typedef decltype(ScanType::echo::dist) distance_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;
      constexpr auto max_distance = std::numeric_limits<distance_type>::max() - 2;

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int c = 0; c < 3; ++c ) {

            const auto & value = scan.slot[s].echo[l][c].area;
            const auto & distance = scan.slot[s].echo[l][c].dist;

            if ( value > 0 && value < max_value && distance > 0 && distance < max_distance ) {
              image[l][s][c] = ((double) value ) * sqrt((double) distance);
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

      typedef decltype(ScanType::echo::peak) value_type;
      typedef decltype(ScanType::echo::dist) distance_type;
      constexpr auto max_value = std::numeric_limits<value_type>::max() - 2;
      constexpr auto max_distance = std::numeric_limits<distance_type>::max() - 2;

      cv::Mat3f image(scan.NUM_LAYERS, scan.NUM_SLOTS);

      for( int l = 0; l < scan.NUM_LAYERS; ++l ) {
        for( int s = 0; s < scan.NUM_SLOTS; ++s ) {
          for( int c = 0; c < 3; ++c ) {

            const auto &value = scan.slot[s].echo[l][c].peak;
            const auto &distance = scan.slot[s].echo[l][c].dist;

            if( value > 0 && value < max_value && distance > 0 && distance < max_distance ) {
              image[l][s][c] = ((double) value) * sqrt((double) distance);
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

      if( ifhd_.read_payload(&format_version, sizeof(format_version)) == sizeof(format_version) ) {

        static constexpr uint32_t scanV1Offset = 336;
        static constexpr uint32_t scanV3Offset = 1328;
        static constexpr uint32_t scanV5Offset = 1120;

        if( payload_size == sizeof(c_vlo_scan1) + scanV1Offset && format_version == 18 ) {
          version_ = VLO_VERSION_1;
          frame_size_ = sizeof(c_vlo_scan1);
        }
        else if( payload_size == sizeof(c_vlo_scan3) + scanV3Offset && format_version == 18 ) {
          version_ = VLO_VERSION_3;
          frame_size_ = sizeof(c_vlo_scan3);
        }
        else if( payload_size == sizeof(c_vlo_scan5) + scanV5Offset && format_version == 5 ) {
          version_ = VLO_VERSION_5;
          frame_size_ = sizeof(c_vlo_scan5);
        }
        else if( payload_size == sizeof(c_vlo_scan5) && format_version == 5 ) {
          version_ = VLO_VERSION_5;
          frame_size_ = sizeof(c_vlo_scan5);
        }

        CF_DEBUG("format_version: %d->%d frame_size_=%zd",
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

    CF_DEBUG("vlo struct sizes: scan1=%zu scan3=%zu scan=%zu",
        sizeof(c_vlo_scan1),
        sizeof(c_vlo_scan3),
        sizeof(c_vlo_scan5));

    CF_DEBUG("vlo format version %u in '%s'",
        format_version, fname);

    switch ((version_ = (VLO_VERSION) (format_version))) {
      case VLO_VERSION_1:
        frame_size_ = sizeof(c_vlo_scan1);
        break;
      case VLO_VERSION_3:
        frame_size_ = sizeof(c_vlo_scan3);
        break;
      case VLO_VERSION_5:
        frame_size_ = sizeof(c_vlo_scan5);
        break;
      case VLO_VERSION_18: {

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

bool c_vlo_reader::read(c_vlo_scan1 * scan)
{
  if( version_ != VLO_VERSION_1 ) {
    errno = EINVAL;
    return false;
  }

  if( fd_ >= 0 ) {
    return ::read(fd_, scan, sizeof(*scan)) == sizeof(*scan);
  }

  if( ifhd_.is_open() ) {
    return ifhd_.read_payload(scan, sizeof(*scan)) == sizeof(*scan);
  }

  errno = EBADF;
  return false;
}

bool c_vlo_reader::read(c_vlo_scan3 * scan)
{
  if( version_ != VLO_VERSION_3 ) {
    errno = EINVAL;
    return false;
  }

  if( fd_ >= 0 ) {
    return ::read(fd_, scan, sizeof(*scan)) == sizeof(*scan);
  }

  if( ifhd_.is_open() ) {
    return ifhd_.read_payload(scan, sizeof(*scan)) == sizeof(*scan);
  }

  errno = EBADF;
  return false;
}

bool c_vlo_reader::read(c_vlo_scan5 * scan)
{
  if( version_ != VLO_VERSION_5 ) {
    errno = EINVAL;
    return false;
  }


  if( fd_ >= 0 ) {
    return ::read(fd_, scan, sizeof(*scan)) == sizeof(*scan);
  }

  if( ifhd_.is_open() ) {
    return ifhd_.read_payload(scan, sizeof(*scan)) == sizeof(*scan);
  }

  errno = EBADF;
  return false;
}

bool c_vlo_reader::read(cv::Mat * image, c_vlo_file::DATA_CHANNEL channel)
{
  switch (version()) {
    case VLO_VERSION_1: {
      std::unique_ptr<c_vlo_scan1> scan(new c_vlo_scan1());
      if( read(scan.get()) ) {
        return !(*image = ::get_image(*scan, channel)).empty();
      }
      break;
    }

    case VLO_VERSION_3: {
      std::unique_ptr<c_vlo_scan3> scan(new c_vlo_scan3());
      if( read(scan.get()) ) {
        return !(*image = ::get_image(*scan, channel)).empty();
      }
      break;
    }

    case VLO_VERSION_5: {
      std::unique_ptr<c_vlo_scan5> scan(new c_vlo_scan5());
      if( read(scan.get()) ) {
        return !(*image = ::get_image(*scan, channel)).empty();
      }
      break;
    }
  }

  return false;
}

cv::Mat c_vlo_reader::get_image(const c_vlo_scan1 & scan, DATA_CHANNEL channel)
{
  return ::get_image(scan, channel);
}

cv::Mat c_vlo_reader::get_image(const c_vlo_scan3 & scan, DATA_CHANNEL channel)
{
  return ::get_image(scan, channel);
}

cv::Mat c_vlo_reader::get_image(const c_vlo_scan5 & scan, DATA_CHANNEL channel)
{
  return ::get_image(scan, channel);
}

cv::Mat c_vlo_reader::get_thumbnail_image(const std::string & filename)
{
  cv::Mat image;
  c_vlo_reader vlo;

  if( vlo.open(filename) ) {

    switch (vlo.version()) {
      case VLO_VERSION_1: {
        std::unique_ptr<c_vlo_scan1> scan(new c_vlo_scan1());
        if( vlo.read(scan.get()) ) {
          image = get_image(*scan, DATA_CHANNEL_AMBIENT);
        }
        break;
      }

      case VLO_VERSION_3: {
        std::unique_ptr<c_vlo_scan3> scan(new c_vlo_scan3());
        if( vlo.read(scan.get()) ) {
          image = get_image(*scan, DATA_CHANNEL_AMBIENT);
        }
        break;
      }

      case VLO_VERSION_5: {
        std::unique_ptr<c_vlo_scan5> scan(new c_vlo_scan5());
        if( vlo.read(scan.get()) ) {
          image = get_image(*scan, DATA_CHANNEL_AMBIENT);
        }
        break;
      }
    }
  }

  if( !image.empty() ) {
    autoclip(image, cv::noArray(),
        0.5, 99.5,
        0, 255);
    image.convertTo(image,
        CV_8U);
  }

  return image;
//  return cv::Mat();
}

