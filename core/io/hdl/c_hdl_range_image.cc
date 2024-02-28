/*
 * c_hdl_range_image.cc
 *
 *  Created on: Feb 28, 2024
 *      Author: amyznikov
 */

#include "c_hdl_range_image.h"

#if HAVE_TBB && !defined(Q_MOC_RUN)
# include <tbb/tbb.h>
  typedef tbb::blocked_range<int> range;
#endif

#include <core/debug.h>


namespace {

inline float square(float x)
{
  return x * x;
}

inline double square(double x)
{
  return x * x;
}

inline cv::Vec4f compute_cartesian2(const c_hdl_point & p)
{
  const double azimuth = p.azimuth;
  const double elevation = p.elevation;
  const double distance = p.distance;

  return cv::Vec4f(distance * cos(elevation) * sin(azimuth),
      distance * cos(elevation) * cos(azimuth),
      distance * sin(elevation),
      p.intensity);
}

} // namespace


c_hdl_range_image::c_hdl_range_image()
{
  update_image_size();
}

/* c'tor with lidar specifcation */
c_hdl_range_image::c_hdl_range_image(const c_hdl_specification * lidar) :
    hdl_(lidar)
{
  update_image_size();
}

/* c'tor with lidar specifcation and start azimith */
c_hdl_range_image::c_hdl_range_image(const c_hdl_specification * lidar, double start_azimuth) :
    hdl_(lidar), start_azimuth_(start_azimuth)
{
  update_image_size();
}

c_hdl_range_image::c_hdl_range_image(const c_hdl_specification * lidar, double azimuthal_resolution, double start_azimuth) :
    hdl_(lidar), azimuthal_resolution_(azimuthal_resolution), start_azimuth_(start_azimuth)
{
  update_image_size();
}

void c_hdl_range_image::update_image_size()
{
  if( !hdl_ || azimuthal_resolution_ <= 0 ) {
    sin_elevation_table_.clear();
    cos_elevation_table_.clear();
    tan_elevation_table_.clear();
    image_size_.width = 0;
    image_size_.height = 0;
  }
  else {

    image_size_ = cv::Size(hdl_->lasers.size(),
        (int) ceil(2 * CV_PI / azimuthal_resolution_));

    sin_elevation_table_.resize(hdl_->lasers.size());
    cos_elevation_table_.resize(hdl_->lasers.size());
    tan_elevation_table_.resize(hdl_->lasers.size());

    for( uint i = 0, n = hdl_->lasers.size(); i < n; ++i ) {
      const c_hdl_lasers_table &laser = hdl_->lasers[i];
      sin_elevation_table_[laser.laser_ring] = sin(laser.vert_correction * CV_PI / 180);
      cos_elevation_table_[laser.laser_ring] = cos(laser.vert_correction * CV_PI / 180);
      tan_elevation_table_[laser.laser_ring] = tan(laser.vert_correction * CV_PI / 180);
    }
  }
}

/** set current HDL parameters */
void c_hdl_range_image::set_lidar_specifcation(const c_hdl_specification * lidar)
{
  hdl_ = lidar;
  update_image_size();
}

/** get current HDL parameters */
const c_hdl_specification* c_hdl_range_image::lidar_specifcation() const
{
  return hdl_;
}

/** set desired azimuthal resolution in [radians/px] */
void c_hdl_range_image::set_azimuthal_resolution(double radians_per_pixel)
{
  azimuthal_resolution_ = radians_per_pixel;
  update_image_size();
}

/** get current azimuthal resolution in [radians/px] */
double c_hdl_range_image::azimuthal_resolution() const
{
  return azimuthal_resolution_;
}

/** set start azimuth (the azimuth of very first image column) in radians. */
void c_hdl_range_image::set_start_azimuth(double radians)
{
  start_azimuth_ = radians;
}

/** get current start azimuth (the azimuth of very first image column) in radians. */
double c_hdl_range_image::start_azimuth() const
{
  return start_azimuth_;
}

bool c_hdl_range_image::create_output_images(cv::OutputArray output_range_image, cv::Mat1b * mask, int dtype) const
{
  if( !hdl_ ) {
    CF_ERROR("HDL LiDAR specification pointer is not set");
    return false;
  }

  if( azimuthal_resolution_ <= 0 ) {
    CF_ERROR("invalid range image azimuthal resolution specified: %g [radian/pix]",
        azimuthal_resolution_);
    return false;
  }

  if( image_size_.empty() ) {
    CF_ERROR("APP BUG: invalid computed range image size %dx%d.\n"
        "Most probably the c_hdl_range_image::update_image_size() was not called properly\n"
        "Consult amyznikov how to fix this issue",
        image_size_.width, image_size_.height);
    return false;
  }



  if( output_range_image.needed() ) {

    if( output_range_image.fixedType() ) {
      dtype = output_range_image.type();
    }
    else if( dtype < 0 ) {
      CF_ERROR("No range image output type specified: dtype=%d", dtype);
      return false;
    }

    output_range_image.create(image_size_, dtype);
    output_range_image.setTo(0);
  }

  if( mask ) {
    mask->create(image_size_);
    mask->setTo(0);
  }

  return true;
}


/** project given HDL point onto range image */
bool c_hdl_range_image::project(const c_hdl_point & p, int * outr, int * outc) const
{
  double a;

  const int c = p.laser_ring;
  if( c < 0 || c >= image_size_.width ) {
    return false;
  }

  if( (a = p.azimuth - start_azimuth_) < 0 ) {
    a += 2 * CV_PI;
  }

  const int r = (int) (a / azimuthal_resolution_);
  if( r < 0 || r >= image_size_.height ) {
    return false;
  }

  *outr = r;
  *outc = c;

  return true;
}

bool c_hdl_range_image::projectncr(const c_hdl_point & p, int * outr, int * outc) const
{
  double a;

  if( (a = p.azimuth - start_azimuth_) < 0 ) {
    a += 2 * CV_PI;
  }

  const int r = (const int) (a / azimuthal_resolution_);
  if( r >= 0 && r < image_size_.height ) {
    *outr = r;
    *outc = p.laser_ring;
    return true;
  }

  return false;
}

/** get azimuth for given range image row, in radians */
double c_hdl_range_image::azimuth(int r) const
{
  double a = r * azimuthal_resolution_ + start_azimuth_;
  if( a >= 2 * CV_PI ) {
    a -= 2 * CV_PI;
  }
  if( a < 0 ) {
    a += 2 * CV_PI;
  }
  return a;
}

/** get azimuth for given range image column, in radians */
double c_hdl_range_image::elevation(int c) const
{
  if ( hdl_ ) {
    for ( const c_hdl_lasers_table & t : hdl_->lasers ) {
      if ( t.laser_ring == c ) {
        return t.vert_correction * CV_PI / 180;
      }
    }
  }
  return 0;
}



/** build range image where each pixel is the distance to HDL point  */
bool c_hdl_range_image::build_distances(const std::vector<c_hdl_point> & points,
    /* out*/cv::Mat1f & distances,
    /* out, opt */ cv::Mat1b * mask,
    const std::vector<uint8_t> * filter) const
{
  if( filter && !filter->empty() && filter->size() != points.size() ) {
    CF_ERROR("Invalid filter size=%zu, expected %zu", filter->size(), points.size());
    return false;
  }

  if ( !create_output_images(distances, mask) ) {
    CF_ERROR("create_output_images() fails");
    return false;
  }

  int r, c;

  if ( !filter || filter->empty() ) {
    for( const c_hdl_point &p : points ) {
      if( p.distance > 0 && project(p, &r, &c) ) {
        if( !distances[r][c] || p.distance < distances[r][c] ) {
          distances[r][c] = p.distance;
          if( mask ) {
            (*mask)[r][c] = 255;
          }
        }
      }
    }
  }
  else {
    for( uint i = 0, n = points.size(); i < n; ++i ) {
      if( (*filter)[i] ) {
        const c_hdl_point &p = points[i];
        if( p.distance > 0 && project(p, &r, &c) ) {
          if( !distances[r][c] || p.distance < distances[r][c] ) {
            distances[r][c] = p.distance;
            if( mask ) {
              (*mask)[r][c] = 255;
            }
          }
        }
      }
    }
  }

  return true;
}

/** build range image where each pixel is the horizontal distance (depth) to HDL point  */
bool c_hdl_range_image::build_depths(const std::vector<c_hdl_point> & points,
    /* out*/cv::Mat1f & depths,
    /* out, opt */cv::Mat1b * mask,
    const std::vector<uint8_t> * filter) const
{
  if( filter && !filter->empty() && filter->size() != points.size() ) {
    CF_ERROR("Invalid filter size=%zu, expected %zu", filter->size(), points.size());
    return false;
  }

  if ( !create_output_images(depths, mask) ) {
    CF_ERROR("create_output_images() fails");
    return false;
  }

  int r, c;

  if ( !filter || filter->empty() ) {
    for( const c_hdl_point &p : points ) {
      if( p.distance > 0 && project(p, &r, &c) ) {
        const double d = compute_depth(p);
        if( !depths[r][c] || d < depths[r][c] ) {
          depths[r][c] = d;
          if( mask ) {
            (*mask)[r][c] = 255;
          }
        }
      }
    }
  }
  else {
    for( uint i = 0, n = points.size(); i < n; ++i ) {
      if( (*filter)[i] ) {
        const c_hdl_point &p = points[i];
        if( p.distance > 0 && project(p, &r, &c) ) {
          const double d = compute_depth(p);
          if( !depths[r][c] || d < depths[r][c] ) {
            depths[r][c] = d;
            if( mask ) {
              (*mask)[r][c] = 255;
            }
          }
        }
      }
    }
  }

  return true;
}

/** build range image where each pixel is the horizontal distance (depth) to HDL point
 * scaled to 8 bit */
bool c_hdl_range_image::build_depths(const std::vector<c_hdl_point> & points, double max_distance,
    /* out */ cv::Mat1b & depths,
    /* in, opt */ const std::vector<uint8_t> * filter) const
{
  if( filter && !filter->empty() && filter->size() != points.size() ) {
    CF_ERROR("Invalid filter size=%zu, expected %zu", filter->size(), points.size());
    return false;
  }

  if ( !create_output_images(depths) ) {
    CF_ERROR("create_output_images() fails");
    return false;
  }

  const double distance_scale = 255 / max_distance;
  int r, c;

  if ( !filter || filter->empty() ) {

    for( const c_hdl_point &p : points ) {

      if( p.distance > 0 && projectncr(p, &r, &c) ) {

        const uint8_t d = cv::saturate_cast<uint8_t>(
            p.distance * distance_scale * cos_elevation_table_[p.laser_ring]);

        if( !depths[r][c] || d < depths[r][c] ) {
          depths[r][c] = d;
        }
      }
    }

  }
  else {
    for( uint i = 0, n = points.size(); i < n; ++i ) {
      if( (*filter)[i] ) {

        const c_hdl_point & p =
            points[i];

        if( p.distance > 0 && projectncr(p, &r, &c) ) {

          const uint8_t d = cv::saturate_cast<uint8_t>(p.distance * distance_scale *
              cos_elevation_table_[p.laser_ring]);

          if( !depths[r][c] || d < depths[r][c] ) {
            depths[r][c] = d;
          }
        }
      }
    }
  }

  return true;

}


/** build range image where each pixel is the height of HDL point  */
bool c_hdl_range_image::build_heights(const std::vector<c_hdl_point> & points,
    /* out*/cv::Mat1f & heights,
    /* out, opt */ cv::Mat1b * mask,
    const std::vector<uint8_t> * filter) const
{
  if( filter && !filter->empty() && filter->size() != points.size() ) {
    CF_ERROR("Invalid filter size=%zu, expected %zu", filter->size(), points.size());
    return false;
  }

  if( !create_output_images(heights, mask) ) {
    CF_ERROR("create_output_images() fails");
    return false;
  }

  cv::Mat1f distances;
  int r, c;

  distances.create(heights.size());
  distances.setTo(0);

  if ( !filter || filter->empty() ) {
    for( const c_hdl_point &p : points ) {
      if( p.distance > 0 && project(p, &r, &c) ) {
        if( !distances[r][c] || p.distance < distances[r][c] ) {
          distances[r][c] = p.distance;
          heights[r][c] = compute_height(p);
          if( mask ) {
            (*mask)[r][c] = 255;
          }
        }
      }
    }
  }
  else {
    for( uint i = 0, n = points.size(); i < n; ++i ) {
      if( (*filter)[i] ) {
        const c_hdl_point &p = points[i];
        if( p.distance > 0 && project(p, &r, &c) ) {
          if( !distances[r][c] || p.distance < distances[r][c] ) {
            distances[r][c] = p.distance;
            heights[r][c] = compute_height(p);
            if( mask ) {
              (*mask)[r][c] = 255;
            }
          }
        }
      }
    }
  }

  return true;
}

/** build range image where each pixel is the intensity of HDL point  */
bool c_hdl_range_image::build_intensity(const std::vector<c_hdl_point> & points,
    /* out*/cv::Mat1f & intensity,
    /* out, opt */cv::Mat1b * mask,
    const std::vector<uint8_t> * filter) const
{
  if( filter && !filter->empty() && filter->size() != points.size() ) {
    CF_ERROR("Invalid filter size=%zu, expected %zu", filter->size(), points.size());
    return false;
  }

  if( !create_output_images(intensity, mask) ) {
    CF_ERROR("create_output_images() fails");
    return false;
  }

  cv::Mat1f distances;
  int r, c;

  distances.create(intensity.size());
  distances.setTo(0);

  if ( !filter || filter->empty() ) {
    for( const c_hdl_point &p : points ) {
      if( p.distance > 0 && project(p, &r, &c) ) {
        if( !distances[r][c] || p.distance < distances[r][c] ) {
          distances[r][c] = p.distance;
          intensity[r][c] = p.intensity;
          if( mask ) {
            (*mask)[r][c] = 255;
          }
        }
      }
    }
  }
  else {
    for( uint i = 0, n = points.size(); i < n; ++i ) {
      if( (*filter)[i] ) {
        const c_hdl_point &p = points[i];
        if( p.distance > 0 && project(p, &r, &c) ) {
          if( !distances[r][c] || p.distance < distances[r][c] ) {
            distances[r][c] = p.distance;
            intensity[r][c] = p.intensity;
            if( mask ) {
              (*mask)[r][c] = 255;
            }
          }
        }
      }
    }
  }

  return true;
}


/** build range image where each pixel is the elevation of HDL point in radians */
bool c_hdl_range_image::build_elevations(const std::vector<c_hdl_point> & points,
    /* out*/ cv::Mat1f & elevations,
    /* out, opt */ cv::Mat1b * mask,
    const std::vector<uint8_t> * filter) const
{
  if( filter && !filter->empty() && filter->size() != points.size() ) {
    CF_ERROR("Invalid filter size=%zu, expected %zu", filter->size(), points.size());
    return false;
  }

  if( !create_output_images(elevations, mask) ) {
    CF_ERROR("create_output_images() fails");
    return false;
  }

  cv::Mat1f distances;
  int r, c;

  distances.create(elevations.size());
  distances.setTo(0);

  if ( !filter || filter->empty() ) {
    for( const c_hdl_point &p : points ) {
      if( p.distance > 0 && project(p, &r, &c) ) {
        if( !distances[r][c] || p.distance < distances[r][c] ) {
          distances[r][c] = p.distance;
          elevations[r][c] = p.elevation;
          if( mask ) {
            (*mask)[r][c] = 255;
          }
        }
      }
    }
  }
  else {
    for( uint i = 0, n = points.size(); i < n; ++i ) {
      if( (*filter)[i] ) {
        const c_hdl_point &p = points[i];
        if( p.distance > 0 && project(p, &r, &c) ) {
          if( !distances[r][c] || p.distance < distances[r][c] ) {
            distances[r][c] = p.distance;
            elevations[r][c] = p.elevation;
            if( mask ) {
              (*mask)[r][c] = 255;
            }
          }
        }
      }
    }
  }

  return true;
}

/** build range image where each pixel is the azimuth of HDL point in radians */
bool c_hdl_range_image::build_azimuths(const std::vector<c_hdl_point> & points,
    /* out*/ cv::Mat1f & azimuths,
    /* out, opt */ cv::Mat1b * mask,
    const std::vector<uint8_t> * filter) const
{
  if( filter && !filter->empty() && filter->size() != points.size() ) {
    CF_ERROR("Invalid filter size=%zu, expected %zu", filter->size(), points.size());
    return false;
  }

  if( !create_output_images(azimuths, mask) ) {
    CF_ERROR("create_output_images() fails");
    return false;
  }

  cv::Mat1f distances;
  int r, c;

  distances.create(azimuths.size());
  distances.setTo(0);

  if ( !filter || filter->empty() ) {
    for( const c_hdl_point &p : points ) {
      if( p.distance > 0 && project(p, &r, &c) ) {
        if( !distances[r][c] || p.distance < distances[r][c] ) {
          distances[r][c] = p.distance;
          azimuths[r][c] = p.azimuth;
          if( mask ) {
            (*mask)[r][c] = 255;
          }
        }
      }
    }
  }
  else {
    for( uint i = 0, n = points.size(); i < n; ++i ) {
      if( (*filter)[i] ) {
        const c_hdl_point &p = points[i];
        if( p.distance > 0 && project(p, &r, &c) ) {
          if( !distances[r][c] || p.distance < distances[r][c] ) {
            distances[r][c] = p.distance;
            azimuths[r][c] = p.azimuth;
            if( mask ) {
              (*mask)[r][c] = 255;
            }
          }
        }
      }
    }
  }

  return true;
}

/** build range image where each pixel is the laser_id of HDL point */
bool c_hdl_range_image::build_lazerids(const std::vector<c_hdl_point> & points,
    /* out*/ cv::Mat1b & lazerids,
    /* out, opt */ cv::Mat1b * mask,
    const std::vector<uint8_t> * filter) const
{
  if( filter && !filter->empty() && filter->size() != points.size() ) {
    CF_ERROR("Invalid filter size=%zu, expected %zu", filter->size(), points.size());
    return false;
  }

  if( !create_output_images(lazerids, mask) ) {
    CF_ERROR("create_output_images() fails");
    return false;
  }

  cv::Mat1f distances;
  int r, c;

  distances.create(lazerids.size());
  distances.setTo(0);

  if ( !filter || filter->empty() ) {
    for( const c_hdl_point &p : points ) {
      if( p.distance > 0 && project(p, &r, &c) ) {
        if( !distances[r][c] || p.distance < distances[r][c] ) {
          distances[r][c] = p.distance;
          lazerids[r][c] = p.laser_id;
          if( mask ) {
            (*mask)[r][c] = 255;
          }
        }
      }
    }
  }
  else {
    for( uint i = 0, n = points.size(); i < n; ++i ) {
      if( (*filter)[i] ) {
        const c_hdl_point &p = points[i];
        if( p.distance > 0 && project(p, &r, &c) ) {
          if( !distances[r][c] || p.distance < distances[r][c] ) {
            distances[r][c] = p.distance;
            lazerids[r][c] = p.laser_id;
            if( mask ) {
              (*mask)[r][c] = 255;
            }
          }
        }
      }
    }
  }

  return true;
}


/** build range image where each pixel is the laser_ring of HDL point */
bool c_hdl_range_image::build_lazer_rings(const std::vector<c_hdl_point> & points,
    /* out*/ cv::Mat1b & rings,
    /* out, opt */ cv::Mat1b * mask,
    const std::vector<uint8_t> * filter) const
{
  if( filter && !filter->empty() && filter->size() != points.size() ) {
    CF_ERROR("Invalid filter size=%zu, expected %zu", filter->size(), points.size());
    return false;
  }

  if( !create_output_images(rings, mask) ) {
    CF_ERROR("create_output_images() fails");
    return false;
  }

  cv::Mat1f distances;
  int r, c;

  distances.create(rings.size());
  distances.setTo(0);

  if ( !filter || filter->empty() ) {
    for( const c_hdl_point &p : points ) {
      if( p.distance > 0 && project(p, &r, &c) ) {
        if( !distances[r][c] || p.distance < distances[r][c] ) {
          distances[r][c] = p.distance;
          rings[r][c] = p.laser_ring;
          if( mask ) {
            (*mask)[r][c] = 255;
          }
        }
      }
    }
  }
  else {
    for( uint i = 0, n = points.size(); i < n; ++i ) {
      if( (*filter)[i] ) {
        const c_hdl_point &p = points[i];
        if( p.distance > 0 && project(p, &r, &c) ) {
          if( !distances[r][c] || p.distance < distances[r][c] ) {
            distances[r][c] = p.distance;
            rings[r][c] = p.laser_ring;
            if( mask ) {
              (*mask)[r][c] = 255;
            }
          }
        }
      }
    }
  }

  return true;

}


/** build range image where each pixel is the data block index of HDL point */
bool c_hdl_range_image::build_datablocks(const std::vector<c_hdl_point> & points,
    /* out*/ cv::Mat1b & datablocks,
    /* out, opt */ cv::Mat1b * mask,
    const std::vector<uint8_t> * filter) const
{
  if( filter && !filter->empty() && filter->size() != points.size() ) {
    CF_ERROR("Invalid filter size=%zu, expected %zu", filter->size(), points.size());
    return false;
  }

  if( !create_output_images(datablocks, mask) ) {
    CF_ERROR("create_output_images() fails");
    return false;
  }

  cv::Mat1f distances;
  int r, c;

  distances.create(datablocks.size());
  distances.setTo(0);

  if ( !filter || filter->empty() ) {
    for( const c_hdl_point &p : points ) {
      if( p.distance > 0 && project(p, &r, &c) ) {
        if( !distances[r][c] || p.distance < distances[r][c] ) {
          distances[r][c] = p.distance;
          datablocks[r][c] = p.datablock;
          if( mask ) {
            (*mask)[r][c] = 255;
          }
        }
      }
    }
  }
  else {
    for( uint i = 0, n = points.size(); i < n; ++i ) {
      if( (*filter)[i] ) {
        const c_hdl_point &p = points[i];
        if( p.distance > 0 && project(p, &r, &c) ) {
          if( !distances[r][c] || p.distance < distances[r][c] ) {
            distances[r][c] = p.distance;
            datablocks[r][c] = p.datablock;
            if( mask ) {
              (*mask)[r][c] = 255;
            }
          }
        }
      }
    }
  }

  return true;
}


/** build range image where each pixel is the packet index of HDL point */
bool c_hdl_range_image::build_pkts(const std::vector<c_hdl_point> & points,
    /* out*/ cv::Mat1i & pkts,
    /* out, opt */ cv::Mat1b * mask,
    const std::vector<uint8_t> * filter) const
{
  if( filter && !filter->empty() && filter->size() != points.size() ) {
    CF_ERROR("Invalid filter size=%zu, expected %zu", filter->size(), points.size());
    return false;
  }

  if( !create_output_images(pkts, mask) ) {
    CF_ERROR("create_output_images() fails");
    return false;
  }

  cv::Mat1f distances;
  int r, c;

  distances.create(pkts.size());
  distances.setTo(0);

  if ( !filter || filter->empty() ) {
    for( const c_hdl_point &p : points ) {
      if( p.distance > 0 && project(p, &r, &c) ) {
        if( !distances[r][c] || p.distance < distances[r][c] ) {
          distances[r][c] = p.distance;
          pkts[r][c] = p.pkt;
          if( mask ) {
            (*mask)[r][c] = 255;
          }
        }
      }
    }
  }
  else {
    for( uint i = 0, n = points.size(); i < n; ++i ) {
      if( (*filter)[i] ) {
        const c_hdl_point &p = points[i];
        if( p.distance > 0 && project(p, &r, &c) ) {
          if( !distances[r][c] || p.distance < distances[r][c] ) {
            distances[r][c] = p.distance;
            pkts[r][c] = p.pkt;
            if( mask ) {
              (*mask)[r][c] = 255;
            }
          }
        }
      }
    }
  }

  return true;
}

/** build range image where each pixel is the timestamp of HDL point in seconds */
bool c_hdl_range_image::build_timestamps(const std::vector<c_hdl_point> & points,
    /* out*/ cv::Mat1f & timestamps,
    /* out, opt */ cv::Mat1b * mask,
    const std::vector<uint8_t> * filter) const
{
  if( filter && !filter->empty() && filter->size() != points.size() ) {
    CF_ERROR("Invalid filter size=%zu, expected %zu", filter->size(), points.size());
    return false;
  }

  if( !create_output_images(timestamps, mask) ) {
    CF_ERROR("create_output_images() fails");
    return false;
  }

  cv::Mat1f distances;
  int r, c;

  distances.create(timestamps.size());
  distances.setTo(0);

  if ( !filter || filter->empty() ) {
    for( const c_hdl_point &p : points ) {
      if( p.distance > 0 && project(p, &r, &c) ) {
        if( !distances[r][c] || p.distance < distances[r][c] ) {
          distances[r][c] = p.distance;
          timestamps[r][c] = p.timestamp;
          if( mask ) {
            (*mask)[r][c] = 255;
          }
        }
      }
    }
  }
  else {
    for( uint i = 0, n = points.size(); i < n; ++i ) {
      if( (*filter)[i] ) {
        const c_hdl_point &p = points[i];
        if( p.distance > 0 && project(p, &r, &c) ) {
          if( !distances[r][c] || p.distance < distances[r][c] ) {
            distances[r][c] = p.distance;
            timestamps[r][c] = p.timestamp;
            if( mask ) {
              (*mask)[r][c] = 255;
            }
          }
        }
      }
    }
  }

  return true;
}



/** build range image where each pixel counts the number of aliased points */
bool c_hdl_range_image::build_aliasing(const std::vector<c_hdl_point> & points,
    /* out*/ cv::Mat1b & counter,
    const std::vector<uint8_t> * filter) const
{
  if( filter && !filter->empty() && filter->size() != points.size() ) {
    CF_ERROR("Invalid filter size=%zu, expected %zu", filter->size(), points.size());
    return false;
  }

  if( !create_output_images(counter) ) {
    CF_ERROR("create_output_images() fails");
    return false;
  }

  int r, c;

  if ( !filter || filter->empty() ) {
    for( const c_hdl_point &p : points ) {
      if( p.distance > 0 && project(p, &r, &c) ) {
        ++counter[r][c];
      }
    }
  }
  else {
    for( uint i = 0, n = points.size(); i < n; ++i ) {
      if( (*filter)[i] ) {
        const c_hdl_point &p = points[i];
        if( p.distance > 0 && project(p, &r, &c) ) {
          ++counter[r][c];
        }
      }
    }
  }

  return true;
}


/** build range image where each pixel is the cartesian coordintes (x,y,z) of HDL point  */
bool c_hdl_range_image::build_cartesizan(const std::vector<c_hdl_point> & points,
    /* out*/cv::Mat3f & cartesian,
    /* out, opt */cv::Mat1b * mask)
{
  if( !create_output_images(cartesian, mask) ) {
    CF_ERROR("create_output_images() fails");
    return false;
  }

  cv::Mat1f distances;
  int r, c;

  distances.create(cartesian.size());
  distances.setTo(0);

  for( const c_hdl_point &p : points ) {
    if( p.distance > 0 && project(p, &r, &c) ) {
      if( !distances[r][c] || p.distance < distances[r][c] ) {
        distances[r][c] = p.distance;
        cartesian[r][c] = compute_cartesian(p);
        if( mask ) {
          (*mask)[r][c] = 255;
        }
      }
    }
  }

  return true;
}

/** build range image where each pixel is the Cartesian coordinates and intensity (x,y,z, w) of HDL point  */
bool c_hdl_range_image::build_cartesizan(const std::vector<c_hdl_point> & points,
    /* out*/cv::Mat4f & cartesian,
    /* out, opt */cv::Mat1b * mask)
{
  if( !create_output_images(cartesian, mask) ) {
    CF_ERROR("create_output_images() fails");
    return false;
  }

  cv::Mat1f distances;
  int r, c;

  distances.create(cartesian.size());
  distances.setTo(0);

  for( const c_hdl_point &p : points ) {
    if( p.distance > 0 && project(p, &r, &c) ) {
      if( !distances[r][c] || p.distance < distances[r][c] ) {
        distances[r][c] = p.distance;
        cartesian[r][c] = compute_cartesian2(p);
        if( mask ) {
          (*mask)[r][c] = 255;
        }
      }
    }
  }

  return true;
}


/** build range image where each pixel is the local surface slope to the horizontal plane (in radians) */
bool c_hdl_range_image::build_gslopes(const std::vector<c_hdl_point> & points,
    /* out*/ cv::Mat1f & gslopes,
    /* out, opt */ cv::Mat1b * mask,
    /* out, opt */ cv::Mat1f * output_also_distances,
    /* out, opt */ cv::Mat1f * output_also_heights) const
{
  cv::Mat1f distances;

  if ( !build_distances(points, distances, mask) ) {
    CF_ERROR("build_distances() fails");
    return false;
  }

  gslopes.create(distances.size());
  gslopes.setTo(CV_PI / 2);

  if ( output_also_heights ) {
    output_also_heights->create(distances.size());
    output_also_heights->setTo(0);
  }

  static const auto next_col =
      [](int x, int y, const cv::Mat1f & mask) -> int {
        while ( x < mask.cols && !mask[y][x] ) {
          ++x;
        }
        return x;
      };

  static const auto prev_col =
      [](int x, int y, const cv::Mat1f & mask) -> int {
        while ( x >= 0 && !mask[y][x] ) {
          --x;
        }
        return x;
      };

  static const auto prev_row_wrapped =
      [](int x, int y, const cv::Mat1f & mask) -> int {

        const int y0 = y --;
        while ( y >= 0 && !mask[y][x] ) {
          --y;
        }

        if ( y < 0 ) {
          y += mask.rows;
          while ( y > y0 && !mask[y][x] ) {
            --y;
          }
          if ( y <= y0 ) {
            y = -1;
          }
        }

        return y;
      };

  static const auto next_row_wrapped =
      [](int x, int y, const cv::Mat1f & mask) -> int {

        const int y0 = y ++;
        while ( y < mask.rows && !mask[y][x] ) {
          ++y;
        }

        if ( y >= mask.rows ) {
          y = 0;
          while ( y < y0 && !mask[y][x] ) {
            ++y;
          }
          if ( y >= y0 ) {
            y = -1;
          }
        }

        return y;
      };

#if HAVE_TBB
  tbb::parallel_for(0, distances.rows,
      [this, &distances, &gslopes, output_also_heights ](int yc) {
#else
  for ( int yc = 0; yc < distances.rows; ++yc ) {
#endif

    double rp, hp, rc, hc, rn, hn;
    double dh, dr, c;

    const double ARES = azimuthal_resolution_;


    for ( int xc = 0; xc < distances.cols; ++xc ) {
      if ( distances[yc][xc] > 0 ) {

        if ( output_also_heights ) {
          (*output_also_heights)[yc][xc] =
              distances[yc][xc] * sin_elevation_table_[xc];
        }

        const int xp = prev_col(xc - 1, yc, distances);
        const int xn = next_col(xc + 1, yc, distances);

        if( xp < 0 && xn >= distances.cols ) {
          continue; // no any data in row
        }

        const int yp = prev_row_wrapped(xc, yc, distances);
        const int yn = next_row_wrapped(xc, yc, distances);

        if( yp >= 0 && yn >= 0 ) {
          const double drp = (distances[yc][xc] - distances[yp][xc]) * cos_elevation_table_[xc];
          const double drn = (distances[yn][xc] - distances[yc][xc]) * cos_elevation_table_[xc];
          const double wp = (1. / (0.1 + square(drp)));
          const double wn = (1. / (0.1 + square(drn)));
          const double cp = (1. + square(drp / (distances[yc][xc] * cos_elevation_table_[xc] * ARES)));
          const double cn = (1. + square(drn / (distances[yc][xc] * cos_elevation_table_[xc] * ARES)));
          c = (cp * wp + cn * wn) / (wp + wn);
        }
        else if( yp >= 0 ) {
          c = 1. + square(distances[yc][xc] - distances[yp][xc] / (distances[yc][xc] * ARES));
        }
        else if( yn >= 0 ) {
          c = 1. + square(distances[yn][xc] - distances[yc][xc] / (distances[yc][xc] * ARES));
        }
        else {
          c = 1.;
        }

        rc = distances[yc][xc];
        hc = distances[yc][xc] * sin_elevation_table_[xc];

        if( xp >= 0 ) {
          rp = distances[yc][xp];
          hp = distances[yc][xp] * sin_elevation_table_[xp];
        }
        else {
          rp = rc;
          hp = hc;
        }

        if( xn < distances.cols ) {
          rn = distances[yc][xn];
          hn = distances[yc][xn] * sin_elevation_table_[xn];
        }
        else {
          rn = distances[yc][xc];
          hn = distances[yc][xc] * sin_elevation_table_[xc];
        }

        const double wp = 1. / (0.1 + square(rc - rp));
        const double wn = 1. / (0.1 + square(rn - rc));
        const double w = 1. / (wp + wn);

        dh = ((hc - hp) * wp + (hn - hc) * wn) * w;
        dr = ((rc - rp) * wp + (rn - rc) * wn) * w;

        gslopes[yc][xc] =
            atan2(dh * sqrt(c), dr);
      }
    }
  }
#if HAVE_TBB
  );
#endif

  if( output_also_distances ) {
    *output_also_distances =
        std::move(distances);
  }

  return true;
}


/** build range image where each pixel is the local surface slope to the horizontal plane (in radians) */
bool c_hdl_range_image::depth2gslopes(const cv::Mat1f & depths,
    /* out*/ cv::Mat1f & gslopes,
    /* out, opt */ cv::Mat1f * output_also_heights) const
{

  if( depths.cols != sin_elevation_table_.size() ) {
    CF_ERROR("Input image size not match: depths=%dx%d sin_elevation_table_.size=%zu",
        depths.cols, depths.rows,
        sin_elevation_table_.size());
    return false;
  }

  gslopes.create(depths.size());
  gslopes.setTo(CV_PI / 2);

  if ( output_also_heights ) {
    output_also_heights->create(depths.size());
    output_also_heights->setTo(0);
  }

  static const auto next_col =
      [](int x, int y, const cv::Mat1f & mask) -> int {
        while ( x < mask.cols && !mask[y][x] ) {
          ++x;
        }
        return x;
      };

  static const auto prev_col =
      [](int x, int y, const cv::Mat1f & mask) -> int {
        while ( x >= 0 && !mask[y][x] ) {
          --x;
        }
        return x;
      };

  static const auto prev_row_wrapped =
      [](int x, int y, const cv::Mat1f & mask) -> int {

        const int y0 = y --;
        while ( y >= 0 && !mask[y][x] ) {
          --y;
        }

        if ( y < 0 ) {
          y += mask.rows;
          while ( y > y0 && !mask[y][x] ) {
            --y;
          }
          if ( y <= y0 ) {
            y = -1;
          }
        }

        return y;
      };

  static const auto next_row_wrapped =
      [](int x, int y, const cv::Mat1f & mask) -> int {

        const int y0 = y ++;
        while ( y < mask.rows && !mask[y][x] ) {
          ++y;
        }

        if ( y >= mask.rows ) {
          y = 0;
          while ( y < y0 && !mask[y][x] ) {
            ++y;
          }
          if ( y >= y0 ) {
            y = -1;
          }
        }

        return y;
      };

#if HAVE_TBB
  tbb::parallel_for(0, depths.rows,
      [this, &depths, &gslopes, output_also_heights ](int yc) {
#else
  for ( int yc = 0; yc < depths.rows; ++yc ) {
#endif

    double rp, hp, rc, hc, rn, hn;
    double dh, dr, c;

    const double ARES = azimuthal_resolution_;


    for ( int xc = 0; xc < depths.cols; ++xc ) {
      if ( depths[yc][xc] > 0 ) {

        if ( output_also_heights ) {
          (*output_also_heights)[yc][xc] =
              depths[yc][xc] * sin_elevation_table_[xc] / cos_elevation_table_[xc];
        }

        const int xp = prev_col(xc - 1, yc, depths);
        const int xn = next_col(xc + 1, yc, depths);

        if( xp < 0 && xn >= depths.cols ) {
          continue; // no any data in row
        }

        const int yp = prev_row_wrapped(xc, yc, depths);
        const int yn = next_row_wrapped(xc, yc, depths);

        if( yp >= 0 && yn >= 0 ) {
          const double drp = (depths[yc][xc] - depths[yp][xc]);
          const double drn = (depths[yn][xc] - depths[yc][xc]);
          const double wp = (1. / (0.1 + square(drp)));
          const double wn = (1. / (0.1 + square(drn)));
          const double cp = (1. + square(drp / (depths[yc][xc] * ARES)));
          const double cn = (1. + square(drn / (depths[yc][xc] * ARES)));
          c = (cp * wp + cn * wn) / (wp + wn);
        }
        else if( yp >= 0 ) {
          c = 1. + square(depths[yc][xc] - depths[yp][xc] / (depths[yc][xc] * ARES));
        }
        else if( yn >= 0 ) {
          c = 1. + square(depths[yn][xc] - depths[yc][xc] / (depths[yc][xc] * ARES));
        }
        else {
          c = 1.;
        }

        rc = depths[yc][xc];
        hc = depths[yc][xc] * sin_elevation_table_[xc] / cos_elevation_table_[xc];

        if( xp >= 0 ) {
          rp = depths[yc][xp];
          hp = depths[yc][xp] * sin_elevation_table_[xp] / cos_elevation_table_[xp];
        }
        else {
          rp = rc;
          hp = hc;
        }

        if( xn < depths.cols ) {
          rn = depths[yc][xn];
          hn = depths[yc][xn] * sin_elevation_table_[xn] / cos_elevation_table_[xn];
        }
        else {
          rn = depths[yc][xc];
          hn = depths[yc][xc] * sin_elevation_table_[xc] / cos_elevation_table_[xc];
        }

        const double wp = 1. / (0.1 + square(rc - rp));
        const double wn = 1. / (0.1 + square(rn - rc));
        const double w = 1. / (wp + wn);

        dh = ((hc - hp) * wp + (hn - hc) * wn) * w;
        dr = ((rc - rp) * wp + (rn - rc) * wn) * w;

        gslopes[yc][xc] =
            atan2(dh * sqrt(c), dr);
      }
    }
  }
#if HAVE_TBB
  );
#endif

  return true;
}


/** build range images required for c_hdl_ground_filter */
bool c_hdl_range_image::build_image_for_ground_detection(const std::vector<c_hdl_point> & points, double sensor_height,
    /* out */ cv::Mat3f & dhs,
    /* out */ cv::Mat1b & mask,
    /* in, opt */ const std::vector<uint8_t> * filter,
    /* out, opt */ std::vector<cv::Point> * projected_coords)
{

  INSTRUMENT_REGION("");

  static const auto next_col =
      [](int x, const cv::Vec3f dhs[], int n) -> int {
        while ( x < n && !dhs[x][0] ) {
          ++x;
        }
        return x;
      };

  if( filter && !filter->empty() && filter->size() != points.size() ) {
    CF_ERROR("Invalid filter size: %zu. points.size()=%zu",
        filter->size(), points.size());
    return false;
  }


  if( !create_output_images(dhs, &mask) ) {
    CF_ERROR("create_output_images() fails");
    return false;
  }

  if ( projected_coords ) {
    projected_coords->clear();
    projected_coords->resize(points.size());
  }

  if( !filter || filter->empty() ) {

    for( uint i = 0, n = points.size(); i < n; ++i ) {

      const c_hdl_point &p = points[i];
      int r = -1, c = -1;

      if( p.distance > 0 && projectncr(p, &r, &c) ) {
        if( !dhs[r][c][0] || p.distance < dhs[r][c][0] ) {
          dhs[r][c][0] = p.distance;
        }
      }

      if( projected_coords ) {
        cv::Point &pp = (*projected_coords)[i];
        pp.y = r;
        pp.x = c;
      }
    }
  }
  else {
    for( uint i = 0, n = points.size(); i < n; ++i ) {

      int r = -1, c = -1;

      if( (*filter)[i] ) {
        const c_hdl_point &p = points[i];

        if( p.distance > 0 && projectncr(p, &r, &c) ) {
          if( !dhs[r][c][0] || p.distance < dhs[r][c][0] ) {
            dhs[r][c][0] = p.distance;
          }
        }
      }

      if( projected_coords ) {
        cv::Point &pp = (*projected_coords)[i];
        pp.x = c;
        pp.y = r;
      }
    }
  }

#if HAVE_TBB
  tbb::parallel_for(range(0, dhs.rows, 64),
      [this, &dhs, &mask](const range & r) {
  for ( int yc = r.begin(), ny = r.end(); yc < ny; ++yc ) {
#else
  for( int yc = 0; yc < dhs.rows; ++yc ) {
#endif
          for( int xc = 0, nc = dhs.cols; xc < nc; ++xc ) {
            const float d = dhs[yc][xc][0];
            if ( d > 0 ) {
              dhs[yc][xc][0] = d * cos_elevation_table_[xc];
              dhs[yc][xc][1] = d * sin_elevation_table_[xc];
              dhs[yc][xc][2] = (float)(CV_PI / 2);
              mask[yc][xc] = 255;
            }
          }
        }
#if HAVE_TBB
  });
#endif

  const float SH = sensor_height;

#if HAVE_TBB
  tbb::parallel_for(range(0, dhs.rows, 64),
      [&dhs, SH](const range & r) {
  for ( int yc = r.begin(), ny = r.end(); yc < ny; ++yc ) {
#else
  for( int yc = 0; yc < dhs.rows; ++yc ) {
#endif

    const int nx = dhs.cols;
    cv::Vec3f * scanline = dhs[yc];

    int xc, xp;
    float rp, hp, rc, hc;
    float dh, dr;

    xp = -1;
    hp = -SH;
    rp = 0;

    for( ; (xc = next_col(xp + 1, scanline, nx)) < nx; xp = xc, rp = rc, hp = hc) {

      rc = scanline[xc][0];
      hc = scanline[xc][1];

      dh = (hc - hp);
      dr = (rc - rp);

      scanline[xc][2] =
        atan2f(dh , dr);
    }
  }
#if HAVE_TBB
  });
#endif


  return true;
}

/** build range images required for c_hdl_ground_filter */
bool c_hdl_range_image::build_images_for_ground_detection(const std::vector<c_hdl_point> & points, double sensor_height,
    /* out */ cv::Mat1f & depths,
    /* out */ cv::Mat1f & heights,
    /* out */ cv::Mat1f & slopes,
    /* out */ cv::Mat1b & mask,
    /* in, opt */ const std::vector<uint8_t> * filter,
    /* out, opt */ std::vector<cv::Point> * projected_coords)
{

  INSTRUMENT_REGION("");

  static const auto next_col =
      [](int x, const float depths[], int n) -> int {
        while ( x < n && !depths[x] ) {
          ++x;
        }
        return x;
      };

  if( filter && !filter->empty() && filter->size() != points.size() ) {
    CF_ERROR("Invalid filter size: %zu. points.size()=%zu",
        filter->size(), points.size());
    return false;
  }


  if( !create_output_images(depths, &mask) ) {
    CF_ERROR("create_output_images(depths) fails");
    return false;
  }

  if ( projected_coords ) {
    projected_coords->clear();
    projected_coords->resize(points.size());
  }

  if( !filter || filter->empty() ) {

    for( uint i = 0, n = points.size(); i < n; ++i ) {

      const c_hdl_point &p = points[i];
      int r = -1, c = -1;

      if( p.distance > 0 && projectncr(p, &r, &c) ) {
        if( !depths[r][c] || p.distance < depths[r][c] ) {
          depths[r][c] = p.distance;
        }
      }

      if( projected_coords ) {
        cv::Point &pp = (*projected_coords)[i];
        pp.y = r;
        pp.x = c;
      }
    }
  }
  else {
    for( uint i = 0, n = points.size(); i < n; ++i ) {

      int r = -1, c = -1;

      if( (*filter)[i] ) {
        const c_hdl_point &p = points[i];

        if( p.distance > 0 && projectncr(p, &r, &c) ) {
          if( !depths[r][c] || p.distance < depths[r][c] ) {
            depths[r][c] = p.distance;
          }
        }
      }

      if( projected_coords ) {
        cv::Point &pp = (*projected_coords)[i];
        pp.x = c;
        pp.y = r;
      }
    }
  }

  heights.create(depths.size()), heights.setTo(0);

#if HAVE_TBB
  tbb::parallel_for(range(0, depths.rows, 64),
      [this, &depths, &heights, &mask](const range & r) {
  for ( int yc = r.begin(), ny = r.end(); yc < ny; ++yc ) {
#else
  for( int yc = 0; yc < depths.rows; ++yc ) {
#endif
          for( int xc = 0, nc = depths.cols; xc < nc; ++xc ) {
            const float d = depths[yc][xc];
            if ( d > 0 ) {
              depths[yc][xc] = d * cos_elevation_table_[xc];
              heights[yc][xc] = d * sin_elevation_table_[xc];
              mask[yc][xc] = 255;
            }
          }
        }
#if HAVE_TBB
  });
#endif

  const float SH = sensor_height;

  slopes.create(depths.size()), slopes.setTo(CV_PI / 2);

#if HAVE_TBB
  tbb::parallel_for(range(0, depths.rows, 64),
      [&depths, &heights, &slopes, SH](const range & r) {
  for ( int yc = r.begin(), ny = r.end(); yc < ny; ++yc ) {
#else
  for( int yc = 0; yc < dhs.rows; ++yc ) {
#endif

    const int nx = depths.cols;
    const float * dptr = depths[yc];
    const float * hptr = heights[yc];
    float * sptr = slopes[yc];

    int xc, xp;
    float rp, hp, rc, hc;
    float dh, dr;

    xp = -1;
    hp = -SH;
    rp = 0;

    for( ; (xc = next_col(xp + 1, dptr, nx)) < nx; xp = xc, rp = rc, hp = hc) {

      rc = dptr[xc];
      hc = hptr[xc];

      dh = (hc - hp);
      dr = (rc - rp);

      sptr[xc] = atan2f(dh, dr);
    }
  }
#if HAVE_TBB
  });
#endif


  return true;
}

void c_hdl_range_image::median_inpaint(cv::InputOutputArray image, int kradius)
{
  INSTRUMENT_REGION("");

  cv::Mat median_image, mask;

  cv::medianBlur(image,
      median_image,
      2 * kradius + 1);

  cv::compare(image, 0, mask,
      cv::CMP_LE);

  median_image.copyTo(image,
      mask);
}
