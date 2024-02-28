/*
 * c_hdl_frame.h
 *
 *  Created on: Mar 15, 2022
 *      Author: amyznikov
 */

#ifndef __c_hdl_frame_h__
#define __c_hdl_frame_h__

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include <cmath>

/**
 * Additional optional flags for each lidar point, like dual return etc.
 */
enum c_hdl_point_flags
{
};

/**
 * single lidar point as received from packet.
 * */
struct c_hdl_point
{
  int laser_id;
  int laser_ring;
  int pkt;
  int datablock;
  int flags;
  double azimuth;
  double elevation;
  double distance;
  double intensity;
  double timestamp;
};

/**
 * HDL frame is the list of HDL points collected during single lidar rotation
 * */
class c_hdl_frame
{
public:
  typedef c_hdl_frame this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  // sequential index of frame in source stream
  size_t index = 0;

//  // source hash, typically intended for multi-lidar processing
//  // to distinct frames originated from different lidars
//  uint64_t source = 0;

// array of acquired lidar points (point cloud)
  std::vector<c_hdl_point> points;
};

/**
 * lidar point cloud represented in Cartesian XYZ coordinates
 * */
class c_hdl_point_cloud
{
public:
  typedef c_hdl_point_cloud this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  // x, y, z, r, t, flag
  std::vector<cv::Vec6f> points;
};

/**
 * convert spherical (raw) lidar points to Cartesian coordinates
 */
template<class T>
void convert_to_cartesian(const std::vector<c_hdl_point> & lidar_points, std::vector<cv::Vec<T, 3>> & positions)
{
  positions.clear();
  for( const c_hdl_point & p : lidar_points ) {
    if( p.distance <= 0 ) {
      positions.emplace_back(0, 0, 0);
    }
    else {
      positions.emplace_back(
          p.distance * std::cos(p.elevation) * std::sin(p.azimuth),
          p.distance * std::cos(p.elevation) * std::cos(p.azimuth),
          p.distance * std::sin(p.elevation));
    }
  }
}

/**
 * convert spherical (raw) lidar points to Cartesian coordinates
 */
inline void convert_to_cartesian(const std::vector<c_hdl_point> & hdl_points, cv::OutputArray output_positions)
{
  output_positions.create(hdl_points.size(), 1,
      CV_32FC3);

  cv::Mat3f positions =
      output_positions.getMatRef();

  for( int i = 0, n = hdl_points.size(); i < n; ++i ) {

    const c_hdl_point & p =
        hdl_points[i];

    if( p.distance <= 0 ) {
      positions[i][0][0] = 0;
      positions[i][0][1] = 0;
      positions[i][0][2] = 0;
    }
    else {
      positions[i][0][0] = p.distance * std::cos(p.elevation) * std::sin(p.azimuth);
      positions[i][0][1] = p.distance * std::cos(p.elevation) * std::cos(p.azimuth);
      positions[i][0][2] = p.distance * std::sin(p.elevation);
    }
  }
}

/**
 * convert spherical (raw) lidar points to Cartesian coordinates with time stamps in sec
 */
template<class T>
void convert_to_cartesian(const std::vector<c_hdl_point> & hdl_points,
    std::vector<cv::Vec<T, 4>> & positions_with_timestamps)
{
  positions_with_timestamps.clear();
  for( const c_hdl_point & p : hdl_points ) {
    if( p.distance <= 0 ) {
      positions_with_timestamps.emplace_back(0, 0, p.timestamp * 1e-6);
    }
    else {
      positions_with_timestamps.emplace_back(
          p.distance * std::cos(p.elevation) * std::sin(p.azimuth),
          p.distance * std::cos(p.elevation) * std::cos(p.azimuth),
          p.distance * std::sin(p.elevation),
          p.timestamp * 1e-6);
    }
  }
}

inline cv::Vec3f depth_to_cartesian(double depth, double azimuth, double elevation)
{
  return depth > 0 ?
      cv::Vec3f(depth * sin(azimuth),
          depth * cos(azimuth),
          depth * tan(elevation)) :
      cv::Vec3f::all(0);
}

inline cv::Vec3f compute_cartesian(const c_hdl_point & p)
{
  return p.distance > 0 ?
      cv::Vec3f(p.distance * cos(p.elevation) * sin(p.azimuth),
          p.distance * cos(p.elevation) * cos(p.azimuth),
          p.distance * sin(p.elevation)) :
      cv::Vec3f::all(0);
}

inline double compute_depth(const c_hdl_point & p)
{
  return p.distance > 0 ? p.distance * cos(p.elevation) : 0;
}

inline double compute_distance(const c_hdl_point & p)
{
  return p.distance;
}

inline double compute_height(const c_hdl_point & p)
{
  return p.distance > 0 ? p.distance * sin(p.elevation) : 0;
}

inline double compute_intensity(const c_hdl_point & p)
{
  return p.intensity;
}

inline double compute_elevation(const c_hdl_point & p)
{
  return p.elevation;
}

inline double compute_azimuth(const c_hdl_point & p)
{
  return p.azimuth;
}

inline double compute_laser_id(const c_hdl_point & p)
{
  return p.laser_id;
}

inline double compute_datablock(const c_hdl_point & p)
{
  return p.datablock;
}

inline double compute_packet(const c_hdl_point & p)
{
  return p.pkt;
}

inline double compute_tstamp(const c_hdl_point & p)
{
  return p.timestamp;
}



#endif /* __c_hdl_frame_h__ */
