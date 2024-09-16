/*
 * c_ego_motion_correction.h
 *
 *  Created on: Oct 10, 2018
 *      Authors: amyznikov, kolyarage
 *
 * @see Jens Rieken and Markus Maurer,
 *    Sensor scan timing compensation in environment models for automated road vehicles
 *
 * @see Pierre Merriaux, Yohan Dupuis et. all,
 *    LiDAR point clouds correction acquired from a moving car based on CAN-bus data
 *
 *
 * The purpose of this module is to provide EGO motion compensation (correction)
 * for individual point clouds acquired from moving rotating LIDAR.
 *
 *  It is well known that point cloud acquired from moving rotating LIDAR
 *  is affected by deformations caused by EGO LIDAR movement
 *  and varying orientation in yaw, pitch and roll.
 *
 *  The routines in this module use the GPS/IMU (OxTS) information
 *  for car trajectory estimation during EGO movement within
 *  single LIDAR frame (typically 100 ms for 10Hz rotation rate).
 *
 *  Estimated car trajectory is then used to correct coordinates of individual points.
 *
 *  The most convenient routine for usage by callers is the c_ego_motion_correction::compensate_ego_motion().
 *  On input it accepts raw (deformed by moving car) point cloud, on output - the same cloud with
 *  point coordinates corrected for EGO motion.
 *
 * The rest of routines are auxiliary, user usually SHOULD NOT call them directly in sprite of they are
 * declared in public interface. They MAY be used for some special purposes, experimentations or debugs.
 *
 *
 * POINT TYPES:
 *  The routines in this module are templates parametrized by conventional PointType as individual point representation.
 *  The only MANDATORY requirement for PointType is to have floating-point public fields named x, y, and z.
 *
 *  It is also DESIRABLE to have double-precision field named 'ts', representing individual point time stamp,
 *  but for the cases when this field is unavailable for your PointType, the time stamps may be passed
 *  via external array argument.
 *
 *
 *
 * COORDINATE AND TIME SYSTEMS:
 *  It is CRITICALY important to provide input clouds given in correct coordinate systems.
 *  The output of routines is UNDEFINED if input coordinates are given in incorrect coordinate system.
 *
 *  The cloud coordinate system SHOULD be local ISO8855 VS located at IMU box zero point,
 *  and oriented exactly as in KITTI OxTS. This requirement is because of the way
 *  how we utilize the OxTS track information for car trajectory estimation.
 *
 *    ** X axis -> forward (usually in direction of car movement)
 *    ** Y axis -> left
 *    ** Z axis -> up
 *
 *  The unit of coordinate measurement MUST be meters.
 *  The unit of time measurements MUST be seconds.
 *  The velocities in oxts[] data MUST be meters per second (m/s).
 *  The angles in oxts[] data MUST be in radians (see comments to oxts_data members).
 *
 *
 *  In order to transform original point cloud (given in Velodyne coordinate system) to IMU coordinate system
 *  the utility class c_imu2velo CAN be used. Note also that for KITTI datasets the mapping between
 *  IMU and Velodyne coordinate systems is given in the 'calib_imu_to_velo.txt' file.
 *  For Zenuity data we guessed this mapping from very limited information we have, the matrix and translation
 *  vector is hardcoded in zu_dataset C++ class constructor.
 *
 *  The example code how to do this is present below.
 *
 * IMPLEMENTATION NUANCES:
 *  For performance reasons the points in the cloud are grouped into several groups
 *  according to their time stamps. Each group of points is corrected as whole.
 *
 *  There is parameter which controls this grouping, it is named 'time_step'.
 *
 *  All points with time stamp differences within specified time_step interval
 *  are considered as single group of points, and the correction is computed
 *  for middle time stamp of the group.
 *
 *  The default time_step value is 1 ms which is more than enough for total majority of use cases.
 *
 *  If you are extremal man and you indeed want to compute individual correction for each individual point,
 *  then you can specify time_step = DBL_EPSILON, and get the correction for each point
 *  individually.
 *
 *
 *
 *
 * USAGE:
 *  There are several overloads to the conventional cloud correction routine compensate_ego_motion().
 *  User may call any appropriate for his situation.
 *  See detailed description of arguments for each particular overload.
 *
 *  The conventional usage steps are follows:
 *
 *  1) Declare the c_ego_motion_correction object instance
 *
 *    c_ego_motion_correction emc;
 *    c_imu2velo imu2velo; // for conversion to IMU system
 *
 *  2) Setup emc oxts[] data pointer, and velo->imu calibration:
 *
 *   imu2velo.set_imu2velo(my_dataset->imu2velo.R, my_dataset->imu2velo.T); // reverse mapping will computed automatically
 *   emc.set_oxts_data(&my_dataset->oxts);
 *
 * 3) Optional: setup grouping time interval:
 *
 emc.set_time_step(0.01); // 10 msec
 *
 * 4) Use appropriate from several overloads of emc.compensate_ego_motion():
 *
 *    std::vector<lidar_point> my_cloud;
 *    my_dataset->frames[i]->load_points(&my_cloud);
 *
 *    imu2velo.convert_to_imu(my_cloud);
 *    emc.compensate_ego_motion(my_cloud, my_dataset->frames[i]->ts);
 *    imu2velo.convert_to_velo(my_cloud); // optionally convert back to velodyne system, if required
 *
 *
 *    std::vector<lidar_point> origina_cloud, corrected_cloud;
 *
 *    my_dataset->frames[i]->load_points(&origina_cloud);
 *
 *    imu2velo.convert_to_imu(origina_cloud, corrected_cloud);
 *    emc.compensate_ego_motion(corrected_cloud, my_dataset->frames[i]->ts);
 *    imu2velo.convert_to_velo(corrected_cloud); // optionally convert back to velodyne system, if required
 *
 *
 *    And so on...
 */

#pragma once
#ifndef __c_ego_motion_correction_h__
#define __c_ego_motion_correction_h__

#include <string>
#include <numeric>
#include <algorithm>

#include <core/io/kitti/oxts.h>
#include <core/proc/gps/gps.h>
#include <core/debug.h>

//! @addtogroup emc
//! @{

/**
 * @brief Several methods exist to estimate the car trejectory from
 * vehicular motion estimations. Runge-Kutta method can be regarded
 * as the most computationally efficient and stable method.
 * But for the cases of lack precise IMU accelerators data
 * the pure GPS-based position estimator is added too
 */
enum TRAJECTORY_ESTIMATION_METHOD
{
  TRAJECTORY_ESTIMATION_INVALID = -1,

  // stub for silently skip ego motion compensation
  TRAJECTORY_ESTIMATION_NONE = 0,

  // uses only gps positions lat, lon, alt, and orientation roll, pitch, yaw for trajectory estimation
  TRAJECTORY_ESTIMATION_GPS = 1,

  // uses only velocities vf, vl, vu  and orientation roll, pitch, yaw for trajectory estimation,
  // simplified linear motion approximation (does not account the changes in roll, pitch, yaw during  LIDAR frame acquisition)
  TRAJECTORY_ESTIMATION_IMU = 2,

  // Euler integration of vf, vl, vu  and orientation roll, pitch, yaw for trajectory estimation,
  // uses oxts[] measurements within single LIDAR frame if available for more precise trajectory estimation.
  // Note that the Norway trace example does not expose strong yaws, therefore resulting trajectory
  // is mostly identical to one build from simplified linear approximation TRAJECTORY_ESTIMATION_IMU.
  TRAJECTORY_ESTIMATION_IMUE = 3,

// FIXME: add combined GPS + IMU trajectory estimation method for further improvements
};

/**
 * @brief Car trajectory estimation and LIDAR point cloud correction
 @see Jens Rieken and Markus Maurer,
 *    Sensor scan timing compensation in environment models for automated road vehicles
 * @see Pierre Merriaux, Yohan Dupuis et. all,
 *    LiDAR point clouds correction acquired from a moving car based on CAN-bus data
 *
 *  @paragraph module MODULE PURPOSES
 *  The purpose of this module is to provide EGO motion compensation (correction)
 *  for individual point clouds acquired from moving rotating LIDAR.
 *  It is well known that point cloud acquired from moving rotating LIDAR
 *  is affected by deformations caused by EGO LIDAR movement
 *  and varying orientation in yaw, pitch and roll.
 *  The routines in this module use the GPS/IMU (OxTS) information
 *  for car trajectory estimation during EGO movement within
 *  single LIDAR frame (typically 100 ms for 10Hz rotation rate).
 *  Estimated car trajectory is then used to correct coordinates of individual points.
 *  The most convenient routine for usage by callers is the c_ego_motion_correction::compensate_ego_motion().
 *  On input it accepts raw (deformed by moving car) point cloud, on output - the same cloud with
 *  point coordinates corrected for EGO motion.
 *  The rest of routines are auxiliary, user usually SHOULD NOT call them directly in sprite of they are
 *  declared in public interface. They MAY be used for some special purposes, experimentations or debugs.
 *
 *  @paragraph point POINT TYPES:
 *  The routines in this module are templates parametrized by conventional PointType as individual point representation.
 *  The only MANDATORY requirement for PointType is to have floating-point public fields named x, y, and z.
 *  It is also DESIRABLE to have double-precision field named 'ts', representing individual point time stamp,
 *  but for the cases when this field is unavailable for your PointType, the time stamps may be passed
 *  via external array argument.
 *
 *  @paragraph coord COORDINATE AND TIME SYSTEMS:
 *  It is CRITICALY important to provide input clouds given in correct coordinate systems.
 *  The output of routines is UNDEFINED if input coordinates are given in incorrect coordinate system.
 *  The cloud coordinate system SHOULD be local ISO8855 VS located at IMU box zero point,
 *  and oriented exactly as in KITTI OxTS. This requirement is because of the way
 *  how we utilize the OxTS track information for car trajectory estimation.
 *  X axis -> forward (usually in direction of car movement)
 *  Y axis -> left
 *  Z axis -> up
 *  The unit of coordinate measurement MUST be meters.
 *  The unit of time measurements MUST be seconds.
 *  The velocities in oxts[] data MUST be meters per second (m/s).
 *  The angles in oxts[] data MUST be in radians (see comments to oxts_data members).
 *  In order to transform original point cloud (given in Velodyne coordinate system) to IMU coordinate system
 *  the utility class c_imu2velo CAN be used. Note also that for KITTI datasets the mapping between
 *  IMU and Velodyne coordinate systems is given in the 'calib_imu_to_velo.txt' file.
 *  For Zenuity data we guessed this mapping from very limited information we have, the matrix and translation
 *  vector is hardcoded in zu_dataset C++ class constructor.
 *  The example code how to do this is present below.
 *
 *  @paragraph nuances IMPLEMENTATION NUANCES:
 *  For performance reasons the points in the cloud are grouped into several groups
 *  according to their time stamps. Each group of points is corrected as a whole.
 *  There is parameter which controls this grouping, it is named 'time_step'.
 *  All points with time stamp differences within specified time_step interval
 *  are considered as single group of points, and the correction is computed
 *  for middle time stamp of the group.
 *  The default time_step value is 1 ms which is more than enough for total majority of use cases.
 *  If you are extremal man and you indeed want to compute individual correction for each individual point,
 *  then you can specify time_step = DBL_EPSILON, and get the correction for each point
 *  individually.
 *
 *  @paragraph usage USAGE:
 *  There are several overloads to the conventional cloud correction routine compensate_ego_motion().
 *  User may call any appropriate for his situation.
 *  See detailed description of arguments for each particular overload.
 *  The conventional usage steps are follows:
 *
 *  @code 1) Declare the c_ego_motion_correction object instance
 *
 *     c_ego_motion_correction emc;
 *     c_imu2velo imu2velo; // for conversion to IMU system
 *
 *  2) Setup emc oxts[] data pointer, and velo->imu calibration:
 *
 *     imu2velo.set_imu2velo(my_dataset->imu2velo.R, my_dataset->imu2velo.T); // reverse mapping will computed automatically
 *     emc.set_oxts_data(&my_dataset->oxts);
 *
 *  3) Optional: setup grouping time interval:
 *
 *     emc.set_time_step(0.01); // 10 msec
 *
 *  4) Use appropriate from several overloads of emc.compensate_ego_motion():
 *
 *     std::vector<lidar_point> my_cloud;
 *     my_dataset->frames[i]->load_points(&my_cloud);
 *
 *     imu2velo.convert_to_imu(my_cloud);
 *     emc.compensate_ego_motion(my_cloud, my_dataset->frames[i]->ts);
 *     imu2velo.convert_to_velo(my_cloud); // optionally convert back to velodyne system, if required
 *
 *
 *     std::vector<lidar_point> origina_cloud, corrected_cloud;
 *
 *     my_dataset->frames[i]->load_points(&origina_cloud);
 *
 *     imu2velo.convert_to_imu(origina_cloud, corrected_cloud);
 *     emc.compensate_ego_motion(corrected_cloud, my_dataset->frames[i]->ts);
 *     imu2velo.convert_to_velo(corrected_cloud); // optionally convert back to velodyne system, if required
 *     @endcode
 *
 *
 *    And so on...
 **/
class c_ego_motion_compensation
{
  double time_step_ = 1e-4; // 100 us
  enum TRAJECTORY_ESTIMATION_METHOD trajectory_estimation_method_ = TRAJECTORY_ESTIMATION_GPS;
  const std::vector<oxts_data> * trajectory_ = nullptr;
  //double tts_offset_ = 0; // [sec]

  // Forward mapping IMU -> VELO
  cv::Vec3d imu2velo_translation_ = cv::Vec3d::all(0);
  cv::Matx33d imu2velo_rotation_ = cv::Matx33d::eye();
  // Reverse mapping VELO -> IMU
  cv::Vec3d velo2imu_translation_ = cv::Vec3d::all(0);
  cv::Matx33d velo2imu_rotation_ = cv::Matx33d::eye();

  // Use VELO as input / output
  bool enable_imu2velo_ = false;

  struct c_index_entry
  {
    double ts;
    size_t p;
    c_index_entry(double _ts, size_t _p) : ts(_ts), p(_p)
    {
    }
    static bool less_by_time(const c_index_entry & lhs, const c_index_entry & rhs)
    {
      return lhs.ts < rhs.ts;
    }
  };

  typedef std::vector<c_index_entry>
  PointIndex;

public: // properties
  /**
   * @brief Specify OxTS data array which wll be used for trajectory estimation
   * The OxTS array MUST be set BEFORE any call to any of compensate_ego_motion() methods.
   * the oxts[] array MUST be already sorted by time stamp, otherwise the result of
   * ego motion compensation is UNDEFINED.
   *
   * The conventional way to set oxts_data pointer is to get it from lidar_dataset,
   * where it is correctly sorted by default.
   * @code
   *   c_ego_motion_correction emc;
   *   emc.set_oxts_data(&my_dataset->oxts);
   * @endcode
   */
  void set_trajectory(const std::vector<oxts_data> * trajectory)
  {
    trajectory_ = trajectory;
  }

  /**
   * @brief Get pointer to currently used oxts_data[] array
   * @return vector of oxts_data structures
   */
  const std::vector<oxts_data>* trajectory() const
  {
    return trajectory_;
  }

  /**
   * @brief Specify time interval in seconds used to group individual points according to time stamp.
   *
   * Default 1 ms is MORE THAN enough for total majority of use cases.
   * Set this interval to 5ms or 10 ms for better performance for the cost
   * of some (actually small) loss of precision.
   * The time measurement unit is seconds (sec).
   * @param ts time step interval
   */
  void set_time_step(double ts)
  {
    time_step_ = ts;
  }

  ///@brief Get current time_step value.
  double time_step() const
  {
    return time_step_;
  }

  /**
   * @brief Specify one of implemented car trajectory estimation methods.
   * For more detailed discussion see comments to enum TRAJETORY_ESTIMATION_METHOD.
   * @param v trajectory estimation method
   */
  void set_trajectory_estimation_method(enum TRAJECTORY_ESTIMATION_METHOD v)
  {
    trajectory_estimation_method_ = v;
  }

  ///@brief Get current method used for car trajectory estimation.
  enum TRAJECTORY_ESTIMATION_METHOD trajectory_estimation_method() const
  {
    return trajectory_estimation_method_;
  }

  /**
   * @brief Use VELO as input / output coordinate system
   * If this option is set to true, then The IMU <--> VELO mapping must be set too
   * @param true if IMU <--> VELO mapping enabled, false otherwise
   */
  void set_enable_imu2velo(bool v)
  {
    enable_imu2velo_ = v;
  }

  /**
   * @brief get if IMU <--> VELO mapping was enabled
   * @return true if IMU <--> VELO mapping enabled
   */
  bool enable_imu2velo() const
  {
    return enable_imu2velo_;
  }

  /**
   * @brief Sets the mapping from IMU system to VELODYNE
   *
   * The mapping is assumed to be rotation followed by positive translation like this:
   * velo_pos = imu2velo_rotation * imu_pos + imu2velo_translation
   * @param rotation 3x3 rotation IMU <--> VELO matrix
   * @param translation 3d IMU <--> VELO vector translation
   * @param update_reverse true if enables computing of reverse transform automatically
   * as inverse of provided rotation and translation vector
   */
  void set_imu2velo(const cv::Matx33d & rotation, const cv::Vec3d & translation, bool update_reverse = true)
  {
    imu2velo_rotation_ = rotation;
    imu2velo_translation_ = translation;
    if( update_reverse ) {
      velo2imu_rotation_ = rotation.inv();
      velo2imu_translation_ = -velo2imu_rotation_ * imu2velo_translation_;
    }
  }

  /// @brief Overload version of the above routine
  void set_imu2velo(const cv::Matx34d & RT, bool update_reverse = true)
  {
    cv::Matx33d R;
    cv::Vec3d T;
    split_rt_matrix(RT, R, T);
    set_imu2velo(R, T, update_reverse);
  }

  /**
   * @brief Set the mapping from VELODYNE to IMU system
   *
   * The conversion is assumed to be rotation followed by positive translation like this:
   * imu_pos = velo2imu_rotation * velo_pos + velo2imu_translation
   * @param rotation 3x3 VELO <--> IMU rotation matrix
   * @param translation 3d VELO <--> IMU translation vector
   * @param update_reverse true if enabled computing of reverse transform automatically
   * as inverse of provided rotation matrix and translation vector
   */
  void set_velo2imu(const cv::Matx33d & rotation, const cv::Vec3d & translation, bool update_reverse = true)
  {
    velo2imu_rotation_ = rotation;
    velo2imu_translation_ = translation;
    if( update_reverse ) {
      imu2velo_rotation_ = rotation.inv();
      imu2velo_translation_ = -imu2velo_rotation_ * velo2imu_translation_;
    }
  }

  /// @brief Overload version of the above routine
  void set_velo2imu(const cv::Matx34d & RT, bool update_reverse = true)
  {
    cv::Matx33d R;
    cv::Vec3d T;
    split_rt_matrix(RT, R, T);
    set_velo2imu(R, T, update_reverse);
  }

  ///@brief Reference to forward mapping IMU -> VELO translation vector
  const cv::Vec3d& imu2velo_translation() const
  {
    return imu2velo_translation_;
  }

  ///@brief Reference to forward mapping IMU -> VELO rotation matrix
  const cv::Matx33d& imu2velo_rotation() const
  {
    return imu2velo_rotation_;
  }

  ///@brief Reference to reverse mapping VELO -> IMU translation vector
  const cv::Vec3d& velo2imu_translation() const
  {
    return velo2imu_translation_;
  }

  ///@brief Reference to reverse mapping VELO -> IMU rotation matrix
  const cv::Matx33d& velo2imu_rotation() const
  {
    return velo2imu_rotation_;
  }

  ///@brief wrap for glv::oxts_interpolate_for_timestamp()
  void oxts_interpolate_for_timestamp(double ts, /*out*/oxts_data * oxtsi) const
  {
    if( !trajectory_ || trajectory_->empty() ) {
      CF_FATAL("c_ego_motion_correction: OxTS data was not set correctly: oxts_data_=%p size=%zu",
          (void* ) trajectory_, trajectory_ ? 0 : trajectory_->size());
    }
    else {
      ::oxts_interpolate_for_timestamp(*trajectory_, ts /*+ tts_offset_*/, oxtsi);
    }
  }

public: // conventional EGO motion correction methods
  /**
   * @brief Compensates point coordinate in given cloud "in place".
   *
   * On input the point time stamps MUST be specified in the floating-point member PointType::ts in seconds.
   * On output the point coordinates will refer to specified reference (target) time stamp target_ts,
   * as if whole cloud was acquired momentary at moment target_ts.
   * The target_ts SHOULD be usually somewhere WITHIN the LiDAR frame time stamps, but it is not mandatory.
   * NOTE: The routine DOES NOT modify original point time stamps, only coordinates, x, y, z are modified.
   * Caller routine CAN modify the output time stamps manually if it need this, as every output point is referred
   * to the same time stamp target_ts
   * @tparam PointType template parameter
   * @param cloud point cloud to compensate
   * @param target_ts time stamp
   * @param out_target_oxts modified (compensated) lidar points
   * @param mask lidar point's mask
   * @return true if compensation was successful, false otherwise
   */
  template<class PointType>
  bool compensate_ego_motion(std::vector<PointType> & cloud, double target_ts,
      /*out, opt*/oxts_data * out_target_oxts = nullptr,
      const std::vector<bool> & mask = std::vector<bool>() /* don't touch points outside of mask*/) const
  {
    if( !trajectory_ || trajectory_->empty() ) {
      CF_FATAL("APP BUG: Empty Oxts in c_ego_motion_correction");
      return false;
    }

    if( !mask.empty() && mask.size() != cloud.size() ) {
      CF_ERROR("Invalid mask specified: mask.size=%zu src.size=%zu", mask.size(), cloud.size());
      return false;
    }

    oxts_data target_oxts;
    oxts_interpolate_for_timestamp(target_ts, &target_oxts);
    if( out_target_oxts ) {
      *out_target_oxts = target_oxts;
    }

    switch (trajectory_estimation_method_) {
      case TRAJECTORY_ESTIMATION_NONE:
        return true; // silently skip compensation
      case TRAJECTORY_ESTIMATION_GPS:
        return compensate_ego_motion_using_gps(cloud, cloud, mask, target_oxts);
      case TRAJECTORY_ESTIMATION_IMU:
        return compensate_ego_motion_using_imu(cloud, cloud, mask, target_oxts);
      case TRAJECTORY_ESTIMATION_IMUE:
        return compensate_ego_motion_using_imue(cloud, cloud, mask, target_oxts);
      default:
        break;
    }

    return false;
  }

  /**
   * @brief Compensates point coordinates in given cloud "in place".
   * This overload assumes that user-provided PointType doesn't have member named 'ts' which provides point time stamps.
   * @tparam PointType template parameter
   * @param cloud point cloud to compensate
   * @param target_ts target time stamp
   * @param time_stamps explicitly specified time_stamp array, since PointType doesn't have member 'ts'
   * @param out_target_oxts modified (compensated) point cloud
   * @param mask point mask
   * @return true if compensation was successful
   */
  template<class PointType>
  bool compensate_ego_motion(std::vector<PointType> & cloud, double target_ts, const std::vector<double> & time_stamps,
      /*out, opt*/oxts_data * out_target_oxts = nullptr,
      const std::vector<bool> & mask = std::vector<bool>() /* don't touch points outside of mask*/) const
  {
    if( !trajectory_ || trajectory_->empty() ) {
      CF_FATAL("APP BUG: Empty Oxts in c_ego_motion_correction");
      return false;
    }

    if( !mask.empty() && mask.size() != cloud.size() ) {
      CF_ERROR("Invalid mask specified: mask.size=%zu cloud.size=%zu", mask.size(), cloud.size());
      return false;
    }

    oxts_data target_oxts;
    oxts_interpolate_for_timestamp(target_ts, &target_oxts);
    if( out_target_oxts ) {
      *out_target_oxts = target_oxts;
    }

    switch (trajectory_estimation_method_) {
      case TRAJECTORY_ESTIMATION_NONE:
        return true; // silently skip compensation
      case TRAJECTORY_ESTIMATION_GPS:
        return compensate_ego_motion_using_gps(cloud, cloud, mask, time_stamps, target_oxts);
      case TRAJECTORY_ESTIMATION_IMU:
        return compensate_ego_motion_using_imu(cloud, cloud, mask, time_stamps, target_oxts);
      case TRAJECTORY_ESTIMATION_IMUE:
        return compensate_ego_motion_using_imue(cloud, cloud, mask, time_stamps, target_oxts);
      default:
        break;
    }

    return false;
  }

  /**
   * @brief Compensates point coordinates in given cloud src and writes into output cloud dst.
   * The clouds src and dst CAN point to the same C++ object, in this case 'in place' compensation will be done.
   * @tparam PointType parameter type
   * @param src source read only point cloud to compensate
   * @param dst destination cloud point which will holds compensation results
   * @param target_ts target timestamp
   * @param out_target_oxts compensated oxts data
   * @param mask point cloud mask
   * @return true if compensation was successfull
   */
  template<class PointType>
  bool compensate_ego_motion(const std::vector<PointType> & src, std::vector<PointType> & dst, double target_ts,
      /*out, opt*/oxts_data * out_target_oxts = nullptr,
      const std::vector<bool> & mask = std::vector<bool>() /* don't touch points outside of mask*/) const
  {
    if( !trajectory_ || trajectory_->empty() ) {
      CF_FATAL("APP BUG: Empty Oxts in c_ego_motion_correction");
      return false;
    }

    if( !mask.empty() && mask.size() != src.size() ) {
      CF_ERROR("Invalid mask specified: mask.size=%zu src.size=%zu", mask.size(), src.size());
      return false;
    }

    oxts_data target_oxts;
    oxts_interpolate_for_timestamp(target_ts, &target_oxts);
    if( out_target_oxts ) {
      *out_target_oxts = target_oxts;
    }

    switch (trajectory_estimation_method_) {
      case TRAJECTORY_ESTIMATION_NONE:
        if( &src != &dst ) {
          if( mask.empty() ) {
            dst = src;
          }
          else {
            dst.clear(), dst.reserve(src.size());
            for( int i = 0, n = (int) src.size(); i < n; ++i ) {
              if( mask[i] ) {
                dst.emplace_back(src[i]);
              }
            }
          }
        }
        else if( !mask.empty() ) {
          std::vector<PointType> tmp;
          tmp.reserve(src.size());
          for( int i = 0, n = (int) src.size(); i < n; ++i ) {
            if( mask[i] ) {
              tmp.emplace_back(src[i]);
            }
          }
          dst = std::move(tmp);
        }
        return true; // silently skip motion compensation
      case TRAJECTORY_ESTIMATION_GPS:
        return compensate_ego_motion_using_gps(src, dst, mask, target_oxts);
      case TRAJECTORY_ESTIMATION_IMU:
        return compensate_ego_motion_using_imu(src, dst, mask, target_oxts);
      case TRAJECTORY_ESTIMATION_IMUE:
        return compensate_ego_motion_using_imue(src, dst, mask, target_oxts);
      default:
        break;
    }

    return false;
  }

  /**
   * @brief Compensates point coordinates in given cloud and writes into output cloud dst
   * without OxTS interpolation
   * @tparam PointType parameter type
   * @param src point cloud to compensate
   * @param dst destination to hold compensated points
   * @param target_oxts OxTS data
   * @param mask point cloud mask
   * @return true if compensation was successfull
   */
  template<class PointType>
  bool compensate_ego_motion(const std::vector<PointType> & src, std::vector<PointType> & dst,
      const oxts_data & target_oxts,
      const std::vector<bool> & mask = std::vector<bool>()) const
  {
    if( !mask.empty() && mask.size() != src.size() ) {
      CF_ERROR("Invalid mask specified: mask.size=%zu src.size=%zu", mask.size(), src.size());
      return false;
    }

    switch (trajectory_estimation_method_) {
      case TRAJECTORY_ESTIMATION_NONE:
        if( &src != &dst ) {
          if( mask.empty() ) {
            dst = src;
          }
          else {
            dst.clear(), dst.reserve(src.size());
            for( int i = 0, n = (int) src.size(); i < n; ++i ) {
              if( mask[i] ) {
                dst.emplace_back(src[i]);
              }
            }
          }
        }
        else if( !mask.empty() ) {
          std::vector<PointType> tmp;
          tmp.reserve(src.size());
          for( int i = 0, n = (int) src.size(); i < n; ++i ) {
            if( mask[i] ) {
              tmp.emplace_back(src[i]);
            }
          }
          dst = std::move(tmp);
        }
        return true; // silently skip motion compensation
      case TRAJECTORY_ESTIMATION_GPS:
        return compensate_ego_motion_using_gps(src, dst, mask, target_oxts);
      case TRAJECTORY_ESTIMATION_IMU:
        return compensate_ego_motion_using_imu(src, dst, mask, target_oxts);
      case TRAJECTORY_ESTIMATION_IMUE:
        return compensate_ego_motion_using_imue(src, dst, mask, target_oxts);
      default:
        break;
    }

    return false;
  }

  /**
   * @brief Compensates point coordinates in given cloud src and writes into output cloud dst
   * Clouds src and dst CAN point to the same C++ object, in this case 'in place' compensation will be done.
   * Assumes that user-provided PointType doesn't have member named 'ts'. Instead, array of time stamps provided
   * separately.
   * @tparam PointType parameter type
   * @param src point cloud to be compensated
   * @param dst destination point cloud to hold compensated results
   * @param target_ts target time stamp
   * @param time_stamps array of point's time stamp in point cloud
   * @param out_target_oxts target OxTS
   * @param mask point cloud mask
   * @return true if compensation was successfull, false otherwise
   */
  template<class PointType>
  bool compensate_ego_motion(const std::vector<PointType> & src, std::vector<PointType> & dst, double target_ts,
      const std::vector<double> & time_stamps, /*out, opt*/oxts_data * out_target_oxts = nullptr,
      const std::vector<bool> & mask = std::vector<bool>() /* don't touch points outside of mask*/) const
  {
    if( !trajectory_ || trajectory_->empty() ) {
      CF_FATAL("APP BUG: Empty Oxts in c_ego_motion_correction");
      return false;
    }

    if( !mask.empty() && mask.size() != src.size() ) {
      CF_ERROR("Invalid mask specified: mask.size=%zu src.size=%zu", mask.size(), src.size());
      return false;
    }

    oxts_data target_oxts;
    oxts_interpolate_for_timestamp(target_ts, &target_oxts);
    if( out_target_oxts ) {
      *out_target_oxts = target_oxts;
    }

    switch (trajectory_estimation_method_) {
      case TRAJECTORY_ESTIMATION_NONE:
        if( &src != &dst ) {
          if( mask.empty() ) {
            dst = src;
          }
          else {
            dst.clear(), dst.reserve(src.size());
            for( int i = 0, n = src.size(); i < n; ++i ) {
              if( mask[i] ) {
                dst.emplace_back(src[i]);
              }
            }
          }
        }
        else if( !mask.empty() ) {
          std::vector<PointType> tmp;
          tmp.reserve(src.size());
          for( int i = 0, n = src.size(); i < n; ++i ) {
            if( mask[i] ) {
              tmp.emplace_back(src[i]);
            }
          }
          dst = std::move(tmp);
        }
        return true; // silently skip compensation
      case TRAJECTORY_ESTIMATION_GPS:
        return compensate_ego_motion_using_gps(src, dst, mask, time_stamps, target_oxts);
      case TRAJECTORY_ESTIMATION_IMU:
        return compensate_ego_motion_using_imu(src, dst, mask, time_stamps, target_oxts);
      case TRAJECTORY_ESTIMATION_IMUE:
        return compensate_ego_motion_using_imue(src, dst, mask, time_stamps, target_oxts);
      default:
        break;
    }

    return false;
  }

public: // Implementation-specific auxiliary subroutines,
        // Don't call them directly if you are not sure what you are doing,
        // in sprite of they are declared in public interface.

  /**
   * @brief make point cloud compensation (correction) based on GPS trajectory
   * @tparam PointType template parameter for lidar point type
   * @param src point cloud to be compensated
   * @param dst destination to hold compensated resuslts
   * @param mask point cloud mask
   * @param target_oxts target OxTS
   * @return
   */
  template<class PointType>
  bool compensate_ego_motion_using_gps(const std::vector<PointType> & src, std::vector<PointType> & dst,
      const std::vector<bool> & mask, const oxts_data & target_oxts) const
  {
    PointIndex index;
    if( build_index<PointType>(src, index, &mask) ) {
      return compensate_ego_motion_using_gps_trajectory(src, dst, index, target_oxts);
    }
    return true;
  }

  /**
   * @brief make point cloud compensation based on GPS trajectory. Assumed points in point cloud don't have
   * 'ts' field. Instead there is input vector which provide timestamp array separately
   * @tparam PointType template parameter type which indicates point type to work with
   * @tparam TimeType template parameter type which indicates time type to work with
   * @param src LiDAR point cloud to be compensated
   * @param dst destination point cloud to hold compensated point cloud
   * @param mask point cloud mask
   * @param target_oxts target OxTS
   * @param tstamps separately provided timestamps for point cloud points
   * @return true if compensation was successfull or build point indices failed, false otherwise
   */
  template<class PointType, class TimeType>
  bool compensate_ego_motion_using_gps(const std::vector<PointType> & src, std::vector<PointType> & dst,
      const std::vector<bool> & mask, const oxts_data & target_oxts, std::vector<TimeType> & tstamps) const
  {
    PointIndex index;
    if( build_index<PointType>(src, tstamps, index, &mask) ) {
      return compensate_ego_motion_using_gps_trajectory(src, dst, index, target_oxts);
    }
    return true;
  }

  /**
   * @brief compensate ego motion based on linearized inertial measurement unit (IMU) trajectory
   * @tparam PointType template parameter type
   * @param src point cloud to be compensated
   * @param dst destination point cloud to hold comepnsation results
   * @param mask point cloud mask
   * @param target_oxts target OxTS
   * @return true if compensation was successful
   */
  template<class PointType>
  bool compensate_ego_motion_using_imu(const std::vector<PointType> & src, std::vector<PointType> & dst,
      const std::vector<bool> & mask, const oxts_data & target_oxts) const
  {
    PointIndex index;
    if( build_index<PointType>(src, index, &mask) ) {
      return compensate_ego_motion_using_linearized_imu_trajectory(src, dst, index, target_oxts);
    }
    return true;
  }

  /**
   * @brief compensate ego motion based on linearized inertial measurement unit (IMU) data. Assumed point cloud points
   * do not have timestamp. Instead, those data provided in separate vector
   * @tparam PointType template parameter type which indicates point type to work with
   * @tparam TimeType template parameter type which indicates time type to work with
   * @param src source point cloud to be compensated
   * @param dst destination point cloud to hold compensated results
   * @param mask point cloud mask
   * @param target_oxts target OxTS
   * @param tstamps separate time stamp array for point cloud
   * @return true if compensated was successful
   */
  template<class PointType, class TimeType>
  bool compensate_ego_motion_using_imu(const std::vector<PointType> & src, std::vector<PointType> & dst,
      const std::vector<bool> & mask, const oxts_data & target_oxts, std::vector<TimeType> & tstamps) const
  {
    PointIndex index;
    if( build_index<PointType>(src, tstamps, index, &mask) ) {
      return compensate_ego_motion_using_linearized_imu_trajectory(src, dst, index, target_oxts);
    }
    return true;
  }

  /**
   * @brief compensates ego motion based on full inertial measurement unit (IMU) trajectory
   * @tparam PointType template parameter type which indicates point type
   * @param src source point cloud to be compensated
   * @param dst destination point cloud to hold compensated results
   * @param mask point cloud mask
   * @param target_oxts target OxTS
   * @return true if compensation was successful, false otherwise
   */
  template<class PointType>
  bool compensate_ego_motion_using_imue(const std::vector<PointType> & src, std::vector<PointType> & dst,
      const std::vector<bool> & mask, const oxts_data & target_oxts) const
  {
    PointIndex index;
    if( build_index<PointType>(src, index, &mask) ) {
      return compensate_ego_motion_using_full_imu_trajectory(src, dst, index, target_oxts);
    }
    return true;
  }

  /**
   * @brief compensates ego motion based on inertial measurement unit (IMU) trajectory.
   * Assumed point in point cloud doesn't have 'ts' field. Instead provided time stamp array separately
   * @tparam PointType template parameter type which indicates point type
   * @tparam TimeType template parameter type which indicates time type
   * @param src source point cloud to be compensated
   * @param dst destination point cloud to hold compensated point cloud
   * @param mask point cloud mask
   * @param target_oxts target OxTS
   * @param tstamps separately provided time stamp array for point cloud
   * @return
   */
  template<class PointType, class TimeType>
  bool compensate_ego_motion_using_imue(const std::vector<PointType> & src, std::vector<PointType> & dst,
      const std::vector<bool> & mask, const oxts_data & target_oxts, std::vector<TimeType> & tstamps) const
  {
    PointIndex index;
    if( build_index<PointType>(src, tstamps, index, &mask) ) {
      return compensate_ego_motion_using_full_imu_trajectory(src, dst, index, target_oxts);
    }
    return true;
  }

private:
  /**
   * @brief create array of c_index_entry from point cloud's timestamps and sort it in ascending order by time
   * @tparam PointType template parameter type which indicates point type of point cloud
   * @param cloud point cloud
   * @param index output array of c_index_entry
   * @param mask point cloud mask
   * @return true if output array non-empty
   */
  template<class PointType>
  bool build_index(const std::vector<PointType> & cloud, PointIndex & index,
      const std::vector<bool> * mask) const
  {
    index.reserve(cloud.size());
    if( !mask || mask->empty() ) {
      for( size_t i = 0, n = cloud.size(); i < n; ++i ) {
        index.emplace_back(cloud[i].t, i);
      }
    }
    else {
      assert(mask->size() == cloud.size());
      for( size_t i = 0, n = cloud.size(); i < n; ++i ) {
        if( (*mask)[i] ) {
          index.emplace_back(cloud[i].t, i);
        }
      }
    }

    // sort index over points time
    std::sort(index.begin(), index.end(), c_index_entry::less_by_time);

    return !index.empty();
  }

  /**
   * @brief create array of c_index_entry from separately provided timestamps array and sort it in ascending order
   * by time.
   * @tparam PointType template parameter which indicates point type of point cloud
   * @tparam TimeType template parameter which indicates time type
   * @param cloud input point cloud
   * @param tstamps array of timestamps
   * @param index output array of c_index_entry
   * @param mask point cloud mask
   * @return true if output array non-empty
   */
  template<class PointType, class TimeType>
  bool build_index(const std::vector<PointType> & cloud, const std::vector<TimeType> & tstamps,
      PointIndex & index, const std::vector<bool> * mask) const
  {
    index.reserve(cloud.size());
    if( !mask || mask->empty() ) {
      for( size_t i = 0, n = cloud.size(); i < n; ++i ) {
        index.emplace_back(tstamps[i], i);
      }
    }
    else {
      assert(mask->size() == cloud.size());
      for( size_t i = 0, n = cloud.size(); i < n; ++i ) {
        if( (*mask)[i] ) {
          index.emplace_back(tstamps[i], i);
        }
      }
    }

    // sort index over points time
    std::sort(index.begin(), index.end(), c_index_entry::less_by_time);

    return !index.empty();
  }

  /**
   * @brief find last point in a group of points. Group of points determined by their accessory to range of
   * time between [first_point_in_group + time_step]
   * @param index array of c_index_entry structures
   * @param first_point_in_group first point in a point group
   * @return
   */
  size_t find_last_point_in_group(const PointIndex & index, size_t first_point_in_group) const
  {
    size_t last_point_in_group = first_point_in_group;
    const double last_timestamp = index[first_point_in_group].ts + time_step_;
    while (last_point_in_group + 1 < index.size() && index[last_point_in_group + 1].ts <= last_timestamp) {
      ++last_point_in_group;
    }
    return last_point_in_group;
  }

  /**
   * @brief Uses pure gps for ego motion compensation. Ego motion compensation calculates by applying total rotation and
   * total translation matrices to every point in point cloud.
   * @tparam PointType template parameter type indiciates point type
   * @param src point cloud to be compensated
   * @param dst output point cloud to hold compensated results
   * @param index array of c_index_entry structures
   * @param target_oxts target OxTS
   * @return
   */
  template<class PointType>
  bool compensate_ego_motion_using_gps_trajectory(const std::vector<PointType> & src, std::vector<PointType> & dst,
      const PointIndex & index, const oxts_data & target_oxts) const
  {
    using namespace cv;

    Vec3d target_ecef_position;
    Matx33d target_ecef_to_vs_rotation;

    std::vector<PointType> tmp;
    std::vector<PointType> & dstp = ((&src == &dst && src.size() == index.size()) || (&src != &dst)) ? dst : tmp;
    if( &dstp == &src ) {
    }
    else if( src.size() == index.size() ) {
      dstp = src;
    }
    else {
      dstp.clear(), dstp.reserve(index.size());
    }

    target_ecef_position =
        gps2ecef(target_oxts.lat, target_oxts.lon, target_oxts.alt);

    target_ecef_to_vs_rotation =
        ecef2iso8855vs_rotation_matrix(target_oxts.lat, target_oxts.lon, target_oxts.alt,
            target_oxts.roll, target_oxts.pitch, target_oxts.yaw);

    size_t first_point_in_group = 0;

    while (first_point_in_group < index.size()) {

      // Find end of the group
      const size_t last_point_in_group = find_last_point_in_group(index, first_point_in_group);
      const double group_timestamp = 0.5 * (index[first_point_in_group].ts + index[last_point_in_group].ts);

      if( group_timestamp == target_oxts.ts ) {
        if( dstp.size() != src.size() ) {
          for( size_t i = first_point_in_group; i <= last_point_in_group; ++i ) {
            dst.emplace_back(src[index[i].p]);
          }
        }
      }
      else {

        oxts_data group_oxts;
        Vec3d group_orign;
        Matx33d group_vs_to_ecef_rotation;
        Matx33d group_vs_to_reference_rotation;

        // Interpolate oxts data for the middle group time stamp
        oxts_interpolate_for_timestamp(group_timestamp, &group_oxts);

        // Compute group position in ECEF and convert to offset relative to reference VS
        group_orign =
            target_ecef_to_vs_rotation
                * (gps2ecef(group_oxts.lat, group_oxts.lon, group_oxts.alt) - target_ecef_position);

        // Compute ISO8855VS -> ECEF rotation matrix
        iso8855vs2ecef(group_oxts.lat, group_oxts.lon, group_oxts.alt,
            group_oxts.roll, group_oxts.pitch, group_oxts.yaw,
            &group_vs_to_ecef_rotation, NULL);

        group_vs_to_reference_rotation = target_ecef_to_vs_rotation * group_vs_to_ecef_rotation;

        // Update positions of each point in selected group

        const Matx33d TotalRotation = enable_imu2velo_ ?
            imu2velo_rotation_ * group_vs_to_reference_rotation * velo2imu_rotation_ :
            group_vs_to_reference_rotation;

        const Vec3d TotalTranslation =
            enable_imu2velo_ ?
                imu2velo_rotation_ * (group_vs_to_reference_rotation * velo2imu_translation_ + group_orign)
                    + imu2velo_translation_ :
                group_orign;

        // Update positions of each point in selected group of points
        if( src.size() == dstp.size() ) { // in-place correction correction preserving points order
          for( size_t i = first_point_in_group; i <= last_point_in_group; ++i ) {
            PointType & p = dst[index[i].p];
            const Vec3d new_pos(TotalRotation * Vec3d(p.x, p.y, p.z) + TotalTranslation);
            p.x = new_pos[0], p.y = new_pos[1], p.z = new_pos[2];
          }
        }
        else {
          for( size_t i = first_point_in_group; i <= last_point_in_group; ++i ) {
            PointType p = src[index[i].p];
            const Vec3d new_pos(TotalRotation * Vec3d(p.x, p.y, p.z) + TotalTranslation);
            p.x = new_pos[0], p.y = new_pos[1], p.z = new_pos[2];
            dstp.emplace_back(p);
          }
        }

      }

      first_point_in_group = last_point_in_group + 1;
    }

    if( &dstp == &tmp ) {
      dst = std::move(tmp);
    }

    return true;
  }

  /**
   * @brief Uses pure IMU for ego motion compensation
   *
   * Adapted from kolyarage usnpin.cpp:
   * 1) don't reorder (sort) points in cloud,
   * 2) ignore rotation angle because it could be invalid or unavailable in some datasets/PointTypes
   * 3) use point time stamps instead of rotation angle for adjustable grouping controlled by time_step_
   * @tparam PointType template parameter indicates point type
   * @param src point cloud to be compensated
   * @param dst point cloud to store compensated results
   * @param index array of c_index_entry structures
   * @param target_oxts target OxTS
   * @return true if always return true
   */
  template<class PointType>
  bool compensate_ego_motion_using_linearized_imu_trajectory(const std::vector<PointType> & src,
      std::vector<PointType> & dst,
      const PointIndex & index, const oxts_data & target_oxts) const
  {
    using namespace cv;

    oxts_data group_oxts;
    Matx33d R;
    Vec3d T;

    std::vector<PointType> tmp;
    std::vector<PointType> & dstp = ((&src == &dst && src.size() == index.size()) || (&src != &dst)) ? dst : tmp;
    if( &dstp == &src ) {
    }
    else if( src.size() == index.size() ) {
      dstp = src;
    }
    else {
      dstp.clear(), dstp.reserve(index.size());
    }

    size_t first_point_in_group = 0;

    while (first_point_in_group < index.size()) {

      // Find end of the group based on time_step_.
      // Use set_time_step(DBL_EPSILON) if you need exact match (slow computation)
      const size_t last_point_in_group = find_last_point_in_group(index, first_point_in_group);

      // use middle as group time stamp.
      const double group_timestamp = 0.5 * (index[first_point_in_group].ts + index[last_point_in_group].ts);

      if( group_timestamp == target_oxts.ts ) {
        if( dstp.size() != src.size() ) {
          for( size_t i = first_point_in_group; i <= last_point_in_group; ++i ) {
            dst.emplace_back(src[index[i].p]);
          }
        }
      }
      else {

        oxts_interpolate_for_timestamp(group_timestamp, &group_oxts);
        compute_correction(group_oxts, target_oxts, R, T);

        // Update positions of each point in selected group
        if( src.size() == dstp.size() ) { // in-place correction correction preserving points order
          for( size_t i = first_point_in_group; i <= last_point_in_group; ++i ) {
            PointType & p = dst[index[i].p];
            const Vec3d new_pos(R * Vec3d(p.x, p.y, p.z) + T);
            p.x = new_pos[0], p.y = new_pos[1], p.z = new_pos[2];
          }
        }
        else {
          for( size_t i = first_point_in_group; i <= last_point_in_group; ++i ) {
            PointType p = src[index[i].p];
            const Vec3d new_pos(R * Vec3d(p.x, p.y, p.z) + T);
            p.x = new_pos[0], p.y = new_pos[1], p.z = new_pos[2];
            dstp.emplace_back(p);
          }
        }
      }

      first_point_in_group = last_point_in_group + 1;
    }

    if( &dstp == &tmp ) {
      dst = std::move(tmp);
    }

    return true;
  }

  // Adapted from kolyarage usnpin.cpp and optimized a bit for better performance
  /**
   * Computed rotation and translation supposed to be applied to group of points to move them into target
   * coordianate system: target_pos = R * source_pos + T
   * @param group_oxts OxTS data for group of point
   * @param target_oxts target OxTS data
   * @param TotalRotation output total rotation 3x3 matrix
   * @param TotalTranslation output total 3d translation vector
   */
  void compute_correction(const oxts_data & group_oxts, const oxts_data & target_oxts,
      cv::Matx33d & TotalRotation, cv::Vec3d & TotalTranslation) const
  {
    using namespace cv;

    Matx33d L, T, Y;

    build_rotation((group_oxts.roll - target_oxts.roll),
        (group_oxts.pitch - target_oxts.pitch),
        (group_oxts.yaw - target_oxts.yaw),
        &L, &T, &Y);

    const Matx33d ImuRotation = Y * T * L;

    const cv::Vec3d ImuTranslation = 0.5 * (group_oxts.ts - target_oxts.ts) * Vec3d(
        target_oxts.vf + group_oxts.vf,
        target_oxts.vl + group_oxts.vl,
        target_oxts.vu + group_oxts.vu);

    if( !enable_imu2velo_ ) {
      TotalRotation = ImuRotation;
      TotalTranslation = ImuTranslation;
    }
    else {
      TotalRotation = imu2velo_rotation_ * ImuRotation * velo2imu_rotation_;
      TotalTranslation = imu2velo_rotation_ * (ImuRotation * velo2imu_translation_ + ImuTranslation)
          + imu2velo_translation_;
    }
  }

  //@brief IMUE, TEST FOR OXTS EULER INTEGRATION (FIRST-ORDER RUGE-KUTTA)
  // FIXME: consider reimplement the oxts_integrate() to allow usage of output R and T as 'target_pos = R * src_pos + T';
  template<class PointType>
  bool compensate_ego_motion_using_full_imu_trajectory(const std::vector<PointType> & src, std::vector<PointType> & dst,
      const PointIndex & index, const oxts_data & target_oxts) const
  {
    using namespace cv;

    oxts_data group_oxts;
    Matx33d R;
    Vec3d T;

    std::vector<PointType> tmp;
    std::vector<PointType> & dstp = ((&src == &dst && src.size() == index.size()) || (&src != &dst)) ? dst : tmp;
    if( &dstp == &src ) {
    }
    else if( src.size() == index.size() ) {
      dstp = src;
    }
    else {
      dstp.clear(), dstp.reserve(index.size());
    }

    size_t first_point_in_group = 0;

    while (first_point_in_group < index.size()) {

      // Find end of the group based on time_step_.
      // Use set_time_step(DBL_EPSILON) if you need exact match (slow computation)
      const size_t last_point_in_group = find_last_point_in_group(index, first_point_in_group);

      // use middle as group time stamp.
      const double group_timestamp = 0.5 * (index[first_point_in_group].ts + index[last_point_in_group].ts);

      if( group_timestamp == target_oxts.ts ) {
        if( dstp.size() != src.size() ) {
          for( size_t i = first_point_in_group; i <= last_point_in_group; ++i ) {
            dst.emplace_back(src[index[i].p]);
          }
        }
      }
      else {

        oxts_interpolate_for_timestamp(group_timestamp, &group_oxts);
        oxts_integrate(group_oxts, target_oxts, *trajectory_, &T, &R);

        // Update positions of each point in selected group of points
        if( src.size() == dstp.size() ) { // in-place correction correction preserving points order
          for( size_t i = first_point_in_group; i <= last_point_in_group; ++i ) {
            PointType & p = dst[index[i].p];
            const Vec3d new_pos(R * (Vec3d(p.x, p.y, p.z) - T));
            p.x = new_pos[0], p.y = new_pos[1], p.z = new_pos[2];
          }
        }
        else {
          for( size_t i = first_point_in_group; i <= last_point_in_group; ++i ) {
            PointType p = src[index[i].p];
            const Vec3d new_pos(R * (Vec3d(p.x, p.y, p.z) - T));
            p.x = new_pos[0], p.y = new_pos[1], p.z = new_pos[2];
            dstp.emplace_back(p);
          }
        }
      }

      first_point_in_group = last_point_in_group + 1;
    }

    if( &dstp == &tmp ) {
      dst = std::move(tmp);
    }

    return true;
  }

public: // additional utility methods which may be usefull for applications
  /**
   * @brief Computes total rotation and translation between source and target OxTS points.
   * Computed rotation and translation supposed to be applied to group of points to move them into target coordinate
   * system: Target_pos = TotalRotation * Source_pos + TotalTranslation, e.g. we firstly ROTATE axes into target system,
   * and then ADD offset between source and target systems.
   * @param source_oxts source OxTS data
   * @param target_oxts target OxTS data
   * @param TotalRotation output 3x3 rotation matrix
   * @param TotalTranslation output 3d translation vector
   */
  void compute_transform(const oxts_data & source_oxts, const oxts_data & target_oxts,
      cv::Matx33d & TotalRotation, cv::Vec3d & TotalTranslation) const
  {
    using namespace cv;

    switch (trajectory_estimation_method()) {
      case TRAJECTORY_ESTIMATION_INVALID:
        case TRAJECTORY_ESTIMATION_NONE: {
        TotalRotation = cv::Matx33d::eye();
        TotalTranslation = Vec3d::all(0);
        break;
      }
      case TRAJECTORY_ESTIMATION_IMU:
        case TRAJECTORY_ESTIMATION_IMUE: {
        Matx33d L, T, Y;

        build_rotation((source_oxts.roll - target_oxts.roll),
            (source_oxts.pitch - target_oxts.pitch),
            (source_oxts.yaw - target_oxts.yaw),
            &L, &T, &Y);

        const Matx33d ImuRotation = Y * T * L;

        const cv::Vec3d ImuTranslation = 0.5 * (source_oxts.ts - target_oxts.ts) * Vec3d(
            target_oxts.vf + source_oxts.vf,
            target_oxts.vl + source_oxts.vl,
            target_oxts.vu + source_oxts.vu);

        if( !enable_imu2velo() ) {
          TotalRotation = ImuRotation;
          TotalTranslation = ImuTranslation;
        }
        else {
          TotalRotation = imu2velo_rotation() * ImuRotation * velo2imu_rotation();
          TotalTranslation = imu2velo_rotation() * (ImuRotation * velo2imu_translation() + ImuTranslation)
              + imu2velo_translation();
        }
        break;
      }

      case TRAJECTORY_ESTIMATION_GPS: {

        Vec3d source_orign;
        Vec3d target_ecef_position;
        Matx33d source_vs_to_ecef_rotation;
        Matx33d source_vs_to_reference_rotation;
        Matx33d target_ecef_to_vs_rotation;

        target_ecef_position =
            gps2ecef(target_oxts.lat, target_oxts.lon, target_oxts.alt);

        source_orign =
            gps2ecef(source_oxts.lat, source_oxts.lon, source_oxts.alt);

        target_ecef_to_vs_rotation =
            ecef2iso8855vs_rotation_matrix(target_oxts.lat, target_oxts.lon, target_oxts.alt,
                target_oxts.roll, target_oxts.pitch, target_oxts.yaw);

        source_vs_to_ecef_rotation =
            iso8855vs2ecef_rotation_matrix(source_oxts.lat, source_oxts.lon, source_oxts.alt,
                source_oxts.roll, source_oxts.pitch, source_oxts.yaw);

        source_orign = target_ecef_to_vs_rotation * (source_orign - target_ecef_position);
        source_vs_to_reference_rotation = target_ecef_to_vs_rotation * source_vs_to_ecef_rotation;

        TotalRotation = enable_imu2velo() ?
            imu2velo_rotation() * source_vs_to_reference_rotation * velo2imu_rotation() :
            source_vs_to_reference_rotation;

        TotalTranslation =
            enable_imu2velo() ?
                imu2velo_rotation() * (source_vs_to_reference_rotation * velo2imu_translation() + source_orign)
                    + imu2velo_translation() :
                source_orign;

        break;
      }
    }
  }

  /**
   * @brief Computes total rotation and translation between source and target OxTS points
   * and return resuls in a single (R | T) matrix
   * @param source_oxts source OxTS
   * @param target_oxts target OxTS
   * @return 3x4 combined rotation-translation matrix
   */
  cv::Matx34d compute_transform(const oxts_data & source_oxts, const oxts_data & target_oxts) const
  {
    cv::Matx33d R;
    cv::Vec3d T;

    compute_transform(source_oxts, target_oxts, R, T);

    return cv::Matx34d(
        R(0, 0), R(0, 1), R(0, 2), T(0),
        R(1, 0), R(1, 1), R(1, 2), T(1),
        R(2, 0), R(2, 1), R(2, 2), T(2));
  }

  /**
   * @brief computes total rotation and translation between source and target OxTS points and return
   * single combined rotation-translation matrix.
   * @param source_oxts source OxTS
   * @param target_ts target timestamp
   * @param _target_oxts output target_oxts data (optional)
   * @return single combined 3x4 rotation-translation matrix
   */
  cv::Matx34d compute_transform(const oxts_data & source_oxts, double target_ts,
      oxts_data * _target_oxts = nullptr) const
  {
    oxts_data target_oxts;
    oxts_interpolate_for_timestamp(target_ts, &target_oxts);
    if( _target_oxts ) {
      *_target_oxts = target_oxts;
    }
    return compute_transform(source_oxts, target_oxts);
  }
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  struct target_cache
  {
    enum TRAJECTORY_ESTIMATION_METHOD method = TRAJECTORY_ESTIMATION_NONE;
    oxts_data target_oxts;
    cv::Vec3d target_ecef_position; // Used with TRAJECTORY_ESTIMATION_GPS
    cv::Matx33d target_ecef_to_vs_rotation; // Used with TRAJECTORY_ESTIMATION_GPS
  };

  void cache_target(const oxts_data & target_oxts, struct target_cache * cache) const
  {
    cache->target_oxts = target_oxts;
    switch (cache->method = this->trajectory_estimation_method_) {
      case TRAJECTORY_ESTIMATION_GPS:
        cache->target_ecef_position =
            gps2ecef(target_oxts.lat, target_oxts.lon, target_oxts.alt);

        cache->target_ecef_to_vs_rotation =
            ecef2iso8855vs_rotation_matrix(target_oxts.lat, target_oxts.lon, target_oxts.alt,
                target_oxts.roll, target_oxts.pitch, target_oxts.yaw);
        break;
      default:
        break;
    }
  }

  void compensate_ego_motion(double src_x, double src_y, double src_z, const oxts_data & src_oxts,
      const struct target_cache & cache,
      double * x, double * y, double * z) const
  {
    constexpr double time_eps = 1e-4; // 0.1 ms

    switch (cache.method) {
      case TRAJECTORY_ESTIMATION_GPS:
        if( fabs(src_oxts.ts - cache.target_oxts.ts) < time_eps ) {
          *x = src_x, *y = src_y, *z = src_z;
        }
        else {
          cv::Vec3d src_orign;
          cv::Matx33d src_vs_to_ecef_rotation;
          cv::Matx33d src_vs_to_reference_rotation;

          // Compute src position in ECEF and convert to offset relative to reference VS
          src_orign =
              cache.target_ecef_to_vs_rotation * (gps2ecef(src_oxts.lat, src_oxts.lon, src_oxts.alt) -
                  cache.target_ecef_position);

          // Compute ISO8855VS -> ECEF rotation matrix
          iso8855vs2ecef(src_oxts.lat, src_oxts.lon, src_oxts.alt,
              src_oxts.roll, src_oxts.pitch, src_oxts.yaw,
              &src_vs_to_ecef_rotation, NULL);

          src_vs_to_reference_rotation = cache.target_ecef_to_vs_rotation * src_vs_to_ecef_rotation;

          const cv::Matx33d TotalRotation =
              enable_imu2velo_ ?
                  imu2velo_rotation_ * src_vs_to_reference_rotation * velo2imu_rotation_ :
                  src_vs_to_reference_rotation;

          const cv::Vec3d TotalTranslation =
              enable_imu2velo_ ?
                  imu2velo_rotation_ * (src_vs_to_reference_rotation * velo2imu_translation_ + src_orign)
                      + imu2velo_translation_ :
                  src_orign;

          // Compute transformed position
          const cv::Vec3d corrected_pos(TotalRotation * cv::Vec3d(src_x, src_y, src_z) + TotalTranslation);

          *x = corrected_pos[0];
          *y = corrected_pos[1];
          *z = corrected_pos[2];
        }
        break;

      case TRAJECTORY_ESTIMATION_IMU:
        if( fabs(src_oxts.ts - cache.target_oxts.ts) < time_eps ) {
          *x = src_x, *y = src_y, *z = src_z;
        }
        else {
          cv::Matx33d R;
          cv::Vec3d T;
          compute_correction(src_oxts, cache.target_oxts, R, T);
          const cv::Vec3d corrected_pos(R * cv::Vec3d(src_x, src_y, src_z) + T);
          *x = corrected_pos[0];
          *y = corrected_pos[1];
          *z = corrected_pos[2];
        }
        break;

      case TRAJECTORY_ESTIMATION_IMUE:
        if( fabs(src_oxts.ts - cache.target_oxts.ts) < time_eps ) {
          *x = src_x, *y = src_y, *z = src_z;
        }
        else {
          cv::Matx33d R;
          cv::Vec3d T;
          oxts_integrate(src_oxts, cache.target_oxts, *trajectory_, &T, &R);
          const cv::Vec3d corrected_pos(R * cv::Vec3d(src_x, src_y, src_z) + T);
          *x = corrected_pos[0];
          *y = corrected_pos[1];
          *z = corrected_pos[2];
        }
        break;

      default:
        *x = src_x;
        *y = src_y;
        *z = src_z;
        break;
    }
  }

  // Simple experimental function which switches trajectory estimation method - GPS for "fast" moving object, IMU for "slow"
  static enum TRAJECTORY_ESTIMATION_METHOD get_trajectory_estimation_method_by_velocity(
      const std::vector<oxts_data> * oxts, double source_ts, double target_ts, double thr = 9.5)
  {

    auto iter_target = std::lower_bound(
        oxts->begin(), oxts->end(), target_ts,
        [](const oxts_data & oxts_, const double ts) {
          return oxts_.ts < ts;
        });

    int idx_target = (int) (iter_target - oxts->begin() - 1);
    if( idx_target < 0 || idx_target >= (int) oxts->size() ) {
      CF_WARNING("get_trajectory_estimation_method_by_velocity: target_ts %.3f is out of ego oxts timerange",
          target_ts);
      return TRAJECTORY_ESTIMATION_NONE;
    }
    auto iter_source = std::lower_bound(
        oxts->begin(), oxts->end(), source_ts,
        [](const oxts_data & oxts_, const double ts) {
          return oxts_.ts < ts;
        });
    int idx_source = (int) (iter_source - oxts->begin() - 1);
    if( idx_source < 0 || idx_source >= (int) oxts->size() ) {
      CF_WARNING("get_trajectory_estimation_method_by_velocity: source_ts %.3f is out of ego oxts timerange",
          source_ts);
      return TRAJECTORY_ESTIMATION_NONE;
    }
    if( idx_source == idx_target ) {
      return oxts->at(idx_source).vf <= thr ? TRAJECTORY_ESTIMATION_IMU : TRAJECTORY_ESTIMATION_GPS;
    }
    bool is_future = idx_source < idx_target;
    int start = is_future ? idx_source : idx_target;
    int end = is_future ? idx_target : idx_source;
    double vf_integrated = 0.;
    double t = 0.;
    for( int i = start; i < end; i++ ) {
      double dt = oxts->at(i + 1).ts - oxts->at(i).ts;
      vf_integrated += (oxts->at(i + 1).vf - oxts->at(i).vf) * dt / 2.;
      t += dt;
    }
    double vf_mean = vf_integrated / t;
    return vf_mean <= thr ? TRAJECTORY_ESTIMATION_IMU : TRAJECTORY_ESTIMATION_GPS;
  }

};

//! @} emc

#endif /* __c_ego_motion_correction_h__ */
