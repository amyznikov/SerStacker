/*
 * c_hdl_lidar_specifcation.h
 *
 *  Created on: Mar 20, 2022
 *      Author: amyznikov
 */

#ifndef __c_hdl_lidar_specifcation_h__
#define __c_hdl_lidar_specifcation_h__

#include <vector>
#include <string>

enum HDLSensorType
{
  HDLSensor_unknown = -1,
  HDLSensor_VLP16 = 0x22,      // decimal: 34
  HDLSensor_VLP16HiRes = 0x24, // decimal: 36
  HDLSensor_VLP32AB = 0x23,    // decimal: 35
  HDLSensor_VLP32C = 0x28,     // decimal: 40
  HDLSensor_HDL32E = 0x21,     // decimal: 33
  HDLSensor_HDL64 = 0xA0,  // decimal: 160
  HDLSensor_VLS128 = 0xA1, // decimal: 161
};


enum HDLReturnMode
{
  HDLReturnMode_unknown = -1,
  HDL_STRONGEST_RETURN = 0x37,
  HDL_LAST_RETURN = 0x38,
  HDL_DUAL_RETURN = 0x39,
  HDL_TRIPLE_RETURN = 0x3A,
  HDL_DUAL_RETURN_WITH_CONFIDENCE = 0x3B,
};

enum HDLFramingMode {
  HDLFraming_Rotation = 0,
  HDLFraming_Packet = 1,
  //  HDLFraming_Firing_Sequence = 2,
  HDLFraming_DataBlock = 3,
};

struct c_hdl_lasers_table
{
  // laser index in packet data block
  int laser_id;

  // ring (row) index for range image
  int laser_ring;

  // Rotational (azimuth) correction in Degrees
  double rot_correction;

  // Vertical elevation in Degrees
  double vert_correction;

  // Vertical offset in [m].
  // This refers to the physical vertical offset of each of the laser blocks.
  double vert_offset;

  // Horizontal offset in [m].
  // This refers to the physical horizontal (i.e. left or right) offset of each of the laser blocks.
  double horz_offset;

  double distance_correction; // [m]

  // two-point calibration when available
  double dist_correction_x; // [m]
  double dist_correction_y; // [m]
  double focal_distance; // [m]
  double focal_slope;
  double close_slope; // Used in HDL-64 only

};

struct c_hdl_specification
{
  HDLSensorType sensor = HDLSensor_unknown;
  double distance_resolution = 0; // [m]
  std::vector<c_hdl_lasers_table> lasers;
};

inline constexpr int num_lasers(HDLSensorType sensorType)
{
  switch (sensorType)
  {
  case HDLSensor_HDL64:
    return 64;
  case HDLSensor_HDL32E:
    case HDLSensor_VLP32AB:
    case HDLSensor_VLP32C:
    return 32;
  case HDLSensor_VLP16:
    case HDLSensor_VLP16HiRes:
    return 16;
  case HDLSensor_VLS128:
    return 128;
  default:
    break;
  }

  return 0;
}

bool set_hdl_lidar_specification_config_file(HDLSensorType sensor_type,
    const std::string & config_file_pathname);

std::string get_hdl_lidar_specification_config_file(
    HDLSensorType sensor_type);

const c_hdl_specification * get_default_hdl_lidar_specification(
    HDLSensorType sensor_type);

#endif /* __c_hdl_lidar_specifcation_h__ */
