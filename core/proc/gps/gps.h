/*
 * gps.h
 *
 *  Created on: Sep 27, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __gps_h__
#define __gps_h__

#include <core/proc/pose.h>

#define HAVE_c_gps_position 1

struct c_gps_position
{
  double timestamp = 0; // [sec]
  double latitude = 40.725563 * CV_PI / 180; // [radians]
  double longitude = -74.536627 * CV_PI / 180;  // [radians]
  double altitude = 100; // [meters]
  double roll = 0; // [radians]
  double pitch = 0; // [radians]
  double yaw = 0; // [radians]
  double sensor_height = 4; // [m]
};


//@brief GPS to ECEF
//@see <https://en.wikipedia.org/wiki/Geographic_coordinate_conversion>
//@see S.P. Drake DSTO-TN-0432
cv::Vec3d gps2ecef(double lat, double lon, double alt);

//@brief ECEF to GPS.
// J. Zhu, "Conversion of Earth-centered Earth-fixed coordinates
//        to geodetic coordinates," IEEE Transactions on Aerospace and Electronic Systems,
//          vol. 30, pp. 957-961, 1994.
void ecef2gps(double x, double y, double z,
    double * lat, double * lon, double * alt);

void ecef2gps(const cv::Vec3d & ecef,
    double * lat, double * lon, double * alt);

// gps output order is [lat, lon, alt]
cv::Vec3d ecef2gps(const cv::Vec3d & ecef);


//@brief Compute rotation matrix from ENU to ECEF (inverse of ECEF to ENU rotation)
//@see   <https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ENU_to_ECEF>
cv::Matx33d enu2ecef_rotation_matrix(double lat, double lon);

//@brief ECEF to ENU Rotation matrix
//@see   <https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ECEF_to_ENU>
cv::Matx33d ecef2enu_rotation_matrix(double lat, double lon);

//@brief OxTS coordinate system conversions:
//     ISO 8855 ENU earth-fixed system (East North Up) -> ISO 8855 vehicle system
//  @see conversion between OxTS navigation and vehicle frames
//    <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
cv::Matx33d enu2velo_rotation_matrix(double roll, double pitch, double yaw);

//@brief OxTS coordinate system conversions:
//  ISO 8855 ENU earth-fixed system (East North Up) -> ISO 8855 vehicle system
//@see conversion between OxTS navigation and vehicle frames
//  <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
void enu2velo(double roll, double pitch, double yaw, const cv::Vec3d & Venu,
    /* out */cv::Vec3d * Vxyz, /* out, opt */cv::Matx33d * M = nullptr);

//@brief ENU to ECEF coordinates
//@see   <https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ENU_to_ECEF>
cv::Vec3d enu2ecef(double lat0, double lon0, double alt0,
    const cv::Vec3d & enu);

cv::Matx34d enu2ecef_matrix(double lat0, double lon0, double alt0);

//@brief OxTS coordinate system conversions:
//     ISO 8855 vehicle system -> ISO 8855 ENU earth-fixed system (East North Up)
//@see conversion between OxTS navigation and vehicle frames
//    <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
cv::Matx33d velo2enu_rotation_matrix(double roll, double pitch, double yaw);

//@brief OxTS coordinate system conversions:
// ISO 8855 vehicle system -> ISO 8855 ENU earth-fixed system (East North Up)
//@see conversion between OxTS navigation and vehicle frames
// <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
void velo2enu(double roll, double pitch, double yaw, const cv::Vec3d & Vxyz,
    /* out */cv::Vec3d * Venu, /* out, opt */cv::Matx33d * M = nullptr);


//@brief OxTS coordinate system conversions:
//     ECEF Rotation matrix ISO 8855 vehicle system -> ECEF (EARTH-CENTERED, EARTH-FIXED) XYZ
//@see conversion between OxTS navigation and vehicle frames
//    <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
//@see From_ENU_to_ECEF
//    <https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ENU_to_ECEF>
cv::Matx33d velo2ecef_rotation_matrix(double lat, double lon, double alt, double roll, double pitch, double yaw);

//@brief inverse of iso8855vs2ecef rotation
cv::Matx33d ecef2velo_rotation_matrix(double lat, double lon, double alt,
    double roll, double pitch, double yaw);

//@brief OxTS coordinate system conversions:
//     ECEF Position and/or Rotation matrix ISO 8855 vehicle system -> ECEF (EARTH-CENTERED, EARTH-FIXED) XYZ
//
//  Given position of some point vs_pos in ISO8855 VS centered at GPS lat, lon, alt, and directed along roll, pitch, yaw,
//  the ECEF position of the point can be computed as Rotation followed by positive Translation:
//@code
//    cv::Matx33d vs2ecef_rotation;
//    cv::Vec3d ecef_zeropoint_position;
//
//    iso8855vs2ecef(lat, lon, alt, roll, pitch, yaw, &vs2ecef_rotation, &ecef_zeropoint_position);
//    ecef_pos = vs2ecef_rotation * vs_pos + ecef_zeropoint_position;
//
//@see conversion between OxTS navigation and vehicle frames
//    <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
//@see From_ENU_to_ECEF
//    <https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ENU_to_ECEF>
void velo2ecef(double lat, double lon, double alt, double roll, double pitch, double yaw,
    /*out, opt*/ cv::Matx33d * Rotation,
    /*out, opt*/ cv::Vec3d * ecefPosition = nullptr);

//@brief Velocity conversion:
//     ISO 8855 ENU earth-fixed system (East North Up) -> ISO 8855 intermediate system
//@see conversion between OxTS navigation and vehicle frames
//    <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
void enu2velo_velocity(double ve, double vn, double vu, double yaw,
    /* out */ double * _vf, /* out */ double * _vl, /* out */ double * _vu,
    /* out, opt */ cv::Matx33d * M = NULL );

//@brief ISO 8855 ENU earth-fixed system (East North Up) -> ISO 8855 vehicle system
//@see conversion between OxTS navigation and vehicle frames
//   <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
void enu2velo_velocity(double ve, double vn, double vu,
    double roll, double pitch, double yaw,
    /*out*/double * vx, /*out*/double * vy, /*out*/double * vz,
    /* out, opt */cv::Matx33d * M = nullptr);

//@brief OxTS POSE :
//  ISO 8855 ENU earth-fixed system (East North Up) -> ISO 8855 vehicle system POSE
//@see conversion between OxTS navigation and vehicle frames
//    <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
void enu2velo_pose(double roll, double pitch, double yaw,
    /*in, out*/ cv::Vec3d axis[], uint nbaxis,
    /* out, opt */cv::Matx33d * M = nullptr);

//@brief OxTS POSE :
//  ISO 8855 ENU earth-fixed system (East North Up) -> ISO 8855 vehicle system POSE
//@see conversion between OxTS navigation and vehicle frames
//    <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
void velo2enu_pose(double roll, double pitch, double yaw,
    /*in, out*/ cv::Vec3d axis[], uint nbaxis,
    /* out, opt */cv::Matx33d * M = nullptr);

//@brief OxTS coordinate system conversions:
//     ISO 8855 ENU earth-fixed system (East North Up) -> ISO 8855 vehicle system
//@see conversion between OxTS navigation and vehicle frames
//    <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
cv::Matx33d enu2iso8855vs_rotation_matrix(double roll, double pitch, double yaw);

//@brief OxTS coordinate system conversions:
//     ISO 8855 vehicle system -> ISO 8855 ENU earth-fixed system (East North Up)
//  @see conversion between OxTS navigation and vehicle frames
//    <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
cv::Matx33d iso8855vs2enu_rotation_matrix(double roll, double pitch, double yaw);

//@brief inverse of iso8855vs2ecef rotation
cv::Matx33d ecef2iso8855vs_rotation_matrix(double lat, double lon, double /*alt*/,
    double roll, double pitch, double yaw);

//@brief OxTS coordinate system conversions:
//     ECEF Rotation matrix ISO 8855 vehicle system -> ECEF (EARTH-CENTERED, EARTH-FIXED) XYZ
//@see conversion between OxTS navigation and vehicle frames
//    <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
//@see From_ENU_to_ECEF
//    <https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ENU_to_ECEF>
cv::Matx33d iso8855vs2ecef_rotation_matrix(double lat, double lon, double /*alt*/,
    double roll, double pitch, double yaw);

//@brief OxTS coordinate system conversions:
//     ECEF Position and/or Rotation matrix ISO 8855 vehicle system -> ECEF (EARTH-CENTERED, EARTH-FIXED) XYZ
//
//  Given position of some point vs_pos in ISO8855 VS centered at GPS lat, lon, alt, and directed along roll, pitch, yaw,
//  the ECEF position of the point can be computed as Rotation followed by positive Translation:
//@code
//    cv::Matx33d vs2ecef_rotation;
//    cv::Vec3d ecef_zeropoint_position;
//
//    iso8855vs2ecef(lat, lon, alt, roll, pitch, yaw, &vs2ecef_rotation, &ecef_zeropoint_position);
//    ecef_pos = vs2ecef_rotation * vs_pos + ecef_zeropoint_position;
//
//@see conversion between OxTS navigation and vehicle frames
//    <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
//@see From_ENU_to_ECEF
//    <https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ENU_to_ECEF>
void iso8855vs2ecef(double lat, double lon, double alt, double roll, double pitch, double yaw,
    /*out, opt*/ cv::Matx33d * Rotation, /*out, opt*/ cv::Vec3d * ecefPosition);

#endif /* __gps_h__ */
