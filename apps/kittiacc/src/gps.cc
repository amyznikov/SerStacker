/*
 * gps.cc
 *
 *  Created on: Sep 27, 2022
 *      Author: amyznikov
 *
 *  See also:
 *    https://github.com/ethz-asl/geodetic_utils
 *
 */

#include "gps.h"
#include <cmath>


static constexpr double gEarthSemimajorAxis = 6378137.0;
static constexpr double gEarthSemiminorAxis = 6356752.3142;
static constexpr double gEarthFirstEccentricitySquared = 6.69437999014e-3;
static constexpr double gEarthSecondEccentricitySquared = 6.73949674228e-3;


//@brief GPS to ECEF
//@see <https://en.wikipedia.org/wiki/Geographic_coordinate_conversion>
//@see S.P. Drake DSTO-TN-0432
cv::Vec3d gps2ecef(double lat, double lon, double alt)
{
  // Earth parameters.
  // @see <http://wiki-1-1930356585.us-east-1.elb.amazonaws.com/wiki/index.php/Geodetic_system>
  const double a = gEarthSemimajorAxis; // semi-major axis in meters
  const double e2 = gEarthFirstEccentricitySquared; // First eccentricity squared

  const double h =
      sqrt(1.0 - e2 * sin(lat) * sin(lat));

  return cv::Vec3d(
      (a / h + alt) * cos(lat) * cos(lon),
      (a / h + alt) * cos(lat) * sin(lon),
      (a * (1 - e2) / h + alt) * sin(lat));
}



//@brief ECEF to GPS.
// J. Zhu, "Conversion of Earth-centered Earth-fixed coordinates
//        to geodetic coordinates," IEEE Transactions on Aerospace and Electronic Systems,
//          vol. 30, pp. 957-961, 1994.
void ecef2gps(double x, double y, double z, double * lat, double * lon, double * alt)
{
  const double r = sqrt(x * x + y * y);
  const double Esq = gEarthSemimajorAxis * gEarthSemimajorAxis - gEarthSemiminorAxis * gEarthSemiminorAxis;
  const double F = 54 * gEarthSemiminorAxis * gEarthSemiminorAxis * z * z;
  const double G = r * r + (1 - gEarthFirstEccentricitySquared) * z * z - gEarthFirstEccentricitySquared * Esq;
  const double C = (gEarthFirstEccentricitySquared * gEarthFirstEccentricitySquared * F * r * r) / pow(G, 3);
  const double S = cbrt(1 + C + sqrt(C * C + 2 * C));
  const double P = F / (3 * pow((S + 1 / S + 1), 2) * G * G);
  const double Q = sqrt(1 + 2 * gEarthFirstEccentricitySquared * gEarthFirstEccentricitySquared * P);

  const double r_0 = -(P * gEarthFirstEccentricitySquared * r) / (1 + Q)
      + sqrt( 0.5 * gEarthSemimajorAxis * gEarthSemimajorAxis * (1 + 1.0 / Q)
              - P * (1 - gEarthFirstEccentricitySquared) * z * z / (Q * (1 + Q)) - 0.5 * P * r * r);

  const double U = sqrt(pow((r - gEarthFirstEccentricitySquared * r_0), 2) + z * z);
  const double V = sqrt(pow((r - gEarthFirstEccentricitySquared * r_0), 2) + (1 - gEarthFirstEccentricitySquared) * z * z);
  const double Z_0 = gEarthSemiminorAxis * gEarthSemiminorAxis * z / (gEarthSemimajorAxis * V);

  *alt = U * (1 - gEarthSemiminorAxis * gEarthSemiminorAxis / (gEarthSemimajorAxis * V));
  *lat = atan((z + gEarthSecondEccentricitySquared * Z_0) / r);
  *lon = atan2(y, x);
}

void ecef2gps(const cv::Vec3d & ecef, double* lat, double* lon, double* alt)
{
  ecef2gps(ecef[0], ecef[1], ecef[2], lat, lon, alt);
}

cv::Vec3d ecef2gps(const cv::Vec3d & ecef)
{
  cv::Vec3d gps;
  ecef2gps(ecef, &gps[0], &gps[1], &gps[2]);
  return gps;
}


//@brief Compute rotation matrix from ENU to ECEF (inverse of ECEF to ENU rotation)
//@see   <https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ENU_to_ECEF>
cv::Matx33d enu2ecef_rotation_matrix(double lat, double lon)
{
  double clat, slat, clon, slon;

  sincos(lat, &slat, &clat);
  sincos(lon, &slon, &clon);

  return cv::Matx33d(
      -sin(lon), -sin(lat) * cos(lon), cos(lat) * cos(lon),
      cos(lon), -sin(lat) * sin(lon), cos(lat) * sin(lon),
      0, cos(lat), sin(lat));
}

//@brief ECEF to ENU Rotation matrix
//@see   <https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ECEF_to_ENU>
cv::Matx33d ecef2enu_rotation_matrix(double lat, double lon)
{
  double clat, slat, clon, slon;

  sincos(lat, &slat, &clat);
  sincos(lon, &slon, &clon);

  return cv::Matx33d(
      -slon, clon, 0,
      -slat * clon, -slat * slon, clat,
      clat * clon, clat * slon, slat);
}


//@brief OxTS coordinate system conversions:
//     ISO 8855 ENU earth-fixed system (East North Up) -> ISO 8855 vehicle system
//@see conversion between OxTS navigation and vehicle frames
//    <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
cv::Matx33d enu2velo_rotation_matrix(double roll, double pitch, double yaw)
{
  cv::Matx33d L, T, Y;

  build_rotation(-roll, -pitch, -yaw,
      &L, &T, &Y);

  return L * T * Y;
}

//@brief OxTS coordinate system conversions:
//     ISO 8855 vehicle system -> ISO 8855 ENU earth-fixed system (East North Up)
//  @see conversion between OxTS navigation and vehicle frames
//    <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
cv::Matx33d velo2enu_rotation_matrix(double roll, double pitch, double yaw)
{
  cv::Matx33d L, T, Y;

  build_rotation(roll, pitch, yaw,
      &L, &T, &Y);

  return  Y * T * L;
}

//@brief OxTS coordinate system conversions:
// ISO 8855 ENU earth-fixed system (East North Up) -> ISO 8855 vehicle system
//@see conversion between OxTS navigation and vehicle frames
// <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
void enu2velo(double roll, double pitch, double yaw, const cv::Vec3d & Venu,
    /* out */cv::Vec3d * Vxyz, /* out, opt */cv::Matx33d * M )
{
  const cv::Matx33d R =
      enu2velo_rotation_matrix(roll, pitch, yaw);

  *Vxyz =
      R * Venu;

  if ( M ) {
    *M = R;
  }
}

//@brief ENU to ECEF coordinates
//@see   <https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ENU_to_ECEF>
cv::Vec3d enu2ecef(double lat0, double lon0, double alt0, const cv::Vec3d & enu)
{
  const cv::Matx33d Rotation =
      enu2ecef_rotation_matrix(lat0, lon0);

  const cv::Vec3d Translation =
      gps2ecef(lat0, lon0, alt0);

  return Rotation * enu + Translation;
}

cv::Matx34d enu2ecef_matrix(double lat0, double lon0, double alt0)
{
  const cv::Matx33d R =
      enu2ecef_rotation_matrix(lat0, lon0);

  const cv::Vec3d T =
      gps2ecef(lat0, lon0, alt0);

  return cv::Matx34d(
      R(0,0), R(0,1), R(0,2), T(0),
      R(1,0), R(1,1), R(1,2), T(1),
      R(2,0), R(2,1), R(2,2), T(2));

}

//@brief OxTS coordinate system conversions:
// ISO 8855 vehicle system -> ISO 8855 ENU earth-fixed system (East North Up)
//@see conversion between OxTS navigation and vehicle frames
// <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
void velo2enu(double roll, double pitch, double yaw, const cv::Vec3d & Vxyz,
    /* out */cv::Vec3d * Venu, /* out, opt */cv::Matx33d * M )
{
  const cv::Matx33d R =
      velo2enu_rotation_matrix(roll, pitch, yaw);

  *Venu =
      R * Vxyz;

  if ( M ) {
    *M = R;
  }
}



//@brief ISO 8855 ENU earth-fixed system (East North Up) -> ISO 8855 vehicle system
//@see conversion between OxTS navigation and vehicle frames
//   <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
void enu2velo_velocity(double ve, double vn, double vu,
    double roll, double pitch, double yaw,
    /*out*/double * vx, /*out*/double * vy, /*out*/double * vz,
    /* out, opt */cv::Matx33d * _M )
{
  cv::Vec3d Vxyz;

  enu2velo(roll, pitch, yaw,
      cv::Vec3d(ve, vn, vu),
      &Vxyz,
      _M);

  if ( vx ) {
    *vx = Vxyz[0];
  }

  if ( vy ) {
    *vy = Vxyz[1];
  }

  if ( vz ) {
    *vz = Vxyz[2];
  }
}


//@brief ISO 8855 ENU earth-fixed system (East North Up) -> ISO 8855 vehicle system POSE
//@see conversion between OxTS navigation and vehicle frames
//    <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
void enu2velo_pose(double roll, double pitch, double yaw,
    /*in, out*/ cv::Vec3d axis[], uint nbaxis,
    /* out, opt */cv::Matx33d * /*M*/)
{
  const cv::Matx33d R =
      enu2velo_rotation_matrix(-roll, -pitch, -yaw);

  for ( uint i = 0; i < nbaxis; ++i ) {
    axis[i] = R * axis[i];
  }
}

//@brief OxTS POSE :
//     ISO 8855 ENU earth-fixed system (East North Up) -> ISO 8855 vehicle system POSE
//@see conversion between OxTS navigation and vehicle frames
//    <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
void velo2enu_pose(double roll, double pitch, double yaw,
    /*in, out*/ cv::Vec3d axis[], uint nbaxis,
    /* out, opt */cv::Matx33d * /*M*/)
{
  const cv::Matx33d R =
      velo2enu_rotation_matrix(roll, pitch, yaw);

  for ( uint i = 0; i < nbaxis; ++i ) {
    axis[i] = R * axis[i];
  }
}


//@brief Velocity conversion:
//     ISO 8855 ENU earth-fixed system (East North Up) -> ISO 8855 intermediate system
//@see conversion between OxTS navigation and vehicle frames
//    <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
void enu2velo_velocity(double ve, double vn, double vu, double yaw,
    /* out, opt */double * _vf,
    /* out, opt */double * _vl,
    /* out, opt */double * _vu,
    /* out, opt */cv::Matx33d * M)
{
  cv::Matx33d Y;

  build_rotation(0., 0., -yaw,
      (cv::Matx33d *)nullptr,
      (cv::Matx33d *)nullptr,
      &Y);

  if ( M ) {
    *M = Y;
  }

  if ( _vf || _vl || _vu ) {

    const cv::Vec3d Vh =
        Y * cv::Vec3d(ve, vn, vu);

    if ( _vf ) {
      *_vf = Vh[0];
    }
    if ( _vl ) {
      *_vl = -Vh[1];
    }
    if ( _vu ) {
      *_vu = Vh[2];
    }
  }
}

//@brief inverse of iso8855vs2ecef rotation
cv::Matx33d ecef2iso8855vs_rotation_matrix(double lat, double lon, double /*alt*/,
    double roll, double pitch, double yaw)
{
  cv::Matx33d ecef_to_enu_rotation, enu_to_vs_rotation;

  ecef_to_enu_rotation = ecef2enu_rotation_matrix(lat, lon);
  enu_to_vs_rotation = enu2iso8855vs_rotation_matrix(roll, pitch, yaw);
  return enu_to_vs_rotation * ecef_to_enu_rotation;
}



//@brief OxTS coordinate system conversions:
//     ECEF Rotation matrix ISO 8855 vehicle system -> ECEF (EARTH-CENTERED, EARTH-FIXED) XYZ
//@see conversion between OxTS navigation and vehicle frames
//    <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
//@see From_ENU_to_ECEF
//    <https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ENU_to_ECEF>
cv::Matx33d velo2ecef_rotation_matrix(double lat, double lon, double /*alt*/, double roll, double pitch, double yaw)
{
  const cv::Matx33d vs_to_enu_rotation =
      velo2enu_rotation_matrix(roll, pitch, yaw);

  const cv::Matx33d enu_to_ecef_rotation =
      enu2ecef_rotation_matrix(lat, lon);

 return enu_to_ecef_rotation * vs_to_enu_rotation;
}

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
    /*out, opt*/ cv::Matx33d * Rotation, /*out, opt*/ cv::Vec3d * ecefPosition)
{
  if( ecefPosition ) {

    *ecefPosition =
        gps2ecef(lat, lon, alt);
  }

  if( Rotation ) {
    *Rotation =
        velo2ecef_rotation_matrix(lat, lon, alt,
            roll, pitch, yaw);
  }
}



//@brief inverse of iso8855vs2ecef rotation
cv::Matx33d ecef2velo_rotation_matrix(double lat, double lon, double /*alt*/,
    double roll, double pitch, double yaw)
{
  const cv::Matx33d ecef_to_enu_rotation =
      ecef2enu_rotation_matrix(lat, lon);

  const cv::Matx33d enu_to_vs_rotation =
      enu2velo_rotation_matrix(roll, pitch, yaw);

  return enu_to_vs_rotation * ecef_to_enu_rotation;
}


//@brief OxTS coordinate system conversions:
//     ISO 8855 ENU earth-fixed system (East North Up) -> ISO 8855 vehicle system
//@see conversion between OxTS navigation and vehicle frames
//    <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
cv::Matx33d enu2iso8855vs_rotation_matrix(double roll, double pitch, double yaw)
{
  cv::Matx33d L, T, Y;
  build_rotation(-roll, -pitch, -yaw, &L, &T, &Y);
  return L * T * Y;
}

//@brief OxTS coordinate system conversions:
//     ISO 8855 vehicle system -> ISO 8855 ENU earth-fixed system (East North Up)
//  @see conversion between OxTS navigation and vehicle frames
//    <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
cv::Matx33d iso8855vs2enu_rotation_matrix(double roll, double pitch, double yaw)
{
  cv::Matx33d L, T, Y;
  build_rotation(roll, pitch, yaw, &L, &T, &Y);
  return Y * T * L;
}



//@brief OxTS coordinate system conversions:
//     ECEF Rotation matrix ISO 8855 vehicle system -> ECEF (EARTH-CENTERED, EARTH-FIXED) XYZ
//@see conversion between OxTS navigation and vehicle frames
//    <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
//@see From_ENU_to_ECEF
//    <https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ENU_to_ECEF>
cv::Matx33d iso8855vs2ecef_rotation_matrix(double lat, double lon, double /*alt*/, double roll, double pitch, double yaw)
{
  cv::Matx33d vs_to_enu_rotation;
  cv::Matx33d enu_to_ecef_rotation;

  vs_to_enu_rotation = iso8855vs2enu_rotation_matrix(roll, pitch, yaw);
  enu_to_ecef_rotation = enu2ecef_rotation_matrix(lat, lon);

  return enu_to_ecef_rotation * vs_to_enu_rotation;
}

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
    /*out, opt*/ cv::Matx33d * Rotation, /*out, opt*/ cv::Vec3d * ecefPosition)
{
  if ( ecefPosition ) {
    *ecefPosition = gps2ecef(lat, lon, alt);
  }

  if( Rotation ) {
    *Rotation = iso8855vs2ecef_rotation_matrix(lat, lon, alt, roll, pitch, yaw);
  }
}
