/*
 * oxts-kitti.h
 *
 *  Created on: Sep 6, 2018
 *      Author: amyznikov
 */

#pragma once
#ifndef __oxts_kitti_h__
#define __oxts_kitti_h__

#include "oxts.h"

///////////////////////////////////////////////////////////////////////////////

//! @addtogroup kitti
//! @{

/**@brief In-memory representation of KITTI  GPS/IMU 3D localization unit
 *
 *  See: readme.txt in
 *    http://kitti.is.tue.mpg.de/kitti/devkit_raw_data.zip
 *
 * The GPS/IMU information is given in a single small text file
 * which is written for each synchronized frame.
 * Each text file contains 30 values which are:
 * */
struct kitti_oxts_data {
  double ts;    //  time stamp in julian seconds
  double lat;   //  latitude of the oxts-unit (deg)
  double lon;   //  longitude of the oxts-unit (deg)
  double alt;   //  altitude of the oxts-unit (m)
  double roll;  //  roll angle (rad),  0 = level, positive = left side up (-pi..pi)
  double pitch; //  pitch angle (rad), 0 = level, positive = front down (-pi/2..pi/2)
  double yaw;   //  heading (rad),     0 = east,  positive = counter clockwise (-pi..pi)
  double vn;    //  velocity towards north (m/s)
  double ve;    //  velocity towards east (m/s)
  double vf;    //  forward velocity, i.e. parallel to earth-surface (m/s)
  double vl;    //  leftward velocity, i.e. parallel to earth-surface (m/s)
  double vu;    //  upward velocity, i.e. perpendicular to earth-surface (m/s)
  double ax;    //  acceleration in x, i.e. in direction of vehicle front (m/s^2)
  double ay;    //  acceleration in y, i.e. in direction of vehicle left (m/s^2)
  double az;    //  acceleration in z, i.e. in direction of vehicle top (m/s^2)
  double af;    //  forward acceleration (m/s^2)
  double al;    //  leftward acceleration (m/s^2)
  double au;    //  upward acceleration (m/s^2)
  double wx;    //  angular rate around x (rad/s)
  double wy;    //  angular rate around y (rad/s)
  double wz;    //  angular rate around z (rad/s)
  double wf;    //  angular rate around forward axis (rad/s)
  double wl;    //  angular rate around leftward axis (rad/s)
  double wu;    //  angular rate around upward axis (rad/s)
  double posacc;    //  position accuracy (north/east in m)
  double velacc;    //  velocity accuracy (north/east in m/s)
  int navstat;    //  navigation status
  int numsats;    //  number of satellites tracked by primary GPS receiver
  int posmode;    //  position mode of primary GPS receiver
  int velmode;    //  velocity mode of primary GPS receiver
  int orimode;    //  orientation mode of primary GPS receiver
};


/** @brief just resets all oxts data fields to zeros
 * */
void clear_oxts_data(kitti_oxts_data * oxts);



/** @brief Read oxts_data from specified KITTI oxts data stream

@code
  struct kitti_oxts_data oxts;
  if ( ! load_kitti_oxts_data(&oxts, stdin) ) {
    CF_FATAL("Can't load OXTS data stream, see error log for details");
  }
@endcode
 */
bool load_oxts_data(kitti_oxts_data * oxts,
    FILE * fp);




/**@brief Read oxts_data from specified KITTI oxts file

@code
  struct kitti_oxts_data oxts;
  string fname = "/data/kitti/2011_09_26/2011_09_26_drive_0001_extract/oxts/data/0000000000.txt"
  if ( ! load_kitti_oxts_data(&oxts, fname) ) {
    CF_FATAL("Can't load OXTS file, see error log for details");
  }
@endcode
 * */
bool load_oxts_data(kitti_oxts_data * oxts,
    const std::string & fname);




/** @brief Read kitti_oxts_data from specified set of KITTI OXTS and time stamp files

  The oxts_folder_path must point to the KITTI oxts/ folder inside of a some KITTI dataset,
  containing the oxts/timestamps.txt and oxts/data/xxxxxxxxx.txt files.
  To something like 2011_09_26/2011_09_26_drive_0001_extract/oxts

  This routine reads OXTS data from oxts/data/[*.txt] files
   and time stamps from and oxts/timestamps.txt  files


@code
  vector<struct kitti_oxts_data> oxts;
  string oxts_path = "/data/kitti/2011_09_26/2011_09_26_drive_0001_extract/oxts"

  if ( ! load_oxts_data(&oxts, oxts_path) ) {
    CF_FATAL("Can't load OXTS directory, see error log for details");
  }
@endcode
 */
bool load_oxts_data(std::vector<kitti_oxts_data> * oxts,
    const std::string & oxts_folder_path);


/**@brief Convert kitti_oxts_data to general glv representation

   For KITTI there is no conversion need, just copy data fields
 */
void oxts_to_glv(const kitti_oxts_data & src,
    oxts_data * dst);


/**@brief Convert kitti_oxts_data to general glv representation

   For KITTI there is no conversion need, just copy data fields, but for ZU and for synthetics
   some additional conversion may require.

   Note - this function will NOT clear existing data under destination, but ADD new data from source instead.

@code
  vector<struct kitti_oxts_data> kitti_oxts;
  load_oxts_data(&kitti_oxts, kitti_oxts_path);

  vector<struct oxts_data> oxts;
  oxts_to_glv(kitti_oxts, &oxts);
@endcode
 */
void oxts_to_glv(const std::vector<kitti_oxts_data> & src,
    std::vector<oxts_data> * dst);

//! @} kitti
///////////////////////////////////////////////////////////////////////////////
#endif /* __oxts_kitti_h__ */
