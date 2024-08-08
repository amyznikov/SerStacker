/*
 * oxts-kitti.cc
 *
 *  Created on: Sep 5, 2018
 *      Author: amyznikov
 */

#ifdef _MSC_VER
# pragma warning (disable:4996)
# define _CRT_SECURE_NO_WARNINGS
# define _USE_MATH_DEFINES
# define strcasecmp(a, b) 	_stricmp(a, b)
#endif 

#include "oxts-kitti.h"
#include "kitti-timestamp.h"

#include <core/ssprintf.h>
#include <core/readdir.h>
#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////
using namespace std;



/** @brief just resets all oxts data fields to zeros
 * */
void clear_oxts_data(kitti_oxts_data * c)
{
  memset(c, 0, sizeof(*c));
}


/** @brief Read oxts_data from specified KITTI oxts file
 * */
bool load_oxts_data(kitti_oxts_data * oxts, FILE * fp)
{
  /* There is a bug in *SOME* KITTI oxts data files, when integers are saved as floats,
   * the below is very stupid workaround */
  double navstat = 0;
  double numsats = 0;
  double posmode = 0;
  double velmode = 0;
  double orimode = 0;

  clear_oxts_data(oxts);

  int n = fscanf(fp, ""
      " %lf"  // double lat;   //  latitude of the oxts-unit (deg)
      " %lf"  // double lon;   //  longitude of the oxts-unit (deg)
      " %lf"  // double alt;   //  altitude of the oxts-unit (m)
      " %lf"  // double roll;  //  roll angle (rad),  0 = level, positive = left side up (-pi..pi)
      " %lf"  // double pitch; //  pitch angle (rad), 0 = level, positive = front down (-pi/2..pi/2)
      " %lf"  // double yaw;   //  heading (rad),     0 = east,  positive = counter clockwise (-pi..pi)
      " %lf"  // double vn;    //  velocity towards north (m/s)
      " %lf"  // double ve;    //  velocity towards east (m/s)
      " %lf"  // double vf;    //  forward velocity, i.e. parallel to earth-surface (m/s)
      " %lf"  // double vl;    //  leftward velocity, i.e. parallel to earth-surface (m/s)
      " %lf"  // double vu;    //  upward velocity, i.e. perpendicular to earth-surface (m/s)
      " %lf"  // double ax;    //  acceleration in x, i.e. in direction of vehicle front (m/s^2)
      " %lf"  // double ay;    //  acceleration in y, i.e. in direction of vehicle left (m/s^2)
      " %lf"  // double az;    //  acceleration in z, i.e. in direction of vehicle top (m/s^2)
      " %lf"  // double af;    //  forward acceleration (m/s^2)
      " %lf"  // double al;    //  leftward acceleration (m/s^2)
      " %lf"  // double au;    //  upward acceleration (m/s^2)
      " %lf"  // double wx;    //  angular rate around x (rad/s)
      " %lf"  // double wy;    //  angular rate around y (rad/s)
      " %lf"  // double wz;    //  angular rate around z (rad/s)
      " %lf"  // double wf;    //  angular rate around forward axis (rad/s)
      " %lf"  // double wl;    //  angular rate around leftward axis (rad/s)
      " %lf"  // double wu;    //  angular rate around upward axis (rad/s)
      " %lf"  // double posacc;    //  velocity accuracy (north/east in m)
      " %lf"  // double velacc;    //  velocity accuracy (north/east in m/s)
      " %lf"  // int navstat;    //  navigation status
      " %lf"  // int numsats;    //  number of satellites tracked by primary GPS receiver
      " %lf"  // int posmode;    //  position mode of primary GPS receiver
      " %lf"  // int velmode;    //  velocity mode of primary GPS receiver
      " %lf", // int orimode;    //  orientation mode of primary GPS receiver
      &oxts->lat,
      &oxts->lon,
      &oxts->alt,
      &oxts->roll,
      &oxts->pitch,
      &oxts->yaw,
      &oxts->vn,
      &oxts->ve,
      &oxts->vf,
      &oxts->vl,
      &oxts->vu,
      &oxts->ax,
      &oxts->ay,
      &oxts->az,
      &oxts->af,
      &oxts->al,
      &oxts->au,
      &oxts->wx,
      &oxts->wy,
      &oxts->wz,
      &oxts->wf,
      &oxts->wl,
      &oxts->wu,
      &oxts->posacc,
      &oxts->velacc,
      &navstat,
      &numsats,
      &posmode,
      &velmode,
      &orimode);

  if ( n != 30 ) {
    errno = EPROTO;
    CF_FATAL("fscanf(oxts_data) fails: n=%d errno=%s", n, strerror(errno));
    return false;
  }

  oxts->navstat = (int)navstat;
  oxts->numsats = (int)numsats;
  oxts->posmode = (int)posmode;
  oxts->velmode = (int)velmode;
  oxts->orimode = (int)orimode;

  /* convert from KITTI to GLV OXTS units */
  oxts->lat *= M_PI / 180; // degs -> radians
  oxts->lon *= M_PI / 180; // degs -> radians

  return true;
}


/** @brief Read oxts_data from specified KITTI oxts file
 * */
bool load_oxts_data(kitti_oxts_data * c, const string & fname)
{
  FILE * fp = NULL;
  bool fOk = false;

  if ( !(fp = fopen(fname.c_str(), "rt")) ) {
    CF_FATAL("fopen('%s') fails: %s", fname.c_str(), strerror(errno));
  }
  else if ( !(fOk = load_oxts_data(c, fp)) ) {
    CF_FATAL("load_oxts_data('%s') fails: %s", fname.c_str(), strerror(errno));
  }

  if ( fp ) {
    fclose(fp);
  }

  return fOk;
}





/** @brief Read kitti_oxts_data from specified set of KITTI oxts and time stamp files
 *
 *  The oxts_folder_path must point to the KITTI oxts folder inside of a some dataset,
 *  containing the oxts/timestamps.txt and oxts/data/xxxxxxxxx.txt files,
 *  for example 2011_09_26/2011_09_26_drive_0001_extract/oxts/
 *
 *  This routine reads OXTS data from oxts/data/[*.txt] files
 *  and time stamps from and oxts/timestamps.txt  files
 *
 *  The time stamps are converted to julian seconds
 *
 * */
bool load_oxts_data(vector<kitti_oxts_data> * c, const string & oxts_folder_path)
{
  // format path like to 2011_09_26/2011_09_26_drive_0001_extract/oxts/data
  const string oxts_data_folder_path =
      ssprintf("%s/data", oxts_folder_path.c_str());

  // format path like to 2011_09_26/2011_09_26_drive_0001_extract/oxts/timestamps.txt
  const string ts_file_name =
      ssprintf("%s/timestamps.txt", oxts_folder_path.c_str());

  vector<string> oxts_files;
  vector<double> tstamps;
  kitti_oxts_data oxts;


  //
  // try to list the oxts files
  //
  if ( readdir(&oxts_files, oxts_data_folder_path, "??????????.txt", true, DT_REG) < 0 ) {
    CF_FATAL("readdir('%s') fails", oxts_data_folder_path.c_str());
    return false;
  }

  if ( oxts_files.empty() ) {
    CF_FATAL("No KITI OXTS files found under '%s' directory", oxts_data_folder_path.c_str());
    return false;
  }




  //
  // try to load time stamps
  //
  if ( !read_kitti_velodyne_timestamps(ts_file_name, tstamps) ) {
    CF_FATAL("read_kitti_velodyne_timestamps('%s') fails", ts_file_name.c_str());
    return false;
  }

  if ( tstamps.size() != oxts_files.size() ) {
    CF_FATAL("ERROR: The number of time stamps does not match to number of oxts files under '%s'",
        oxts_folder_path.c_str());
    return false;
  }

  //
  // read oxts data and assign time stamps
  //
  sort(oxts_files.begin(), oxts_files.end());

  for ( size_t i = 0, n = tstamps.size(); i < n; ++i ) {
    if ( !load_oxts_data(&oxts, oxts_files[i]) ) {
      CF_FATAL("ERROR: load_kitti_oxts_data() fails for '%s'", oxts_files[i].c_str());
    }
    else {
      oxts.ts = tstamps[i];
      c->emplace_back(oxts);
    }
  }

  return true;
}

/**@brief Convert kitti_oxts_data to general glv representation

   For KITTI there is no conversion need, just copy data fields
 */
void oxts_to_glv(const kitti_oxts_data & src, oxts_data * dst)
{
  dst->ts = src.ts;    //  time stamp in julian seconds
  dst->lat = src.lat;   //  latitude of the oxts-unit (deg)
  dst->lon = src.lon;   //  longitude of the oxts-unit (deg)
  dst->alt = src.alt;   //  altitude of the oxts-unit (m)
  dst->roll = src.roll;  //  roll angle (rad),  0 = level, positive = left side up (-pi..pi)
  dst->pitch = src.pitch; //  pitch angle (rad), 0 = level, positive = front down (-pi/2..pi/2)
  dst->yaw = src.yaw;   //  heading (rad),     0 = east,  positive = counter clockwise (-pi..pi)
  dst->vn = src.vn;    //  velocity towards north (m/s)
  dst->ve = src.ve;    //  velocity towards east (m/s)
  dst->vf = src.vf;    //  forward velocity, i.e. parallel to earth-surface (m/s)
  dst->vl = src.vl;    //  leftward velocity, i.e. parallel to earth-surface (m/s)
  dst->vu = src.vu;    //  upward velocity, i.e. perpendicular to earth-surface (m/s)
  dst->ax = src.ax;    //  acceleration in x, i.e. in direction of vehicle front (m/s^2)
  dst->ay = src.ay;    //  acceleration in y, i.e. in direction of vehicle left (m/s^2)
  dst->az = src.az;    //  acceleration in z, i.e. in direction of vehicle top (m/s^2)
  dst->af = src.af;    //  forward acceleration (m/s^2)
  dst->al = src.al;    //  leftward acceleration (m/s^2)
  dst->au = src.au;    //  upward acceleration (m/s^2)
  dst->wx = src.wx;    //  angular rate around x (rad/s)
  dst->wy = src.wy;    //  angular rate around y (rad/s)
  dst->wz = src.wz;    //  angular rate around z (rad/s)
  dst->wf = src.wf;    //  angular rate around forward axis (rad/s)
  dst->wl = src.wl;    //  angular rate around leftward axis (rad/s)
  dst->wu = src.wu;    //  angular rate around upward axis (rad/s)
  dst->posacc = src.posacc;    //  velocity accuracy (north/east in m)
  dst->velacc = src.velacc;    //  velocity accuracy (north/east in m/s)
  dst->navstat = src.navstat;    //  navigation status
  dst->numsats = src.numsats;    //  number of satellites tracked by primary GPS receiver
  dst->posmode = src.posmode;    //  position mode of primary GPS receiver
  dst->velmode = src.velmode;    //  velocity mode of primary GPS receiver
  dst->orimode = src.orimode;    //  orientation mode of primary GPS receiver

}


/**@brief Convert kitti_oxts_data to general glv representation

   For KITTI there is no conversion need, just copy data fields
 */
void oxts_to_glv(const std::vector<kitti_oxts_data> & src, std::vector<struct oxts_data> * dst)
{
  const size_t src_size = src.size();
  const size_t start_index = dst->size();

  dst->resize(dst->size() + src.size());

  for ( size_t i = 0; i < src_size; ++i ) {
    oxts_to_glv(src[i], &(*dst)[i + start_index]);
  }
}













///////////////////////////////////////////////////////////////////////////////




