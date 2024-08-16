/*
 * c_kitti_dataset.cc
 *
 *  Created on: Aug 4, 2024
 *      Author: amyznikov
 */

#include "c_kitti_dataset.h"
#include "oxts-kitti.h"
#include <core/readdir.h>
#include <core/ssprintf.h>
#include <core/debug.h>




static ssize_t get_file_size(FILE * fp)
{
  ssize_t curpos, size;

  if ( (curpos = ftell(fp) ) >= 0 && fseek(fp, 0, SEEK_END) == 0 )  {
    size = ftell(fp);
#ifdef _WIN32
    fseek(fp, (long)curpos, SEEK_SET);
#else
    fseek(fp, curpos, SEEK_SET);
#endif
    return size;
  }
  return -1;
}

/** @brief Read velodine time stamps from specified file
 *
 * The expected time stamp format looks like this:
 *  2011-09-29 12:19:46.230015023
 *
 *  It is then converted to julian date as floating-point seconds
 */
static bool read_kitti_velodyne_timestamps(const std::string & tsfilename, std::vector<double> & stamps)
{
  FILE * fp;
  char line[1024];
  int year, month, day, hour, minute;
  double seconds;

  struct tm t;
  memset(&t, 0, sizeof(t));

  if( !(fp = fopen(tsfilename.c_str(), "rt")) ) {
    CF_FATAL("fopen('%s') fails: %s", tsfilename.c_str(), strerror(errno));
    return false;
  }

  while (fgets(line, sizeof(line), fp)) {
    if( sscanf(line, "%4d-%2d-%2d %2d:%2d:%lf", &year, &month, &day, &hour, &minute, &seconds) == 6 ) {

      t.tm_year = year - 1900;  // Year - 1900
      t.tm_mon = month - 1;     // months since January - [0,11]
      t.tm_mday = day;          // day of the month - [1,31]
      t.tm_hour = hour;         // hours since midnight - [0,23]
      t.tm_min = minute;        // minutes after the hour - [0,59]
      t.tm_sec = 0;             // seconds after the minute - [0,59]
      t.tm_isdst = 0;           // DST.   [-1/0/1]

      stamps.emplace_back(mktime(&t) + seconds);
    }
  }

  fclose(fp);

  return true;
}


/**@brief The same as above but after reading assigns time stamps to corresponding KITTI frame fields

  See the code of kitti_dataset::load() for details
*/
static bool read_velodyne_time_stamps(const std::string & tsfilename,
    std::vector<c_kitti_frame::uptr> & frames, double c_kitti_frame::*ts)
{
  std::vector<double> stamps;
  bool fOk = false;

  if( !read_kitti_velodyne_timestamps(tsfilename, stamps) ) {
    CF_FATAL("read_velodyne_time_stamps() fails for '%s'", tsfilename.c_str());
  }
  else if( stamps.size() != frames.size() ) {
      CF_FATAL("CRITICAL WARNING: "
          "Number of velodyne frames not match to the number of time stamps in %s.\n"
          "Time stamps will not available\n",
          tsfilename.c_str());
  }
  else {
    for( size_t i = 0, n = frames.size(); i < n; ++i ) {
      (frames[i].get())->*ts = stamps[i];
    }
    fOk = true;
  }

//  else {
//
//    if( stamps.size() != frames.size() ) {
//      CF_FATAL("CRITICAL WARNING: "
//          "Number of velodyne frames not match to the number of time stamps in %s.\n"
//          "Time stamps will not available\n",
//          tsfilename.c_str());
//    }
//
//    CF_DEBUG("H '%s' frames.size()=%zu stamps.size()=%zu", tsfilename.c_str(), frames.size(), stamps.size());
//    for( size_t i = 0, n = frames.size(); i < n; ++i ) {
//
//      const std::string fname =
//          get_file_name(frames[i]->filename());
//
//      int frame_index = -1;
//
//      CF_DEBUG("fname='%s'", fname.c_str());
//
//      int k = sscanf(fname.c_str(), "%d", &frame_index);
//
//      CF_DEBUG("k=%d frame_index=%d", k, frame_index);
//
//      if ( k != 1 ) {
//        CF_FATAL("Can not parse frame index from filename '%s'", fname.c_str());
//      }
//      else if ( frame_index < 0 || frame_index >= stamps.size() ) {
//        CF_FATAL("Invalid frame index parsed: %d for filename '%s'. stamps.size()=%zu",
//            frame_index, fname.c_str(), stamps.size());
//      }
//      else {
//        (frames[i].get())->*ts = stamps[frame_index];
//      }
//    }
//    fOk = true;
//  }

  return fOk;
}


static inline double compute_point_timestamp(const cv::Vec3f & pos, double ts, double te, double tff)
{
  double azimuth = atan2(pos[1], pos[0]); // gives [-pi..pi]
  return tff - azimuth * (te - ts) / (2 * CV_PI);
}



/**
 * Load KITTI lidar point from xxx_sync/velodyne_points/data/xxxxx.bin
 *
 * x,y, z are in meters;
 * r is reflectance;
 * a ts sweep (horizontal) angle in radians [0..2PI)
 * t is set to 0;
 *
 * For details on a and t see compute_velodyne_point_time_stamps()
 * */
static bool load_velodyne_points(FILE * fp, std::vector<cv::Vec3f> & pts,
    std::vector<float> & reflectances, std::vector<double> & timestamps,
    double tff, double tss, double tse)
{
#pragma pack(push,1)
  struct
  {
    float x, y, z, r;
  } buf[4 * 1024];
#pragma pack(pop)

  ssize_t file_size;
  size_t n;

  if( (file_size = get_file_size(fp)) % sizeof(buf[0]) ) {
    CF_FATAL("Invalid file size: %zd is not a multiple of %zu bytes", file_size, sizeof(buf[0]));
    return false;
  }

  const ssize_t reserve =
      file_size > 0 ? file_size / sizeof(buf[0]) : 100000;

  pts.reserve(reserve);
  reflectances.reserve(reserve);
  timestamps.reserve(reserve);

  while ((n = fread(buf, sizeof(buf[0]), sizeof(buf) / sizeof(buf[0]), fp)) > 0) {

    for( size_t i = 0; i < n; ++i ) {

      pts.emplace_back(buf[i].x, buf[i].y, buf[i].z);
      reflectances.emplace_back(buf[i].r);
      timestamps.emplace_back(compute_point_timestamp(pts.back(), tss, tse, tff));
    }

  }

  return true;
}


/**
 * The same as above but from file name specified
 * */
static bool load_velodyne_points(const std::string & fname, std::vector<cv::Vec3f> & pts,
    std::vector<float> & reflectances, std::vector<double> & timestamps,
    double tff, double tss, double tse)
{
  FILE * fp = nullptr;
  bool fOk = false;

  if ( !(fp = fopen(fname.c_str(), "rb")) ) {
    CF_FATAL("fopen('%s') fails: %s", fname.c_str(), strerror(errno));
  }
  else {
    fOk = load_velodyne_points(fp, pts, reflectances, timestamps, tff, tss, tse);
    fclose(fp);
  }

  return fOk;
}


bool c_kitti_frame::load_points(std::vector<cv::Vec3f> & pts, std::vector<float> & reflectances,
    std::vector<double> & timestamps)
{
  return load_velodyne_points(_filename, pts, reflectances, timestamps,
      _tsf, _tss, _tse);
}

void c_kitti_dataset::clear()
{
  _frames.clear();
  _oxts_track.clear();
}

bool c_kitti_dataset::load(const std::string & input_directory)
{
  clear();

  if ( !input_directory.empty() ) {
    _input_directory = input_directory;
  }

  setlocale(LC_ALL, "en_US.UTF-8");


  while ( !_input_directory.empty() && _input_directory.back() == '/' ) {
    _input_directory.pop_back();
  }

  if ( _input_directory.empty() ) {
    CF_ERROR("kitti_dataset: No valid dataset path specified");
    return false;
  }


  std::string parent_directory =
      get_parent_directory(_input_directory);

  while ( !parent_directory.empty() && parent_directory.back() == '/' ) {
    parent_directory.pop_back();
  }

  _velodyne_directory = ssprintf("%s/velodyne_points", _input_directory.c_str());
  _oxts_directory = ssprintf("%s/oxts", _input_directory.c_str());
  _cam2cam_file_name = ssprintf("%s/calib_cam_to_cam.txt", parent_directory.c_str());
  _velo2cam_file_name = ssprintf("%s/calib_velo_to_cam.txt", parent_directory.c_str());
  _imu2velo_file_name = ssprintf("%s/calib_imu_to_velo.txt", parent_directory.c_str());

  //
  // Read available velodyne frames
  //
  {
    std::vector<std::string> filenames;
    int status;

    const std::string datapath =
        ssprintf("%s/data", _velodyne_directory.c_str());

    if( (status = readdir(&filenames, datapath, "*.bin", true, DT_REG)) < 0 ) {
      CF_FATAL("readdir(%s) fails: %s",
          datapath.c_str(),
          strerror(errno));
      return false;
    }

    if ( filenames.empty() ) {
      CF_WARNING("WARNING: NO KITTI LIDAR FILES FOUND UNDER %s",
          datapath.c_str());
    }
    else {

      CF_DEBUG("%zu lidar files found", filenames.size());

      sort(filenames.begin(), filenames.end());

      for( const std::string & filename : filenames ) {
        _frames.emplace_back(c_kitti_frame::create_uptr(filename));
      }
    }
  }


  //
  // Read time stamps for each frame, they are in separate files for KITTI
  //
  {
    const std::string timestamps_start_file_name =
        ssprintf("%s/velodyne_points/timestamps_start.txt",
            _input_directory.c_str());

    if ( !read_velodyne_time_stamps(timestamps_start_file_name, _frames, &c_kitti_frame::_tss) ) {
      CF_FATAL("read_velodyne_time_stamps() fails: '%s'", timestamps_start_file_name.c_str());
    }

    const std::string timestamps_end_file_name =
        ssprintf("%s/velodyne_points/timestamps_end.txt",
            _input_directory.c_str());

    if( !read_velodyne_time_stamps(timestamps_end_file_name, _frames, &c_kitti_frame::_tse) ) {
      CF_FATAL("read_velodyne_time_stamps() fails: '%s'", timestamps_end_file_name.c_str());
    }

    const std::string timestamps_facing_forward_file_name =
        ssprintf("%s/velodyne_points/timestamps.txt",
            _input_directory.c_str());

    if ( !read_velodyne_time_stamps(timestamps_facing_forward_file_name, _frames, &c_kitti_frame::_tsf) ) {
      CF_FATAL("read_velodyne_time_stamps() fails: '%s'", timestamps_facing_forward_file_name.c_str());
    }
  }
  //
  // Read OXTS data
  //
  {
    std::vector<kitti_oxts_data> kitti_oxts;
    if ( !load_oxts_data(&kitti_oxts, _oxts_directory)) {
      CF_FATAL("load_oxts_data('%s') fails", _oxts_directory.c_str());
    }
    else {
      if ( kitti_oxts.size() != _frames.size() ) {
        CF_FATAL("CRITICAL WARNING: OXTS size does not match to lidar frame count:\n"
            "kitti_oxts.size()=%zu frames.size()=%zu",
            kitti_oxts.size(), _frames.size());
      }
      oxts_to_glv(kitti_oxts, &_oxts_track.oxts());
    }
  }


  return true;
}

