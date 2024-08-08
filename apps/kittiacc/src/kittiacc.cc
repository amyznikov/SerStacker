/*
 * kittiacc.cc
 *
 *  Created on: Aug 4, 2024
 *      Author: amyznikov
 */

#include <core/io/c_sply_file.h>
#include <core/readdir.h>
#include <core/ssprintf.h>

#include "c_kitti_dataset.h"
#include "c_ego_motion_compensation.h"
#include <core/debug.h>
#include "c_z_grid3d.h"

struct c_lidar_point
{
  float x, y, z, r;
  double t;

  c_lidar_point(float _x, float _y, float _z, float _r, double _t) :
      x(_x), y(_y), z(_z), r(_r), t(_t)
  {
  }

};

struct c_voxel_type
{
  float x = 0;
  float y = 0;
  float z = 0;
  float r = 0;
  double t = 0;
  int n = 0;
};

static void convert(const std::vector<cv::Vec3f> & points, const std::vector<float> & colors,
    const std::vector<double> & timestamps,
    std::vector<c_lidar_point> * lidar_points)
{

  lidar_points->clear();
  lidar_points->reserve(points.size());

  for ( size_t i = 0, n = points.size(); i < n; ++i ) {
    lidar_points->emplace_back(points[i][0], points[i][1], points[i][2], colors[i], timestamps[i]);
  }
}

static void convert(const std::vector<c_lidar_point> & lidar_points,
    std::vector<cv::Vec3f> * points, std::vector<float> * colors,
    std::vector<double> * timestamps )
{

  const size_t n =
      lidar_points.size();

  points->clear();
  points->reserve(n);

  colors->clear();
  colors->reserve(n);

  timestamps->clear();
  timestamps->reserve(n);

  for ( size_t i = 0; i < n; ++i ) {

    const c_lidar_point & p =
        lidar_points[i];

    points->emplace_back(p.x, p.y, p.z);
    colors->emplace_back(p.r);
    timestamps->emplace_back(p.t);
  }
}


int main(int argc, char *argv[])
{
  std::string drive_directory;
  std::string output_file_name;

  c_kitti_dataset dataset;
  c_ego_motion_compensation emc;
  oxts_data target_oxts;


  c_sply_writer sply;
  c_sply_writer splyacc;



  std::vector<c_lidar_point> lidar_points;
  std::vector<cv::Vec3f> points;
  std::vector<float> colors;
  std::vector<double> timestamps;

  std::vector<cv::Vec3f> accpoints;
  std::vector<float> acccolors;
  std::vector<double> acctimestamps;



  c_voxel_grid3d<c_voxel_type> grid;

  for ( int i = 1; i < argc; ++i ) {

    if ( strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "--help") == 0 ) {
      fprintf(stdout,
          "Usage:\n"
          "   kitti2sply [OPTIONS] path/to/kitti/xxx_drive_sync/ -o output_file.sply\n"
          "\n"
          "OPTIONS:\n"
          "\n"
      );

      return 0;
    }

    if ( strcmp(argv[i], "-o") == 0 ) {
      if ( (++i >= argc) ) {
        fprintf(stderr, "Output file name expected: %s\n", argv[i - 1]);
        return 1;
      }

      output_file_name =
          argv[i];

      continue;
    }

    if ( drive_directory.empty() && is_directory(argv[i]) ) {

      drive_directory =
          argv[i];

      continue;
    }

    fprintf(stderr, "Invalid argument or not an input directory specified: %s\n", argv[i]);
    return 1;
  }

  if ( drive_directory.empty() ) {
    fprintf(stderr, "No input directory specified\n");
    return 1;
  }

  if ( output_file_name.empty() ) {
    output_file_name = "acc.sply";
  }

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);


  if ( !dataset.load(drive_directory) ) {
    CF_ERROR("dataset.load(drive_directory='%s') fails", drive_directory.c_str());
    return 1;
  }

  if ( dataset.frames().empty()) {
    CF_ERROR("dataset.is empty");
    return 1;
  }
  else {

    const c_kitti_frame::uptr & reference_frame =
        dataset.frames()[dataset.frames().size()/2];

    if( !oxts_interpolate_for_timestamp(dataset.oxts_track().oxts(), reference_frame->tsf(), &target_oxts) ) {
      CF_ERROR("oxts_interpolate_for_timestamp(frame->tsf()=%g) fails", reference_frame->tsf());
      return 1;
    }
  }


  emc.set_trajectory(&dataset.oxts_track().oxts());
  emc.set_trajectory_estimation_method(TRAJECTORY_ESTIMATION_GPS);

  for ( const c_kitti_frame::uptr & frame : dataset.frames() ) {

    CF_DEBUG("%s", frame->cfilename());

    points.clear();
    colors.clear();
    timestamps.clear();

    if ( !frame->load_points(points, colors, timestamps) ) {
      CF_DEBUG("frame->load_points() fails");
      continue;
    }

    convert(points, colors, timestamps, &lidar_points);

    if ( !emc.compensate_ego_motion(lidar_points, lidar_points, target_oxts ) ) {
      CF_ERROR("emc.compensate_ego_motion() fails");
      return 1;
    }

    for( size_t i = 0, n = lidar_points.size(); i < n; ++i ) {

      const c_lidar_point & p =
          lidar_points[i];

      grid.insert(p.x, p.y, p.z,
          [&p](c_voxel_type & v) {
            v.x = (v.x * v.n + p.x)/ (v.n + 1);
            v.y = (v.y * v.n + p.y)/ (v.n + 1);
            v.z = (v.z * v.n + p.z)/ (v.n + 1);
            v.r = (v.r * v.n + p.r)/ (v.n + 1);
            v.t = (v.t * v.n + p.t)/ (v.n + 1);
            ++v.n;
          });
    }

    if( !splyacc.is_open() ) {

      const std::string acc_output_file_name =
          ssprintf("%s.acc.sply", output_file_name.c_str());

      if( !splyacc.create(acc_output_file_name) ) {
        CF_ERROR("splyacc.create('%s') fails: %s", acc_output_file_name.c_str(), strerror(errno));
        return 1;
      }

      if( splyacc.add_stream("PointClouds") != 0 ) {
        CF_ERROR("splyacc.add_stream('PointClouds') fails: %s", strerror(errno));
        return 1;
      }

    }

    accpoints.clear();
    acccolors.clear();
    acctimestamps.clear();

    for( auto xx = grid.xbeg(); xx != grid.xend(); ++xx ) {
      for( auto yy = (*xx)->a.begin(); yy != (*xx)->a.end(); ++yy ) {
        for( auto zz = (*yy)->a.begin(); zz != (*yy)->a.end(); ++zz ) {

          const auto & d =
              (*zz)->data;

          accpoints.emplace_back(d.x, d.y, d.z);
          acccolors.emplace_back(d.r);
          acctimestamps.emplace_back(d.t);
        }
      }
    }

    if ( !splyacc.write(0, accpoints, acccolors, acctimestamps) ) {
      CF_ERROR("splyacc.write() fails: %s", strerror(errno));
      return 1;
    }



    if( !sply.is_open() ) {

      if( !sply.create(output_file_name) ) {
        CF_ERROR("sply.create('%s') fails: %s", output_file_name.c_str(), strerror(errno));
        return 1;
      }

      if( sply.add_stream("PointClouds") != 0 ) {
        CF_ERROR("sply.add_stream('PointClouds') fails: %s", strerror(errno));
        return 1;
      }
    }

    convert(lidar_points, &points, &colors, &timestamps);

    if ( !sply.write(0, points, colors, timestamps) ) {
      CF_ERROR("sply.write() fails: %s", strerror(errno));
      return 1;
    }


  }



  sply.close();
  splyacc.close();

  return 0;
}
