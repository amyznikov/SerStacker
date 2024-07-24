/*
 * kitti2sply.cc
 *
 *  Created on: Jul 24, 2024
 *      Author: amyznikov
 */

#include <core/io/c_sply_file.h>
#include <core/readdir.h>
#include <core/ssprintf.h>
#include <core/debug.h>


/**
 * Load KITTI lidar point from xxx_sync/velodyne_points/data/xxxxx.bin
 * */
static size_t load_velodyne_points(c_file_handle & fd,
    std::vector<cv::Vec3f> & points,
    std::vector<float> & colors)
{
#pragma pack(push,1)
  struct {
    float x, y, z, r;
  } buf[4 * 1024];
#pragma pack(pop)

  size_t ntotal = 0;
  ssize_t file_size;
  ssize_t cb;


  if ( (file_size = fd.size()) < 0 ) {
    // make some sane guess
    points.reserve(100*1000);
    colors.reserve(100*1000);
  }
  else if ( file_size % sizeof(buf[0]) ) {
    CF_FATAL("Invalid file size: is not a multiple of %zu bytes", sizeof(buf[0]));
    return 0;
  }
  else {
   points.reserve(file_size / sizeof(buf[0]));
   colors.reserve(file_size / sizeof(buf[0]));
  }


  fd.seek(0);

  const size_t bytes_to_read =
      sizeof(buf[0]) * (sizeof(buf) / sizeof(buf[0]));

  while ((cb = fd.read(buf, bytes_to_read)) > 0) {


    const size_t num_items =
        cb / sizeof(buf[0]);

    for( size_t i = 0; i < num_items; ++i ) {

      points.emplace_back(buf[i].x, buf[i].y, buf[i].z);
      colors.emplace_back(buf[i].r);
    }

    ntotal += num_items;
  }

  return ntotal;
}


int main(int argc, char *argv[])
{
  std::string input_directory;
  std::vector<std::string> input_file_names;
  std::string output_file_name;
  int status;

  for ( int i = 1; i < argc; ++i ) {

    if ( strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "--help") == 0 ) {
      fprintf(stdout,
          "Usage:\n"
          "   kitti2sply [OPTIONS] path_to_velodyne_points/data -o output_file.sply\n"
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

    if ( input_directory.empty() && is_directory(argv[i]) ) {

      input_directory =
          argv[i];

      continue;
    }

    fprintf(stderr, "Invalid argument or not an input directory specified: %s\n", argv[i]);
    return 1;
  }

  if ( input_directory.empty() ) {
    fprintf(stderr, "No input directory specified\n");
    return 1;
  }

  if ( output_file_name.empty() ) {
    output_file_name = "velopoints.sply";
  }

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);


  if( (status = readdir(&input_file_names, input_directory, "*.bin", true, DT_REG)) < 0 ) {
    CF_ERROR("readdir('%s') fails: %s", input_directory.c_str(), strerror(errno));
    return 1;
  }

  if ( input_file_names.empty() ) {
    CF_ERROR("No input *.bin files found at '%s'", input_directory.c_str());
    return 1;
  }

  c_sply_writer sply;

  std::vector<cv::Vec3f> points;
  std::vector<float> colors;

  std::sort(input_file_names.begin(), input_file_names.end());


  for ( const std::string & input_file_name : input_file_names ) {

    c_file_handle fd;

    if( !fd.open(input_file_name, O_RDONLY) ) {
      CF_ERROR("Can not read '%s' : %s", input_file_name.c_str(), strerror(errno));
      continue;
    }

    points.clear();
    colors.clear();

    const size_t npts =
        load_velodyne_points(fd, points, colors);

    if( !npts ) {
      CF_ERROR("load_velodyne_points('%s') fails: %s", input_file_name.c_str(), strerror(errno));
      continue;
    }

    if ( !sply.is_open() ) {

      if ( !sply.create(output_file_name) ) {
        CF_ERROR("sply.create('%s') fails: %s", output_file_name.c_str(), strerror(errno));
        return 1;
      }

      if ( sply.add_stream("PointCloud") != 0 ) {
        CF_ERROR("sply.add_stream('PointCloud') fails: %s", strerror(errno));
        return 1;
      }
    }

    CF_DEBUG("%s : %zu", c_file_name(input_file_name), npts);

    if ( !sply.write(0, points, colors) ) {
      CF_ERROR("sply.write() fails: %s", strerror(errno));
      return 1;
    }
  }

  return 0;
}

