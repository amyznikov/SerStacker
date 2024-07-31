/*
 * sply_merge.cc
 *
 *  Created on: Jul 28, 2024
 *      Author: amyznikov
 */

#include <core/io/c_sply_file.h>
#include <core/readdir.h>
#include <core/debug.h>


int main(int argc, char *argv[])
{
  std::vector<std::string> input_file_names;
  std::string output_file_name;

  std::vector<c_sply_reader::uptr> sr;
  c_sply_writer sw;

  for ( int i = 1; i < argc; ++i ) {

    if ( strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "--help") == 0 ) {
      fprintf(stdout,
          "Usage:\n"
          "   sply_merge [OPTIONS] -o output_file.sply sply_file1.sply sply_file2.sply ...\n"
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


    if ( is_regular_file(argv[i])) {

      input_file_names.emplace_back(argv[i]);

      continue;
    }

    fprintf(stderr, "Invalid argument or not an regular input file specified: %s\n", argv[i]);
    return 1;
  }

  if ( input_file_names.size() < 1 ) {
    fprintf(stderr, "No input file specified\n");
    return 1;
  }

  if ( output_file_name.empty() ) {
    output_file_name = "merge.sply";
  }


  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

  for ( const std::string & input_file_name : input_file_names )  {

    c_sply_reader::uptr sply(new c_sply_reader() );

    if ( !sply->open(input_file_name) ) {
      CF_ERROR("sply->open(input_file_name='%s') fails: %s", input_file_name.c_str(), strerror(errno));
      return 1;
    }

    if ( !sply->select_stream(sply->find_stream("PointClouds")) ) {
      CF_ERROR("Stream 'PointCloud' is not found in '%s'", input_file_name.c_str());
      return 1;
    }

    sr.emplace_back(std::move(sply));
  }


  std::vector<std::vector<cv::Mat>> points;
  std::vector<std::vector<cv::Mat>> colors;
  std::vector<std::vector<cv::Mat>> timestamps;

  std::vector<cv::Mat> merged_points;
  std::vector<cv::Mat> merged_colors;
  std::vector<cv::Mat> merged_timestamps;

  points.resize(sr.size());
  colors.resize(sr.size());
  timestamps.resize(sr.size());

  while (42) {

    bool have_data = false;

    for( uint32_t i = 0, n = sr.size(); i < n; ++i ) {

      points[i].clear();
      colors[i].clear();
      timestamps[i].clear();

      if( sr[i]->read(points[i], colors[i], timestamps[i]) ) {
        have_data = true;
      }
    }

    if( !have_data ) {
      break;
    }

    if( !sw.is_open() ) {

      if( !sw.create(output_file_name) ) {
        CF_ERROR("sw.create(output_file_name='%s') fails: %s", output_file_name.c_str(), strerror(errno));
        return 1;
      }

      if( sw.add_stream("PointClouds") != 0 ) {
        CF_ERROR("sw.add_stream('PointClouds') fails: %s", strerror(errno));
        return 1;
      }
    }

    merged_points.clear();
    merged_colors.clear();
    merged_timestamps.clear();

    for( uint32_t i = 0, n = sr.size(); i < n; ++i ) {
      merged_points.insert(merged_points.end(), points[i].begin(), points[i].end());
      merged_colors.insert(merged_colors.end(), colors[i].begin(), colors[i].end());
      merged_timestamps.insert(merged_timestamps.end(), timestamps[i].begin(), timestamps[i].end());
    }

    if( !sw.write(0, merged_points, merged_colors, merged_timestamps) ) {
      CF_ERROR("sw.write() fails: %s", strerror(errno));
      return 1;
    }
  }


  return 0;
}
