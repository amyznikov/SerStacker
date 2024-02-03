/*
 * vlo_read.cc
 *
 *  Created on: Nov 20, 2023
 *      Author: amyznikov
 */
#include <core/io/c_vlo_file.h>
#include <core/debug.h>


static bool dump_table(const cv::Mat1f & table, const std::string & filename)
{
  const int src_rows =
      table.rows;

  const int src_cols =
      table.cols;

  FILE * fp =
      fopen(filename.c_str(), "w");
  if ( !fp ) {
    CF_ERROR("fopen('%s') fails: %s", filename.c_str(),
        strerror(errno));
    return false;
  }

  fprintf(fp, "R\tE\tEdeg\n");

  for ( int i = 0; i < src_rows; ++i ) {
    fprintf(fp, "%4d\t%+g\t%+g\n", i, (double) table[i][0], (double) table[i][0] * 180. / CV_PI);
  }

  fclose(fp);

  return true;
}

int main(int argc, char *argv[])
{
  std::string input_filename;
  std::string output_inclinations_filename = "ray_inclinations.txt";
  std::string output_azimuths_filename = "ray_azimuths.txt";

  c_vlo_reader reader;
  c_vlo_scan scan;
  cv::Mat1f table;

  for( int i = 1; i < argc; ++i ) {

    if( strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-help") == 0 ) {
      fprintf(stdout, "Usage:\n"
          " vlo_read <input-file-name.dat> -oi output_inclinations_filename.txt -oa output_azimuths_filename.txt\n"
          "\n"
          "\n");

      return 0;
    }

    if( input_filename.empty() ) {
      input_filename= argv[i];
      continue;
    }

    if( strcmp(argv[i], "-oi") == 0 ) {

      if( ++i >= argc ) {
        fprintf(stderr, "Missing output file name after -oi option\n");
        return 1;
      }

      output_inclinations_filename = argv[i];
      continue;
    }

    if( strcmp(argv[i], "-oa") == 0 ) {

      if( ++i >= argc ) {
        fprintf(stderr, "Missing output file name after -oa option\n");
        return 1;
      }

      output_azimuths_filename = argv[i];
      continue;
    }

    fprintf(stderr, "Invalid argument %s\n",
        argv[i]);

    return 1;
  }

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

  if( !reader.open(input_filename) ) {
    CF_ERROR("reader.open(input_filename='%s') fails",
        input_filename.c_str());
    return 1;
  }

  if ( !reader.read(&scan) ) {
    CF_ERROR("reader.read(&scan) fails");
    return 1;
  }

  table = scan.elevation;
//  if( !get_vlo_ray_inclinations_table(scan, table) ) {
//    CF_ERROR("c_vlo_file::get_ray_inclinations_table() fails");
//    return 1;
//  }

  if ( !dump_table(table, output_inclinations_filename) ) {
    CF_ERROR("dump_table('%s') fails", output_inclinations_filename.c_str());
    return 1;
  }

  table = scan.azimuth;
//  if( !get_vlo_ray_azimuths_table(scan, table) ) {
//    CF_ERROR("c_vlo_file::get_ray_azimuths_table() fails");
//    return 1;
//  }

  if ( !dump_table(table, output_azimuths_filename) ) {
    CF_ERROR("dump_table('%s') fails", output_azimuths_filename.c_str());
    return 1;
  }


  return 0;
}
