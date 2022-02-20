/*
 * ffmpeg_read.cc
 *
 *  Created on: Dec 2, 2020
 *      Author: amyznikov
 */

#include <core/io/c_ffmpeg_file.h>
#include <core/readdir.h>
#include <core/debug.h>


int main(int argc, char *argv[])
{
  std::string input_file_name;
  std::string output_path;
  std::string input_options;

  for ( int i = 1; i < argc; ++i ) {

    if ( strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "--help") == 0 ) {
      fprintf(stdout,
          "Usage:\n"
          "   ffmpeg_read [OPTIONS] input_video -o output_path\n"
          "\n"
          "OPTIONS:\n"
          "  -opts \"ffmpeg input options\"\n"
          "\n"
      );

      return 0;
    }

    if ( strcmp(argv[i], "-o") == 0 ) {
      if ( (++i >= argc) ) {
        fprintf(stderr, "Output file name expected: %s\n", argv[i - 1]);
        return 1;
      }
      output_path = argv[i];
    }


    else if ( strcmp(argv[i], "-opts") == 0 ) {
      if ( (++i >= argc) ) {
        fprintf(stderr, "ffmpeg input options expected: %s\n", argv[i - 1]);
        return 1;
      }
      input_options = argv[i];
    }


    else if ( input_file_name.empty() && is_regular_file(argv[i]) ) {
      input_file_name = argv[i];
    }


    else {
      fprintf(stderr, "Invalid argument or not existimng input file specified: %s\n", argv[i]);
      return 1;
    }
  }

  if ( input_file_name.empty() ) {
    fprintf(stderr, "No input SER file specified\n");
    return 1;
  }

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);


  c_ffmpeg_reader ffmpeg_reader;


  if ( !ffmpeg_reader.open(input_file_name) ) {
    fprintf(stderr, "ser_reader.open(%s) fails: %s\n",
        input_file_name.c_str(),
        strerror(errno));
    return 1;
  }

  if ( output_path.empty() ) {
    output_path = "./frames";
  }

  if ( !create_path(output_path) ) {
    fprintf(stderr, "create_path(%s) fails: %s\n",
        output_path.c_str(),
        strerror(errno));
    return 1;
  }


  const cv::Size frame_size = ffmpeg_reader.frame_size();

  fprintf(stderr, "Input: %dx%d; num_frames=%d; duration=%g sec; fps=%g\n",
      frame_size.width, frame_size.height,
      ffmpeg_reader.num_frames(),
      ffmpeg_reader.duration(),
      ffmpeg_reader.fps());


  ffmpeg_reader.close();

  return 0;
}
