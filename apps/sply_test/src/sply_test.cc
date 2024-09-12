/*
 * sply_test.cc
 *
 *  Created on: Sep 7, 2024
 *      Author: amyznikov
 */

#include <core/io/sply/c_sply_video_file.h>
#include <core/io/c_ffmpeg_file.h>
#include <core/proc/gps/gpx.h>
#include <core/readdir.h>
#include <core/debug.h>


///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[])
{
  std::string gpx_file_name;
  std::string video_file_name;
  std::string output_file_name;

  c_gpx_track gpx;
  c_sply_video_frame frame;
  double pts = 0;


  for( int i = 1; i < argc; ++i ) {

    if( strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "--help") == 0 ) {
      fprintf(stdout,
          "Usage:\n"
              "   sply_test [OPTIONS] -gpx gpx_file.gpx -v video_file_name.avi -o output_file.sply\n"
              "\n"
              "OPTIONS:\n"
              "\n"
          );

      return 0;
    }

    if( strcmp(argv[i], "-gpx") == 0 ) {
      if( (++i >= argc) ) {
        fprintf(stderr, "gpsx file name expected: %s\n", argv[i - 1]);
        return 1;
      }

      gpx_file_name =
          argv[i];

      continue;
    }

    if( strcmp(argv[i], "-v") == 0 ) {
      if( (++i >= argc) ) {
        fprintf(stderr, "video file name expected: %s\n", argv[i - 1]);
        return 1;
      }

      video_file_name =
          argv[i];

      continue;
    }

    if( strcmp(argv[i], "-o") == 0 ) {
      if( (++i >= argc) ) {
        fprintf(stderr, "Output file name expected: %s\n", argv[i - 1]);
        return 1;
      }

      output_file_name =
          argv[i];

      continue;
    }


    fprintf(stderr, "Invalid argument or not an regular input file specified: %s\n", argv[i]);
    return 1;
  }

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

  if( gpx_file_name.empty() ) {
    fprintf(stderr, "No gpsx input file specified\n");
    return 1;
  }

  if( video_file_name.empty() ) {
    fprintf(stderr, "No video input file specified\n");
    return 1;
  }

  if( output_file_name.empty() ) {
    fprintf(stderr, "No output file name specified\n");
    return 1;
  }

  if ( output_file_name == gpx_file_name ) {
    fprintf(stderr, "Output and GPX file names are the same. Specify different file names\n");
    return 1;
  }

  if ( output_file_name == video_file_name ) {
    fprintf(stderr, "Output and video file names are the same. Specify different file names\n");
    return 1;
  }




  if( !load_gpx_track_xml(gpx_file_name, &gpx) ) {
    CF_ERROR("load_gpx_track_xml('%s') fails", gpx_file_name.c_str());
    return 1;
  }

  CF_DEBUG("gpx: name='%s' author='%s' src='%s' created='%s' length=%g duration=%g",
      gpx.name.c_str(),
      gpx.author.c_str(),
      gpx.src.c_str(),
      gpx.created.c_str(),
      gpx.length,
      gpx.duration);




  if ( true ) {

    c_ffmpeg_reader ffmpeg;
    c_sply_video_writer sply;

    if ( !ffmpeg.open(video_file_name) ) {
      CF_ERROR("ffmpeg.open(video_file_name='%s') fails", video_file_name.c_str());
      return 1;
    }

    CF_DEBUG("ffmpeg: size=%d",
        ffmpeg.num_frames());

    if ( !sply.create(output_file_name) ) {
      CF_ERROR("sply.create(output_file_name='%s') fails", output_file_name.c_str());
      return 1;
    }

    while ( ffmpeg.read(frame.image, &pts) ) {

      frame.ts = pts;

      if ( !sply.write(frame) ) {
        CF_ERROR("sply.write() fails: %s", strerror(errno));
        break;
      }

      if ( (ffmpeg.curpos() % 8) == 0 ) {
        CF_DEBUG("F %9zd", (ssize_t)ffmpeg.curpos());
      }

    }

    sply.close();
  }

  if ( false ) {

    c_sply_video_reader sply;

    if ( !sply.open(output_file_name) ) {
      CF_ERROR("sply.open(output_file_name='%s') fails", output_file_name.c_str());
      return 1;
    }

    while ( sply.read(frame) ) {
      CF_DEBUG("frame");
    }


    sply.close();
  }

  return 0;
}
