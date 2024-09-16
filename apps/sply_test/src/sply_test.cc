/*
 * sply_test.cc
 *
 *  Created on: Sep 7, 2024
 *      Author: amyznikov
 */

#include <core/io/sply/c_sply_video_file.h>
#include <core/io/c_ffmpeg_file.h>
#include <core/io/c_stdio_file.h>
#include <core/proc/gps/gpx.h>
#include <core/readdir.h>
#include <core/debug.h>


///////////////////////////////////////////////////////////////////////////////

static bool save_gps_output_file(const c_gpx_track & gpx, const std::string & gps_output_file_name)
{
  c_stdio_file fp;

  if( !fp.open(gps_output_file_name, "wt") ) {
    CF_ERROR("Can not create file '%s' : %s", gps_output_file_name.c_str(), strerror(errno));
    return false;
  }

  fprintf(fp, "I\tTS\tRTS\tLAT\tLON\tALT\n");

  const c_gps_position & gps0 =
      gpx.pts[0];

  for ( size_t i = 0, n = gpx.pts.size(); i < n; ++i ) {

    const c_gps_position & gps =
        gpx.pts[i];

    fprintf(fp, "%8zu"
        "\t%12.3f\t%12.3f"
        "\t%+12.8f\t%+12.8f\t%+12.3f\n",
        i,
        gps.timestamp, gps.timestamp - gps0.timestamp,
        gps.latitude * 180 / CV_PI, gps.longitude * 180 / CV_PI, gps.altitude);

  }

  fp.close();

  return true;
}

///////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[])
{
  std::string gpx_input_file_name;
  std::string gps_output_file_name;
  std::string video_input_file_name;
  std::string sply_output_file_name;

  c_gpx_track gpx;
  c_sply_video_frame frame;
  double pts = 0;


  for( int i = 1; i < argc; ++i ) {

    if( strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "--help") == 0 ) {
      fprintf(stdout,
          "Usage:\n"
              "   sply_test [OPTIONS] -gpx gpx_file.gpx [-gps gps_output_file_name.txt]  -v video_file_name.avi -o output_file.sply\n"
              "\n"
              "OPTIONS:\n"
              "\n"
          );

      return 0;
    }

    if( strcmp(argv[i], "-gpx") == 0 ) {
      if( (++i >= argc) ) {
        fprintf(stderr, "gpx input file name expected: %s\n", argv[i - 1]);
        return 1;
      }

      gpx_input_file_name =
          argv[i];

      continue;
    }

    if( strcmp(argv[i], "-gps") == 0 ) {
      if( (++i >= argc) ) {
        fprintf(stderr, "gps output file name expected: %s\n", argv[i - 1]);
        return 1;
      }

      gps_output_file_name =
          argv[i];

      continue;
    }

    if( strcmp(argv[i], "-v") == 0 ) {
      if( (++i >= argc) ) {
        fprintf(stderr, "video file name expected: %s\n", argv[i - 1]);
        return 1;
      }

      video_input_file_name =
          argv[i];

      continue;
    }

    if( strcmp(argv[i], "-o") == 0 ) {
      if( (++i >= argc) ) {
        fprintf(stderr, "Output file name expected: %s\n", argv[i - 1]);
        return 1;
      }

      sply_output_file_name =
          argv[i];

      continue;
    }


    fprintf(stderr, "Invalid argument or not an regular input file specified: %s\n", argv[i]);
    return 1;
  }

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

  if( gpx_input_file_name.empty() ) {
    fprintf(stderr, "No gpsx input file specified\n");
    return 1;
  }

  if( !load_gpx_track_xml(gpx_input_file_name, &gpx) ) {
    CF_ERROR("load_gpx_track_xml('%s') fails", gpx_input_file_name.c_str());
    return 1;
  }

  CF_DEBUG("gpx: name='%s' author='%s' src='%s' created='%s' length=%g duration=%g",
      gpx.name.c_str(),
      gpx.author.c_str(),
      gpx.src.c_str(),
      gpx.created.c_str(),
      gpx.length,
      gpx.duration);


  if ( !gps_output_file_name.empty() && !gpx.pts.empty() ) {
    if ( !save_gps_output_file(gpx, gps_output_file_name) ) {
      CF_ERROR("save_gps_output_file('%s') fails", gps_output_file_name.c_str());
      return 1;
    }
  }



  if( video_input_file_name.empty() ) {
    fprintf(stderr, "No video input file specified\n");
    return 1;
  }

  if( sply_output_file_name.empty() ) {
    fprintf(stderr, "No output file name specified\n");
    return 1;
  }

  if ( sply_output_file_name == gpx_input_file_name ) {
    fprintf(stderr, "Output and GPX file names are the same. Specify different file names\n");
    return 1;
  }

  if ( sply_output_file_name == video_input_file_name ) {
    fprintf(stderr, "Output and video file names are the same. Specify different file names\n");
    return 1;
  }








  if ( true ) {

    c_ffmpeg_reader ffmpeg;
    c_sply_video_writer sply;

    if ( !ffmpeg.open(video_input_file_name) ) {
      CF_ERROR("ffmpeg.open(video_file_name='%s') fails", video_input_file_name.c_str());
      return 1;
    }

    CF_DEBUG("ffmpeg: size=%d",
        ffmpeg.num_frames());

    if ( !sply.create(sply_output_file_name) ) {
      CF_ERROR("sply.create(output_file_name='%s') fails", sply_output_file_name.c_str());
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

    if ( !sply.open(sply_output_file_name) ) {
      CF_ERROR("sply.open(output_file_name='%s') fails", sply_output_file_name.c_str());
      return 1;
    }

    while ( sply.read(frame) ) {
      CF_DEBUG("frame");
    }


    sply.close();
  }

  return 0;
}
