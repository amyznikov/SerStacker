/*
 * darkbayer.cc
 *
 *  Created on: Nov 4, 2022
 *      Author: amyznikov
 */

#include <core/io/c_input_sequence.h>
#include <core/io/save_image.h>
#include <core/proc/reduce_channels.h>
#include <core/readdir.h>
#include <core/debug.h>

static void average_frames(const std::string & input_file_name,
    const std::string & output_file_name);


int main(int argc, char *argv[])
{
  std::string input_file_name;
  std::string output_file_name;
  std::string output_path;
  std::string operation;


  for ( int i = 1; i < argc; ++i ) {

    if ( strcasecmp(argv[i], "-help") == 0 || strcasecmp(argv[i], "--help") == 0 ) {
      fprintf(stdout,
          "Usage:\n"
          "   darkbayer operation [OPTIONS] input_file -o output_file\n"
          "\n"
          "Operations:\n"
          " avg\n"
          " sub\n"
          "\n"
          "Options for avg:\n"
          "\n"
          "Options for sub:\n"
          "\n"
          "\n"
      );

      return 0;
    }

    if ( operation.empty() ) {
      operation = argv[i];
      continue;
    }

    if ( strcasecmp(argv[i], "-o") == 0 ) {

      if ( ++i >= argc ) {
        fprintf(stderr, "Output file name expected\n");
        return 1;
      }

      output_file_name =
          argv[i];

      continue;
    }

    if ( input_file_name.empty() ) {

      input_file_name =
          argv[i];

      if ( !file_exists(input_file_name) || is_directory(input_file_name) ) {

        fprintf(stdout, "Input file '%s' not exists or is not a regular file\n",
            input_file_name.c_str());

        return 1;
      }

      continue;
    }


    fprintf(stderr, "Unknow or invalid argument: %s\n", argv[i]);

    return 1;
  }


  if ( operation.empty() ) {
    fprintf(stderr, "Operation not specified\n");
    return 1;
  }

  if ( input_file_name.empty() ) {
    fprintf(stderr, "input file not specified\n");
    return 1;
  }

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

  if ( strcasecmp(operation.c_str(), "avg")  == 0 ) {

    average_frames(input_file_name, output_file_name);

  }
  else {
    fprintf(stderr, "Unknown or not yet implemented operation specified: %s\n",
        operation.c_str());
    return 1;

  }


  return 0;
}


static void average_frames(const std::string & input_file_name, const std::string & output_file_name)
{
  c_input_sequence::sptr input_sequence =
      c_input_sequence::create(input_file_name);

  if( !input_sequence ) {
    CF_ERROR("c_input_sequence::create(input_file_name='%s') fails",
        input_file_name.c_str());
    return;
  }

  input_sequence->set_auto_debayer(
      DEBAYER_DISABLE);

  if ( !input_sequence->open() ) {
    CF_ERROR("input_sequence->open(%s) fails",
        input_file_name.c_str());
    return;

  }

  const int total_frames =
      input_sequence->size();


  cv::Mat current_frame;
  cv::Mat current_mask;

  cv::Mat cumulative_frame;
  cv::Mat cumuative_counter;
  int num_frames_processed  = 0;

  for( int i = 0; i < total_frames; ++i ) {

    if( !input_sequence->read(current_frame, &current_mask) ) {
      CF_ERROR("input_sequence->read(pos=%d) fails", i);
      break;
    }

    if( cumulative_frame.empty() ) {

      current_frame.convertTo(cumulative_frame,
          CV_32F);

      if( current_mask.empty() ) {
        cumuative_counter =
            cv::Mat::ones(current_frame.size(),
                CV_MAKETYPE(CV_32F, current_frame.channels()));
      }
      else {
        current_mask.convertTo(cumuative_counter,
            CV_MAKETYPE(CV_32F, current_frame.channels()),
            1. / 255);
      }

      ++num_frames_processed;
      continue;
    }

    if( current_frame.size() != cumulative_frame.size() || current_frame.channels() != cumulative_frame.channels() ) {

      CF_ERROR("input frame (%dx%d c=%d d=%d) not match to cumuative frame (%dx%d c=%d d=%d).\n",
          current_frame.cols, current_frame.rows, current_frame.channels(), current_frame.depth(),
          cumulative_frame.cols, cumulative_frame.rows, cumulative_frame.channels(), cumulative_frame.depth());

      break;
    }

    cv::add(current_frame,
        cumulative_frame,
        cumulative_frame,
        current_mask,
        cumulative_frame.type());

    cv::add(cumuative_counter, 1,
        cumuative_counter,
        current_mask);


    if ( num_frames_processed++ % 16 == 0 ) {
      fprintf(stderr, "%6d frames\n",
          num_frames_processed);
    }

  }

  fprintf(stderr, "%6d frames\n",
      num_frames_processed);

  if ( num_frames_processed < 1 ) {
    CF_ERROR("No input frames processed");
    return;
  }

  cv::Mat cumulative_mask =
      cumuative_counter > 0.99;

  cv::divide(cumulative_frame, cumuative_counter, cumulative_frame);
  cumulative_frame.setTo(0, ~cumulative_mask);

  double min, max;
  cv::Scalar m, s;

  cv::minMaxLoc(cumulative_frame, &min, &max);
  cv::meanStdDev(cumulative_frame, m, s, cumulative_mask);

  CF_DEBUG("min=%g max=%g m=(%g %g %g %g) stdev=(%g %g %g %g)", min, max,
      m[0], m[1], m[2], m[3],
      s[0], s[1], s[2], s[3]);


  if ( cv::countNonZero(cumulative_mask) == cumulative_mask.size().area() ) {

    if ( !save_image(cumulative_frame, output_file_name) ) {
      CF_ERROR("save_image(output_file_name=%s) fails",
          output_file_name.c_str());
    }

  }
  else {

    if ( cumulative_mask.channels() > 1 ) {

      reduce_color_channels(cumulative_mask,
          cumulative_mask,
          cv::REDUCE_MIN);
    }

    if ( !save_image(cumulative_frame, cumulative_mask, output_file_name) ) {
      CF_ERROR("save_image(output_file_name=%s) fails",
          output_file_name.c_str());
    }
  }

}
