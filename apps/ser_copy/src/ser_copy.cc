/*
 * ser_copy.cc
 *
 *  Created on: Dec 1, 2020
 *      Author: amyznikov
 */

#include <core/io/c_ser_file.h>
#include <core/readdir.h>
#include <core/debug.h>


int main(int argc, char *argv[])
{
  std::string input_file_name;
  std::string output_file_name;
  std::string output_path;

  int start_frame = 0;
  int end_frame = -1;

  bool fix_corrupted_header = false;


  for ( int i = 1; i < argc; ++i ) {

    if ( strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "--help") == 0 ) {
      fprintf(stdout,
          "Usage:\n"
          "   ser_copy [OPTIONS] input_file.ser -o output_file.ser [-f start[:end]]\n"
          "\n"
          "OPTIONS:\n"
          "  --fix "
          "      Fix header of corrupted file\n"
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

    if ( strcmp(argv[i], "-f") == 0 ) {
      if ( (++i >= argc) ) {
        fprintf(stderr, "frame range start[:end] is expected after %s option\n", argv[i - 1]);
        return 1;
      }

      if ( sscanf(argv[i], "%d:%d", &start_frame, &end_frame) < 1 ) {
        fprintf(stderr, "frame range start[:end] is expected after %s option\n", argv[i - 1]);
        return 1;
      }

      continue;
    }

    if ( strcmp(argv[i], "--fix") == 0 ) {
      fix_corrupted_header = true;
      continue;
    }


    if ( input_file_name.empty() && is_regular_file(argv[i]) ) {

      input_file_name =
          argv[i];

      continue;
    }

    fprintf(stderr, "Invalid argument or not existimng input file specified: %s\n", argv[i]);
    return 1;
  }

  if ( input_file_name.empty() ) {
    fprintf(stderr, "No input SER file specified\n");
    return 1;
  }

  if ( output_file_name.empty() ) {
    fprintf(stderr, "No output SER file name specified\n");
    return 1;
  }

  if ( input_file_name == output_file_name ) {
    fprintf(stderr, "Input and Output file names can not be same\n");
    return 1;
  }

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);


  if( end_frame > 0 && end_frame < start_frame ) {
    fprintf(stderr, "Invalid end frame specified: %d less tnah start frame %d\n",
        end_frame, start_frame);
    return 1;
  }


  c_ser_reader ser_reader;
  c_ser_writer ser_writer;


  if ( !ser_reader.open(input_file_name) ) {
    fprintf(stderr, "ser_reader.open(%s) fails: %s\n",
        input_file_name.c_str(),
        strerror(errno));
    return 1;
  }

  CF_DEBUG("ser_reader.timestamps().size = %zu", ser_reader.timestamps().size());

  if ( fix_corrupted_header ) {
    CF_DEBUG("file is opened");
    return 0;
  }

  if ( start_frame < 0 ) {
    start_frame = 0;
  }
  else if( start_frame >= ser_reader.num_frames() ) {
    fprintf(stderr, "Invalid start frame specified: %d. num_frames=%d\n",
        start_frame, ser_reader.num_frames());
    return 1;
  }

  if( end_frame < 0 || end_frame >= ser_reader.num_frames() ) {
    end_frame = ser_reader.num_frames() - 1;
  }

  if( start_frame > 0 && !ser_reader.seek(start_frame) ) {
    fprintf(stderr, "ser_reader.seek(start_frame=%d) fails: %s\n",
        start_frame,
        strerror(errno));
    return 1;
  }


  if ( !(output_path = get_parent_directory(output_file_name)).empty() && !create_path(output_path) ) {
    fprintf(stderr, "create_path(%s) fails: %s\n",
        output_path.c_str(),
        strerror(errno));
    return 1;
  }

  fprintf(stderr, "Input: %dx%d;  %d channels; %d bpp\n",
      ser_reader.image_width(), ser_reader.image_height(),
      ser_reader.channels(),
      ser_reader.bits_per_plane());


  if ( !ser_writer.create(output_file_name,ser_reader.image_width(), ser_reader.image_height(),
      ser_reader.color_id(), ser_reader.bits_per_plane()) ) {

    fprintf(stderr, "ser_writer.create(%s) fails: %s\n",
        output_file_name.c_str(),
        strerror(errno));
    return 1;
  }

  cv::Mat current_frame;
  int num_frames = 0;


  for ( int i = start_frame ; i <= end_frame && ser_reader.read(current_frame); ++i ) {

    if ( !ser_writer.write(current_frame) ) {
      fprintf(stderr, "ser_writer.write() fails: %s\n", strerror(errno));
      break;
    }

    if ( num_frames++ % 32 == 0 )  {
      fprintf(stderr, "%16d frames\r",
          num_frames);
      fflush(stderr);
    }
  }

  fprintf(stderr, "%16d frames\n",
      num_frames);

  return 0;
}
