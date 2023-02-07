/*
 * alpha.cc
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */

#include <core/proc/eccalign.h>
#include <core/improc/c_image_processor.h>
#include <core/improc/c_unsharp_mask_routine.h>
#include <core/improc/c_align_color_channels_routine.h>
#include <core/improc/c_rangeclip_routine.h>
#include <core/io/save_image.h>
#include <core/io/load_image.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/downstrike.h>
#include <core/proc/unsharp_mask.h>
#include <core/proc/morphology.h>
#include <core/proc/geo-reconstruction.h>
#include <core/proc/planetary-disk-detection.h>
#include <core/proc/jupiter.h>
#include <core/proc/fft.h>
#include <core/settings.h>
#include <core/readdir.h>
#include <core/get_time.h>
#include <core/proc/inpaint.h>
#include <tbb/tbb.h>
#include <core/proc/laplacian_pyramid.h>
#include <core/debug.h>



int main(int argc, char *argv[])
{
  std::string input_file_name;
  std::string output_file_name;
  std::string output_directory;

  cv::Mat image;
  std::vector<cv::Mat> pyramid;


  for ( int i = 1; i < argc; ++i ) {

    if ( strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-help") == 0 ) {

      fprintf(stdout, "Test for Laplacian pyramid.\n"
          "Usage:\n"
          "   alpha <input_image> [-o <output_directory>]\n"
          "\n");

      return 0;
    }

    if( strcmp(argv[i], "-o") == 0 ) {
      if( ++i >= argc ) {
        fprintf(stderr, "Output directory name expected after %s argument\n", argv[i - 1]);
        return 1;
      }

      output_directory = argv[i];
      continue;
    }

    if ( input_file_name.empty() ) {
      input_file_name = argv[i];
      continue;
    }

    fprintf(stderr, "Invalid argument: %s\n", argv[i]);
    return 1;
  }

  if( input_file_name.empty() ) {
    fprintf(stderr, "No input file specified\n");
    return 1;
  }

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);


  if ( !load_image(input_file_name, image) ) {
    CF_ERROR("load_image('%s') fails", input_file_name.c_str());
    return 1;
  }


  build_laplacian_pyramid(image, pyramid);

  if ( output_directory.empty() ) {
    output_directory = "./pyramid";
  }

  for ( int i = 0, n = pyramid.size(); i < n; ++i ) {

    output_file_name =
        ssprintf("%s/L%03d.tiff",
            output_directory.c_str(), i);

    if ( !save_image(pyramid[i], output_file_name) ) {
      CF_ERROR("save_image('%s') fails", output_file_name.c_str());
      return 1;
    }
  }


  cv::Mat reconstructed_image;
  reconstruct_laplacian_pyramid(reconstructed_image, pyramid);

  output_file_name =
      ssprintf("%s/reconstructed_image.tiff", output_directory.c_str());

  if ( !save_image(reconstructed_image, output_file_name) ) {
    CF_ERROR("save_image('%s') fails", output_file_name.c_str());
    return 1;
  }

  return 0;
}

