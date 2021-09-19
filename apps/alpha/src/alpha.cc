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
#include <core/ssprintf.h>
#include <core/readdir.h>
#include <core/get_time.h>
#include <core/proc/inpaint.h>
#include <core/registration/c_planetary_disk_registration.h>
#include <tbb/tbb.h>
#include <core/debug.h>


int main(int argc, char *argv[])
{
  std::string filename;
  cv::Mat image, mask;
  double scale = 10;

  for ( int i = 1; i < argc; ++i ) {

    if ( strcmp(argv[i], "--help") == 0 ) {
      printf("Usage: alpha <input-file-name.tiff> [-scale <scale>]\n");
      return 0;
    }

    if ( strcmp(argv[i], "-scale") == 0 ) {
      if ( ++i >= argc ) {
        fprintf(stderr, "Missing scale value in command line\n");
        return 1;
      }

      if ( sscanf(argv[i], "%lf", &scale)  != 1 ) {
        fprintf(stderr, "Invalid scale value in command line: %s\n", argv[i]);
        return 1;
      }
    }
    else if ( filename.empty() ) {
      filename = argv[i];
    }
    else {
      fprintf(stderr, "Invalid argument : %s\n", argv[i]);
      return 1;
    }
  }

  if ( filename.empty() ) {
    fprintf(stderr, "No input file name specified\n");
    return 1;
  }

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);


  if ( !load_image(filename, image, mask) ) {
    CF_ERROR("load_image(%s) fails", filename.c_str());
    return 1;
  }


  fftSharpenR1(image, image, scale);

  save_image(image, "fft_sharpen_image.tiff");



  return 0;
}

