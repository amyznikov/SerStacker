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
#include <core/registration/c_feature_based_registration.h>
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
  std::string filenames[2];
  cv::Mat images[2], masks[2];
  cv::Mat feature_image, feature_mask;

  for ( int i = 1; i < argc; ++i ) {

    if ( strcmp(argv[i], "--help") == 0 ) {
      printf("Usage: alpha <input-file-name1.tiff> <input-file-name2.tiff> \n");
      return 0;
    }

    else if ( filenames[0].empty() ) {
      filenames[0] = argv[i];
    }
    else if ( filenames[1].empty() ) {
      filenames[1] = argv[i];
    }
    else {
      fprintf(stderr, "Invalid argument : %s\n", argv[i]);
      return 1;
    }
  }

  if ( filenames[0].empty() || filenames[1].empty() ) {
    fprintf(stderr, "Two input file names expected\n");
    return 1;
  }

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);


  for ( int i = 0; i < 2; ++i ) {
    if ( !load_image(filenames[i], images[i], masks[i]) ) {
      CF_ERROR("load_image(%s) fails", filenames[i].c_str());
      return 1;
    }
  }

  return 0;
}

