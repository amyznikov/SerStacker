/*
 * alpha.cc
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */

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
#include <core/proc/fft.h>
#include <core/settings.h>
#include <core/readdir.h>
#include <core/get_time.h>
#include <core/proc/inpaint.h>
//#include <tbb/tbb.h>
#include <core/proc/lpg.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/stereo/c_sweepscan_stereo_matcher.h>

#include <core/proc/laplacian_pyramid.h>
#include <core/proc/image_registration/ecc2.h>
#include <core/proc/image_registration/ecc_motion_model.h>
#include <core/proc/reduce_channels.h>
#include <core/proc/pyrscale.h>
#include <core/proc/bfgs.h>
#include <core/io/hdl/c_hdl_frame_reader.h>

#include <core/proc/c_line_estimate.h>
#include <core/proc/c_quad_estimate.h>

#include <core/debug.h>

int main(int argc, char *argv[])
{
//  std::string address;
//  std::string options;
//
//  for ( int i = 1; i < argc; ++i ) {
//    if ( strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "--help") == 0 ) {
//      fprintf(stdout, "Usage: alpha path_to_pcap_file.pcap\n");
//      return 0;
//
//    }
//
//    if ( address.empty() ) {
//      address = argv[i];
//      continue;
//    }
//
//    fprintf(stderr, "Invalid arg: '%s'\n", argv[i]);
//    return 1;
//  }


  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

  return 0;
}


