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
#include <core/debug.h>
#include <core/proc/bfgs.h>

#include "../../../core/io/hdl/c_hdl_frame_reader.h"


int main(int argc, char *argv[])
{
  std::string address;
  std::string options;

  for ( int i = 1; i < argc; ++i ) {
    if ( strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "--help") == 0 ) {
      fprintf(stdout, "Usage: alpha path_to_pcap_file.pcap\n");
      return 0;

    }

    if ( address.empty() ) {
      address = argv[i];
      continue;
    }

    fprintf(stderr, "Invalid arg: '%s'\n", argv[i]);
    return 1;
  }


  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

  if ( address.empty() ) {
    CF_ERROR("No pcap file specified");
    return 1;
  }
//
//
//  c_hdl_offline_pcap_reader reader;
//
//
//  if( !reader.open(address, options) ) {
//    CF_ERROR("reader.open('%s') fails", address.c_str());
//    return 1;
//  }
//
////  return 0;
//
//  CF_DEBUG("reader.streams=%zu", reader.streams().size());
//
//  if ( !reader.select_stream(0) ) {
//    CF_ERROR("reader.select_stream(0) fails");
//    return 1;
//  }
//
//  const int n =
//      reader.num_frames();
//
//  CF_DEBUG("num_frames=%d", n);
//
//  for ( int i = 0; i < 10; ++i ) {
//
//    reader.seek(i);
//
//    c_hdl_frame::sptr frame =
//        reader.read();
//
//    if ( !frame ) {
//      CF_ERROR("reader.read() fails");
//      break;
//    }
//
//    CF_DEBUG("frame[%d].pts=%zu", i, frame->points.size());
//
//  }
//
//


  return 0;
}


