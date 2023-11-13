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

#include <core/io/c_las_file.h>


int main(int argc, char *argv[])
{
//  std::string filename;
//
//  for( int i = 1; i < argc; ++i ) {
//
//    if( strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-help") == 0 ) {
//      fprintf(stdout, "Usage:\n"
//          " alpha <input-file-name>\n"
//          "\n"
//          "\n");
//
//      return 0;
//    }
//
//    if( filename.empty() ) {
//      filename = argv[i];
//      continue;
//    }
//
//    fprintf(stderr, "Invalid argument %s\n",
//        argv[i]);
//
//    return 1;
//  }
//
//
//
//  cf_set_logfile(stderr);
//  cf_set_loglevel(CF_LOG_DEBUG);
//
//  c_las_reader las_;
//
//  if ( !las_.open(filename) ) {
//    CF_ERROR("las_.open('%s') fails", filename.c_str());
//    return 1;
//  }
//
//  const LASheader  * lh =
//      las_.header();
//
//  const LASpoint * p;
//
//  while ((p = las_.read_point())) {
//
//    const F64 x = p->get_x();
//    const F64 y = p->get_y();
//    const F64 z = p->get_z();
//
//    fprintf(stdout, "%g %g %g\n", x, y, z);
//  }


  return 0;
}


