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
#include <core/proc/c_minmaxacc.h>
#include <core/proc/c_math_expression.h>


int main(int argc, char *argv[])
{
  std::string s;

  for ( int i = 1; i < argc; ++i ) {
    if ( strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "--help") == 0 ) {
      fprintf(stdout, "Usage: alpha 'expression'\n");
      return 0;

    }

    if ( s.empty() ) {
      s = argv[i];
      continue;
    }

    fprintf(stderr, "Invalid arg: '%s'\n", argv[i]);
    return 1;

  }


  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

  c_math_expression m;

  m.add_argument(0, "x", "x");

  if( !m.parse(s) ) {

    CF_ERROR("m.parse() fails: %s err=%s",
        m.error_message().c_str(),
        m.pointer_to_syntax_error());

    return 1;
  }

  for ( double x = -10; x <= 10; x += 1 ) {

    double args[] = {x};

    double f = m.eval(args);

    fprintf(stdout, "%g\t%g\n", x, f);

  }

  CF_DEBUG("H");


  return 0;
}


