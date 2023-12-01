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


int main(int argc, char *argv[])
{

  c_minacc<float, int> maxacc(5);

  float maxkey = 0;
  int  maxval = 0;

  for ( int i = 0; i < 100; ++i ) {

    float key = rand() % 100;
    float val = i;

    maxacc.update(key, val);

    if ( key >= maxkey ) {
      maxkey = key;
      maxval = val;
    }
  }

  fprintf(stdout, "key\tvalue\n");

  for ( const auto & item : maxacc.items() ) {

    fprintf(stdout, "%g\t%d\n", (double) item.key, (int)item.value);

  }



  return 0;
}


