/*
 * alpha.cc
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */

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
#include <tbb/tbb.h>
#include <core/proc/lpg.h>
#include <core/proc/stereo/c_sweepscan_stereo_matcher.h>
#include <core/proc/camera_calibration/camera_pose.h>
#include <core/proc/laplacian_pyramid.h>
#include <core/proc/image_registration/ecc2.h>
//#include <core/proc/image_registration/ecc_motion_model.h>
#include <core/proc/reduce_channels.h>
#include <core/proc/pyrscale.h>
#include <core/proc/bfgs.h>
#include <core/io/hdl/c_hdl_frame_reader.h>
#include <core/io/image/c_regular_image_input_source.h>
#include <core/io/image/c_ffmpeg_input_source.h>

#include <core/proc/c_linear_regression3.h>
#include <core/proc/c_line_estimate.h>
#include <core/proc/c_quad_estimate.h>
#include <core/proc/fit_exponential.h>
#include <core/proc/fit_decreasing_exponent.h>
#include <core/proc/extract_channel.h>

#include <core/pipeline/c_cte_pipeline/c_cte_pipeline.h>

#include <limits>
#include <core/debug.h>

namespace temp {


}


int main(int argc, char *argv[])
{
  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

  c_linear_regression3d l;

  const double a0 = -10, b0 = 0.20, c0 = -0.1;
  double a = 0, b = 0, c = 0;

  const size_t n = 20;
  double x[n];
  double y[n];

  for( size_t i = 0; i < n; ++i ) {
    x[i] = i;
    y[i] = (a0 * x[i] + b0) / (c0 * x[i] + 1);
  }

  for( size_t i = 0; i < n; ++i ) {
    l.update(x[i], 1, -x[i] * y[i], y[i]);
  }

  l.compute(a, b, c);

  CF_DEBUG("computed: a=%g b=%g c=%g", a, b, c);

  return 0;
}

