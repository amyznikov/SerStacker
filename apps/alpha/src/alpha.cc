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

#include <core/proc/c_line_estimate.h>
#include <core/proc/c_quad_estimate.h>
#include <core/proc/fit_exponential.h>
#include <core/proc/fit_decreasing_exponent.h>
#include <core/proc/extract_channel.h>

#include <core/pipeline/c_cte_pipeline/c_cte_pipeline.h>

#include <limits>
#include <core/debug.h>

namespace {


}



int main(int argc, char *argv[])
{
  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

  UVec3d uv1(0, 0);
  UVec3d uv2(0, 0);
  UVec3d uv = uv1 - uv2;

  cv::Vec3d v1 = uv1.vec();
  cv::Vec3d v2 = uv2.vec();
  cv::Vec3d v = uv.vec();


//  CF_DEBUG("\n"
//      "v2={%g %g %g) norm=%g\n",
//      v2(0), v2(1), v2(2), cv::norm(v2));


  CF_DEBUG("\n"
      "v1={%g %g %g) norm=%g\n"
      "v2={%g %g %g) norm=%g\n"
      "v={%g %g %g) norm=%g\n",
      v1(0), v1(1), v1(2), cv::norm(v1),
      v2(0), v2(1), v2(2), cv::norm(v2),
      v(0), v(1), v(2), cv::norm(v));

  return 0;
}

