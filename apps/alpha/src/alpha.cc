/*
 * alpha.cc
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */

#include <core/improc/c_image_processor.h>
#include <core/improc/c_unsharp_mask_routine.h>
#include <core/improc/c_align_color_channels_routine.h>
#include <core/improc/pixel/c_rangeclip_routine.h>
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


int main(int argc, char *argv[])
{
  class c_bfgs_callback :
      public c_bfgs::callback
  {
  public:
    bool compute(const std::vector<double> & p, double * f,
        std::vector<double> * g = nullptr, bool * have_g = nullptr) const override
    {
      const int n = p.size();
      double fx = 0.0;

      if( g ) {
        g->resize(n);
        *have_g = true;
      }

      for( int i = 0; i < n; i += 2 ) {

        double t1 = 1.0 - p[i];
        double t2 = 10.0 * (p[i + 1] - p[i] * p[i]);

        if( g ) {
          (*g)[i + 1] = 20.0 * t2;
          (*g)[i + 0] = -2.0 * (p[i] * (*g)[i + 1] + t1);
        }

        if( f ) {
          fx += t1 * t1 + t2 * t2;
        }
      }

      if( f ) {
        *f = fx;
      }

      return true;
    }

  };

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

  /* Initialize the variables. */
  constexpr int N = 100;

  std::vector<double> p(N);
  double fx;

  for( int i = 0; i < N; i += 2 ) {
    p[i] = -1.2;
    p[i+1] = 1.5;
  }

  c_bfgs bfgs(1000);
  c_bfgs_callback cb;

  c_bfgs::STATUS status =
      bfgs.run(cb, p);

  cb.compute(p, &fx, nullptr, nullptr);

  printf("L-BFGS optimization terminated: %d iterations status=%d %s (%s)\n", bfgs.iterations(),
      (int) status, toString(status), comment_for(status));

  printf("  fx = %f, x[0] = %f, x[1] = %f\n", fx, p[0], p[1]);

  return 0;
}


