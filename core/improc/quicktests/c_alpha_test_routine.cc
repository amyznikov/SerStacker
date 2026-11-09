/*
 * c_alpha_test_routine.cc
 *
 *  Created on: Jun 26, 2026
 *      Author: amyznikov
 */

#include "c_alpha_test_routine.h"
#include <core/proc/feature2d/planetary-disk-detection.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/morphology.h>
#include <core/proc/gradient.h>
#include <core/proc/fft.h>
#include <core/proc/fast_gaussian_blur.h>
#include <core/proc/histogram-tools.h>
#include <core/proc/downstrike.h>
#include <core/ssprintf.h>
#include <core/proc/inpaint/average_pyramid_inpaint.h>
#include <core/io/c_stdio_file.h>
#include <core/proc/c_linear_regression.h>
#include <core/proc/c_line_estimate.h>
#include <core/proc/run-loop.h>
#include <core/proc/divide.h>
#include <core/readdir.h>
#include <random>
#include <core/io/c_stdio_file.h>
#include <core/proc/divide.h>
#include <core/proc/multiply.h>


//template<>
//const c_enum_member * members_of<c_alpha_test_routine::DISPLAY>()
//{
//  static const c_enum_member members[] = {
//      { c_alpha_test_routine::DISPLAY_SRC1, "SRC1", "" },
//      { c_alpha_test_routine::DISPLAY_SRC2, "SRC2", "" },
//      { c_alpha_test_routine::DISPLAY_SPEC1, "SPEC1", "" },
//      { c_alpha_test_routine::DISPLAY_SPEC2, "SPEC2", "" },
//      { c_alpha_test_routine::DISPLAY_SPEC3, "SPEC3", "" },
//      { c_alpha_test_routine::DISPLAY_CROSS_SPEC13_DIFF, "SPEC13_DIFF", "" },
//      { c_alpha_test_routine::DISPLAY_CROSS_SPEC13, "CROSS_SPEC13", "" },
//      { c_alpha_test_routine::DISPLAY_CORR_MAP13, "CORR_MAP13", "" },
//      { c_alpha_test_routine::DISPLAY_SRC2}
//  };
//  return members;
//}

/////////////////////////////////////
namespace {
}

bool c_alpha_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
//    SERIALIZE_OPTION(settings, save, *this, sigma);
//    SERIALIZE_OPTION(settings, save, *this, eps);
//    SERIALIZE_OPTION(settings, save, *this, kradius);
//    SERIALIZE_OPTION(settings, save, *this, dscale);
//    SERIALIZE_OPTION(settings, save, *this, uscale);
    return true;
  }
  return false;
}

void c_alpha_test_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
//  ctlbind(ctls, "sigma", CTL_CONTEXT(ctx, sigma), "");
//  ctlbind(ctls, "eps", CTL_CONTEXT(ctx, eps), "");
//  ctlbind(ctls, "kradius", CTL_CONTEXT(ctx, kradius), "");
//  ctlbind(ctls, "dscale", CTL_CONTEXT(ctx, dscale), "");
//  ctlbind(ctls, "uscale", CTL_CONTEXT(ctx, uscale), "");
}

bool c_alpha_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat msk;
  cv::Scalar mv, sv;

  const int cn = image.channels();

  if( !mask.empty() ) {
    if( mask.depth() == CV_8U ) {
      msk = mask.getMat();
    }
    else {
      cv::compare(mask, 0, msk, cv::CMP_GT);
    }
  }


  cv::meanStdDev(image, mv, sv, msk);

  for ( int c = 0; c < cn; ++c ) {
    if ( sv[c] != 0 ) {
      sv[c] = 1.0 / sv[c];
    }
  }

  const cv::Scalar bias = -mv.mul(sv);

  cv::Mat1f M = cv::Mat1f::zeros(cn, cn + 1);
  for (int i = 0; i < cn; ++i) {
    M(i, i) = sv[i];
    M(i, cn) = bias[i];
  }

  cv::transform(image, image, M);

  return true;
}
