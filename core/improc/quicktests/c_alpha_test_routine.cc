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
//      { c_alpha_test_routine::DISPLAY_CURRENT_IMAGE, "CURRENT_IMAGE", "" },
//      { c_alpha_test_routine::DISPLAY_DRIZZLED_IMAGE, "DRIZZLED_IMAGE", "" },
//      { c_alpha_test_routine::DISPLAY_DRIZZLE_ACCUMULATOR, "DRIZZLE_ACCUMULATOR", "" },
//      { c_alpha_test_routine::DISPLAY_DRIZZLE_WEIGHTS, "DRIZZLE_WEIGHTS", "" },
//      { c_alpha_test_routine::DISPLAY_CURRENT_IMAGE}
//  };
//  return members;
//}

/////////////////////////////////////
namespace {
} // namespace

bool c_alpha_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {

    return true;
  }
  return false;
}

void c_alpha_test_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  //ctlbind(ctls, "Display", CTL_CONTEXT(ctx, _display), "Select image to display");
}

bool c_alpha_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat src;
  if ( image.channels() == 1 ) {
    src = image.getMat();
  }
  else {
    cv::cvtColor(image, src, cv::COLOR_BGR2GRAY);
  }

  if ( src.depth() != CV_32F ) {
    src.convertTo(src, CV_32F);
  }

  cv::Mat IR, IC, IRP, ICP;
  mreduce(src, IR, 0, cv::REDUCE_SUM);
  mreduce(src, IC, 1, cv::REDUCE_SUM);
  cv::repeat(IR, src.cols, 1, IRP);
  cv::repeat(IC, 1, src.rows, ICP);

  cv::Mat IP;
  cv::multiply(IRP, ICP, IP);


//  fftPPSDecompositionCCS(IR, fftGenerateDiscreteLaplacianFilter(IR.size(), false), IRP, cv::noArray());
//  fftPPSDecompositionCCS(IC, fftGenerateDiscreteLaplacianFilter(IC.size(), false), ICP, cv::noArray());
//  fftCCSSpectrumMagnitude(IRP, IRP, true);
//  fftCCSSpectrumMagnitude(ICP, ICP, true);
//
//  cv::Mat IP = ICP * IRP;

  image.move(IP);

  return true;
}


