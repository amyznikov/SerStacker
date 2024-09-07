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
#include <core/proc/extract_channel.h>

#include <core/debug.h>

namespace {

void ecclm_normalize(cv::Mat1f & image, cv::Mat1b & mask)
{
  cv::Mat m, s;

  const double eps =
      100;

  const double sigma =
      15;

  const int ksize =
      2 * ((int) (sigma * 3)) + 1;

  cv::Mat G =
      cv::getGaussianKernel(ksize, sigma,
          CV_32F);

  cv::sepFilter2D(image, m, CV_32F, G, G, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::subtract(image, m, m, cv::noArray(), CV_32F);

  cv::sepFilter2D(cv::abs(m), s, CV_32F, G, G, cv::Point(-1, -1), eps, cv::BORDER_REPLICATE);
  cv::divide(m, s, image);
}

}



int main(int argc, char *argv[])
{
//  cf_set_logfile(stderr);
//  cf_set_loglevel(CF_LOG_DEBUG);
//
//  const std::string input_filenames[2] = {
//      "/home/data/ecclm/F1.png",
//      "/home/data/ecclm/F2.png",
//  };
//
//  cv::Mat input_images[2];
//
//  for ( int i = 0; i < 2; ++i ) {
//
//    if ( !load_image(input_filenames[i], input_images[i]) ) {
//      CF_ERROR("load_image(%s) fails", input_filenames[i].c_str());
//      return 1;
//    }
//
//    extract_channel(input_images[i], input_images[i], cv::noArray(), cv::noArray(), color_channel_gray, 1, CV_32F);
//    cv::GaussianBlur(input_images[i], input_images[i], cv::Size(), 1, 1);
//
//    //cv::morphologyEx(input_image, input_image, cv::MORPH_GRADIENT, cv::Mat1b(5,5, 255));
//  }
//
//  double corr =
//      compute_correlation(input_images[0],
//          input_images[0],
//          cv::noArray() );
//
//  CF_DEBUG("corr=%g", corr);

  return 0;
}

