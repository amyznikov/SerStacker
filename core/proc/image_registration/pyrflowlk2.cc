/*
 * pyrflowlk2.cc
 *
 *  Created on: Oct 3, 2023
 *      Author: amyznikov
 */


#include "pyrflowlk2.h"

#if HAVE_TBB
# include <tbb/tbb.h>
#endif




/**
 * @see <https://en.wikipedia.org/wiki/Finite_difference_coefficient>
 */

static void compute_gradients(cv::InputArray src, cv::Mat & gx, cv::Mat & gy)
{
  static constexpr float k[3 * 5] = {
      +1. / 24, -2. / 6, +0., +2. / 6, -1. / 24,
      +1. / 12, -2. / 3, +0., +2. / 3, -1. / 12,
      +1. / 24, -2. / 6, +0., +2. / 6, -1. / 24,
  };

  static const thread_local cv::Matx<float, 3, 5> Kx =
      0.5 * cv::Matx<float, 3, 5>(k);

  static const thread_local cv::Matx<float, 5, 3> Ky =
      Kx.t();

#if HAVE_TBB
  tbb::parallel_invoke(
      [&]() {
        cv::filter2D(src, gx, CV_32F, Kx, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
      },
      [&]() {
        cv::filter2D(src, gy, CV_32F, Ky, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
      });
#else
  cv::filter2D(src, gx, CV_32F, Kx, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
  cv::filter2D(src, gy, CV_32F, Ky, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
#endif

}


bool pyrflowlk2(cv::InputArray previous_image, cv::InputArray next_image,
    const std::vector<cv::KeyPoint> & previous_keypoints, std::vector<cv::KeyPoint> & predicted_keypoints,
    std::vector<uint8_t> & status,
    std::vector<float> & err,
    const c_pyrflowlk2_options & opts)
{

  std::vector<cv::Mat> previous_pyramid, next_pyramid;
  cv::Mat Ix, Iy, It;

  const int maxlevel =
      std::max(1, opts.maxLevel);

  cv::buildPyramid(previous_image, previous_pyramid, maxlevel, cv::BORDER_REPLICATE);
  cv::buildPyramid(next_image, next_pyramid, maxlevel, cv::BORDER_REPLICATE);


  predicted_keypoints.resize(previous_keypoints.size());

  const int nlevels =
      previous_pyramid.size();

  for ( int l = nlevels-1; l >= 0; --l ) {

    const cv::Mat & I =
        previous_pyramid[l];

    const cv::Mat & J =
        next_pyramid[l];

    compute_gradients(I, Ix, Iy);
    cv::subtract(I, J, It, cv::noArray());



    for ( int i = 0, n = previous_keypoints.size(); i < n; ++i ) {

      const cv::KeyPoint & pp =
          previous_keypoints[i];

      const cv::KeyPoint & cp =
          predicted_keypoints[i];



    }


  }



  // cv::calcOpticalFlowPyrLK(prevImg, nextImg, prevPts, nextPts, status, err, winSize, maxLevel, criteria, flags, minEigThreshold)

  return false;
}


bool load_settings(c_config_setting settings, c_pyrflowlk2_options * opts)
{
  BEGIN_LOAD_OPTIONS(settings)
  LOAD_OPTIONS(settings, *opts, maxLevel);
  LOAD_OPTIONS(settings, *opts, winSize);
  LOAD_OPTIONS(settings, *opts, maxIterations);
  LOAD_OPTIONS(settings, *opts, flags);
  LOAD_OPTIONS(settings, *opts, eps);
  END_LOAD_OPTIONS(settings)
  return true;
}

bool save_settings(c_config_setting settings, const c_pyrflowlk2_options & opts)
{
  SAVE_OPTION(settings, opts, maxLevel);
  SAVE_OPTION(settings, opts, winSize);
  SAVE_OPTION(settings, opts, maxIterations);
  SAVE_OPTION(settings, opts, flags);
  SAVE_OPTION(settings, opts, eps);
  return true;
}
