/*
 * estimate_noise.cc
 *
 *  Created on: Feb 15, 2018
 *      Author: amyznikov
 */

#include "estimate_noise.h"

///////////////////////////////////////////////////////////////////////////////


/*
 * Estimate the standard deviation of the noise in a gray-scale image.
 *  J. Immerkr, Fast Noise Variance Estimation,
 *    Computer Vision and Image Understanding,
 *    Vol. 64, No. 2, pp. 300-302, Sep. 1996
 *
 * Matlab code:
 *  https://www.mathworks.com/matlabcentral/fileexchange/36941-fast-noise-estimation-in-images
 */
void create_noise_map(cv::InputArray src, cv::OutputArray dst, cv::InputArray mask)
{
  /* Compute sum of absolute values of special laplacian */

  constexpr double S = 0.20888568955258338; // sqrt(M_PI_2) / 6.0

  static thread_local float C[3 * 3] = {
      +1 * S, -2 * S, +1 * S,
      -2 * S, +4 * S, -2 * S,
      +1 * S, -2 * S, +1 * S
  };

  static thread_local cv::Mat1f K(3,3, C);

  cv::filter2D(src, dst,
      dst.fixedType() ? dst.type() : src.depth() == CV_64F ? CV_64F : CV_32F,
      K,
      cv::Point(-1,-1),
      0,
      cv::BORDER_REPLICATE);

  if ( !mask.empty() ) {
    cv::Mat mtmp;
    cv::dilate(~mask.getMat(), mtmp, cv::Mat1b(3, 3, 255));
    dst.setTo(0, mtmp);
  }
}

/*
 * Estimate the standard deviation of the noise in a gray-scale image.
 *  J. Immerkr, Fast Noise Variance Estimation,
 *    Computer Vision and Image Understanding,
 *    Vol. 64, No. 2, pp. 300-302, Sep. 1996
 *
 * Matlab code:
 *  https://www.mathworks.com/matlabcentral/fileexchange/36941-fast-noise-estimation-in-images
 */
cv::Scalar estimate_noise(cv::InputArray src, cv::OutputArray dst, cv::InputArray mask)
{
  cv::Mat H;
  cv::Scalar sigma;

  create_noise_map(src, H, cv::noArray());


  if ( H.depth() != CV_8U && H.depth() != CV_16U ) {
    absdiff(H, 0, H);
  }

  if ( dst.needed() ) {
    if ( dst.fixedType() ) {
      H.convertTo(dst, dst.depth());
    }
    else {
      H.copyTo(dst);
    }
  }


  if ( mask.empty() ) {
    // sigma = cv::sum(H) / ((H.cols - 2) * (H.rows - 2));
    sigma = cv::mean(H);
  }
  else {

    cv::Mat emask;

    cv::erode(mask, emask, cv::Mat1b(5, 5, 255),
        cv::Point(-1,-1), 1,
        cv::BORDER_REPLICATE);

    sigma = cv::mean(H, emask);

    if ( dst.needed() ) {
      dst.setTo(0, ~emask);
    }
  }

  return sigma;
}

int extract_channel_with_max_sn_ratio(cv::InputArray src, cv::OutputArray dst, cv::InputArray srcmsk,
    double * s, double * n)
{
  cv::Scalar mean, stdev, noise;

  int nb_channels = src.channels(), best_channel = 0;

  double best_ratio = 0;

  cv::meanStdDev(src, mean, stdev, srcmsk);

  noise = estimate_noise(src, cv::noArray(), srcmsk);

  for ( int i  = 0; i < nb_channels; ++i ) {
    const double ratio = noise[i] > 0 ? stdev[i] / noise[i] : DBL_MAX;
    if ( ratio > best_ratio ) {
      best_ratio = ratio;
      best_channel = i;
    }
  }

  if ( dst.needed() ) {
    cv::extractChannel(src, dst, best_channel);
  }

  if ( s ) {
    *s = stdev[best_channel];
  }

  if ( n ) {
    *n = noise[best_channel];
  }

  return best_channel;
}
