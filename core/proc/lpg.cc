/*
 * lpg.cc
 *
 *  Created on: Apr 7, 2023
 *      Author: amyznikov
 */

#include "lpg.h"
#include <core/proc/reduce_channels.h>
#include <core/debug.h>

static void compute_gradient(const cv::Mat & src, cv::Mat & g)
{
  static thread_local cv::Mat Kx, Ky;
  if( Kx.empty() ) {
    cv::getDerivKernels(Kx, Ky, 1, 0, 3, true, CV_32F);
    Kx *= M_SQRT2;
    Ky *= M_SQRT2;
  }

  cv::Mat gx, gy;

  cv::sepFilter2D(src, gx, CV_32F, Kx, Ky, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::sepFilter2D(src, gy, CV_32F, Ky, Kx, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

  cv::add(gx.mul(gx), gy.mul(gy), g);
}

// https://jblindsay.github.io/ghrg/Whitebox/Help/FilterLaplacian.html
static void compute_laplacian(const cv::Mat & src, cv::Mat & l)
{
  static thread_local cv::Mat Kx, Ky;
  if( Kx.empty() ) {
    cv::getDerivKernels(Kx, Ky, 2, 0, 3, true, CV_32F);
    Kx *= M_SQRT2;
    Ky *= M_SQRT2;
  }

  cv::Mat gx, gy;

  cv::sepFilter2D(src, gx, CV_32F, Kx, Ky, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::sepFilter2D(src, gy, CV_32F, Ky, Kx, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

  cv::add(gx.mul(gx), gy.mul(gy), l);
}


template<class T>
static inline T square(T x)
{
  return x * x;
}

/**
* Optimized local sharpening calculation (Gradient + Laplacian 5x5)
* src: input image (already normalized and downscaled, if necessary)
* dst: output weight map
* alpha, beta: weights for Laplacian and gradient
**/
static void compute_lpg_5x5_parallel(const cv::Mat1f & src, cv::Mat1f & dst, float alpha, float beta, float eps)
{
  if( dst.size() != src.size() ) {
    dst.create(src.size());
  }

  //constexpr float l_norm = 1.0f / 16.0f;
  constexpr float l_norm = 100.0f / 4.0f;
  constexpr float g_norm = 100.0f / 36.0f;

  alpha *= (l_norm * l_norm);
  beta *= (g_norm * g_norm);

  // Row-wise parallelization via TBB
  cv::parallel_for_(cv::Range(2, src.rows - 2),
      [&src, &dst, alpha, beta, eps](const cv::Range & range) {
        for (int y = range.start; y < range.end; ++y) {


          const float* r0 = src[y - 2];
          const float* r1 = src[y - 1];
          const float* r2 = src[y];
          const float* r3 = src[y + 1];
          const float* r4 = src[y + 2];
          float* out = dst[y];

          const int cols = src.cols;
          for (int x = 2; x < cols - 2; ++x) {
            // 1. Extended Sobol Gradient 5x5 (smoothing + derivative)
            // Horizontal component dX
            const float gx =
                (r0[x+2] + 2*r1[x+2] + 4*r2[x+2] + 2*r3[x+2] + r4[x+2]) -
                  (r0[x-2] + 2*r1[x-2] + 4*r2[x-2] + 2*r3[x-2] + r4[x-2]) +
                  2.0f * ((r1[x+1] + 2*r2[x+1] + r3[x+1]) - (r1[x-1] + 2*r2[x-1] + r3[x-1]));

            // Vertical component dY
            const float gy =
                (r4[x-2] + 2*r4[x-1] + 4*r4[x] + 2*r4[x+1] + r4[x+2]) -
                  (r0[x-2] + 2*r0[x-1] + 4*r0[x] + 2*r0[x+1] + r0[x+2]) +
                  2.0f * ((r3[x-1] + 2*r3[x] + r3[x+1]) - (r1[x-1] + 2*r1[x] + r1[x+1]));

            const float grad =
                (gx * gx + gy * gy);

            // 2. 5x5 Noise-Robust Laplacian
            // Sum with weights to highlight high-frequency details
            const float lapl =
                square(16.0f * r2[x] -
                  2.0f * (r1[x] + r3[x] + r2[x-1] + r2[x+1]) -
                  1.0f * (r1[x-1] + r1[x+1] + r3[x-1] + r3[x+1]) -
                  1.0f * (r0[x] + r4[x] + r2[x-2] + r2[x+2]));

            // 3. Linear combination "on the fly"
            out[x] = (float)(alpha * lapl + beta * grad + eps);
          }

          // Fill the edges of the current row (copy the nearest calculated pixel)
          out[0] = out[1] = out[2];
          out[cols - 1] = out[cols - 2] = out[cols - 3];
        }
      });

  // 2. Fill the top and bottom rows (copy the closest calculated ones)
  if( src.rows > 4 ) {
    dst.row(2).copyTo(dst.row(0));
    dst.row(2).copyTo(dst.row(1));
    dst.row(src.rows - 3).copyTo(dst.row(src.rows - 2));
    dst.row(src.rows - 3).copyTo(dst.row(src.rows - 1));
  }
}


static bool downscale(cv::InputArray src, cv::Mat & dst, int level, int border_mode = cv::BORDER_DEFAULT)
{
  if( std::min(src.cols(), src.rows()) < 4 ) {
    src.copyTo(dst);
  }
  else {

    cv::pyrDown(src, dst, cv::Size(), border_mode);

    if( std::min(dst.cols, dst.rows) >= 4 ) {

      for( int l = 1; l < level; ++l ) {

        cv::pyrDown(dst, dst, cv::Size(), border_mode);

        if( std::min(dst.cols, dst.rows) < 4 ) {
          break;
        }
      }
    }
  }

  return true;
}

static bool upscale(cv::Mat & image, cv::Size dstSize)
{
  const cv::Size inputSize = image.size();

  if( inputSize != dstSize ) {

    std::vector<cv::Size> spyramid;

    spyramid.emplace_back(dstSize);

    while (42) {
      const cv::Size nextSize((spyramid.back().width + 1) / 2, (spyramid.back().height + 1) / 2);
      if( nextSize == inputSize ) {
        break;
      }
      if( nextSize.width < inputSize.width || nextSize.height < inputSize.height ) {
        CF_ERROR("FATAL: invalid next size : nextSize=%dx%d inputSize=%dx%d",
            nextSize.width, nextSize.height,
            inputSize.width, inputSize.height);
        return false;
      }
      spyramid.emplace_back(nextSize);
    }

    for( int i = spyramid.size() - 1; i >= 0; --i ) {
      cv::pyrUp(image, image, spyramid[i]);
    }
  }

  return true;
}

static double maxval(int ddepth)
{
  switch (CV_MAT_DEPTH(ddepth)) {
    case CV_8U:
      return UINT8_MAX;
    case CV_8S:
      return INT8_MAX;
    case CV_16U:
      return UINT16_MAX;
    case CV_16S:
      return INT16_MAX;
    case CV_32S:
      return INT32_MAX;
  }

  return 1;
}

/**
 * lpg:
 *
 *  Create the map of
 *      alpha * |laplacian| +  beta * |gradient|
 *
 * with
 *    alpha = k / ( k + 1)
 *    beta = 1 / (k + 1)
 *
 *
 *
 * For noise filtering purposes the input image can be optionally downscaled using cv::pyrDown(),
 * and the output map can be upscaled using cv::pyrUp().
 *
 * When average_color_channels is true the color channels of input image are averaged into
 * single channel before processing.
 *
 */
bool lpg(cv::InputArray image, cv::InputArray mask, cv::OutputArray optional_output_map,
    double k, double p, int dscale, int uscale, bool average_color_channels,
    cv::Scalar * optional_output_sharpness_metric)
{

  const cv::Mat src = image.getMat();

  //cv::Mat s, l, g, m;
  cv::Mat s;
  cv::Mat1f m;
  double min, max;

  if ( src.depth() == CV_32F ) {
    src.copyTo(s);
  }
  else {
    src.convertTo(s, CV_32F, 1. / maxval(src.depth()));
  }

  if( s.channels() > 1 /*&& average_color_channels */) {
    reduce_color_channels(s, s, cv::REDUCE_AVG);
  }

  if( dscale > 0 ) {
    downscale(s, s, dscale);
    cv::multiply(s, cv::Scalar::all(1.0 / (1 + dscale)), s);
  }

#if 1
  compute_lpg_5x5_parallel(s, m, k / (k + 1), 1. / (k + 1), 1e-9);
#else
  cv::Mat l, g;
  compute_laplacian(s, l);
  compute_gradient(s, g);
  cv::addWeighted(l, k / (k + 1.), g, 1. / (k + 1), 0, m);
#endif

  if( uscale > 0 && uscale > dscale ) {
    downscale(m, m, uscale - std::max(0, dscale));
    cv::multiply(m, cv::Scalar::all(uscale - std::max(0, dscale)), m);
  }

  if( p != 0 && p != 1  ) {
    cv::pow(m, p, m);
  }

  if( m.size() != src.size() ) {
    upscale(m, src.size());
  }

  if( optional_output_sharpness_metric ) {

    if( mask.empty() ) {
      *optional_output_sharpness_metric = cv::mean(m);
    }
    else {
      cv::Mat1b msk;
      cv::resize(mask, msk, m.size(), 0, 0, cv::INTER_NEAREST);
      *optional_output_sharpness_metric = cv::mean(m, msk);
    }
  }

  if( optional_output_map.needed() ) {
    optional_output_map.move(m);
  }

  return true;
}

