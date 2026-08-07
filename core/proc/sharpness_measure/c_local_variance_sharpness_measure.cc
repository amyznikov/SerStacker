/*
 * c_mgmap_sharpness_measure.cc
 *
 *  Created on: Jul 30, 2026
 *      Author: amyznikov
 */

#include "c_local_variance_sharpness_measure.h"
#include <core/proc/pixtype.h>
#include <core/proc/run-loop.h>
#include <atomic>

namespace {

static bool pdownscale(cv::InputArray src, cv::Mat & dst, int level, int border_mode = cv::BORDER_DEFAULT)
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

static bool pupscale(cv::Mat & image, cv::Size dstSize)
{
  const cv::Size inputSize = image.size();

  if( inputSize != dstSize ) {

    std::vector<cv::Size> sizes;

    sizes.emplace_back(dstSize);

    while (42) {
      const cv::Size nextSize((sizes.back().width + 1) / 2, (sizes.back().height + 1) / 2);
      if( nextSize == inputSize ) {
        break;
      }
      if( nextSize.width < inputSize.width || nextSize.height < inputSize.height ) {
        CF_ERROR("FATAL: invalid next size : nextSize=%dx%d inputSize=%dx%d",
            nextSize.width, nextSize.height,
            inputSize.width, inputSize.height);
        return false;
      }
      sizes.emplace_back(nextSize);
    }

    for( int i = sizes.size() - 1; i >= 0; --i ) {
      cv::pyrUp(image, image, sizes[i]);
    }
  }

  return true;
}

template<typename _Tp>
static double _compute_sharpness_norm(cv::InputArray src, double depthScale, double W)
{
  const int rows = src.rows();
  const int cols = src.cols();
  const cv::Mat_<_Tp> G = src.getMat();

  std::atomic<float> total_sum(0.0f);

  parallel_for(0, rows, [&, cols](const auto& range) {
    float local_sum = 0.0;

    for (int y = rbegin(range); y != rend(range); ++y) {
      const _Tp* gp = G[y];

      for (int x = 0; x < cols; ++x) {
        const float val = gp[x];
        local_sum += val * val * val * val; // val^4
      }
    }

    float current = total_sum.load(std::memory_order_relaxed);
    while (!total_sum.compare_exchange_weak(current, current + local_sum,
        std::memory_order_relaxed));
  });

  const double scale3 = depthScale * depthScale * depthScale / W;
  return total_sum.load() * scale3;
}

static double compute_sharpness_norm(cv::InputArray src, double depthScale, double W)
{
  CV_DISPATCH(src.depth(), _compute_sharpness_norm, src, depthScale, W);
  return 0.0;
}

/**
 * Can not work in-place, dst must not refer to src
 * */
template<typename _Tp>
static double _compute_sharpness_map(cv::InputArray _src, cv::OutputArray _dst, double depthScale, double W)
{
  const int rows = _src.rows();
  const int cols = _src.cols();
  const cv::Mat_<_Tp> G = _src.getMat();

  _dst.create(rows, cols, CV_32FC1);
  cv::Mat1f dst = _dst.getMatRef();

  const float mapScale = float (depthScale * depthScale * depthScale);

  std::atomic<float> total_sum(0.0f);

  parallel_for(0, rows, [&, cols, mapScale](const auto& range) {
    float local_sum = 0.0f;

    for (int y = rbegin(range); y != rend(range); ++y) {
      const _Tp* g_row = G[y];
      float* __restrict dstp = dst[y];

      for (int x = 0; x < cols; ++x) {
        const float val = g_row[x];
        dstp[x] = val * val * val * mapScale;
        local_sum += val * val * val * val;
      }
    }

    float current = total_sum.load(std::memory_order_relaxed);
    while (!total_sum.compare_exchange_weak(current, current + local_sum,
        std::memory_order_relaxed));
  });

  const double scale3 = depthScale * depthScale * depthScale / W;
  return scale3 * total_sum.load();
}

static double compute_sharpness_map(cv::InputArray src, cv::OutputArray dst, double depthScale, double W)
{
  CV_DISPATCH(src.depth(), _compute_sharpness_map, src, dst, depthScale, W);
  return 0.0;
}

} // namespace

// Return frame quality metric to allow compare consecutive frames for live stacking
// MAP(x,y) = G(x,y) ^ n / sum(G)
// Q = sum(G(x,y) ^ n) / sum(G)
// The reasoning is to compute weighted average of morph gradient cubic
// using gradient module as the weight, thus estimate gradient cubics
// in the regions where the gradients are exists
double compute_local_variance_map(cv::InputArray image, const c_local_variance_map_options & opts,
    cv::OutputArray outputMap /*= cv::noArray()*/)
{
  cv::Mat M, G;

  const int ksize = 2 * std::max(1, opts.kradius) + 1;
  const cv::Mat1b SE(ksize, ksize, 255);
  const double depthScale = 20 * getMaxValForPixelDepth(CV_32F) / getMaxValForPixelDepth(image.depth());

  extract_channel(image, M, cv::noArray(), cv::noArray(), opts.channel, -1, false);
  if( opts.dscale > 0 ) {
    pdownscale(M, M, opts.dscale);
  }

  cv::morphologyEx(M, G, cv::MORPH_GRADIENT, SE, cv::Point(-1, -1), 1,
      cv::BORDER_REPLICATE);

  const double W = cv::norm(G, cv::NORM_L1);
  if( !(W > 0) ) {
    if( outputMap.needed() ) {
      outputMap.create(image.size(), CV_32F);
      outputMap.setTo(0);
    }
    return 0;
  }

  // Optimized path for Q without map
  if( !outputMap.needed() ) {
    return compute_sharpness_norm(G, depthScale, W);
  }

  // Little slower path path for Q with map
  // Add some minimal feasible weight to the map for totally flat areas
  const double Q = compute_sharpness_map(G, M, depthScale, W);
  if( opts.uscale > 0 ) {
    pdownscale(M, M, opts.uscale);
  }
  cv::add(M, 0.05 * Q, M);
  if( G.size() != image.size() ) {
    pupscale(M, image.size());
  }
  outputMap.move(M);

  return Q;
}

bool c_local_variance_sharpness_measure::create_map(cv::InputArray image, cv::OutputArray outputMap,
    const c_local_variance_map_options & opts)
{
  compute_local_variance_map(image, opts, outputMap);
  return true;
}

bool c_local_variance_sharpness_measure::create_map(cv::InputArray image, cv::OutputArray outputMap) const
{
  return create_map(image, outputMap, opts);
}

cv::Scalar c_local_variance_sharpness_measure::compute(cv::InputArray image, cv::InputArray mask) const
{
  return cv::Scalar::all(compute_local_variance_map(image, opts));
}

bool serialize_local_variance_map_options(c_config_setting section, bool save, c_local_variance_map_options & opts)
{
  SERIALIZE_OPTION(section, save, opts, channel);
  SERIALIZE_OPTION(section, save, opts, kradius);
  SERIALIZE_OPTION(section, save, opts, dscale);
  SERIALIZE_OPTION(section, save, opts, uscale);
  return true;
}
