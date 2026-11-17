/*
 * median.cc
 *
 *  Created on: Nov 11, 2016
 *      Author: amyznikov
 */

#include "median.h"
#include <core/proc/pixtype.h>
#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////

template<class _Tp>
static _Tp _median(const cv::Mat & image, cv::InputArray _mask)
{
  std::vector<_Tp> samples;
  cv::Mat mask;
  int nnz;

  if ( (mask = _mask.getMat()).empty() ) {
    // spread Mat to single row
    if ( image.isContinuous() ) {
      image.reshape(0, 1).copyTo(samples);
    }
    else {
      cv::Mat array;
      image.copyTo(array);
      array.reshape(0, 1).copyTo(samples);
    }
  }
  else if ( (nnz = countNonZero(mask)) > 0 ) {
    samples.resize(nnz);
    for ( int y = 0, z = 0; y < mask.rows; ++y ) {
      const uint8_t * mp = mask.ptr<const uint8_t>(y);
      const _Tp * imgp = image.ptr<const _Tp>(y);
      for ( int x = 0; x < mask.cols; ++x ) {
        if ( mp[x] ) {
          samples[z++] = imgp[x];
        }
      }
    }
  }
  else {
    return 0;
  }

  const size_t mid = samples.size() / 2;
  std::nth_element(samples.begin(), samples.begin() + mid, samples.end());
  return samples[mid];

  // call median
  //return median_(&(*vec.begin()), vec.size());
}

double median(const cv::Mat & image, cv::InputArray _mask)
{
  CV_Assert(!image.empty());
  CV_Assert(image.channels() == 1);
  CV_DISPATCH(image.depth(), _median, image, _mask);
  cv::error(cv::Error::BadDepth, "Unsupported image type", CV_Func, __FILE__, __LINE__);
  return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////

// Median of background
template<class _Tp>
static _Tp _estimateMedian(cv::InputArray _src, int stepX, int stepY)
{
  if( _src.empty() || _src.channels() != 1 ) {
    CF_ERROR("Non-empty single-channel input image expected");
    return 0;
  }

  if( stepX <= 0 ) {
    stepX = 4;
  }
  if( stepY <= 0 ) {
    stepY = 4;
  }

  const int width = _src.cols();
  const int height = _src.rows();
  const size_t est_count = size_t((height + stepY - 1) / stepY) * size_t((width + stepX - 1) / stepX);
  if( est_count < 3 ) {
    return 0;
  }

  const cv::Mat src = _src.getMat();
  const uint8_t * src_base = src.ptr();
  const size_t src_stride = src.step;

  std::vector<_Tp> samples;
  samples.reserve(est_count);

  for( int y = 0; y < height; y += stepY ) {
    const _Tp * srcp = (const _Tp*) (src_base + y * src_stride);
    for( int x = 0; x < width; x += stepX ) {
      samples.emplace_back(srcp[x]);
    }
  }

  const size_t mid = samples.size() / 2;
  std::nth_element(samples.begin(), samples.begin() + mid, samples.end());
  return samples[mid];
}

// Median of background + stdev
// The coefficient 1.4826 relates the MAD to the standard deviation of Gaussian noise
template<class _Tp>
static double _estimateMedianStdev(cv::InputArray _src, double & outSigma, int stepX, int stepY)
{
  constexpr double mad_to_sigma = 1.4826;

  if( _src.empty() || _src.channels() != 1 ) {
    CF_ERROR("Non-empty single-channel input image expected");
    return 0;
  }

  if( stepX <= 0 ) {
    stepX = 4;
  }
  if( stepY <= 0 ) {
    stepY = 4;
  }

  const int width = _src.cols();
  const int height = _src.rows();
  const size_t est_count = size_t((height + stepY - 1) / stepY) * size_t((width + stepX - 1) / stepX);
  if( est_count < 3 ) {
    return 0;
  }

  const cv::Mat src = _src.getMat();
  const uint8_t * src_base = src.ptr();
  const size_t src_stride = src.step;

  using _Tpc = std::conditional_t<std::is_floating_point_v<_Tp>, _Tp, float>;
  std::vector<_Tpc> samples;
  samples.reserve(est_count);

  for( int y = 0; y < height; y += stepY ) {
    const _Tp * srcp = (const _Tp*) (src_base + y * src_stride);
    for( int x = 0; x < width; x += stepX ) {
      samples.emplace_back(srcp[x]);
    }
  }

  // Median of background
  const size_t mid = samples.size() / 2;
  std::nth_element(samples.begin(), samples.begin() + mid, samples.end());
  const _Tpc median = samples[mid];

  // Median of absolute deviation (MAD) in O(N)
  cv::absdiff(samples, median, samples);
  std::nth_element(samples.begin(), samples.begin() + mid, samples.end());
  outSigma = samples[mid] * mad_to_sigma;

  return median;
}

double estimateMedianStdev(cv::InputArray _src, double * outSigma, int stepX, int stepY)
{
  INSTRUMENT_REGION("threshold");
  if( outSigma ) {
    CV_DISPATCH(_src.depth(), _estimateMedianStdev, _src, *outSigma, stepX, stepY);
  }
  else {
    CV_DISPATCH(_src.depth(), _estimateMedian, _src, stepX, stepY);
  }
  return 0;
}

