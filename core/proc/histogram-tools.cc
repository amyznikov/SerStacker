/*
 * histogram-tools.cc
 *
 *  Created on: Mar 10, 2026
 *      Author: amyznikov
 */
#include "histogram-tools.h"
#include <core/ssprintf.h>
#include <core/debug.h>


template<>
const c_enum_member * members_of<histogram_normalization_type>()
{
  static const c_enum_member members[] = {
      { normalize_histogram_mean, "mean" },
      { normalize_histogram_median, "median" },
      { normalize_histogram_mode, "mode" },
      { normalize_histogram_mean }  // must  be last
  };

  return members;
}

static cv::Scalar computeImageMode(cv::InputArray image, cv::InputArray mask)
{
  const uint32_t nbins = 0;
  double minv = -1, maxv = -1;
  cv::Mat1d H;

  bool fOK =
      createHistogram(image,
          mask,
          &minv,
          &maxv,
          nbins,
          H,
          false,
          false);

  if( !fOK ) {
    CF_ERROR("createHistogram() fails");
    return cv::Scalar();
  }

  return computeHistogramMode(H, minv, maxv);
}

static cv::Scalar computeImageMedian(cv::InputArray image, cv::InputArray mask)
{
  const uint32_t nbins = 0;
  double minv = -1, maxv = -1;
  cv::Mat1d H;

  bool fOK =
      createHistogram(image,
          mask,
          &minv,
          &maxv,
          nbins,
          H,
          true,
          true);

  if( !fOK ) {
    CF_ERROR("createHistogram() fails");
    return cv::Scalar();
  }

  return computeHistogramMedian(H, minv, maxv);
}

/**
 *  dst = (src - mv) * stretch + offset
 *   => dst = src * stretch + offset - mv * stretch
 */
bool nomalizeImageHistogram(cv::InputArray image, cv::InputArray mask, cv::OutputArray dst,
    const c_histogram_normalization_options & opts,
    enum COLORID src_colorid)
{
  cv::Mat src;
  cv::Mat msk;

  const bool isBayerPattern = is_bayer_pattern(src_colorid);
  if ( !isBayerPattern ) {
    src = image.getMat();
    msk = mask.getMat();
  }
  else {
    if( src.channels() != 1 ) {
      CF_ERROR("Invalid number of image channels %d for bayer pattern %s. Must be 1",
          src.channels(), toCString(src_colorid));
      return false;
    }

    if( (src.rows & 0x1) || (src.cols & 0x1)  ) {
      CF_ERROR("Invalid image size %dx%d for Bayer pattern '%s', must be even",
          src.cols, src.rows,  toCString(src_colorid));
      return false;
    }

    if ( !extract_bayer_planes(image, src, src_colorid) ) {
      CF_ERROR("extract_bayer_planes() fails for colorid=%d ('%s')",
          (int)src_colorid, toCString(src_colorid));
      return false;
    }
  }

  static const auto stretchLevels =
      [](cv::InputArray src, cv::OutputArray dst, const cv::Scalar & mv,
          const c_histogram_normalization_options & opts, bool isBayerPattern) {

            switch (src.channels()) {
              case 1: {
                const cv::Matx12f m(opts.stretch(0), opts.offset(0) - mv(0) * opts.stretch(0));
                cv::transform(src, dst, m);
                break;
              }
              case 2: {
                const cv::Matx23f m(opts.stretch(0), 0, opts.offset(0) - mv(0) * opts.stretch(0),
                    0, opts.stretch(1), opts.offset(1) - mv(1) * opts.stretch(1));
                cv::transform(src, dst, m);
                break;
              }
              case 3: {
                const cv::Matx34f m(opts.stretch(0), 0, 0, opts.offset(0) - mv(0) * opts.stretch(0),
                    0, opts.stretch(1), 0, opts.offset(1) - mv(1) * opts.stretch(1),
                    0, 0, opts.stretch(2), opts.offset(2) - mv(2) * opts.stretch(2));
                cv::transform(src, dst, m);
                break;
              }
              case 4: {
                  typedef cv::Matx<float, 4, 5> Matx45f;
                  Matx45f m = Matx45f::zeros();
                  m(0, 0) = opts.stretch(0), m(0, 4) = opts.offset(0) - mv(0) * opts.stretch(0);
                  m(1, 1) = opts.stretch(1), m(1, 4) = opts.offset(1) - mv(1) * opts.stretch(1);
                  m(2, 2) = opts.stretch(2), m(2, 4) = opts.offset(2) - mv(2) * opts.stretch(2);
                  m(3, 3) = opts.stretch(3), m(3, 4) = opts.offset(3) - mv(3) * opts.stretch(3);

                  if ( !isBayerPattern ) {
                    cv::transform(src, dst, m);
                  }
                  else {
                    cv::Mat tmp;
                    cv::transform(src, tmp, m);
                    bayer_planes_to_bgr(tmp, dst);
                  }
              }
              break;
            }
          };

  switch (opts.norm_type) {

    case normalize_histogram_mean:
      stretchLevels(image, dst, cv::mean(src, msk), opts, isBayerPattern);
      break;
    case normalize_histogram_mode:
      stretchLevels(image, dst, computeImageMode(src, msk), opts, isBayerPattern);
      break;
    case normalize_histogram_median:
      stretchLevels(image, dst, computeImageMedian(src, msk), opts, isBayerPattern);
      break;
    default:
      CF_ERROR("Invalid norm_type=%d requested", opts.norm_type);
      return false;
  }

  return true;
}


/**
* @param qLow - lower quantile (e.g., 0.01 for 1%)
* @param qHigh - upper quantile (e.g., 0.99 for 99%)
*  */
bool histogramClipWhiteBalance(cv::InputArray src, cv::InputArray mask, cv::OutputArray dst,
    double qlow, double qhigh)
{
  if( src.empty() || src.channels() < 2 ) {
    src.copyTo(dst);
    return true;
  }

  const int cn = src.channels();
  const int depth = src.depth();

  cv::Mat1d H;
  double minv = -1.0, maxv = -1.0;
  const uint32_t nbins = depth >= CV_16U ? 16384U : 0; // allow auto for 8bit
  if( !createHistogram(src, mask, &minv, &maxv, nbins, H, true, true) ) {
    return false;
  }

  cv::Scalar lowLvl, highLvl;
  computeHistogramClipLevels(H, minv, maxv, qlow, qhigh, lowLvl, highLvl);

  // Calculate the "safe" global range (covering all channels)
  double globalLow = DBL_MAX;
  double globalHigh = -DBL_MAX;
  for( int c = 0; c < cn; ++c ) {
    if( lowLvl[c] < globalLow ) {
      globalLow = lowLvl[c];
    }
    if( highLvl[c] > globalHigh ) {
      globalHigh = highLvl[c];
    }
  }


  // Apply the correction to each channel individually
  // Formula for channel c: dst_c = (src_c - low_c) * (targetRange / currentRange) + avgLow
  std::vector<cv::Mat> channels;
  cv::split(src.getMat(), channels);

  const double targetRange = globalHigh - globalLow;

  for( int c = 0; c < cn; ++c ) {
    const double channelRange = highLvl[c] - lowLvl[c];
    // If the channel is almost empty, leave it alone to avoid increasing noise
    if( channelRange > 1e-10 ) {
      const double scale = targetRange / channelRange;
      const double shift = globalLow - (lowLvl[c] * scale);
      channels[c].convertTo(channels[c], -1, scale, shift);
    }
  }

  cv::merge(channels, dst);

  return true;
}
