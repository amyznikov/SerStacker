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

/**
 * Histogram-based automatic adjustment of MTF parameters for c_smooth_rational_mtf
 */
#if 1
void autoMtf(const cv::Mat1d & H, double realMinValue, double realMaxValue,
    double * lclip, double * hclip, double * shadow, double * midtones, double * highlights)
{
    cv::Mat1d Hc;
    makeCumulativeHistogram(H, Hc);
    normalizeHistogram(Hc, Hc, true);

    // BASIC CALIBRATION PARAMETERS
    const double DS_DELTA_LIMIT = 0.01;   // Border of pure Dipskaya (needle)
    const double LUNAR_DELTA_LIMIT = 0.10; // Bright surface boundary (hill)
    const double TARGET_DS = 0.50;        // Target brightness for Deepsky
    const double TARGET_LUNAR = 0.38;     // Target brightness for the Moon (slightly increased from 0.35)
    const double MAX_NEG_SHADOW = -0.10;  // Maximum upward "bending" of shadows
    const double SHADOW_DELTA_CUTOFF = 0.04; // Delta above which shadow is always 0
    // -------------------------------------------------------

    cv::Scalar lb, hb;
    computeHistogramClipLevels(Hc, realMinValue, realMaxValue, 0.001, 0.9999, lb, hb);
    *lclip = lb[0];
    *hclip = hb[0];

    const double medianVal = computeHistogramMedian(Hc, realMinValue, realMaxValue)[0];
    const double range = std::max(1e-9, (realMaxValue - realMinValue));
    const double l_norm = (*lclip - realMinValue) / range;
    const double m_norm = (medianVal - realMinValue) / range;

    const double delta = std::max(1e-9, m_norm - l_norm);
    const double t_med_rel = std::clamp((medianVal - *lclip) / (*hclip - *lclip + 1e-9), 0.001, 0.99);

    // Adaptive estimation of target brightness
    const double weight = std::clamp((delta - DS_DELTA_LIMIT) / (LUNAR_DELTA_LIMIT - DS_DELTA_LIMIT), 0.0, 1.0);
    const double target_val = TARGET_DS - weight * (TARGET_DS - TARGET_LUNAR);

    // Estimate midtones (k_needed -> Options.midtones)
    const double k_needed = (t_med_rel * (1.0 - target_val)) / (target_val * (1.0 - t_med_rel) + 1e-9);
    *midtones = std::clamp(1.0 / (k_needed + 1.0), 0.05, 0.98);

    // Adaptive Shadow
    if (delta < SHADOW_DELTA_CUTOFF) {
        *shadow = std::clamp(MAX_NEG_SHADOW * (1.0 - delta / SHADOW_DELTA_CUTOFF), MAX_NEG_SHADOW, 0.0);
    } else {
        *shadow = 0.0;
    }

    *highlights = 0.0;
}

#else
void autoMtf(const cv::Mat1d & H, double realMinValue, double realMaxValue,
    double * lclip, double * hclip, double * shadow, double * midtones, double * highlights)
{
  cv::Mat1d Hc;

  // Estimate the histogram mode (sky background peak)
  const double mode = computeHistogramMode(H, realMinValue, realMaxValue)[0];

  // Preparing a cumulative histogram
  // Clipping (1% below is usually sufficient, but you can try 0.005 to preserve faint nebulae)
  makeCumulativeHistogram(H, Hc);
  normalizeHistogram(Hc, Hc, true);

  cv::Scalar lowBound, highBound;
  computeHistogramClipLevels(Hc, realMinValue, realMaxValue, 0.0001, 0.9999, lowBound, highBound);
  *lclip = lowBound[0];
  *hclip = highBound[0];

  // Estimate the histogram median
  const double medianVal = computeHistogramMedian(Hc, realMinValue, realMaxValue)[0];

  // Relative positions
  const double range = std::max(1e-9, (*hclip - *lclip));
  const double t_med = std::clamp((medianVal - *lclip) / range, 0.01, 0.99);
  const double t_mode = std::clamp((mode - *lclip) / range, 0.0, 0.99);

  // Auto-Midtones (Brightness balance)
  *midtones = std::clamp(1.0 - t_med, 0.02, 0.98);

  // Auto-Shadows (Darkening the sky background) - SOFT HEURISTICS
  const double bg_dist = t_med - t_mode;
  CF_DEBUG("t_med=%g t_mode=%g bg_dist=%g bg_ratio=%g", t_med, t_mode, bg_dist, bg_dist / (t_mode + t_med));

  // The threshold of 0.15 is more typical for deep-pocketing.
  const double threshold = 0.01;
  if( bg_dist >= threshold ) {
    // For photos with a wide histogram (moon and planets)
    *shadow = 0.0;
  }
  else {
    // The closer the median is to the mode, the more aggressively we push the background towards black.
    // Reduce the maximum strength to 0.45 (safer for stacks)
    const double max_shadow = 0.45;
    *shadow = std::clamp(max_shadow * (1.0 - bg_dist / threshold), 0.0, max_shadow);
  }

  // keep zero highlights to don't loss bright details
  *highlights = 0.0;
}
#endif
