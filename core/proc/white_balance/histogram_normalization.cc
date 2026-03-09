/*
 * histogram_normalization.cc
 *
 *  Created on: Jun 4, 2023
 *      Author: amyznikov
 */

#include "histogram_normalization.h"
#include <core/proc/histogram.h>
#include <core/proc/histogram.h>
#include <core/ssprintf.h>
#include <core/debug.h>


template<>
const c_enum_member * members_of<histogram_normalization_type>()
{
  static const c_enum_member members[] = {
      { histogram_normalize_mean, "mean" },
      { histogram_normalize_median, "median" },
      { histogram_normalize_mode, "mode" },
      { histogram_normalize_clip_ramge, "clip" },
      { histogram_normalize_mean }  // must  be last
  };

  return members;
}


static void generate_color_masks(const cv::Size & size, const cv::Mat1b & src_mask, cv::Mat1b masks[4])
{
  for( int i = 0; i < 4; ++i ) {
    masks[i].create(size);
    masks[i].setTo(0);
  }

  if( src_mask.empty() ) {
    for( int y = 0; y < size.height / 2; ++y ) {
      for( int x = 0; x < size.width / 2; ++x ) {
        masks[0][2 * y + 0][2 * x + 0] = 255;
        masks[1][2 * y + 0][2 * x + 1] = 255;
        masks[2][2 * y + 1][2 * x + 0] = 255;
        masks[3][2 * y + 1][2 * x + 1] = 255;
      }
    }
  }

  else {

    for( int y = 0; y < size.height / 2; ++y ) {
      for( int x = 0; x < size.width / 2; ++x ) {
        masks[0][2 * y + 0][2 * x + 0] = src_mask[2 * y + 0][2 * x + 0];
        masks[1][2 * y + 0][2 * x + 1] = src_mask[2 * y + 0][2 * x + 1];
        masks[2][2 * y + 1][2 * x + 0] = src_mask[2 * y + 1][2 * x + 0];
        masks[3][2 * y + 1][2 * x + 1] = src_mask[2 * y + 1][2 * x + 1];
      }
    }

  }
}

template<class T>
static void _normalize_bayer_pattern(cv::Mat & _image, const cv::Scalar & mv,
    const cv::Scalar & stretch, const cv::Scalar & offset)
{
  cv::Mat_<T> image = _image;
  const cv::Size size = _image.size();

  for( int y = 0; y < size.height / 2; ++y ) {
    for( int x = 0; x < size.width / 2; ++x ) {
      image[2 * y + 0][2 * x + 0] = cv::saturate_cast<T>(
          (image[2 * y + 0][2 * x + 0] - mv[0]) * stretch[0] + offset[0]);
      image[2 * y + 0][2 * x + 1] = cv::saturate_cast<T>(
          (image[2 * y + 0][2 * x + 1] - mv[1]) * stretch[1] + offset[1]);
      image[2 * y + 1][2 * x + 0] = cv::saturate_cast<T>(
          (image[2 * y + 1][2 * x + 0] - mv[2]) * stretch[2] + offset[2]);
      image[2 * y + 1][2 * x + 1] = cv::saturate_cast<T>(
          (image[2 * y + 1][2 * x + 1] - mv[3]) * stretch[3] + offset[3]);
    }
  }
}

static void normalize_bayer_pattern(cv::Mat & _image, const cv::Scalar & mv,
    const cv::Scalar & stretch, const cv::Scalar & offset)
{
  switch (_image.depth())
  {
    case CV_8U:
      return _normalize_bayer_pattern<uint8_t>(_image, mv, stretch, offset);
    case CV_8S:
      return _normalize_bayer_pattern<int8_t>(_image, mv, stretch, offset);
    case CV_16U:
      return _normalize_bayer_pattern<uint16_t>(_image, mv, stretch, offset);
    case CV_16S:
      return _normalize_bayer_pattern<int16_t>(_image, mv, stretch, offset);
    case CV_32S:
      return _normalize_bayer_pattern<int32_t>(_image, mv, stretch, offset);
    case CV_32F:
      return _normalize_bayer_pattern<float>(_image, mv, stretch, offset);
    case CV_64F:
      return _normalize_bayer_pattern<double>(_image, mv, stretch, offset);
  }
}

static inline bool is_floating_type(int depth)
{
  return depth == CV_32F || depth == CV_64F;
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

static bool computeImageClips(cv::InputArray image, cv::InputArray mask, double qLow, double qHigh,
    cv::Scalar & lvlLow, cv::Scalar & lvlHigh)
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
    return false;
  }

  return computeHistogramClipLevels(H, minv, maxv, qLow, qHigh, lvlLow, lvlHigh);
}

// dst = (src - mv) * stretch + offset
// => dst = src * stretch + offset - mv * stretch

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

    case histogram_normalize_mean:
      stretchLevels(image, dst, cv::mean(src, msk), opts, isBayerPattern);
      break;

    case histogram_normalize_mode:
      stretchLevels(image, dst, computeImageMode(src, msk), opts, isBayerPattern);
      break;

    case histogram_normalize_median:
      stretchLevels(image, dst, computeImageMedian(src, msk), opts, isBayerPattern);
      break;

    case histogram_normalize_clip_ramge: {

      std::vector<cv::Mat> channels;
      cv::Scalar lvlLow, lvlHigh;
      double globalLow = DBL_MAX;
      double globalHigh = -DBL_MAX;

      computeImageClips(src, msk, opts.clipRange[0], opts.clipRange[1], lvlLow, lvlHigh);

      const int cn = image.channels();
      for( int c = 0; c < cn; ++c ) {
        if( lvlLow[c] < globalLow ) {
          globalLow = lvlLow[c];
        }
        if( lvlHigh[c] > globalHigh ) {
          globalHigh = lvlHigh[c];
        }
      }

      cv::split(image, channels);

      const double targetRange = globalHigh - globalLow;
      for( int c = 0; c < cn; ++c ) {
        const double channelRange = lvlHigh[c] - lvlLow[c];
        if( channelRange > 1e-10 ) {
          const double scale = targetRange / channelRange;
          const double shift = globalLow - (lvlLow[c] * scale);
          channels[c].convertTo(channels[c], -1, scale * opts.stretch[c], shift + opts.offset[c]);
        }
      }

      if ( !isBayerPattern ) {
        cv::merge(channels, dst);
      }
      else {
        cv::merge(channels, src);
        bayer_planes_to_bgr(src, dst);
      }
      break;
    }

    default:
      CF_ERROR("Invalid norm_type=%d requested", opts.norm_type);
      return false;
  }

  return true;
}





// dst = (src - mv) * stretch + offset
// => dst = src * stretch + offset - mv * stretch

bool nomalize_image_histogramm(cv::InputArray _src, cv::InputArray mask, cv::OutputArray dst,
    histogram_normalization_type norm_type,
    const cv::Scalar & stretch,
    const cv::Scalar & offset,
    enum COLORID src_colorid)
{
  const cv::Mat src = _src.getMat();
  const bool is_bayer = is_bayer_pattern(src_colorid);

  if( is_bayer && src.channels() != 1 ) {
    CF_ERROR("Invalid number of image channels %d for bayer pattern %s. Must be 1",
        src.channels(), toCString(src_colorid));
    return false;
  }

  cv::Mat1b bayer_masks[4];
  cv::Scalar mv;

  if( is_bayer ) {
    generate_color_masks(src.size(),
        mask.getMat(),
        bayer_masks);
  }

  switch (norm_type) {
    case histogram_normalize_mean:
      if( !is_bayer ) {
        mv = cv::mean(src, mask);
      }
      else {
        for( int i = 0; i < 4; ++i ) {
          mv[i] = cv::mean(src, bayer_masks[i])[0];
        }
      }
      break;

    case histogram_normalize_mode:
      if( !is_bayer ) {
        mv = computeImageMode(src, mask);
      }
      else {
        for( int i = 0; i < 4; ++i ) {
          mv[i] = computeImageMode(src, bayer_masks[i])[0];
        }
      }
      break;

    case histogram_normalize_median:
      if( !is_bayer ) {
        mv = computeImageMedian(src, mask);
      }
      else {
        for( int i = 0; i < 4; ++i ) {
          mv[i] = computeImageMedian(src, bayer_masks[i])[0];
        }
      }
      break;

    default:
      CF_ERROR("Invalid norm_type=%d requested", norm_type);
      return false;
  }

  if( is_bayer ) {
    src.copyTo(dst);
    normalize_bayer_pattern(dst.getMatRef(), mv, stretch, offset);
  }
  else {

    switch (src.channels()) {
      case 1: {

        const cv::Matx12f m(stretch(0), offset(0) - mv(0) * stretch(0));

        cv::transform(src, dst, m);
        break;
      }
      case 2: {

        const cv::Matx23f m(
            stretch(0), 0, offset(0) - mv(0) * stretch(0),
            0, stretch(1), offset(1) - mv(1) * stretch(1));

        cv::transform(src, dst, m);
        break;
      }
      case 3: {

        const cv::Matx34f m(
            stretch(0), 0, 0, offset(0) - mv(0) * stretch(0),
            0, stretch(1), 0, offset(1) - mv(1) * stretch(1),
            0, 0, stretch(2), offset(2) - mv(2) * stretch(2));

        cv::transform(src, dst, m);
        break;
      }
      case 4: {
        typedef cv::Matx<float, 4, 5> Matx45f;

        Matx45f m = Matx45f::zeros();

        m(0, 0) = stretch(0), m(0, 4) = offset(0) - mv(0) * stretch(0);
        m(1, 1) = stretch(1), m(1, 4) = offset(1) - mv(1) * stretch(1);
        m(2, 2) = stretch(2), m(2, 4) = offset(2) - mv(2) * stretch(2);
        m(3, 3) = stretch(3), m(3, 4) = offset(3) - mv(3) * stretch(3);

        cv::transform(src, dst, m);
        break;
      }
    }
  }

  return true;
}
