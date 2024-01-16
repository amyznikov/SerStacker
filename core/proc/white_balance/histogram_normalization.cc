/*
 * histogram_normalization.cc
 *
 *  Created on: Jun 4, 2023
 *      Author: amyznikov
 */

#include "histogram_normalization.h"
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

static cv::Scalar compute_mode(const cv::Mat & image, cv::InputArray _mask = cv::noArray())
{
  cv::Mat1f H;
  double minval = -1, maxval = -1;
  int nbins = -1;

  bool fOK =
      create_histogram(image,
          _mask,
          H,
          &minval,
          &maxval,
          nbins,
          false,
          false);

  if( !fOK ) {
    CF_ERROR("create_histogram() fails");
    return cv::Scalar();
  }

  cv::Scalar s;

  for( int i = 0, n = (std::min)(4, H.cols); i < n; ++i ) {

    if( H.rows < 2 ) {
      s[i] = minval;
    }
    else {

      int jmax = 0;

      float hmax = H[jmax][i];

      for( int j = 1, m = H.rows; j < m; ++j ) {
        if( H[j][i] > hmax ) {
          jmax = j;
          hmax = H[j][i];
        }
      }

      s[i] = minval + jmax * (maxval - minval) / (H.rows - 1);
    }
  }

  return s;
}

static cv::Scalar compute_median(const cv::Mat & image, cv::InputArray _mask = cv::noArray())
{
  cv::Mat1f H;
  double minval = -1, maxval = -1;
  int nbins = -1;

  bool fOK =
      create_histogram(image,
          _mask,
          H,
          &minval,
          &maxval,
          nbins,
          true,
          true);

  if( !fOK ) {
    CF_ERROR("create_histogram() fails");
    return cv::Scalar();
  }

  cv::Scalar s;

  for( int i = 0, n = (std::min)(4, H.cols); i < n; ++i ) {

    if( H.rows < 2 ) {
      s[i] = minval;
    }
    else {
      for( int j = 0, m = H.rows; j < m; ++j ) {
        if( H[j][i] >= 0.5 ) {
          s[i] = minval + j * (maxval - minval) / (m - 1);
          break;
        }
      }
    }
  }

  return s;
}



// dst = (src - mv) * stretch + offset
// => dst = src * stretch + offset - mv * stretch

bool nomalize_image_histogramm(cv::InputArray _src, cv::InputArray mask, cv::OutputArray dst,
    histogram_normalization_type norm_type,
    const cv::Scalar & stretch,
    const cv::Scalar & offset,
    enum COLORID src_colorid)
{

  const cv::Mat src =
      _src.getMat();

  const bool is_bayer =
      is_bayer_pattern(src_colorid);

  if( is_bayer && src.channels() != 1 ) {
    CF_ERROR("Invalid number of image channels %d for bayer pattern %s. Must be 1",
        src.channels(), toString(src_colorid));
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
        mv = compute_mode(src, mask);
      }
      else {
        for( int i = 0; i < 4; ++i ) {
          mv[i] = compute_mode(src, bayer_masks[i])[0];
        }
      }
      break;

    case histogram_normalize_median:
      if( !is_bayer ) {
        mv = compute_median(src, mask);
      }
      else {
        for( int i = 0; i < 4; ++i ) {
          mv[i] = compute_median(src, bayer_masks[i])[0];
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
