/*
 * pixinsight-mtf.cc
 *
 *  Created on: Dec 14, 2020
 *      Author: amyznikov
 *
 * Midtones Transfer Function (MTF) as it defined in PixInsight
 *
 *   See section 'Midtones balance' at
 *     <http://pixinsight.com/doc/tools/HistogramTransformation/HistogramTransformation.html>
 *
 * The formal definition of an MTF function with midtones balance parameter 'm' is given by
 *
 *  MTF(x,m) =
 *           0.0   for x == 0
 *           0.5   for x == m
 *           1.0   for x == 1
 *           (m - 1) * x / ( (2 * m - 1) * x - m) otherwise
 */

#include "pixinsight-mtf.h"
#include <tbb/tbb.h>
#include <core/debug.h>

/** @brief Pixinsight MTF for nominal range 0 <= x <= 1, 0 <= m <= 1 */
double mtf_pixinsight(double x, double m)
{
  if ( x <= 0 ) {
    return 0;
  }

  if ( x >= 1 ) {
    return 1;
  }

  if ( x == m ) {
    return 0.5;
  }

  return (m - 1) * x / ((2 * m - 1) * x - m);
}

/** @brief Return default pixel values range for given image depth
 */
static bool suggest_levels_range(int depth, double * minval, double * maxval)
{
  switch ( depth ) {
  case CV_8U :
    *minval = 0;
    *maxval = UINT8_MAX;
    break;
  case CV_8S :
    *minval = INT8_MIN;
    *maxval = INT8_MAX;
    break;
  case CV_16U :
    *minval = 0;
    *maxval = UINT16_MAX;
    break;
  case CV_16S :
    *minval = INT16_MIN;
    *maxval = INT16_MAX;
    break;
  case CV_32S :
    *minval = INT32_MIN;
    *maxval = INT32_MAX;
    break;
  case CV_32F :
    *minval = 0;
    *maxval = 1;
    break;
  case CV_64F :
    *minval = 0;
    *maxval = 1;
    break;
  default:
    *minval = 0;
    *maxval = 1;
    return false;
  }

  return true;
}

/** @brief Return default pixel values range for given image depth
 */
static bool suggest_levels_range(cv::InputArray src, double * minval, double * maxval)
{
  switch (src.depth()) {
    case CV_32F:
    case CV_64F:
      cv::minMaxLoc(src, minval, maxval);
      break;
    default:
      return suggest_levels_range(src.depth(), minval, maxval);
  }

  return true;
}

template<class T1, class T2>
static bool apply_mtf__(const cv::Mat & src, cv::Mat & dst,
    double src_min, double src_max,
    double dst_min, double dst_max,
    double shadows, double highlights, double midtones)
{

  using tbb_range = tbb::blocked_range<int>;

  const int ny = src.rows;
  const int nx = src.cols * src.channels();

  const double imin = src_min + shadows * (src_max - src_min);
  const double imax = src_min + highlights * (src_max - src_min);
  const double si = 1. / (imax - imin);
  const double so = (dst_max - dst_min);

  tbb::parallel_for(tbb_range(0, ny, 256),
      [&src, &dst, nx, imin, imax, dst_min, si, so, midtones]

      (const tbb_range & r) {

        for ( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {

          const T1 * srcp = src.ptr<const T1>(y);
          T2 * dstp = dst.ptr<T2>(y);

          for ( int x = 0; x < nx; ++x ) {

            dstp[x] = cv::saturate_cast<T2>(dst_min +
                so * mtf_pixinsight((srcp[x] - imin) * si, midtones));
          }
        }
      });

  return true;
}

template<class T>
static bool apply_mtf_(const cv::Mat & src, cv::Mat & dst,
    double src_min, double src_max,
    double dst_min, double dst_max,
    double shadows, double highlights, double midtones)
{
  switch ( dst.depth() ) {
  case CV_8U :
    return apply_mtf__<T, uint8_t>(src, dst, src_min, src_max, dst_min, dst_max, shadows, highlights, midtones);
  case CV_8S :
    return apply_mtf__<T, int8_t>(src, dst, src_min, src_max, dst_min, dst_max, shadows, highlights, midtones);
  case CV_16U :
    return apply_mtf__<T, uint16_t>(src, dst, src_min, src_max, dst_min, dst_max, shadows, highlights, midtones);
  case CV_16S :
    return apply_mtf__<T, int16_t>(src, dst, src_min, src_max, dst_min, dst_max, shadows, highlights, midtones);
  case CV_32S :
    return apply_mtf__<T, int32_t>(src, dst, src_min, src_max, dst_min, dst_max, shadows, highlights, midtones);
  case CV_32F :
    return apply_mtf__<T, float>(src, dst, src_min, src_max, dst_min, dst_max, shadows, highlights, midtones);
  case CV_64F :
    return apply_mtf__<T, double>(src, dst, src_min, src_max, dst_min, dst_max, shadows, highlights, midtones);
  }

  return false;
}

/** @brief Apply Pixinsight Midtones Transfer Function to image */
bool apply_mtf_pixinsight(cv::InputArray src_image,
    cv::OutputArray dst_image,
    double src_min, double src_max,
    double dst_min, double dst_max,
    double shadows,
    double highlights,
    double midtones,
    int ddepth)
{
  if ( !dst_image.needed() ) {
    return false;
  }

  if ( dst_image.fixedType() ) {
    ddepth = dst_image.depth();
  }
  else if ( ddepth < CV_8U || ddepth > CV_64F ) {
    ddepth = src_image.depth();
  }


  cv::Mat src, dst;

  src = src_image.getMat();

  if ( src.data != dst_image.getMat().data ) {
    // create external storage for output
    dst_image.create(src.size(), CV_MAKETYPE(ddepth, src.channels()));
    dst = dst_image.getMatRef();
  }
  else if ( src.depth() == ddepth ) {
    // process in place
    dst = src;
  }
  else {
    // create internal storage for output
    dst.create(src.size(), CV_MAKETYPE(ddepth, src.channels()));
  }


  if ( src_min >= src_max ) {
    suggest_levels_range(src,
        &src_min, &src_max);
  }

  if ( src.channels() == 2  ) {
    // assume this is optical flow, keep the input range for correct visualization
    cv::minMaxLoc(src, &dst_min, &dst_max);
  }
  else if ( dst_min >= dst_max ) {
    suggest_levels_range(ddepth,
        &dst_min, &dst_max);
  }

  switch ( src.depth() ) {
  case CV_8U :
    apply_mtf_<uint8_t>(src, dst, src_min, src_max, dst_min, dst_max, shadows, highlights, midtones);
    break;
  case CV_8S :
    apply_mtf_<int8_t>(src, dst, src_min, src_max, dst_min, dst_max, shadows, highlights, midtones);
    break;
  case CV_16U :
    apply_mtf_<uint16_t>(src, dst, src_min, src_max, dst_min, dst_max, shadows, highlights, midtones);
    break;
  case CV_16S :
    apply_mtf_<int16_t>(src, dst, src_min, src_max, dst_min, dst_max, shadows, highlights, midtones);
    break;
  case CV_32S :
    apply_mtf_<int32_t>(src, dst, src_min, src_max, dst_min, dst_max, shadows, highlights, midtones);
    break;
  case CV_32F :
    apply_mtf_<float>(src, dst, src_min, src_max, dst_min, dst_max, shadows, highlights, midtones);
    break;
  case CV_64F :
    apply_mtf_<double>(src, dst, src_min, src_max, dst_min, dst_max, shadows, highlights, midtones);
    break;
  default:
    break;
  }

  if ( dst.data != dst_image.getMatRef().data ) {
    dst_image.move(dst);
  }

  return true;
}



/** @brief Convert conventional image histogram H into cumulative by accumulating the bin values along rows.
 *  In conventional image histogram H the bins are places in rows, the channels in columns
 * */
static bool accumulate_histogram(cv::InputArray H, cv::OutputArray cumulative)
{
  cv::Mat1f src;
  cv::Mat1f dst;

  double sums[H.cols()];

  if ( H.channels() != 1 ) {
    CF_FATAL("Invalid arg: Input histogram must be single-channel matrix");
    return false;
  }

  if ( cumulative.fixedType() && cumulative.channels() != 1 ) {
    CF_FATAL("Invalid arg: output cumulative histogram must be single-channel matrix");
    return false;
  }

  if ( H.depth() == src.depth() ) {
    src = H.getMat();
  }
  else {
    H.getMat().convertTo(src, src.depth());
  }

  memset(sums, 0, sizeof(sums));

  dst.create(src.size());

  for ( int y = 0; y < src.rows; ++y ) {
    for ( int x = 0; x < src.cols; ++x ) {
      dst[y][x] = (sums[x] += src[y][x]);
    }
  }

  if ( !cumulative.fixedType() || cumulative.depth() == dst.depth() ) {
    cumulative.move(dst);
  }
  else {
    dst.convertTo(cumulative, cumulative.depth());
  }

  return true;
}

/** @brief Convert input image histogram H into scaled cumulative and search for 0.5-percentile bin
 * */
static cv::Scalar estimate_median_from_histogram(cv::InputArray H)
{
  cv::Mat1f cumulative;
  cv::Scalar median;

  if ( accumulate_histogram(H, cumulative) ) {

    for ( int c = 0, cn = std::min(cumulative.cols, (int) median.channels); c < cn; ++c ) {
      const float mv = 0.5f * cumulative[cumulative.rows - 1][c];
      if ( mv > 0 ) {

        // use 'lower bound' to localize the bin

        int __first = 0, __len = cumulative.rows;
        while ( __len > 0 ) {
          int __half = __len / 2, __middle = __first + __half;
          if ( cumulative[__middle][c] >= mv ) {
            __len = __half;
          }
          else {
            __first = __middle + 1;
            __len = __len - __half - 1;
          }
        }

        int start_bin = __first;
        int end_bin = __first;
        while ( end_bin + 1 < cumulative.rows && cumulative[end_bin + 1][c] <= mv ) {
          ++end_bin;
        }

        median[c] = 0.5 * (start_bin + end_bin);
      }
    }
  }

  return median;
}

/** @brief compute median and median of the absolute deviations from the data's median.
 *
 *  For a univariate data set X1, X2, ..., Xn :
 *    MAD = median (| Xi − median(X) |)
 **/
static bool estimate_mad_from_histogram(cv::InputArray _H,
    cv::Scalar * output_median,
    cv::Scalar * output_mad,
    cv::OutputArray output_histogram_of_abs_devs = cv::noArray() )
{
  cv::Mat1f H;
  cv::Mat1f histogram_of_abs_devs;

  *output_median = estimate_median_from_histogram(_H);

  if ( _H.depth() == H.depth() ) {
    H = _H.getMat();
  }
  else {
    _H.getMat().convertTo(H, H.depth());
  }

  histogram_of_abs_devs.create(H.size());
  histogram_of_abs_devs.setTo(0);

  for ( int r = 0; r < H.rows; ++r ) {
    for ( int c = 0; c < H.cols; ++c ) {
      const int abs_dev = std::min((int) fabs(r - (*output_median)[c]), H.rows - 1);
      histogram_of_abs_devs[abs_dev][c] += H[r][c];
    }
  }

  *output_mad = estimate_median_from_histogram(histogram_of_abs_devs);

  if ( output_histogram_of_abs_devs.needed() ) {
    if ( output_histogram_of_abs_devs.fixedType() && output_histogram_of_abs_devs.depth() != histogram_of_abs_devs.depth()) {
      histogram_of_abs_devs.convertTo(output_histogram_of_abs_devs, output_histogram_of_abs_devs.depth());
    }
    else {
      output_histogram_of_abs_devs.move(histogram_of_abs_devs);
    }
  }

  return true;
}


/** @brief Auto adjust midtones balance for given image histogram.
 *  Based on findMidtonesBalance() from siril source code */
bool find_midtones_balance_pixinsight(cv::InputArray input_image_histogram,
    double * output_shadows,
    double * output_highlights,
    double * output_midtones)
{
  static constexpr double MAD_NORM = 1.4826;
  static constexpr double shadowsClipping = -2.80; /* Shadows clipping point measured in sigma units from the main histogram peak. */
  static constexpr double targetBackground = 0.25; /* final "luminance" of the image for autostretch in the [0,1] range */

  cv::Mat1f H;
  cv::Scalar median, mad;

  double m = 0.0, c0 = 0.0, c1 = 0.0;
  int invertedChannels = 0;

  if ( input_image_histogram.depth() == H.depth() ) {
    H = input_image_histogram.getMat();
  }
  else {
    input_image_histogram.getMat().convertTo(H, H.depth());
  }


  estimate_mad_from_histogram(H, &median, &mad);
  median /= H.rows;
  cv::max(mad * MAD_NORM / H.rows, cv::Scalar::all(0.001), mad); // avoid breakdown point


  const int cn = std::min(H.cols, (int) cv::Scalar::channels);

  for ( int c = 0; c < cn; ++c ) {
    if ( median[c] > 0.5 * H.rows ) {
      ++invertedChannels;
    }
  }

  if ( invertedChannels < cn ) {

    for ( int c = 0; c < cn; ++c ) {
      m += median[c];
      c0 += median[c] + shadowsClipping * mad[c];
    }

    if ( (c0 /= cn) < 0.0 ) {
      c0 = 0.0;
    }

    *output_shadows = c0;
    *output_highlights = 1.0;
    *output_midtones = mtf_pixinsight(m / cn - c0,
        targetBackground);
  }

  else {

    for ( int c = 0; c < cn; ++c ) {
      m += median[c];
      c1 += median[c] - shadowsClipping * mad[c];
    }

    if ( (c1 /= cn) > 1.0 ) {
      c1 = 1.0;
    }

    *output_shadows = 0.0;
    *output_highlights = c1;
    *output_midtones = 1.0 - mtf_pixinsight(c1 - m / cn,
        targetBackground);
  }

  return true;
}
