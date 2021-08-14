/*
 * create_histogram.cc
 *
 *  Created on: Aug 14, 2021
 *      Author: amyznikov
 */
#include "create_histogram.h"
#include <core/debug.h>

template<class T, int cn>
static bool create_histogram__(cv::InputArray _src, cv::Mat1f & H,
    double minval, double maxval, int bn, cv::InputArray _mask)
{
  // FIXME: implement with TBB

  const cv::Mat_<cv::Vec<T, cn>> src =
      _src.getMat();

  const cv::Vec<float, cn> mv =
      cv::Vec<float, cn>::all(minval);

  const float scale =
      bn / (maxval - minval);

  H.create(bn, cn);
  H.setTo(0);

  if ( _mask.empty() ) {

    for ( int y = 0; y < src.rows; ++y ) {
      for ( int x = 0; x < src.cols; ++x ) {

        const cv::Vec<int, cn> bin = cv::Vec<int, cn>(
            (cv::Vec<float, cn>(src[y][x]) - mv) * scale);

        for ( int c = 0; c < cn; ++c ) {
          int b = bin[c];
          if ( b < 0 ) {
            b = 0;
          }
          else if ( b >= bn ) {
            b = bn - 1;
          }
          H[b][c] += 1;
        }
      }
    }
  }
  else {

    const cv::Mat1b mask = _mask.getMat();

    for ( int y = 0; y < src.rows; ++y ) {
      for ( int x = 0; x < src.cols; ++x ) {
        if ( mask[y][x] ) {

          const cv::Vec<int, cn> bin =
              cv::Vec<int, cn>((cv::Vec<float, cn>(src[y][x]) - mv) * scale);

          for ( int c = 0; c < cn; ++c ) {
            int b = bin[c];
            if ( b < 0 ) {
              b = 0;
            }
            else if ( b >= bn ) {
              b = bn - 1;
            }
            H[b][c] += 1;
          }
        }
      }
    }
  }

  return true;
}

static bool create_histogram_(cv::InputArray src, cv::Mat1f & dst,
    double minval, double maxval, int bn, cv::InputArray mask)
{
  switch ( src.type() ) {

  case CV_8UC1 : return create_histogram__<uint8_t, 1>(src, dst, minval, maxval, bn, mask);
  case CV_8UC2 : return create_histogram__<uint8_t, 2>(src, dst, minval, maxval, bn, mask);
  case CV_8UC3 : return create_histogram__<uint8_t, 3>(src, dst, minval, maxval, bn, mask);
  case CV_8UC4 : return create_histogram__<uint8_t, 4>(src, dst, minval, maxval, bn, mask);

  case CV_8SC1 : return create_histogram__<int8_t, 1>(src, dst, minval, maxval, bn, mask);
  case CV_8SC2 : return create_histogram__<int8_t, 2>(src, dst, minval, maxval, bn, mask);
  case CV_8SC3 : return create_histogram__<int8_t, 3>(src, dst, minval, maxval, bn, mask);
  case CV_8SC4 : return create_histogram__<int8_t, 4>(src, dst, minval, maxval, bn, mask);

  case CV_16UC1 : return create_histogram__<uint16_t, 1>(src, dst, minval, maxval, bn, mask);
  case CV_16UC2 : return create_histogram__<uint16_t, 2>(src, dst, minval, maxval, bn, mask);
  case CV_16UC3 : return create_histogram__<uint16_t, 3>(src, dst, minval, maxval, bn, mask);
  case CV_16UC4 : return create_histogram__<uint16_t, 4>(src, dst, minval, maxval, bn, mask);

  case CV_16SC1 : return create_histogram__<int16_t, 1>(src, dst, minval, maxval, bn, mask);
  case CV_16SC2 : return create_histogram__<int16_t, 2>(src, dst, minval, maxval, bn, mask);
  case CV_16SC3 : return create_histogram__<int16_t, 3>(src, dst, minval, maxval, bn, mask);
  case CV_16SC4 : return create_histogram__<int16_t, 4>(src, dst, minval, maxval, bn, mask);

  case CV_32SC1 : return create_histogram__<int32_t, 1>(src, dst, minval, maxval, bn, mask);
  case CV_32SC2 : return create_histogram__<int32_t, 2>(src, dst, minval, maxval, bn, mask);
  case CV_32SC3 : return create_histogram__<int32_t, 3>(src, dst, minval, maxval, bn, mask);
  case CV_32SC4 : return create_histogram__<int32_t, 4>(src, dst, minval, maxval, bn, mask);

  case CV_32FC1 : return create_histogram__<float, 1>(src, dst, minval, maxval, bn, mask);
  case CV_32FC2 : return create_histogram__<float, 2>(src, dst, minval, maxval, bn, mask);
  case CV_32FC3 : return create_histogram__<float, 3>(src, dst, minval, maxval, bn, mask);
  case CV_32FC4 : return create_histogram__<float, 4>(src, dst, minval, maxval, bn, mask);

  case CV_64FC1 : return create_histogram__<double, 1>(src, dst, minval, maxval, bn, mask);
  case CV_64FC2 : return create_histogram__<double, 2>(src, dst, minval, maxval, bn, mask);
  case CV_64FC3 : return create_histogram__<double, 3>(src, dst, minval, maxval, bn, mask);
  case CV_64FC4 : return create_histogram__<double, 4>(src, dst, minval, maxval, bn, mask);

  default :
    CF_FATAL("Invalid argument: Unsupported src image type : %d", src.type());
    break;
  }

  return false;
}



/// @brief  convert conventional image histogram H into cumulative
///         by accumulating the bin values along rows
bool accumulate_histogram(cv::InputArray H, cv::OutputArray cumulative)
{
  cv::Mat1f src;
  cv::Mat1f dst;

  double sums[H.cols()];

  if ( H.channels() != 1 ) {
    CF_FATAL("Invalid argument: Input histogram must be single-channel");
    return false;
  }

  if ( cumulative.fixedType() && cumulative.channels() != 1 ) {
    CF_FATAL("Invalid argument: output cumulative histogram must be single-channel");
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


bool create_histogram(cv::InputArray image, cv::InputArray mask,
    cv::OutputArray dst,
    double * minval,
    double * maxval,
    int nbins,
    bool cumulative,
    bool scaled)
{
  switch ( image.depth() ) {
  //
  case CV_8U :
    if ( *minval >= *maxval ) {
      *minval = 0;
      *maxval = UINT8_MAX;
    }
    if ( *minval < 0 ) {
      *minval = 0;
    }
    if ( *maxval > UINT8_MAX ) {
      *maxval = UINT8_MAX;
    }
    if ( nbins < 1 ) {
      nbins = *maxval - *minval + 1;
    }
    break;

  case CV_8S :
    if ( *minval >= *maxval ) {
      *minval = INT8_MIN;
      *maxval = INT8_MAX;
    }
    if ( *minval < INT8_MIN ) {
      *minval = INT8_MIN;
    }
    if ( *maxval > INT8_MAX ) {
      *maxval = INT8_MAX;
    }
    if ( nbins < 1 ) {
      nbins = *maxval - *minval + 1;
    }
    break;

    //
  case CV_16U :
    if ( *minval >= *maxval ) {
      *minval = 0;
      *maxval = UINT16_MAX;
    }
    if ( *minval < 0 ) {
      *minval = 0;
    }
    if ( *maxval > UINT16_MAX ) {
      *maxval = UINT16_MAX;
    }
    if ( nbins < 1 ) {
      nbins = *maxval - *minval + 1;
    }
    break;

  case CV_16S :
    if ( *minval >= *maxval ) {
      *minval = INT16_MIN;
      *maxval = INT16_MAX;
    }
    if ( *minval < INT16_MIN ) {
      *minval = INT16_MIN;
    }
    if ( *maxval > INT16_MAX ) {
      *maxval = INT16_MAX;
    }
    if ( nbins < 1 ) {
      nbins = *maxval - *minval + 1;
    }
    break;

    //
  case CV_32S :
    if ( *minval >= *maxval ) {
      *minval = INT32_MIN;
      *maxval = INT32_MAX;
    }
    if ( *minval < INT32_MIN ) {
      *minval = INT32_MIN;
    }
    if ( *maxval > INT32_MAX ) {
      *maxval = INT32_MAX;
    }
    if ( nbins < 1 ) {
      nbins = *maxval - *minval + 1;
    }
    break;

    //
  case CV_32F :
    if ( *minval >= *maxval ) {
      cv::minMaxLoc(image, minval, maxval);
    }
    if ( nbins < 1 ) {
      nbins = (int) std::min(65536., ((*maxval - *minval) / FLT_MIN) + 1);
    }
    break;

    //
  case CV_64F :
    if ( *minval >= *maxval ) {
      cv::minMaxLoc(image, minval, maxval);
    }
    if ( nbins < 1 ) {
      nbins = (int) std::min(65536., ((*maxval - *minval) / DBL_MIN) + 1);
    }
    break;

    //
  default :
    CF_FATAL("Invalid argument: Unsupported image depth %d encountered", image.depth());
    return false;
  }

  if ( nbins > 10000 ) {
    nbins = 10000;
  }

  cv::Mat1f H;

  if ( !create_histogram_(image, H, *minval, *maxval, nbins, mask) ) {
    CF_FATAL("create_histogram() fails");
    return false;
  }

  if ( scaled ) {
    cv::Mat1f sums;
    cv::reduce(H, sums, 0, cv::REDUCE_SUM, sums.depth());
    for ( int y = 0; y < H.rows; ++y ) {
      for ( int x = 0; x < H.cols; ++x ) {
        H[y][x] /= sums[0][x];
      }
    }
  }

  if ( cumulative ) {
    accumulate_histogram(H, H);
  }

  if ( dst.fixedType() && dst.depth() != H.depth() ) {
    H.convertTo(dst, dst.depth());
  }
  else {
    dst.move(H);
  }

  return true;
}


