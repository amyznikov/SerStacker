/*
 * inpaint.cc
 *
 *  Created on: Aug 28, 2021
 *      Author: amyznikov
 */

#include "inpaint.h"
#include <core/proc/downstrike.h>
#if HAVE_TBB && !defined(Q_MOC_RUN)
#include <tbb/tbb.h>
#endif
#include <core/debug.h>

#define downstrike  downstrike_even
#define upject      upject_even


template<class T>
static void average_pyramid_filter_(const cv::Mat & src, const cv::Mat1f & srcmask,
    cv::Mat & dst, cv::Mat1f & dstmask)
{
  constexpr enum cv::BorderTypes border_mode =
      cv::BORDER_REPLICATE;

  static thread_local const cv::Mat1f K =
      cv::Mat1f::ones(3, 1);

  cv::sepFilter2D(src, dst, src.depth(), K, K, cv::Point(1, 1),
      0, border_mode);

  cv::sepFilter2D(srcmask, dstmask, dstmask.depth(), K, K, cv::Point(1, 1),
      0, border_mode);


  // Don't use cv::divide() because result of division by zero differs between OpenCV versions
  // Manualy divide and handle zeros instead

#if HAVE_TBB && !defined(Q_MOC_RUN)

  typedef tbb::blocked_range<int> range;
  tbb::parallel_for(range(0, dst.rows, 256),
      [&dst, &dstmask](const range & r) {

        const int cn = dst.channels();

        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {

          T * dstp = dst.ptr<T>(y);
          float * mskp = dstmask.ptr<float>(y);

          for ( int x = 0; x < dst.cols; ++x, ++mskp, dstp += cn ) {
            if ( *mskp ) {
              for ( int c = 0; c < cn; ++c ) {
                dstp[c] /= *mskp;
              }
              *mskp = 1;
            }
          }

        }
      });

#else
  const int cn = dst.channels();

  for ( int y = 0; y < dst.rows; ++y ) {

    T * dstp = dst.ptr<T>(y);
    float * mskp = dstmask.ptr<float>(y);

    for ( int x = 0; x < dst.cols; ++x, ++mskp, dstp += cn ) {
      if ( *mskp ) {
        for ( int c = 0; c < cn; ++c ) {
          dstp[c] /= *mskp;
        }
        *mskp = 1;
      }
    }
  }

#endif
}

static void average_pyramid_filter(const cv::Mat & src, const cv::Mat1f & srcmask,
    cv::Mat & dst, cv::Mat1f & dstmask)
{
  switch ( src.depth() ) {
  case CV_8U :
    average_pyramid_filter_<uint8_t>(src, srcmask, dst, dstmask);
    break;
  case CV_8S :
    average_pyramid_filter_<int8_t>(src, srcmask, dst, dstmask);
    break;
  case CV_16U :
    average_pyramid_filter_<uint16_t>(src, srcmask, dst, dstmask);
    break;
  case CV_16S :
    average_pyramid_filter_<int16_t>(src, srcmask, dst, dstmask);
    break;
  case CV_32S :
    average_pyramid_filter_<int32_t>(src, srcmask, dst, dstmask);
    break;
  case CV_32F :
    average_pyramid_filter_<float>(src, srcmask, dst, dstmask);
    break;
  case CV_64F :
    average_pyramid_filter_<double>(src, srcmask, dst, dstmask);
    break;
  }
}

static void average_pyramid_pyrdown(const cv::Mat & src, const cv::Mat1f & srcmask,
    cv::Mat & dst, cv::Mat1f & dstmask)
{
  average_pyramid_filter(src, srcmask, dst, dstmask);
  downstrike(dst, dst);
  downstrike(dstmask, dstmask);
}

static inline void average_pyramid_pyrup(cv::Mat & image, cv::Size dst_size)
{
//  cv::pyrUp(image, image, dst_size);
  cv::Mat1f zmask;
  upject(image, image, dst_size, &zmask, CV_32F );
  average_pyramid_filter(image, zmask, image, zmask);
}


//static void average_pyramid_recurse(cv::Mat & image, cv::Mat1f & mask)
//{
//  const cv::Mat zmask = mask == 0;
//  if ( cv::countNonZero(zmask) && std::min(image.cols, image.rows) > 1 ) {
//
//    cv::Mat filtered_image;
//    cv::Mat1f filtered_mask;
//
//    average_pyramid_pyrdown(image, mask, filtered_image, filtered_mask);
//    average_pyramid_recurse(filtered_image, filtered_mask);
//    average_pyramid_pyrup(filtered_image, image.size());
//
//    filtered_image.copyTo(image, mask == 0);
//  }
//}

static void average_pyramid_recurse(cv::Mat & image, cv::Mat1f & mask)
{
  if ( std::min(image.cols, image.rows) > 1 ) {

    cv::Mat filtered_image;
    cv::Mat1f filtered_mask;

    average_pyramid_pyrdown(image, mask, filtered_image, filtered_mask);

    if ( cv::countNonZero(filtered_mask) < filtered_mask.size().area() ) {
      average_pyramid_recurse(filtered_image, filtered_mask);
    }

    average_pyramid_pyrup(filtered_image, image.size());
    filtered_image.copyTo(image, mask == 0);
  }
}

void average_pyramid_inpaint(cv::InputArray _src, cv::InputArray _mask,
    cv::OutputArray dst)
{
  const cv::Mat1b mask = _mask.getMat();

  if ( mask.empty() || countNonZero(mask) == mask.size().area() ) {
    if ( !dst.fixedType() || dst.type() == _src.type() ) {
      _src.copyTo(dst);
    }
    else {
      _src.getMat().convertTo(dst, dst.type());
    }
  }
  else {
    cv::Mat src;
    cv::Mat1f msk;

    _src.copyTo(src, mask);
    mask.convertTo(msk, CV_32F, 1. / 255);
    average_pyramid_recurse(src, msk);

    src.convertTo(dst, dst.fixedType() ? dst.type() : _src.type());
  }
}
