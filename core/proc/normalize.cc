/*
 * normalize.cc
 *
 *  Created on: Jun 21, 2021
 *      Author: amyznikov
 */


#include "normalize.h"
#include <tbb/tbb.h>
#include <core/debug.h>


// BORDER_CONSTANT - fill with unmaskedPixelsValue
// BORDER_TRANSPARENT - scale as regular unmasked pixels
// BORDER_ISOLATED - do not touch

template<class T, int cn>
static bool _do_normalize(cv::Mat & _dst,
    double imin, double imax, double omin, double omax,
    const cv::Mat & mask,
    enum cv::BorderTypes maskBorderMode,
    const cv::Scalar & unmaskedPixelsValue)
{
  typedef tbb::blocked_range<int> range;
  const int grain_size = 512;


  const cv::Vec<double, cn> im = cv::Vec<double, cn>::all(imin);
  const cv::Vec<double, cn> om = cv::Vec<double, cn>::all(omin);
  const double S = (omax - omin) / (imax - imin);

  cv::Mat_<cv::Vec<T, cn>> & dst = (cv::Mat_<cv::Vec<T, cn>> &) _dst;

  if ( mask.empty() || maskBorderMode == cv::BORDER_TRANSPARENT || (mask.channels() == 1 && cv::countNonZero(mask) == mask.size().area()) ) {

    tbb::parallel_for(range(0, dst.rows, grain_size),
        [&dst, im, om, S ](const range & r ) {
          for ( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {
            cv::Vec<T, cn> * dstp = dst[y];
            for ( int x = 0, n = dst.cols; x < n; ++x ) {
              dstp[x] = om + (cv::Vec<double, cn>(dstp[x]) - im) * S;
            }
          }
        });

  }
  else if ( mask.channels() == 1 ) {

    const cv::Mat1b & msk = (const cv::Mat1b &)mask;

    tbb::parallel_for(range(0, dst.rows, grain_size),
        [&dst, &msk, im, om, S, maskBorderMode, unmaskedPixelsValue ](const range & r ) {
          for ( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {

            cv::Vec<T, cn> * dstp = dst[y];
            const uint8_t * mskp = msk[y];

            for ( int x = 0, n = dst.cols; x < n; ++x ) {
              if ( mskp[x] ) {
                dstp[x] = om + (cv::Vec<double, cn>(dstp[x]) - im) * S;
              }
              else if ( maskBorderMode == cv::BORDER_CONSTANT ) {
                for ( int c = 0; c < cn; ++c ) {
                  dstp[x][c] = cv::saturate_cast<T>(unmaskedPixelsValue[c]);
                }
              }
            }
          }
        });

  }
  else if ( mask.channels() == cn ) {

    const cv::Mat_<cv::Vec<uint8_t, cn>> & msk = (const cv::Mat_<cv::Vec<uint8_t, cn>> & )mask;

    tbb::parallel_for(range(0, dst.rows, grain_size),
        [&dst, &msk, imin, omin, S, maskBorderMode, unmaskedPixelsValue ](const range & r ) {
          for ( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {
            cv::Vec<T, cn> * dstp = dst[y];
            const cv::Vec<uint8_t, cn> * mskp = msk[y];
            for ( int x = 0, n = dst.cols; x < n; ++x ) {
              for ( int c = 0; c < cn; ++c ) {
                if ( mskp[x][c] ) {
                  dstp[x][c] = cv::saturate_cast<T>(omin + (dstp[x][c] - imin) * S);
                }
                else if ( maskBorderMode == cv::BORDER_CONSTANT ) {
                  dstp[x][c] = cv::saturate_cast<T>(unmaskedPixelsValue[c]);
                }
              }
            }
          }
        });

  }
  else {
    CF_FATAL("Invalid combination of input image and mask channels");
    return false;
  }

  return true;
}

template<class T>
static bool _do_normalize_mc(cv::Mat & dst,
    double imin, double imax, double omin, double omax,
    const cv::Mat & mask,
    enum cv::BorderTypes maskBorderMode,
    const cv::Scalar & unmaskedPixelsValue)
{
  typedef tbb::blocked_range<int> range;
  const int grain_size = 512;

  const double S = (omax - omin) / (imax - imin);
  const int cn = dst.channels();

  if ( mask.empty() || maskBorderMode == cv::BORDER_TRANSPARENT || (mask.channels() == 1 && cv::countNonZero(mask) == mask.size().area()) ) {
    tbb::parallel_for(range(0, dst.rows, grain_size),
        [&dst, S, imin, omin, omax, cn](const range & r ) {
          for ( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {
            T * dstp = dst.ptr<T>(y);
            for ( int x = 0, n = dst.cols * cn; x < n; ++x ) {
              dstp[x] = omin + (dstp[x] - imin) * S;
              if ( dstp[x] < omin ) {
                dstp[x] = omin;
              }
              else if ( dstp[x] > omax ) {
                dstp[x] = omax;
              }
            }
          }
        });
  }
  else if ( mask.channels() == 1 ) {

    const cv::Mat1b & msk = (const cv::Mat1b &)mask;

    tbb::parallel_for(range(0, dst.rows, grain_size),
        [&dst, &msk, S, imin, omin, omax, cn, maskBorderMode, unmaskedPixelsValue](const range & r ) {
          for ( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {
            T * dstp = dst.ptr<T>(y);
            const uint8_t * mskp = msk[y];
            for ( int x = 0; x < dst.cols; ++x ) {

              if( mskp[x] ) {
                for ( int c = 0; c < cn; ++c ) {
                  dstp[x * cn + c] = omin + (dstp[x * cn + c] - imin) * S;
                  if ( dstp[x * cn + c] < omin ) {
                    dstp[x * cn + c] = omin;
                  }
                  else if ( dstp[x * cn + c] > omax ) {
                    dstp[x * cn + c] = omax;
                  }
                }
              }
              else if ( maskBorderMode == cv::BORDER_CONSTANT ) {
                for ( int c = 0; c < cn; ++c ) {
                  dstp[x * cn + c] = cv::saturate_cast<T>(unmaskedPixelsValue[c]);
                }
              }

            }
          }
        });
  }
  else if ( mask.channels() == cn ) {

    tbb::parallel_for(range(0, dst.rows, grain_size),
        [&dst, &mask, S, imin, omin, omax, cn, maskBorderMode, unmaskedPixelsValue](const range & r ) {
          for ( int y = r.begin(), ymax = r.end(); y < ymax; ++y ) {
            const uint8_t * mskp = mask.ptr<const uint8_t>(y);
            T * dstp = dst.ptr<T>(y);
            for ( int x = 0; x < dst.cols; ++x ) {
              for ( int c = 0; c < cn; ++c ) {
                if( mskp[x * cn + c] ) {
                  dstp[x * cn + c] = omin + (dstp[x * cn + c] - imin) * S;
                  if ( dstp[x * cn + c] < omin ) {
                    dstp[x * cn + c] = omin;
                  }
                  else if ( dstp[x * cn + c] > omax ) {
                    dstp[x * cn + c] = omax;
                  }
                }
                else if ( maskBorderMode == cv::BORDER_CONSTANT ) {
                  for ( int c = 0; c < cn; ++c ) {
                    dstp[x * cn + c] = cv::saturate_cast<T>(unmaskedPixelsValue[c]);
                  }
                }
              }
            }
          }
        });
  }
  else {
    CF_FATAL("Invalid combination of input image and mask channels");
    return false;
  }

  return true;
}


bool normalize_image(cv::Mat & image,
    double imin, double imax, double omin, double omax,
    const cv::Mat & mask,
    enum cv::BorderTypes maskBorderMode,
    const cv::Scalar & unmaskedPixelsValue)
{
  if ( image.channels() > 4 ) {
    switch ( image.depth() ) {
    case CV_8U  : return _do_normalize_mc<uint8_t>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    case CV_8S  : return _do_normalize_mc<int8_t>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    case CV_16U : return _do_normalize_mc<uint16_t>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    case CV_16S : return _do_normalize_mc<int8_t>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    case CV_32S : return _do_normalize_mc<int32_t>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    case CV_32F : return _do_normalize_mc<float>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    case CV_64F : return _do_normalize_mc<double>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    }
  }
  else {

    switch ( image.type() ) {
    case CV_8UC1 : return _do_normalize<uint8_t, 1>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    case CV_8UC2 : return _do_normalize<uint8_t, 2>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    case CV_8UC3 : return _do_normalize<uint8_t, 3>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    case CV_8UC4 : return _do_normalize<uint8_t, 4>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);

    case CV_8SC1 : return _do_normalize<int8_t, 1>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    case CV_8SC2 : return _do_normalize<int8_t, 2>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    case CV_8SC3 : return _do_normalize<int8_t, 3>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    case CV_8SC4 : return _do_normalize<int8_t, 4>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);

    case CV_16UC1 : return _do_normalize<uint16_t, 1>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    case CV_16UC2 : return _do_normalize<uint16_t, 2>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    case CV_16UC3 : return _do_normalize<uint16_t, 3>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    case CV_16UC4 : return _do_normalize<uint16_t, 4>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);

    case CV_16SC1 : return _do_normalize<int16_t, 1>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    case CV_16SC2 : return _do_normalize<int16_t, 2>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    case CV_16SC3 : return _do_normalize<int16_t, 3>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    case CV_16SC4 : return _do_normalize<int16_t, 4>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);

    case CV_32SC1 : return _do_normalize<int32_t, 1>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    case CV_32SC2 : return _do_normalize<int32_t, 2>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    case CV_32SC3 : return _do_normalize<int32_t, 3>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    case CV_32SC4 : return _do_normalize<int32_t, 4>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);

    case CV_32FC1 : return _do_normalize<float, 1>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    case CV_32FC2 : return _do_normalize<float, 2>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    case CV_32FC3 : return _do_normalize<float, 3>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    case CV_32FC4 : return _do_normalize<float, 4>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);

    case CV_64FC1 : return _do_normalize<double, 1>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    case CV_64FC2 : return _do_normalize<double, 2>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    case CV_64FC3 : return _do_normalize<double, 3>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    case CV_64FC4 : return _do_normalize<double, 4>(image, imin, imax, omin, omax, mask, maskBorderMode, unmaskedPixelsValue);
    }
  }

  CF_FATAL("Invalid arg: Unsupported image type: %d", image.type());
  return false;
}


bool normalize_image(cv::InputArray src, cv::OutputArray dst,
    double imin, double imax,
    double omin, double omax,
    cv::InputArray mask,
    enum cv::BorderTypes maskBorderMode,
    const cv::Scalar & unmaskedPixelsValue)
{
  if ( !dst.fixedType() ) {
    src.copyTo(dst);
  }
  else {
    src.getMat().convertTo(dst, dst.depth());
  }

  const cv::Mat msk = mask.getMat();

  return normalize_image(dst.getMatRef(),
      imin, imax,
      omin, omax,
      msk,
      maskBorderMode,
      unmaskedPixelsValue);

}


bool normalize_minmax(cv::InputArray src, cv::OutputArray dst, double omin, double omax,
    cv::InputArray mask,
    enum cv::BorderTypes maskBorderMode,
    const cv::Scalar & unmaskedPixelsValue)
{
  std::vector<cv::Mat> channels;
  std::vector<cv::Mat> cmasks;

  const int cn = src.channels();


  if ( cn == 1 ) {
    channels.emplace_back(src.getMat());
    cmasks.emplace_back(mask.getMat());
  }
  else {
    cv::split(src, channels);

    if ( mask.channels() == cn ) {
      cv::split(mask, cmasks);
    }
    else {
      cmasks.emplace_back(mask.getMat());
    }
  }

  double imin = DBL_MAX, imax = -DBL_MAX;
  for ( int i = 0; i < cn; ++i ) {

    double cmin = 0, cmax = 1;
    cv::minMaxLoc(channels[i], &cmin, &cmax);
    if ( cmin < imin ) {
      imin = cmin;
    }
    if ( cmax > imax ) {
      imax = cmax;
    }
  }

  channels.clear(), channels.shrink_to_fit();
  cmasks.clear(), cmasks.shrink_to_fit();

  return normalize_image(src, dst,
      imin, imax,
      omin, omax,
      mask,
      maskBorderMode,
      unmaskedPixelsValue);
}
