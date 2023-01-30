/*
 * linear_interpolation_inpaint.cc
 *
 *  Created on: Jan 29, 2023
 *      Author: amyznikov
 */

#include "linear_interpolation_inpaint.h"

#if HAVE_TBB && !defined(Q_MOC_RUN)
# include <tbb/tbb.h>
#endif

#include <core/debug.h>

template<class T>
static void interpolate_holes_h_(cv::Mat & image, const cv::Mat1b & mask, cv::Mat1f & dists)
{
  dists.create(image.size());
  dists.setTo(0);

  cv::Mat_<T> src =
      image;

  const int cn =
      image.channels();

  const int rows =
      mask.rows;

  const int cols =
      mask.cols;

#if HAVE_TBB && !defined(Q_MOC_RUN)

  typedef tbb::blocked_range<int> range;

  tbb::parallel_for(range(0, rows, 256),
      [&](const range & r) {
  for( int y = r.begin(); y < r.end(); ++y ) {
#else
  for( int y = 0; y < rows; ++y ) {
#endif

    T *srcp = src[y];
    const uint8_t *mskp = mask[y];

    float *distp = dists[y];

    for( int start = 0;; ) {

      while (start < cols && mskp[start]) {
        ++start;
      }

      int end = start + 1;
      while (end < cols && !mskp[end]) {
        ++end;
      }

      if( start > 0 && end < cols ) {

        float sv[cn], ev[cn];
        float kk[cn];

        const int s = start - 1;
        const int e = end;

        for( int c = 0; c < cn; ++c ) {
          sv[c] = srcp[s * cn + c];
          ev[c] = srcp[e * cn + c];
          kk[c] = (ev[c] - sv[c]) / (end - start);
        }

        for( int x = s + 1; x < e; ++x ) {
          distp[x] = std::max(x - s, e - x);
          for( int c = 0; c < cn; ++c ) {
            srcp[x * cn + c] = cv::saturate_cast<T>(
                sv[c] + (x - s) * kk[c]);
          }
        }
      }
      else if( start > 0 ) {

        const int s = start - 1;
        const int e = cols;

        for( int x = s + 1; x < e; ++x ) {
          distp[x] = x - s;
          for( int c = 0; c < cn; ++c ) {
            srcp[x * cn + c] = srcp[s * cn + c];
          }
        }
      }
      else if( end < cols ) {

        const int s = -1;
        const int e = end;

        for( int x = s + 1; x < e; ++x ) {
          distp[x] = e - x;
          for( int c = 0; c < cn; ++c ) {
            srcp[x * cn + c] = srcp[e * cn + c];
          }
        }
      }

      if( (start = end + 1) >= cols ) {
        break;
      }
    }
  }
#if HAVE_TBB && !defined(Q_MOC_RUN)
    });
#endif
}


template<class T>
static void interpolate_holes_v_(cv::Mat & image, const cv::Mat1b & mask, cv::Mat1f & dists)
{
  dists.create(image.size());
  dists.setTo(0);

  cv::Mat_<T> src =
      image;

  const int cn =
      image.channels();

  const int rows =
      mask.rows;

  const int cols =
      mask.cols;


#if HAVE_TBB && !defined(Q_MOC_RUN)

  typedef tbb::blocked_range<int> range;

  tbb::parallel_for(range(0, cols, 128),
      [&](const range & r) {
  for( int x = r.begin(); x < r.end(); ++x ) {
#else
  for( int x = 0; x < cols; ++x ) {
#endif

    for( int start = 0;; ) {

      while (start < rows && mask[start][x]) {
        ++start;
      }

      int end = start + 1;
      while (end < rows && !mask[end][x]) {
        ++end;
      }

      if( start > 0 && end < rows ) {

        float sv[cn], ev[cn];
        float kk[cn];

        const int s = start - 1;
        const int e = end;

        for( int c = 0; c < cn; ++c ) {
          sv[c] = src[s][x * cn + c];
          ev[c] = src[e][x * cn + c];
          kk[c] = (ev[c] - sv[c]) / (end - start);
        }

        for( int y = start; y < e; ++y ) {
          dists[y][x] = std::max(y - s, e - y);
          for( int c = 0; c < cn; ++c ) {
            src[y][x * cn + c] = cv::saturate_cast<T>(
                sv[c] + (y - s) * kk[c]);
          }
        }
      }
      else if( start > 0 ) {

        const int s = start - 1;
        const int e = rows;

        for( int y = start; y < e; ++y ) {
          dists[y][x] = y - s;
          for( int c = 0; c < cn; ++c ) {
            src[y][x * cn + c] = src[s][x * cn + c];
          }
        }
      }
      else if( end < rows ) {

        // const int s = -1;
        const int e = end;

        for( int y = start; y < e; ++y ) {
          dists[y][x] = e - y;
          for( int c = 0; c < cn; ++c ) {
            src[y][x * cn + c] = src[e][x * cn + c];
          }
        }
      }

      if( (start = end + 1) >= rows ) {
        break;
      }
    }
  }
#if HAVE_TBB && !defined(Q_MOC_RUN)
    });
#endif
}

static void interpolate_holes_h(cv::Mat & image, const cv::Mat1b & mask, cv::Mat1f & dists)
{
  switch (image.depth()) {
    case CV_8U:
      interpolate_holes_h_<uint8_t>(image, mask, dists);
      break;
    case CV_8S:
      interpolate_holes_h_<int8_t>(image, mask, dists);
      break;
    case CV_16U:
      interpolate_holes_h_<uint16_t>(image, mask, dists);
      break;
    case CV_16S:
      interpolate_holes_h_<int16_t>(image, mask, dists);
      break;
    case CV_32S:
      interpolate_holes_h_<int32_t>(image, mask, dists);
      break;
    case CV_32F:
      interpolate_holes_h_<float>(image, mask, dists);
      break;
    case CV_64F:
      interpolate_holes_h_<double>(image, mask, dists);
      break;
  }
}

static void interpolate_holes_v(cv::Mat & image, const cv::Mat1b & mask, cv::Mat1f & dists)
{
  switch (image.depth()) {
    case CV_8U:
      interpolate_holes_v_<uint8_t>(image, mask, dists);
      break;
    case CV_8S:
      interpolate_holes_v_<int8_t>(image, mask, dists);
      break;
    case CV_16U:
      interpolate_holes_v_<uint16_t>(image, mask, dists);
      break;
    case CV_16S:
      interpolate_holes_v_<int16_t>(image, mask, dists);
      break;
    case CV_32S:
      interpolate_holes_v_<int32_t>(image, mask, dists);
      break;
    case CV_32F:
      interpolate_holes_v_<float>(image, mask, dists);
      break;
    case CV_64F:
      interpolate_holes_v_<double>(image, mask, dists);
      break;
  }
}


template<class T>
static void fill_holes_(cv::Mat & himage, const cv::Mat & vimage,
    const cv::Mat1f & hdists, const cv::Mat1f & vdists,
    cv::Mat1b & mask)
{
  cv::Mat_<T> hsrc =
      himage;

  const cv::Mat_<T> vsrc =
      vimage;

  const int cn =
      himage.channels();

  const int rows =
      himage.rows;

  const int cols =
      himage.cols;

#if HAVE_TBB && !defined(Q_MOC_RUN)

  typedef tbb::blocked_range<int> range;

  tbb::parallel_for(range(0, rows, 256),
      [&](const range & r) {
  for( int y = r.begin(); y < r.end(); ++y ) {
#else
  for( int y = 0; y < rows; ++y ) {
#endif

    T *hsrcp = hsrc[y];
    uint8_t *mskp = mask[y];

    const T *vsrcp = vsrc[y];

    const float *hdistsp = hdists[y];
    const float *vdistsp = vdists[y];

    for( int x = 0; x < cols; ++x ) {
      if( !mskp[x] ) {

        if( hdistsp[x] && vdistsp[x] ) {

          const float dh = hdistsp[x];
          const float dv = vdistsp[x];
          const float dd = 1 / (dh + dv);

          for( int c = 0; c < cn; ++c ) {
            hsrcp[x * cn + c] =
                cv::saturate_cast<T>(dd * (hsrcp[x * cn + c] * dv +
                    vsrcp[x * cn + c] * dh));
          }

          mskp[x] = 255;
        }
        else if( vdistsp[x] ) {
          for( int c = 0; c < cn; ++c ) {
            hsrcp[x * cn + c] = vsrcp[x * cn + c];
          }
          mskp[x] = 255;
        }
        else if( hdistsp[x] ) {
          mskp[x] = 255;
        }
      }
    }
  }
#if HAVE_TBB && !defined(Q_MOC_RUN)
    });
#endif
}

static void fill_holes(cv::Mat & himage, const cv::Mat & vimage,
    const cv::Mat1f & hdists, const cv::Mat1f & vdists,
    cv::Mat1b & mask)
{
  switch (himage.depth()) {
    case CV_8U:
      fill_holes_<uint8_t>(himage, vimage, hdists, vdists, mask);
      break;
    case CV_8S:
      fill_holes_<int8_t>(himage, vimage, hdists, vdists, mask);
      break;
    case CV_16U:
      fill_holes_<uint16_t>(himage, vimage, hdists, vdists, mask);
      break;
    case CV_16S:
      fill_holes_<int16_t>(himage, vimage, hdists, vdists, mask);
      break;
    case CV_32S:
      fill_holes_<int32_t>(himage, vimage, hdists, vdists, mask);
      break;
    case CV_32F:
      fill_holes_<float>(himage, vimage, hdists, vdists, mask);
      break;
    case CV_64F:
      fill_holes_<double>(himage, vimage, hdists, vdists, mask);
      break;
  }
}

void linear_interpolation_inpaint(cv::InputArray _src, cv::InputArray _mask, cv::OutputArray dst)
{
  INSTRUMENT_REGION("");

  if( _mask.empty() || cv::countNonZero(_mask) == _mask.size().area() ) {
    _src.copyTo(dst);
    return;
  }

  cv::Mat himage, vimage;
  cv::Mat1f hdists, vdists;
  cv::Mat1b mask;

  _src.copyTo(himage);
  _mask.copyTo(mask);

  while (42) {

    himage.copyTo(vimage);
    interpolate_holes_h(himage, mask, hdists);
    interpolate_holes_v(vimage, mask, vdists);
    fill_holes(himage, vimage, hdists, vdists, mask);

    if( cv::countNonZero(mask) == mask.size().area() ) {
      break;
    }
  }

  dst.move(himage);
}

