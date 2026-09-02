/*
 * linear_interpolation_inpaint.cc
 *
 *  Created on: Jan 29, 2023
 *      Author: amyznikov
 */

#include "linear_interpolation_inpaint.h"
#include <core/proc/reduce_channels.h>
#include <core/proc/run-loop.h>
#include <core/proc/pixtype.h>
#include <atomic>
#include <core/debug.h>

template<class _Tp>
static bool _interpolate_holes_h2(const cv::Mat & image, const cv::Mat1b & mask, cv::Mat & dst_inpaint,
    cv::Mat1f & dists)
{
  const int rows = mask.rows;
  const int cols = mask.cols;
  const int cn = image.channels();

  dists.create(image.size());

  const uint8_t * const src_base = image.ptr();
  const size_t src_stride = image.step;

  const uint8_t * const mask_base = mask.ptr();
  const size_t mask_stride = mask.step;

  uint8_t * const inpaint_base = dst_inpaint.ptr();
  const size_t inpaint_step = dst_inpaint.step;

  uint8_t* const dists_base = dists.ptr();
  const size_t dists_stride = dists.step;

  parallel_loop(0, rows, [=](int y) {

    const _Tp * srcp = (const _Tp * )(src_base + y * src_stride);
    const uint8_t * __restrict mskp = mask_base + y * mask_stride;
    _Tp * __restrict inpaintp = (_Tp * )(inpaint_base + y * inpaint_step);
    float * __restrict distp = (float * )(dists_base + y * dists_stride);

    float sv[4] = {0}, ev[4] = {0}, kk[4] = {0};

    memset(distp, 0, cols * sizeof(*distp));

    for( int start = 0;; ) {
      while (start < cols && mskp[start]) {
        ++start;
      }

      if (start >= cols) {
        break;
      }

      int end = start + 1;
      while (end < cols && !mskp[end]) {
        ++end;
      }

      if( start > 0 && end < cols ) {
        const int s = start - 1, e = end;
        const float scale = 1.0f / (end - start);

        const _Tp* sbeg = srcp + s * cn, * send = srcp + e * cn;
        for( int c = 0; c < cn; ++c ) {
          sv[c] = sbeg[c];
          ev[c] = send[c];
          kk[c] = (ev[c] - sv[c]) * scale;
        }

        _Tp* __restrict inpaint = inpaintp + start * cn;
        float* __restrict dist = distp + start;
        for( int x = start; x < e; ++x, ++dist, inpaint += cn ) {
          const float factor = float(x - s);
          *dist = std::max(x - s, e - x);
          for( int c = 0; c < cn; ++c ) {
            inpaint[c] = cv::saturate_cast<_Tp>(sv[c] + factor * kk[c]);
          }
        }
      }
      else if( start > 0 ) {
        const int s = start - 1, e = cols;

        const _Tp* sbeg = srcp + s * cn;
        float* __restrict dist = distp + start;
        _Tp* __restrict inpaint = inpaintp + start * cn;
        for( int x = start; x < e; ++x, ++dist, inpaint += cn ) {
          *dist = (x - s);
          for( int c = 0; c < cn; ++c ) {
            inpaint[c] = sbeg[c];
          }
        }
      }
      else if( end < cols ) {
        const int e = end;

        const _Tp* send = srcp + e * cn;
        float* __restrict dist = distp + start;
        _Tp* __restrict inpaint = inpaintp + start * cn;
        for( int x = start; x < e; ++x, ++dist, inpaint += cn ) {
          *dist = (e - x);
          for( int c = 0; c < cn; ++c ) {
            inpaint[c] = send[c];
          }
        }
      }

      start = end;
      if( start >= cols ) {
        break;
      }
    }
  });

  return true;
}

template<class _Tp>
static bool _interpolate_holes_v2(const cv::Mat & image, const cv::Mat1b & mask, cv::Mat & dst_inpaint,
    cv::Mat1f & dists)
{
  const int rows = mask.rows;
  const int cols = mask.cols;
  const int cn = image.channels();

  dists.create(image.size());

  const uint8_t * const src_base  = image.ptr();
  const size_t src_stride = image.step;

  const uint8_t * const mask_base = mask.ptr();
  const size_t mask_stride = mask.step;

  uint8_t * const inpaint_base  = dst_inpaint.ptr();
  const size_t inpaint_stride = dst_inpaint.step;

  uint8_t* const dists_base  = dists.ptr();
  const size_t dists_stride = dists.step;

  parallel_for(0, cols, [=](const auto & range) {

    float sv[4] = {0}, ev[4] = {0}, kk[4] = {0};

    const int xbeg = rbegin(range);
    const int xend = rend(range);

    for (int y = 0; y < rows; ++y ) {
      memset(dists_base + y * dists_stride + xbeg * sizeof(float), 0,
          (xend - xbeg)* sizeof(float));
    }

    for ( int x = xbeg; x < xend; ++x ) {

      const uint8_t* const srccp = src_base + x * cn * sizeof(_Tp);
      const uint8_t* const mskcp = mask_base + x;
      uint8_t* const inpaintcp = inpaint_base + x * cn * sizeof(_Tp);
      uint8_t* const distscp = dists_base + x * sizeof(float);

      for( int start = 0; start < rows; ) {

        while (start < rows && *(mskcp + start * mask_stride)) {
          ++start;
        }

        if (start >= rows) {
          break;
        }

        int end = start + 1;
        while (end < rows && !*(mskcp + end * mask_stride)) {
          ++end;
        }

        if( start > 0 && end < rows ) {
          const int s = start - 1, e = end;
          const float scale = 1.0f / (end - start);
          const _Tp* sbeg = (const _Tp*)(srccp + s * src_stride);
          const _Tp* send = (const _Tp*)(srccp + e * src_stride);
          for( int c = 0; c < cn; ++c ) {
            sv[c] = sbeg[c];
            ev[c] = send[c];
            kk[c] = (ev[c] - sv[c]) * scale;
          }

          for( int y = start; y < e; ++y ) {
            const float factor = (y - s);
            _Tp* __restrict inpaint = (_Tp*)(inpaintcp + y * inpaint_stride);
            *(float * __restrict)(distscp + y * dists_stride) = std::max(y - s, e - y);
            for( int c = 0; c < cn; ++c ) {
              inpaint[c] = cv::saturate_cast<_Tp>(sv[c] + factor * kk[c]);
            }
          }
        }
        else if( start > 0 ) {
          const int s = start - 1, e = rows;
          const _Tp* sbeg = (const _Tp*)(srccp + s * src_stride);

          for( int y = start; y < e; ++y ) {
            _Tp* __restrict inpaint = (_Tp*)(inpaintcp + y * inpaint_stride);
            *(float * __restrict)(distscp + y * dists_stride) = (y - s);
            for( int c = 0; c < cn; ++c ) {
              inpaint[c] = sbeg[c];
            }
          }
        }
        else if( end < rows ) {
          const int e = end;
          const _Tp* send = (const _Tp*)(srccp + e * src_stride);

          for( int y = start; y < e; ++y ) {
            _Tp* __restrict inpaint = (_Tp*)(inpaintcp + y * inpaint_stride);
            *(float* __restrict)(distscp + y * dists_stride) = (e - y);
            for( int c = 0; c < cn; ++c ) {
              inpaint[c] = send[c];
            }
          }
        }

        start = end;
      }
    }
  });

  return true;
}


template<class _Tp>
static int _fill_holes2(cv::Mat & _image, const cv::Mat & _inpaint_h, const cv::Mat & _inpaint_v,
    const cv::Mat1f & _hdists, const cv::Mat1f & _vdists,
    cv::Mat1b & _mask)
{
  const int cn = _image.channels();
  const int rows = _image.rows;
  const int cols = _image.cols;

  uint8_t * const image_base = _image.ptr();
  const size_t image_stride = _image.step;

  uint8_t * const mask_base = _mask.ptr();
  const size_t mask_stride = _mask.step;

  const uint8_t * h_base = _inpaint_h.ptr();
  const size_t h_stride = _inpaint_h.step;

  const uint8_t * v_base = _inpaint_v.ptr();
  const size_t v_stride = _inpaint_v.step;

  const uint8_t * hdist_base = _hdists.ptr();
  const size_t hdist_stride = _hdists.step;

  const uint8_t * vdist_base = _vdists.ptr();
  const size_t vdist_stride = _vdists.step;

  std::atomic<int> total_filled(0);

  parallel_for(0, rows, [=, &total_filled](const auto & range) {
    int local_filled = 0;

    for( int y = rbegin(range); y < rend(range); ++y ) {

      const _Tp *hp = (const _Tp *)(h_base + y * h_stride);
      const _Tp *vp = (const _Tp *)(v_base + y * v_stride);
      const float * hdistsp  = (const float *)(hdist_base + y * hdist_stride);
      const float * vdistsp  = (const float *)(vdist_base + y * vdist_stride);

      _Tp *__restrict imagep = (_Tp *)(image_base + y * image_stride);
      uint8_t *__restrict mskp = mask_base + y * mask_stride;

      for( int x = 0; x < cols; ++x ) {
        if( !mskp[x] ) {

          const float dh = hdistsp[x];
          const float dv = vdistsp[x];

          if( dh > 0 && dv > 0 ) {
            const float dd = 1.0f / (dh + dv);
            for( int c = 0; c < cn; ++c ) {
              imagep[x * cn + c] = cv::saturate_cast<_Tp>(dd * (hp[x * cn + c] * dv + vp[x * cn + c] * dh));
            }
            mskp[x] = 255;
            local_filled++;
          }
          else if( dv > 0 ) {
            for( int c = 0; c < cn; ++c ) {
              imagep[x * cn + c] = vp[x * cn + c];
            }
            mskp[x] = 255;
            local_filled++;
          }
          else if( dh > 0 ) {
            for( int c = 0; c < cn; ++c ) {
              imagep[x * cn + c] = hp[x * cn + c];
            }
            mskp[x] = 255;
            local_filled++;
          }
          else {
            // no data available at all
          }
        }
      }
    }
    total_filled.fetch_add(local_filled, std::memory_order_relaxed);
  });

  return total_filled.load(std::memory_order_relaxed);
}

static bool interpolate_holes_h2(const cv::Mat & image, const cv::Mat1b & mask, cv::Mat & dst_inpaint,
    cv::Mat1f & dists)
{
  INSTRUMENT_REGION("");
  CV_DISPATCH(image.depth(), _interpolate_holes_h2, image, mask, dst_inpaint, dists);
  return false;
}

static bool interpolate_holes_v2(const cv::Mat & image, const cv::Mat1b & mask, cv::Mat & dst_inpaint, cv::Mat1f & dists)
{
  INSTRUMENT_REGION("");
  CV_DISPATCH(image.depth(), _interpolate_holes_v2, image, mask, dst_inpaint, dists);
  return false;
}

static int fill_holes2(cv::Mat & image, const cv::Mat & inpaint_h, const cv::Mat & inpaint_v,
    const cv::Mat1f & hdists, const cv::Mat1f & vdists,
    cv::Mat1b & mask)
{
  INSTRUMENT_REGION("");
  CV_DISPATCH(image.depth(), _fill_holes2, image, inpaint_h, inpaint_v, hdists, vdists, mask);
  return 0;
}

void linear_interpolation_inpaint(cv::Mat & image, cv::InputArray _mask)
{
  INSTRUMENT_REGION("");

  if ( _mask.empty() ) {
    return;
  }

 cv::Mat1b mask;
 cv::Mat inpaint_h, inpaint_v;
 cv::Mat1f hdists, vdists;

  if ( _mask.channels() == 1 ) {
    if ( _mask.depth() == CV_8U ) {
      _mask.copyTo(mask);
    }
    else {
      cv::compare(_mask, 0, mask, cv::CMP_GT);
    }
  }
  else if ( _mask.depth() == CV_8U ) {
    reduce_color_channels(_mask, mask, cv::REDUCE_MIN);
  }
  else {
    cv::Mat tmp;
    reduce_color_channels(_mask, tmp, cv::REDUCE_MIN);
    cv::compare(_mask, 0, mask, cv::CMP_GT);
  }

  int total_holes = mask.size().area() - cv::countNonZero(mask);
  if( total_holes < 1 ) {
    return;
  }

  inpaint_h.create(image.size(), image.type());
  inpaint_v.create(image.size(), image.type());

  while (total_holes > 0) {
    interpolate_holes_h2(image, mask, inpaint_h, hdists);
    interpolate_holes_v2(image, mask, inpaint_v, vdists);
    const int filled = fill_holes2(image, inpaint_h, inpaint_v, hdists, vdists, mask);
    if( filled < 1 ) {
      break;
    }
    total_holes -= filled;
  }
}

void linear_interpolation_inpaint(cv::InputArray _src, cv::InputArray _mask,
    cv::OutputArray dst)
{
  _src.copyTo(dst);
  linear_interpolation_inpaint(dst.getMatRef(), _mask);
}
