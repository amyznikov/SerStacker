/*
 * linear_interpolation_inpaint.cc
 *
 *  Created on: Jan 29, 2023
 *      Author: amyznikov
 */

#include "linear_interpolation_inpaint.h"
#include <core/proc/run-loop.h>
#include <core/proc/pixtype.h>
#include <atomic>
#include <core/debug.h>

//template<class _Tp>
//static bool _interpolate_holes_h(cv::Mat & image, const cv::Mat1b & mask, cv::Mat1f & dists)
//{
//  const int rows = mask.rows;
//  const int cols = mask.cols;
//  const int cn = image.channels();
//
//  dists.create(image.size());
//
//  _Tp* const src_base = image.ptr<_Tp>();
//  const uint8_t* const mask_base = mask.ptr<uint8_t>();
//  float* const dist_base = dists.ptr<float>();
//
//  const size_t src_step = image.step / sizeof(_Tp);
//  const size_t mask_step = mask.step / sizeof(uint8_t);
//  const size_t dist_step = dists.step / sizeof(float);
//
//  parallel_loop(0, rows, [=](int y) {
//
//    float sv[cn], ev[cn], kk[cn];
//
//    _Tp * __restrict srcp = src_base + y * src_step;
//    const uint8_t * __restrict mskp = mask_base + y * mask_step;
//    float * __restrict distp = dist_base + y * dist_step;
//
//    for( int start = 0;; ) {
//      while (start < cols && mskp[start]) {
//        distp[start] = 0.0f;
//        ++start;
//      }
//
//      if (start >= cols) {
//        break;
//      }
//
//      int end = start + 1;
//      while (end < cols && !mskp[end]) {
//        ++end;
//      }
//
//      if( start > 0 && end < cols ) {
//        const int s = start - 1, e = end;
//
//        const _Tp* p_start = srcp + s * cn;
//        const _Tp* p_end   = srcp + e * cn;
//
//        const float inv_len = 1.0f / (end - start);
//        for( int c = 0; c < cn; ++c ) {
//          sv[c] = p_start[c];
//          ev[c] = p_end[c];
//          kk[c] = (ev[c] - sv[c]) * inv_len;
//        }
//
//        float* __restrict d_out = distp + start;
//        _Tp* __restrict p_out = srcp + start * cn;
//
//        for( int x = start; x < e; ++x ) {
//          *d_out = std::max(x - s, e - x);
//
//          const float factor = x - s;
//          for( int c = 0; c < cn; ++c ) {
//            p_out[c] = cv::saturate_cast<_Tp>(sv[c] + factor * kk[c]);
//          }
//
//          d_out++;
//          p_out += cn;
//        }
//      }
//      else if( start > 0 ) {
//        const int s = start - 1;
//        const int e = cols;
//
//        const _Tp* __restrict p_start = srcp + s * cn;
//        float* __restrict d_out = distp + start;
//        _Tp* __restrict p_out = srcp + start * cn;
//
//        for( int x = start; x < e; ++x ) {
//          *d_out = x - s;
//
//          for( int c = 0; c < cn; ++c ) {
//            p_out[c] = p_start[c];
//          }
//
//          d_out++;
//          p_out += cn;
//        }
//      }
//      else if( end < cols ) {
//        const int e = end;
//
//        const _Tp* __restrict p_end = srcp + e * cn;
//        float* __restrict d_out = distp + start;
//        _Tp* __restrict p_out = srcp + start * cn;
//
//        for( int x = start; x < e; ++x ) {
//          *d_out = e - x;
//          for( int c = 0; c < cn; ++c ) {
//            p_out[c] = p_end[c];
//          }
//
//          d_out++;
//          p_out += cn;
//        }
//      }
//
//      start = end;
//      if( start >= cols ) {
//        break;
//      }
//    }
//  });
//
//  return true;
//}
//
//static bool interpolate_holes_h(cv::Mat & image, const cv::Mat1b & mask, cv::Mat1f & dists)
//{
//  INSTRUMENT_REGION("");
//  CV_DISPATCH(image.depth(), _interpolate_holes_h, image, mask, dists);
//  return false;
//}
//
//template<class _Tp>
//static bool _interpolate_holes_v(cv::Mat & image, const cv::Mat1b & mask, cv::Mat1f & dists)
//{
//  const int cn = image.channels();
//  const int rows = mask.rows;
//  const int cols = mask.cols;
//
//  dists.create(image.size());
//
//  const size_t src_step = image.step / sizeof(_Tp);
//  const size_t dist_step = dists.step / sizeof(float);
//  const size_t mask_step = mask.step / sizeof(uint8_t);
//
//  _Tp* const src_base = image.ptr<_Tp>();
//  float* const dist_base = dists.ptr<float>();
//  const uint8_t*  const mask_base = mask.ptr<uint8_t>();
//
//  parallel_for(0, cols, [=](const auto & range) {
//
//    float sv[cn], ev[cn], kk[cn];
//
//    for ( int x = rbegin(range); x < rend(range); ++x ) {
//
//      const uint8_t* m_ptr_start = mask_base + x;
//      float* d_ptr_start = dist_base + x;
//      _Tp* s_ptr_start = src_base + x * cn;
//
//      for( int start = 0;; ) {
//
//        const uint8_t* __restrict m_curr = m_ptr_start + start * mask_step;
//        float* __restrict d_curr = d_ptr_start + start * dist_step;
//
//        while (start < rows && *m_curr) {
//          *d_curr = 0.0f;
//          m_curr += mask_step;
//          d_curr += dist_step;
//          ++start;
//        }
//
//        if (start >= rows) {
//          break;
//        }
//
//        int end = start + 1;
//        const uint8_t* __restrict m_end = m_ptr_start + end * mask_step;
//        while (end < rows && !*m_end) {
//          m_end += mask_step;
//          ++end;
//        }
//
//        if( start > 0 && end < rows ) {
//          const int s = start - 1;
//          const int e = end;
//
//          const _Tp* __restrict p_start = s_ptr_start + s * src_step;
//          const _Tp* __restrict p_end   = s_ptr_start + e * src_step;
//
//          const float inv_len = 1.0f / (end - start);
//          for( int c = 0; c < cn; ++c ) {
//            sv[c] = p_start[c];
//            ev[c] = p_end[c];
//            kk[c] = (ev[c] - sv[c]) * inv_len;
//          }
//
//          float* __restrict d_out = d_ptr_start + start * dist_step;
//          _Tp* __restrict p_out = s_ptr_start + start * src_step;
//
//          for( int y = start; y < e; ++y ) {
//            *d_out = std::max(y - s, e - y);
//
//            const float factor = y - s;
//            for( int c = 0; c < cn; ++c ) {
//              p_out[c] = cv::saturate_cast<_Tp>(sv[c] + factor * kk[c]);
//            }
//
//            d_out += dist_step;
//            p_out += src_step;
//          }
//        }
//        else if( start > 0 ) {
//          const int s = start - 1;
//          const int e = rows;
//          const _Tp* __restrict p_start = s_ptr_start + s * src_step;
//
//          float* __restrict d_out = d_ptr_start + start * dist_step;
//          _Tp* __restrict p_out = s_ptr_start + start * src_step;
//
//          for( int y = start; y < e; ++y ) {
//            *d_out = y - s;
//            for( int c = 0; c < cn; ++c ) {
//              p_out[c] = p_start[c];
//            }
//
//            d_out += dist_step;
//            p_out += src_step;
//          }
//        }
//        else if( end < rows ) {
//          const int e = end;
//          const _Tp* __restrict p_end = s_ptr_start + e * src_step;
//
//          float* __restrict d_out = d_ptr_start + start * dist_step;
//          _Tp* __restrict p_out = s_ptr_start + start * src_step;
//
//          for( int y = start; y < e; ++y ) {
//            *d_out = e - y;
//            for( int c = 0; c < cn; ++c ) {
//              p_out[c] = p_end[c];
//            }
//
//            d_out += dist_step;
//            p_out += src_step;
//          }
//        }
//
//        start = end;
//        if( start >= rows ) {
//          break;
//        }
//      }
//    }
//  });
//
//  return true;
//}
//
//static bool interpolate_holes_v(cv::Mat & image, const cv::Mat1b & mask, cv::Mat1f & dists)
//{
//  INSTRUMENT_REGION("");
//  CV_DISPATCH(image.depth(), _interpolate_holes_v, image, mask, dists);
//  return false;
//}
//
//template<class _Tp>
//static int _fill_holes(cv::Mat & himage, const cv::Mat & vimage,
//    const cv::Mat1f & hdists, const cv::Mat1f & vdists,
//    cv::Mat1b & mask)
//{
//  const int cn = himage.channels();
//  const int rows = himage.rows;
//  const int cols = himage.cols;
//
//  cv::Mat_<_Tp> hsrc = himage;
//  const cv::Mat_<_Tp> vsrc = vimage;
//
//  std::atomic<int> total_filled(0);
//
//  parallel_for(0, rows, [&](const auto & range) {
//    int local_filled = 0;
//
//    for( int y = rbegin(range); y < rend(range); ++y ) {
//
//      _Tp *hsrcp = hsrc[y];
//      uint8_t *mskp = mask[y];
//
//      const _Tp *vsrcp = vsrc[y];
//      const float * hdistsp = hdists[y];
//      const float * vdistsp = vdists[y];
//
//      for( int x = 0; x < cols; ++x ) {
//        if( !mskp[x] ) {
//
//          if( hdistsp[x] && vdistsp[x] ) {
//            const float dh = hdistsp[x];
//            const float dv = vdistsp[x];
//            const float dd = 1 / (dh + dv);
//            for( int c = 0; c < cn; ++c ) {
//              hsrcp[x * cn + c] = cv::saturate_cast<_Tp>(dd * (hsrcp[x * cn + c] * dv + vsrcp[x * cn + c] * dh));
//            }
//            mskp[x] = 255;
//            ++local_filled;
//          }
//          else if( vdistsp[x] ) {
//            for( int c = 0; c < cn; ++c ) {
//              hsrcp[x * cn + c] = vsrcp[x * cn + c];
//            }
//            mskp[x] = 255;
//            ++local_filled;
//          }
//          else if( hdistsp[x] ) {
//            mskp[x] = 255;
//            ++local_filled;
//          }
//        }
//      }
//    }
//    total_filled.fetch_add(local_filled,
//        std::memory_order_relaxed);
//  });
//
//  return total_filled.load(std::memory_order_relaxed);
//}
//
//static int fill_holes(cv::Mat & himage, const cv::Mat & vimage,
//    const cv::Mat1f & hdists, const cv::Mat1f & vdists,
//    cv::Mat1b & mask)
//{
//  INSTRUMENT_REGION("");
//  CV_DISPATCH(himage.depth(), _fill_holes, himage, vimage, hdists, vdists, mask);
//  return 0;
//}
//
//
//void linear_interpolation_inpaint(cv::InputArray _src, cv::InputArray _mask, cv::OutputArray dst)
//{
//  INSTRUMENT_REGION("");
//
//  int nnz = 0;
//  if( _mask.empty() || (nnz = cv::countNonZero(_mask)) == _mask.size().area() ) {
//    _src.copyTo(dst);
//    return;
//  }
//
//  cv::Mat himage, vimage;
//  cv::Mat1f hdists, vdists;
//  cv::Mat1b mask;
//
//  if ( true ) {
//    INSTRUMENT_REGION("copyTo1");
//    _src.copyTo(himage);
//    _mask.copyTo(mask);
//  }
//
//  const int mask_area = _mask.size().area() ;
//  while (nnz < mask_area ) {
//    if ( true ) {
//      INSTRUMENT_REGION("copyTo2");
//      himage.copyTo(vimage);
//    }
//    interpolate_holes_h(himage, mask, hdists);
//    interpolate_holes_v(vimage, mask, vdists);
//    fill_holes(himage, vimage, hdists, vdists, mask);
//    if ( true ) {
//      INSTRUMENT_REGION("countNonZero");
//      nnz = cv::countNonZero(mask);
//    }
//  }
//
//  dst.move(himage);
//}
//
//void linear_interpolation_inpaint(cv::Mat & image, cv::InputArray _mask)
//{
//  INSTRUMENT_REGION("");
//
//  int nnz = 0;
//
//  const int mask_area = _mask.empty() ? 0 : _mask.size().area();
//  if( (nnz = cv::countNonZero(_mask)) >= mask_area ) {
//    return;
//  }
//
//  cv::Mat vimage;
//  cv::Mat1f hdists, vdists;
//  cv::Mat1b mask;
//
//  if ( true ) {
//    INSTRUMENT_REGION("copy_mask");
//    _mask.getMat().copyTo(mask);
//  }
//
//  while (nnz < mask_area ) {
//    if ( true ) {
//      INSTRUMENT_REGION("copy_image");
//      image.copyTo(vimage);
//    }
//    interpolate_holes_h(image, mask, hdists);
//    interpolate_holes_v(vimage, mask, vdists);
//    nnz += fill_holes(image, vimage, hdists, vdists, mask);
//  }
//}
//
//

////////////////////////////////////////////////////////////////////////////////////
template<class _Tp>
static bool _interpolate_holes_h2(const cv::Mat & image, const cv::Mat1b & mask, cv::Mat & dst_inpaint,
    cv::Mat1f & dists)
{
  const int rows = mask.rows;
  const int cols = mask.cols;
  const int cn = image.channels();

  dists.create(image.size());

  const _Tp* const src_base = image.ptr<_Tp>();
  const uint8_t* const mask_base = mask.ptr<uint8_t>();

  _Tp* const out_base = dst_inpaint.ptr<_Tp>();
  float* const dist_base = dists.ptr<float>();

  const size_t src_step = image.step / sizeof(_Tp);
  const size_t mask_step = mask.step / sizeof(uint8_t);
  const size_t out_step = dst_inpaint.step / sizeof(_Tp);
  const size_t dist_step = dists.step / sizeof(float);

  parallel_loop(0, rows, [=](int y) {
    const _Tp * __restrict srcp = src_base + y * src_step;
    const uint8_t * __restrict mskp = mask_base + y * mask_step;
    _Tp * __restrict outp = out_base + y * out_step;
    float * __restrict distp = dist_base + y * dist_step;

    float sv[4] = {0}, ev[4] = {0}, kk[4] = {0};

    for( int start = 0;; ) {
      while (start < cols && mskp[start]) {
        distp[start] = 0.0f;
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
        const _Tp* __restrict p_start = srcp + s * cn;
        const _Tp* __restrict p_end   = srcp + e * cn;

        const float inv_len = 1.0f / (end - start);
        for( int c = 0; c < cn; ++c ) {
          sv[c] = float(p_start[c]);
          ev[c] = float(p_end[c]);
          kk[c] = (ev[c] - sv[c]) * inv_len;
        }

        float* __restrict d_out = distp + start;
        _Tp* __restrict p_out = outp + start * cn;

        for( int x = start; x < e; ++x ) {
          *d_out = float(std::max(x - s, e - x));
          const float factor = float(x - s);
          for( int c = 0; c < cn; ++c ) {
            p_out[c] = cv::saturate_cast<_Tp>(sv[c] + factor * kk[c]);
          }
          d_out++;
          p_out += cn;
        }
      }
      else if( start > 0 ) {
        const int s = start - 1;
        const int e = cols;
        const _Tp* __restrict p_start = srcp + s * cn;

        float* __restrict d_out = distp + start;
        _Tp* __restrict p_out = outp + start * cn;

        for( int x = start; x < e; ++x ) {
          *d_out = float(x - s);
          for( int c = 0; c < cn; ++c ) {
            p_out[c] = p_start[c];
          }
          d_out++;
          p_out += cn;
        }
      }
      else if( end < cols ) {
        const int e = end;
        const _Tp* p_end = srcp + e * cn;

        float* __restrict d_out = distp + start;
        _Tp* __restrict p_out = outp + start * cn;

        for( int x = start; x < e; ++x ) {
          *d_out = float(e - x);
          for( int c = 0; c < cn; ++c ) {
            p_out[c] = p_end[c];
          }
          d_out++;
          p_out += cn;
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
static bool _interpolate_holes_v2(const cv::Mat & image, const cv::Mat1b & mask, cv::Mat & dst_inpaint, cv::Mat1f & dists)
{
  const int cn = image.channels();
  const int rows = mask.rows;
  const int cols = mask.cols;

  dists.create(image.size());

  const size_t src_step  = image.step / sizeof(_Tp);
  const size_t out_step  = dst_inpaint.step / sizeof(_Tp);
  const size_t dist_step = dists.step / sizeof(float);
  const size_t mask_step = mask.step / sizeof(uint8_t);

  const _Tp* const src_base       = image.ptr<_Tp>();
  const uint8_t* const mask_base  = mask.ptr<uint8_t>();
  _Tp* const out_base             = dst_inpaint.ptr<_Tp>();
  float* const dist_base          = dists.ptr<float>();

  parallel_loop(0, cols, [=](int x) {

    const uint8_t* const m_ptr_start = mask_base + x;
    float* __restrict const d_ptr_start         = dist_base + x;
    const _Tp* __restrict const s_ptr_start     = src_base + x * cn;
    _Tp* __restrict const o_ptr_start           = out_base + x * cn;

    float sv[4] = {0}, ev[4] = {0}, kk[4] = {0};

    for( int start = 0;; ) {
      const uint8_t* __restrict m_curr = m_ptr_start + start * mask_step;
      float* __restrict d_curr = d_ptr_start + start * dist_step;

      while (start < rows && *m_curr) {
        *d_curr = 0.0f;
        m_curr += mask_step;
        d_curr += dist_step;
        ++start;
      }

      if (start >= rows) {
        break;
      }

      int end = start + 1;
      const uint8_t* m_end = m_ptr_start + end * mask_step;
      while (end < rows && !*m_end) {
        m_end += mask_step;
        ++end;
      }

      if( start > 0 && end < rows ) {
        const int s = start - 1;
        const int e = end;

        const _Tp* __restrict p_start = s_ptr_start + s * src_step;
        const _Tp* __restrict p_end   = s_ptr_start + e * src_step;

        const float inv_len = 1.0f / (end - start);
        for( int c = 0; c < cn; ++c ) {
          sv[c] = float(p_start[c]);
          ev[c] = float(p_end[c]);
          kk[c] = (ev[c] - sv[c]) * inv_len;
        }

        float* __restrict d_out = d_ptr_start + start * dist_step;
        _Tp* __restrict p_out   = o_ptr_start + start * out_step;

        for( int y = start; y < e; ++y ) {
          *d_out = float(std::max(y - s, e - y));

          const float factor = float(y - s);
          for( int c = 0; c < cn; ++c ) {
            p_out[c] = cv::saturate_cast<_Tp>(sv[c] + factor * kk[c]);
          }

          d_out += dist_step;
          p_out += out_step;
        }
      }
      else if( start > 0 ) {
        const int s = start - 1;
        const int e = rows;
        const _Tp* __restrict p_start = s_ptr_start + s * src_step;

        float* __restrict d_out = d_ptr_start + start * dist_step;
        _Tp* __restrict p_out   = o_ptr_start + start * out_step;

        for( int y = start; y < e; ++y ) {
          *d_out = float(y - s);

          for( int c = 0; c < cn; ++c ) {
            p_out[c] = p_start[c];
          }

          d_out += dist_step;
          p_out += out_step;
        }
      }
      else if( end < rows ) {
        const int e = end;
        const _Tp* __restrict p_end = s_ptr_start + e * src_step;

        float* __restrict d_out = d_ptr_start + start * dist_step;
        _Tp* __restrict p_out   = o_ptr_start + start * out_step;

        for( int y = start; y < e; ++y ) {
          *d_out = float(e - y);

          for( int c = 0; c < cn; ++c ) {
            p_out[c] = p_end[c];
          }

          d_out += dist_step;
          p_out += out_step;
        }
      }

      start = end;
      if( start >= rows ) {
        break;
      }
    }
  });

  return true;
}

template<class _Tp>
static int _fill_holes2(cv::Mat & image, const cv::Mat & inpaint_h, const cv::Mat & inpaint_v,
    const cv::Mat1f & hdists, const cv::Mat1f & vdists,
    cv::Mat1b & mask)
{
  const int cn = image.channels();
  const int rows = image.rows;
  const int cols = image.cols;

  cv::Mat_<_Tp> io_src = image;
  const cv::Mat_<_Tp> h_in = inpaint_h;
  const cv::Mat_<_Tp> v_in = inpaint_v;

  std::atomic<int> total_filled(0);

  parallel_for(0, rows, [&](const auto & range) {
    int local_filled = 0;

    for( int y = rbegin(range); y < rend(range); ++y ) {

      _Tp *__restrict io_ptr = io_src[y];
      uint8_t *__restrict mskp = mask[y];

      const _Tp *__restrict h_ptr = h_in[y];
      const _Tp *__restrict v_ptr = v_in[y];
      const float * __restrict hdistsp = hdists[y];
      const float * __restrict vdistsp = vdists[y];

      for( int x = 0; x < cols; ++x ) {
        if( !mskp[x] ) {

          if( hdistsp[x] && vdistsp[x] ) {
            const float dh = hdistsp[x];
            const float dv = vdistsp[x];
            const float dd = 1.0f / (dh + dv);

            for( int c = 0; c < cn; ++c ) {
              io_ptr[x * cn + c] = cv::saturate_cast<_Tp>(dd * (h_ptr[x * cn + c] * dv + v_ptr[x * cn + c] * dh));
            }
            mskp[x] = 255;
            local_filled++;
          }
          else if( vdistsp[x] ) {
            for( int c = 0; c < cn; ++c ) {
              io_ptr[x * cn + c] = v_ptr[x * cn + c];
            }
            mskp[x] = 255;
            local_filled++;
          }
          else if( hdistsp[x] ) {
            for( int c = 0; c < cn; ++c ) {
              io_ptr[x * cn + c] = h_ptr[x * cn + c];
            }
            mskp[x] = 255;
            local_filled++;
          }
        }
      }
    }
    total_filled.fetch_add(local_filled,
        std::memory_order_relaxed);
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

  if (_mask.empty()) {
    return;
  }

  const int mask_area = _mask.size().area();
  int nnz = cv::countNonZero(_mask);
  if( nnz >= mask_area ) {
    return;
  }

  cv::Mat1b mask;
  cv::Mat inpaint_h, inpaint_v;
  cv::Mat1f hdists, vdists;

  inpaint_h.create(image.size(), image.type());
  inpaint_v.create(image.size(), image.type());

  _mask.getMat().copyTo(mask);
  while (nnz < mask_area ) {
    interpolate_holes_h2(image, mask, inpaint_h, hdists);
    interpolate_holes_v2(image, mask, inpaint_v, vdists);
    nnz += fill_holes2(image, inpaint_h, inpaint_v, hdists, vdists, mask);
  }
}

void linear_interpolation_inpaint(cv::InputArray _src, cv::InputArray _mask,
    cv::OutputArray dst)
{
  _src.copyTo(dst);
  linear_interpolation_inpaint(dst.getMatRef(), _mask);
}
