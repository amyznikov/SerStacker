/*
 * project_to_radius_vector.cc
 *
 *  Created on: May 31, 2026
 *      Author: amyznikov
 */

#include "project_to_radius_vector.h"
#include "pixtype.h"
#include "run-loop.h"
#include <core/debug.h>

//template<class _Tp>
//static bool _project_to_radius_vector(const cv::Point2f & rp,
//    cv::InputArray _gx, cv::InputArray _gy,
//    cv::OutputArray _gr, cv::OutputArray _gt,
//    double scale)
//{
//  const int rows = _gx.rows();
//  const int cols = _gx.cols();
//  const int cn = _gx.channels();
//
//  const cv::Mat_<_Tp> gx = _gx.getMat();
//  const cv::Mat_<_Tp> gy = _gy.getMat();
//
//  cv::Mat_<_Tp> gr, gt;
//
//  if( _gr.needed() ) {
//    gr = cv::Mat::zeros(rows, cols, _gx.type());
//  }
//  if( _gt.needed() ) {
//    gt = cv::Mat::zeros(rows, cols, _gx.type());
//  }
//
//  cv::parallel_for_(cv::Range(0, rows), [&, scale, cols, cn](const cv::Range & range) {
//    for ( int y = range.start; y < range.end; ++y ) {
//
//      const _Tp * gxp = gx[y];
//      const _Tp * gyp = gy[y];
//      _Tp * __restrict grp = gr.empty() ? nullptr : gr[y];
//      _Tp * __restrict gtp = gt.empty() ? nullptr : gt[y];
//
//      for ( int x = 0; x < cols; ++x ) {
//        const float dx = x - rp.x;
//        const float dy = y - rp.y;
//        const float dr = std::max(2 * FLT_EPSILON, std::sqrt(dx * dx + dy * dy));
//        const float ca = dx * scale / dr;
//        const float sa = dy * scale / dr;
//        for ( int c = 0; c < cn; ++c ) {
//          const float gxv = gxp[x * cn + c];
//          const float gyv = gyp[x * cn + c];
//          if ( grp ) {
//            grp[x * cn + c] = cv::saturate_cast<_Tp>(+gxv * ca + gyv * sa);
//          }
//          if ( gtp ) {
//            gtp[x * cn + c] = cv::saturate_cast<_Tp>(-gyv * ca + gxv * sa);
//          }
//        }
//      }
//    }
//  });
//
//  if( _gr.needed() ) {
//    _gr.move(gr);
//  }
//  if( _gt.needed() ) {
//    _gt.move(gt);
//  }
//  return true;
//}

template<class _Tp>
static bool _project_to_radius_vector(const cv::Point2f & rp,
    cv::InputArray _gx, cv::InputArray _gy,
    cv::OutputArray _gr, cv::OutputArray _gt,
    double scale)
{
  const int rows = _gx.rows();
  const int cols = _gx.cols();
  const int cn = _gx.channels();

  const cv::Mat gx = _gx.getMat();
  const cv::Mat gy = _gy.getMat();
  cv::Mat gr, gt;

  if( _gr.needed() ) {
    gr = cv::Mat::zeros(rows, cols, _gx.type());
  }
  if( _gt.needed() ) {
    gt = cv::Mat::zeros(rows, cols, _gx.type());
  }

  const uint8_t * gx_base = gx.ptr();
  const size_t gx_stride = gx.step;

  const uint8_t * gy_base = gy.ptr();
  const size_t gy_stride = gy.step;

  const uint8_t * gr_base = gr.empty() ? nullptr : gr.ptr();
  const size_t gr_stride = gr.empty() ? 0 : gr.step;

  const uint8_t * gt_base = gt.empty() ? nullptr : gt.ptr();
  const size_t gt_stride = gt.empty() ? 0 : gt.step;

  parallel_for(0, rows, [=](const auto & range) {
    for ( int y = rbegin(range); y < rend(range); ++y ) {

      const _Tp * gxp = (const _Tp * )(gx_base + y * gx_stride);
      const _Tp * gyp = (const _Tp * )(gy_base + y * gy_stride);

      _Tp * __restrict grp = (_Tp *)(gr_base ? gr_base + y * gr_stride : nullptr);
      _Tp * __restrict gtp = (_Tp *)(gt_base ? gt_base + y * gt_stride : nullptr);

      for ( int x = 0; x < cols; ++x ) {
        const float dx = x - rp.x, dy = y - rp.y;
        const float dr = std::max(2 * FLT_EPSILON, std::sqrt(dx * dx + dy * dy));
        const float ca = dx * scale / dr;
        const float sa = dy * scale / dr;
        for ( int c = 0; c < cn; ++c ) {
          const float gxv = gxp[x * cn + c];
          const float gyv = gyp[x * cn + c];
          if ( grp ) {
            grp[x * cn + c] = cv::saturate_cast<_Tp>(+gxv * ca + gyv * sa);
          }
          if ( gtp ) {
            gtp[x * cn + c] = cv::saturate_cast<_Tp>(-gyv * ca + gxv * sa);
          }
        }
      }
    }
  });

  if( _gr.needed() ) {
    _gr.move(gr);
  }
  if( _gt.needed() ) {
    _gt.move(gt);
  }
  return true;
}

bool project_to_radius_vector(const cv::Point2f & rp, cv::InputArray gx, cv::InputArray gy,
    cv::OutputArray gr, cv::OutputArray gt,
    double scale)
{
  CV_DISPATCH(gx.depth(), _project_to_radius_vector, rp, gx, gy, gr, gt, scale);
  return false;
}
