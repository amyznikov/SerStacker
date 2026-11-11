/*
 * asinh.cc
 *
 *  Created on: Nov 9, 2026
 *      Author: amyznikov
 */
#include "asinh.h"
#include <core/proc/run-loop.h>
#include <core/proc/pixtype.h>
#include <core/debug.h>

template<class _Tp1, class _Tp2>
static bool _asinh(cv::InputArray _src, cv::OutputArray _dst, double _scale, double _offset)
{
  using _Tpc = std::common_type_t<_Tp1,_Tp2>;
  using _Tcomp = std::conditional_t<std::is_floating_point_v<_Tpc>, _Tpc, float>;

  const int cn = _src.channels();
  const cv::Size size = _src.size();
  const int total_width = size.width * cn;

  const int dtype = CV_MAKETYPE(cv::DataType<_Tp2>::depth, cn);
  if ( _dst.fixedType() && _dst.type() != dtype )  {
    CF_ERROR("Bad fixed type %d for destination. Must be %d", _dst.type(), dtype);
    return false;
  }

  const cv::Mat src = _src.getMat();
  const uint8_t* const src_base = src.ptr();
  const size_t src_stride = src.step;

  _dst.create(size, dtype);
  cv::Mat dst = _dst.getMatRef();
  uint8_t* const dst_base = dst.ptr();
  const size_t dst_stride = dst.step;

  const _Tcomp scale = _Tcomp(_scale);
  const _Tcomp offset = _Tcomp(_offset);

  parallel_for(0, size.height, [=](const auto & range) {
    for (int y = rbegin(range); y < rend(range); ++y) {
      const _Tp1 * srcp = (const _Tp1*)(src_base + y * src_stride);
      _Tp2 * __restrict dstp = (_Tp2*)(dst_base + y * dst_stride);
      for (int x = 0; x < total_width; ++x) {
        const _Tcomp srcv = srcp[x];
        dstp[x] = cv::saturate_cast<_Tp2>(std::asinh(srcv * scale + offset));
      }
    }
  });

  return true;
}

bool asinh(cv::InputArray _src, cv::OutputArray _dst, double _scale, double _offset)
{
  const int ddepth = _dst.fixedType() ? _dst.depth() : _src.depth();
  CV_DISPATCH2(_src.depth(), ddepth, _asinh, _src, _dst, _scale, _offset);
  CF_ERROR("Not supported combination of src (%d) and dst (%d) types", _src.depth(), ddepth);
  return false;
}
