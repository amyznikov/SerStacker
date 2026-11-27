/*
 * debayer_ea.cc
 *
 *  Created on: Nov 26, 2026
 *      Author: amyznikov
 */

#include "debayer.h"
#include <core/proc/run-loop.h>
#include <core/proc/pixtype.h>
#include <core/ssprintf.h>
#include <core/debug.h>

#if defined(__GNUC__) || defined(__clang__)
#  define ALWAYS_INLINE __attribute__((always_inline)) inline
#elif defined(_MSC_VER)
#  define ALWAYS_INLINE __forceinline
#else
#  define ALWAYS_INLINE inline
#endif


template<typename _Tps>
static ALWAYS_INLINE _Tps ea_div2(_Tps v)
{
  if constexpr ( std::is_integral_v<_Tps> ) {
    return (v + 1) >> 1;
  }
  return v * _Tps(0.5);
}

template<typename _Tps>
static ALWAYS_INLINE _Tps ea_div4(_Tps v)
{
  if constexpr ( std::is_integral_v<_Tps> ) {
    return (v + 2) >> 2;
  }
  return v * _Tps(0.25);
}

template<typename _Tps>
static ALWAYS_INLINE _Tps ea_div16(_Tps v)
{
  if constexpr ( std::is_integral_v<_Tps> ) {
    return (v + 8) >> 4;
  }
  return v * _Tps(0.0625);
}

template<typename _Tps>
static ALWAYS_INLINE
_Tps ea_green(_Tps g_l, _Tps g_r, _Tps g_u, _Tps g_d,
    _Tps c_0, _Tps c_l2, _Tps c_r2, _Tps c_u2, _Tps c_d2)
{
  const _Tps lap_h = 2 * c_0 - c_l2 - c_r2;
  const _Tps lap_v = 2 * c_0 - c_u2 - c_d2;
  const _Tps dh = std::abs(g_l - g_r) + std::abs(lap_h);
  const _Tps dv = std::abs(g_u - g_d) + std::abs(lap_v);

  if ( dh < dv ) {
    return ea_div4(2 * (g_l + g_r) + lap_h);
  }
  if ( dv < dh ) {
    return ea_div4(2 * (g_u + g_d) + lap_v);
  }
  return ea_div16(4 * (g_l + g_r + g_u + g_d) + lap_h + lap_v);
}

template<typename _Tps>
static ALWAYS_INLINE
_Tps ea_chroma_hv(_Tps c1, _Tps c2, _Tps g0, _Tps g1, _Tps g2)
{
  return g0 + ea_div2((c1 - g1) + (c2 - g2));
}

template<typename _Tps>
static ALWAYS_INLINE
_Tps ea_chroma_diag(_Tps c_nw, _Tps c_ne, _Tps c_sw, _Tps c_se,
    _Tps g_0, _Tps g_nw, _Tps g_ne, _Tps g_sw, _Tps g_se)
{
  const _Tps d_nw = c_nw - g_nw;
  const _Tps d_ne = c_ne - g_ne;
  const _Tps d_sw = c_sw - g_sw;
  const _Tps d_se = c_se - g_se;

  const _Tps grad_nw_se = std::abs(d_nw - d_se) + std::abs(g_nw - g_se);
  const _Tps grad_ne_sw = std::abs(d_ne - d_sw) + std::abs(g_ne - g_sw);

  if ( grad_nw_se < grad_ne_sw ) {
    return g_0 + ea_div2(d_nw + d_se);
  }
  if ( grad_ne_sw < grad_nw_se ) {
    return g_0 + ea_div2(d_ne + d_sw);
  }
  return g_0 + ea_div4(d_nw + d_ne + d_sw + d_se);
}

template<typename _Tp1, typename _Tps>
static ALWAYS_INLINE void ea_compute_g_row(bool GOnEven,
    const _Tp1 * __restrict sm2,
    const _Tp1 * __restrict sm1,
    const _Tp1 * __restrict s0,
    const _Tp1 * __restrict sp1,
    const _Tp1 * __restrict sp2,
    _Tps * __restrict g,
    int w)
{
  const int x2 = 2 < w ? 2 : 0;
  const int x3 = 3 < w ? 3 : 1;

  if ( GOnEven ) {
    *(g + 0) = _Tps(*(s0 + 0));
    *(g + 1) = ea_green<_Tps>(
        *(s0 + 0), *(s0 + x2), *(sm1 + 1), *(sp1 + 1),
        *(s0 + 1), *(s0 + 1), *(s0 + x3), *(sm2 + 1), *(sp2 + 1));

    for ( int x = 2; x < w - 2; x += 2 ) {
      g[x] = _Tps(s0[x]);
      g[x + 1] = ea_green<_Tps>(
          s0[x], s0[x + 2], sm1[x + 1], sp1[x + 1],
          s0[x + 1], s0[x - 1], s0[x + 3], sm2[x + 1], sp2[x + 1]);
    }

    const int x = w - 2;
    g[x] = _Tps(s0[x]);
    g[x + 1] = ea_green<_Tps>(
        s0[x], s0[x], sm1[x + 1], sp1[x + 1],
        s0[x + 1], s0[x - 1], s0[x - 1], sm2[x + 1], sp2[x + 1]);
  }
  else {
    *(g + 0) = ea_green<_Tps>(
        *(s0 + 1), *(s0 + 1), *(sm1 + 0), *(sp1 + 0),
        *(s0 + 0), *(s0 + x2), *(s0 + x2), *(sm2 + 0), *(sp2 + 0));
    *(g + 1) = _Tps(*(s0 + 1));

    for ( int x = 2; x < w - 2; x += 2 ) {
      g[x] = ea_green<_Tps>(
          s0[x - 1], s0[x + 1], sm1[x], sp1[x],
          s0[x], s0[x - 2], s0[x + 2], sm2[x], sp2[x]);
      g[x + 1] = _Tps(s0[x + 1]);
    }

    const int x = w - 2;
    g[x] = ea_green<_Tps>(
        s0[x - 1], s0[x + 1], sm1[x], sp1[x],
        s0[x], s0[x - 2], s0[x - 2], sm2[x], sp2[x]);
    g[x + 1] = _Tps(s0[x + 1]);
  }
}

template<typename _Tp1, typename _Tp2, typename _Tps>
static ALWAYS_INLINE
void ea_write_bgr_pair(bool SwapRB, bool GOnEven,
    const _Tp1 * __restrict sm1,
    const _Tp1 * __restrict s0,
    const _Tp1 * __restrict sp1,
    const _Tps * __restrict gm1,
    const _Tps * __restrict g0,
    const _Tps * __restrict gp1,
    _Tp2 * __restrict dst,
    int x, int xm, int xp)
{
  const int idx_c1 = SwapRB ? 0 : 2;
  const int idx_c2 = SwapRB ? 2 : 0;

  if ( !GOnEven ) {
    // Row of the form [C1  G]
    const _Tps c1_0 = _Tps(s0[x]);
    const _Tps c2_0 = ea_chroma_diag<_Tps>(sm1[xm], sm1[x + 1], sp1[xm], sp1[x + 1],
        g0[x], gm1[xm], gm1[x + 1], gp1[xm], gp1[x + 1]);

    const _Tps c1_1 = ea_chroma_hv<_Tps>(c1_0, s0[xp], g0[x + 1], g0[x], g0[xp]);
    const _Tps c2_1 = ea_chroma_hv<_Tps>(sm1[x + 1], sp1[x + 1], g0[x + 1], gm1[x + 1], gp1[x + 1]);

    dst[3 * x + idx_c1]     = cv::saturate_cast<_Tp2>(c1_0);
    dst[3 * x + 1]          = cv::saturate_cast<_Tp2>(g0[x]);
    dst[3 * x + idx_c2]     = cv::saturate_cast<_Tp2>(c2_0);
    dst[3 * x + 3 + idx_c1] = cv::saturate_cast<_Tp2>(c1_1);
    dst[3 * x + 4]          = cv::saturate_cast<_Tp2>(g0[x + 1]);
    dst[3 * x + 3 + idx_c2] = cv::saturate_cast<_Tp2>(c2_1);
  }
  else {
    // Row of the form [G  C1]
    const _Tps c1_1 = _Tps(s0[x + 1]);
    const _Tps c1_0 = ea_chroma_hv<_Tps>(s0[xm], c1_1, g0[x], g0[xm], g0[x + 1]);
    const _Tps c2_0 = ea_chroma_hv<_Tps>(sm1[x], sp1[x], g0[x], gm1[x], gp1[x]);

    const _Tps c2_1 = ea_chroma_diag<_Tps>(sm1[x], sm1[xp], sp1[x], sp1[xp],
        g0[x + 1], gm1[x], gm1[xp], gp1[x], gp1[xp]);

    dst[3 * x + idx_c1]     = cv::saturate_cast<_Tp2>(c1_0);
    dst[3 * x + 1]          = cv::saturate_cast<_Tp2>(g0[x]);
    dst[3 * x + idx_c2]     = cv::saturate_cast<_Tp2>(c2_0);
    dst[3 * x + 3 + idx_c1] = cv::saturate_cast<_Tp2>(c1_1);
    dst[3 * x + 4]          = cv::saturate_cast<_Tp2>(g0[x + 1]);
    dst[3 * x + 3 + idx_c2] = cv::saturate_cast<_Tp2>(c2_1);
  }
}

template<typename _Tp1, typename _Tp2, typename _Tps>
static ALWAYS_INLINE
void ea_assemble_bgr_row(bool SwapRB, bool GOnEven,
    const _Tp1 * __restrict sm1,
    const _Tp1 * __restrict s0,
    const _Tp1 * __restrict sp1,
    const _Tps * __restrict gm1,
    const _Tps * __restrict g0,
    const _Tps * __restrict gp1,
    _Tp2 * __restrict dst,
    int w)
{
  ea_write_bgr_pair<_Tp1, _Tp2, _Tps>(SwapRB, GOnEven, sm1, s0, sp1, gm1, g0, gp1, dst, 0, 1, 2);

  for ( int x = 2; x < w - 2; x += 2 ) {
    ea_write_bgr_pair<_Tp1, _Tp2, _Tps>(SwapRB, GOnEven, sm1, s0, sp1, gm1, g0, gp1, dst, x, x - 1, x + 2);
  }

  ea_write_bgr_pair<_Tp1, _Tp2, _Tps>(SwapRB, GOnEven, sm1, s0, sp1, gm1, g0, gp1, dst, w - 2, w - 3, w - 2);
}

static ALWAYS_INLINE
int ea_mirror_y(int y, int h)
{
  if ( y < 0 ) {
    return -y;
  }
  if ( y >= h ) {
    return 2 * h - 2 - y;
  }
  return y;
}

template<typename _Tp1, typename _Tp2>
static void _debayer_ea_kernel(const cv::Mat & src, cv::Mat & dst,
    bool EvenRowIsR, bool EvenRowGOnEven)
{
  using _Tpmax = std::common_type_t<_Tp1, _Tp2>;
  using _Tps = std::conditional_t<std::is_floating_point_v<_Tpmax>, _Tpmax, int>;

  const int w = src.cols;
  const int h = src.rows;
  const uint8_t * const src_base = src.data;
  uint8_t * const dst_base = dst.data;
  const size_t src_stride = src.step;
  const size_t dst_stride = dst.step;

  parallel_for(0, h, [=](const auto & range) {
    const int y_start = rbegin(range);
    const int y_end = rend(range);
    if ( y_start >= y_end ) {
      return;
    }

    cv::AutoBuffer<_Tps, 8192> g_ring(3 * w);
    _Tps * g_rows[3] = { g_ring.data(), g_ring.data() + w, g_ring.data() + 2 * w };

    for ( int init_y = y_start - 1; init_y <= y_start; ++init_y ) {
      const int cy = ea_mirror_y(init_y, h);
      const _Tp1 * sm2 = (const _Tp1 *)(src_base + ea_mirror_y(cy - 2, h) * src_stride);
      const _Tp1 * sm1 = (const _Tp1 *)(src_base + ea_mirror_y(cy - 1, h) * src_stride);
      const _Tp1 * s0  = (const _Tp1 *)(src_base + cy * src_stride);
      const _Tp1 * sp1 = (const _Tp1 *)(src_base + ea_mirror_y(cy + 1, h) * src_stride);
      const _Tp1 * sp2 = (const _Tp1 *)(src_base + ea_mirror_y(cy + 2, h) * src_stride);
      _Tps * g_dst = g_rows[(init_y - (y_start - 1)) % 3];

      if ( (cy & 1) == 0 ) {
        ea_compute_g_row<_Tp1, _Tps>(EvenRowGOnEven, sm2, sm1, s0, sp1, sp2, g_dst, w);
      }
      else {
        ea_compute_g_row<_Tp1, _Tps>(!EvenRowGOnEven, sm2, sm1, s0, sp1, sp2, g_dst, w);
      }
    }

    int ring_idx = 2;

    for ( int y = y_start; y < y_end; ++y ) {
      const int cy_next = ea_mirror_y(y + 1, h);
      const _Tp1 * n_sm2 = (const _Tp1 *)(src_base + ea_mirror_y(cy_next - 2, h) * src_stride);
      const _Tp1 * n_sm1 = (const _Tp1 *)(src_base + ea_mirror_y(cy_next - 1, h) * src_stride);
      const _Tp1 * n_s0  = (const _Tp1 *)(src_base + cy_next * src_stride);
      const _Tp1 * n_sp1 = (const _Tp1 *)(src_base + ea_mirror_y(cy_next + 1, h) * src_stride);
      const _Tp1 * n_sp2 = (const _Tp1 *)(src_base + ea_mirror_y(cy_next + 2, h) * src_stride);

      if ( (cy_next & 1) == 0 ) {
        ea_compute_g_row<_Tp1, _Tps>(EvenRowGOnEven, n_sm2, n_sm1, n_s0, n_sp1, n_sp2, g_rows[ring_idx], w);
      }
      else {
        ea_compute_g_row<_Tp1, _Tps>(!EvenRowGOnEven, n_sm2, n_sm1, n_s0, n_sp1, n_sp2, g_rows[ring_idx], w);
      }

      const _Tps * gm1 = g_rows[(ring_idx + 1) % 3];
      const _Tps * g0  = g_rows[(ring_idx + 2) % 3];
      const _Tps * gp1 = g_rows[ring_idx];

      const _Tp1 * sm1 = (const _Tp1 *)(src_base + ea_mirror_y(y - 1, h) * src_stride);
      const _Tp1 * s0  = (const _Tp1 *)(src_base + y * src_stride);
      const _Tp1 * sp1 = (const _Tp1 *)(src_base + ea_mirror_y(y + 1, h) * src_stride);
      _Tp2 * dstp = (_Tp2 *)(dst_base + y * dst_stride);

      if ( (y & 1) == 0 ) {
        ea_assemble_bgr_row<_Tp1, _Tp2, _Tps>(!EvenRowIsR, EvenRowGOnEven, sm1, s0, sp1, gm1, g0, gp1, dstp, w);
      }
      else {
        ea_assemble_bgr_row<_Tp1, _Tp2, _Tps>(EvenRowIsR, !EvenRowGOnEven, sm1, s0, sp1, gm1, g0, gp1, dstp, w);
      }

      ring_idx = (ring_idx + 1) % 3;
    }
  });
}

template<typename _Tp1, typename _Tp2>
static bool _debayer_ea(cv::InputArray _src, cv::OutputArray _dst, enum COLORID colorid)
{
  const cv::Mat src = _src.getMat();
  _dst.create(src.size(), CV_MAKETYPE(cv::DataType<_Tp2>::depth, 3));
  cv::Mat & dst = _dst.getMatRef();

  switch ( colorid ) {
    case COLORID_BAYER_MYYC:
    case COLORID_BAYER_RGGB:
      _debayer_ea_kernel<_Tp1, _Tp2>(src, dst, true, false);
      return true;
    case COLORID_BAYER_YMCY:
    case COLORID_BAYER_GRBG:
      _debayer_ea_kernel<_Tp1, _Tp2>(src, dst, true, true);
      return true;
    case COLORID_BAYER_YCMY:
    case COLORID_BAYER_GBRG:
      _debayer_ea_kernel<_Tp1, _Tp2>(src, dst, false, true);
      return true;
    case COLORID_BAYER_CYYM:
    case COLORID_BAYER_BGGR:
      _debayer_ea_kernel<_Tp1, _Tp2>(src, dst, false, false);
      return true;
    default:
      CF_ERROR("Not supported colorid=%d (%s) for debayer", (int)colorid, toCString(colorid));
      return false;
  }
}

bool debayer_ea(cv::InputArray src, cv::OutputArray dst, enum COLORID colorid, int ddepth)
{
  INSTRUMENT_REGION("");

  if ( !is_bayer_pattern(colorid) ) {
    CF_ERROR("colorid=%d (%s) is not bayer pattern", (int)colorid, toCString(colorid));
    return false;
  }

  if ( (src.cols() & 0x1) || (src.rows() & 0x1) || src.channels() != 1 ) {
    CF_ERROR("Can not make debayer for uneven image size %dx%dx%d",
        src.cols(), src.rows(), src.channels());
    return false;
  }

  if ( dst.fixedType() ) {
    ddepth = dst.depth();
    if ( dst.channels() != 3 ) {
      CF_ERROR("Output image channels=%d. Must be 3", dst.channels());
      return false;
    }
  }
  else if ( ddepth < 0 ) {
    ddepth = src.depth();
  }

  CV_DISPATCH2(src.depth(), ddepth, _debayer_ea, src, dst, colorid);

  CF_ERROR("Not supported combination of src.depth()=%d and ddepth=%d",
      src.depth(), ddepth);
  return false;
}

