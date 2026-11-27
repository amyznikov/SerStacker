/*
 * debayer_planes_ea.cc
 *
 *  Created on: Nov 27, 2026
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

// In a 4-channel image each pixel (x, y) corresponds to a 2x2 cell from the original Bayer pattern.
// Vec4T channel indices: TopLeft=0, TopRight=1, BotLeft=2, BotRight=3.
template<typename _Tp1, typename _Tps>
static ALWAYS_INLINE
void ea_compute_g_pair_4ch(bool CTopLeft,
    const cv::Vec<_Tp1, 4> * __restrict s0,
    const cv::Vec<_Tp1, 4> * __restrict s1,
    const cv::Vec<_Tp1, 4> * __restrict s2,
    _Tps * __restrict g_top,
    _Tps * __restrict g_bot,
    int x0, int x1, int x2)
{
  const _Tp1 * p01 = s0[x1].val;
  const _Tp1 * p10 = s1[x0].val;
  const _Tp1 * p11 = s1[x1].val;
  const _Tp1 * p12 = s1[x2].val;
  const _Tp1 * p21 = s2[x1].val;

  if ( CTopLeft ) {
    // 0 = C_top, 1 = G1, 2 = G2, 3 = C_bot
    g_top[2 * x1 + 0] = ea_green<_Tps>(*(p10 + 1), *(p11 + 1), *(p01 + 2), *(p11 + 2),
        *(p11 + 0), *(p10 + 0), *(p12 + 0), *(p01 + 0), *(p21 + 0));
    g_top[2 * x1 + 1] = _Tps(*(p11 + 1));

    g_bot[2 * x1 + 0] = _Tps(*(p11 + 2));
    g_bot[2 * x1 + 1] = ea_green<_Tps>(*(p11 + 2), *(p12 + 2), *(p11 + 1), *(p21 + 1),
        *(p11 + 3), *(p10 + 3), *(p12 + 3), *(p01 + 3), *(p21 + 3));
  }
  else {
    // 0 = G1, 1 = C_top, 2 = C_bot, 3 = G2
    g_top[2 * x1 + 0] = _Tps(*(p11 + 0));
    g_top[2 * x1 + 1] = ea_green<_Tps>(*(p11 + 0), *(p12 + 0), *(p01 + 3), *(p11 + 3),
        *(p11 + 1), *(p10 + 1), *(p12 + 1), *(p01 + 1), *(p21 + 1));

    g_bot[2 * x1 + 0] = ea_green<_Tps>(*(p10 + 3), *(p11 + 3), *(p11 + 0), *(p21 + 0),
        *(p11 + 2), *(p10 + 2), *(p12 + 2), *(p01 + 2), *(p21 + 2));
    g_bot[2 * x1 + 1] = _Tps(*(p11 + 3));
  }
}

template<typename _Tp1, typename _Tps>
static ALWAYS_INLINE
void ea_compute_g_rows_4ch(bool CTopLeft,
    const cv::Vec<_Tp1, 4> * __restrict s0,
    const cv::Vec<_Tp1, 4> * __restrict s1,
    const cv::Vec<_Tp1, 4> * __restrict s2,
    _Tps * __restrict g_top,
    _Tps * __restrict g_bot,
    int cols)
{
  ea_compute_g_pair_4ch<_Tp1, _Tps>(CTopLeft, s0, s1, s2, g_top, g_bot, 0, 0, cols > 1 ? 1 : 0);

  for ( int x = 1; x < cols - 1; ++x ) {
    ea_compute_g_pair_4ch<_Tp1, _Tps>(CTopLeft, s0, s1, s2, g_top, g_bot, x - 1, x, x + 1);
  }

  if ( cols > 1 ) {
    ea_compute_g_pair_4ch<_Tp1, _Tps>(CTopLeft, s0, s1, s2, g_top, g_bot, cols - 2, cols - 1, cols - 1);
  }
}


template<typename _Tp1, typename _Tp2, typename _Tps>
static ALWAYS_INLINE
void ea_write_2x2_4ch(bool TopHasR, bool CTopLeft,
    const cv::Vec<_Tp1, 4> * __restrict s0,
    const cv::Vec<_Tp1, 4> * __restrict s1,
    const cv::Vec<_Tp1, 4> * __restrict s2,
    const _Tps * __restrict g_prev_bot,
    const _Tps * __restrict g_curr_top,
    const _Tps * __restrict g_curr_bot,
    const _Tps * __restrict g_next_top,
    _Tp2 * __restrict d0,
    _Tp2 * __restrict d1,
    int x0, int x1, int x2)
{
  const int idx_ct = TopHasR ? 2 : 0;
  const int idx_cb = TopHasR ? 0 : 2;

  const _Tp1 * p00 = s0[x0].val; const _Tp1 * p01 = s0[x1].val; const _Tp1 * p02 = s0[x2].val;
  const _Tp1 * p10 = s1[x0].val; const _Tp1 * p11 = s1[x1].val; const _Tp1 * p12 = s1[x2].val;
  const _Tp1 * p20 = s2[x0].val; const _Tp1 * p21 = s2[x1].val; const _Tp1 * p22 = s2[x2].val;

  const int xl = 2 * x1 + 0;
  const int xr = 2 * x1 + 1;
  const int x_pr = 2 * x0 + 1;
  const int x_nl = 2 * x2 + 0;

  _Tps ct_tl, cb_tl, ct_tr, cb_tr, ct_bl, cb_bl, ct_br, cb_br;

  if ( CTopLeft ) {
    // 0 = C_top, 1 = G1, 2 = G2, 3 = C_bot
    ct_tl = _Tps(*(p11 + 0));
    cb_br = _Tps(*(p11 + 3));

    ct_tr = ea_chroma_hv<_Tps>(ct_tl, *(p12 + 0), g_curr_top[xr], g_curr_top[xl], g_curr_top[x_nl]);
    cb_tr = ea_chroma_hv<_Tps>(*(p01 + 3), cb_br, g_curr_top[xr], g_prev_bot[xr], g_curr_bot[xr]);

    ct_bl = ea_chroma_hv<_Tps>(ct_tl, *(p21 + 0), g_curr_bot[xl], g_curr_top[xl], g_next_top[xl]);
    cb_bl = ea_chroma_hv<_Tps>(*(p10 + 3), cb_br, g_curr_bot[xl], g_curr_bot[x_pr], g_curr_bot[xr]);

    cb_tl = ea_chroma_diag<_Tps>(*(p00 + 3), *(p01 + 3), *(p10 + 3), cb_br,
        g_curr_top[xl], g_prev_bot[x_pr], g_prev_bot[xr], g_curr_bot[x_pr], g_curr_bot[xr]);
    ct_br = ea_chroma_diag<_Tps>(
        ct_tl, *(p12 + 0), *(p21 + 0), *(p22 + 0),
        g_curr_bot[xr], g_curr_top[xl], g_curr_top[x_nl], g_next_top[xl], g_next_top[x_nl]);
  }
  else {
    // 0 = G1, 1 = C_top, 2 = C_bot, 3 = G2
    ct_tr = _Tps(*(p11 + 1));
    cb_bl = _Tps(*(p11 + 2));

    ct_tl = ea_chroma_hv<_Tps>(*(p10 + 1), ct_tr, g_curr_top[xl], g_curr_top[x_pr], g_curr_top[xr]);
    cb_tl = ea_chroma_hv<_Tps>(*(p01 + 2), cb_bl, g_curr_top[xl], g_prev_bot[xl], g_curr_bot[xl]);

    ct_br = ea_chroma_hv<_Tps>(ct_tr, *(p21 + 1), g_curr_bot[xr], g_curr_top[xr], g_next_top[xr]);
    cb_br = ea_chroma_hv<_Tps>(cb_bl, *(p12 + 2), g_curr_bot[xr], g_curr_bot[xl], g_curr_bot[x_nl]);

    cb_tr = ea_chroma_diag<_Tps>(*(p01 + 2), *(p02 + 2), cb_bl, *(p12 + 2),
        g_curr_top[xr], g_prev_bot[xl], g_prev_bot[x_nl], g_curr_bot[xl], g_curr_bot[x_nl]);
    ct_bl = ea_chroma_diag<_Tps>(*(p10 + 1), ct_tr, *(p20 + 1), *(p21 + 1),
        g_curr_bot[xl], g_curr_top[x_pr], g_curr_top[xr], g_next_top[x_pr], g_next_top[xr]);
  }

  d0[6 * x1 + idx_ct]     = cv::saturate_cast<_Tp2>(ct_tl);
  d0[6 * x1 + 1]          = cv::saturate_cast<_Tp2>(g_curr_top[xl]);
  d0[6 * x1 + idx_cb]     = cv::saturate_cast<_Tp2>(cb_tl);

  d0[6 * x1 + 3 + idx_ct] = cv::saturate_cast<_Tp2>(ct_tr);
  d0[6 * x1 + 4]          = cv::saturate_cast<_Tp2>(g_curr_top[xr]);
  d0[6 * x1 + 3 + idx_cb] = cv::saturate_cast<_Tp2>(cb_tr);

  d1[6 * x1 + idx_ct]     = cv::saturate_cast<_Tp2>(ct_bl);
  d1[6 * x1 + 1]          = cv::saturate_cast<_Tp2>(g_curr_bot[xl]);
  d1[6 * x1 + idx_cb]     = cv::saturate_cast<_Tp2>(cb_bl);

  d1[6 * x1 + 3 + idx_ct] = cv::saturate_cast<_Tp2>(ct_br);
  d1[6 * x1 + 4]          = cv::saturate_cast<_Tp2>(g_curr_bot[xr]);
  d1[6 * x1 + 3 + idx_cb] = cv::saturate_cast<_Tp2>(cb_br);
}

template<typename _Tp1, typename _Tp2>
static void _interpolate_bayer_planes_ea_kernel(const cv::Mat & src, cv::Mat & dst,
    bool TopHasR, bool CTopLeft)
{
  using Vec4T = cv::Vec<_Tp1, 4>;
  using _Tpmax = std::common_type_t<_Tp1, _Tp2>;
  using _Tps = std::conditional_t<std::is_floating_point_v<_Tpmax>, _Tpmax, int>;

  const int src_rows = src.rows;
  const int src_cols = src.cols;
  const int dst_w = 2 * src_cols;

  const uint8_t * const src_base = src.data;
  uint8_t * const dst_base = dst.data;
  const size_t src_stride = src.step;
  const size_t dst_stride = dst.step;

  parallel_for(0, src_rows, [=](const auto & range) {

    const int y_start = rbegin(range), y_end = rend(range);
    if ( y_start >= y_end ) {
      return;
    }

    cv::AutoBuffer<_Tps, 16384> g_ring(6 * dst_w);
    _Tps * g_top[3] = { g_ring.data() + 0 * dst_w, g_ring.data() + 2 * dst_w, g_ring.data() + 4 * dst_w };
    _Tps * g_bot[3] = { g_ring.data() + 1 * dst_w, g_ring.data() + 3 * dst_w, g_ring.data() + 5 * dst_w };

    for ( int init_y = y_start - 1; init_y <= y_start; ++init_y ) {
      const int y1 = std::max(0, std::min(src_rows - 1, init_y));
      const int y0 = std::max(0, y1 - 1);
      const int y2 = std::min(src_rows - 1, y1 + 1);
      const int slot = init_y - (y_start - 1);

      ea_compute_g_rows_4ch<_Tp1, _Tps>(CTopLeft,
          (const Vec4T *)(src_base + y0 * src_stride),
          (const Vec4T *)(src_base + y1 * src_stride),
          (const Vec4T *)(src_base + y2 * src_stride),
          g_top[slot], g_bot[slot], src_cols);
    }

    int cur_slot = 1;

    for ( int y1 = y_start; y1 < y_end; ++y1 ) {
      const int next_slot = (cur_slot + 1) % 3;
      const int prev_slot = (cur_slot + 2) % 3;

      const int ny1 = std::min(src_rows - 1, y1 + 1);
      const int ny0 = std::max(0, ny1 - 1);
      const int ny2 = std::min(src_rows - 1, ny1 + 1);

      ea_compute_g_rows_4ch<_Tp1, _Tps>(CTopLeft,
          (const Vec4T *)(src_base + ny0 * src_stride),
          (const Vec4T *)(src_base + ny1 * src_stride),
          (const Vec4T *)(src_base + ny2 * src_stride),
          g_top[next_slot], g_bot[next_slot], src_cols);

      const int y0 = std::max(0, y1 - 1);
      const int y2 = std::min(src_rows - 1, y1 + 1);

      const Vec4T * s0 = (const Vec4T *)(src_base + y0 * src_stride);
      const Vec4T * s1 = (const Vec4T *)(src_base + y1 * src_stride);
      const Vec4T * s2 = (const Vec4T *)(src_base + y2 * src_stride);

      _Tp2 * d0 = (_Tp2 *)(dst_base + (2 * y1 + 0) * dst_stride);
      _Tp2 * d1 = (_Tp2 *)(dst_base + (2 * y1 + 1) * dst_stride);

      ea_write_2x2_4ch<_Tp1, _Tp2, _Tps>(TopHasR, CTopLeft,
          s0, s1, s2, g_bot[prev_slot], g_top[cur_slot], g_bot[cur_slot], g_top[next_slot],
          d0, d1, 0, 0, src_cols > 1 ? 1 : 0);

      for ( int x1 = 1; x1 < src_cols - 1; ++x1 ) {
        ea_write_2x2_4ch<_Tp1, _Tp2, _Tps>(TopHasR, CTopLeft,
            s0, s1, s2, g_bot[prev_slot], g_top[cur_slot], g_bot[cur_slot], g_top[next_slot],
            d0, d1, x1 - 1, x1, x1 + 1);
      }

      if ( src_cols > 1 ) {
        ea_write_2x2_4ch<_Tp1, _Tp2, _Tps>(TopHasR, CTopLeft,
            s0, s1, s2, g_bot[prev_slot], g_top[cur_slot], g_bot[cur_slot], g_top[next_slot],
            d0, d1, src_cols - 2, src_cols - 1, src_cols - 1);
      }

      cur_slot = next_slot;
    }
  });
}

template<typename _Tp1, typename _Tp2>
static bool _interpolate_bayer_planes_ea(cv::InputArray _src, cv::OutputArray _dst, enum COLORID colorid)
{
  if ( _src.rows() <= 0 || _src.cols() <= 0 ) {
    CF_ERROR("Invalid source image size %dx%d", _src.cols(), _src.rows());
    return false;
  }

  const cv::Mat src = _src.getMat();
  _dst.create(2 * src.rows, 2 * src.cols, CV_MAKETYPE(cv::DataType<_Tp2>::depth, 3));
  cv::Mat & dst = _dst.getMatRef();

  switch ( colorid ) {
    case COLORID_BAYER_MYYC:
    case COLORID_BAYER_RGGB:
      _interpolate_bayer_planes_ea_kernel<_Tp1, _Tp2>(src, dst, true, true);
      return true;
    case COLORID_BAYER_YMCY:
    case COLORID_BAYER_GRBG:
      _interpolate_bayer_planes_ea_kernel<_Tp1, _Tp2>(src, dst, true, false);
      return true;
    case COLORID_BAYER_YCMY:
    case COLORID_BAYER_GBRG:
      _interpolate_bayer_planes_ea_kernel<_Tp1, _Tp2>(src, dst, false, false);
      return true;
    case COLORID_BAYER_CYYM:
    case COLORID_BAYER_BGGR:
      _interpolate_bayer_planes_ea_kernel<_Tp1, _Tp2>(src, dst, false, true);
      return true;
    default:
      CF_ERROR("Invalid or not supported colorid = %d (%s)", colorid, toCString(colorid));
      return false;
  }
}


bool interpolate_bayer_planes_ea(cv::InputArray src, cv::OutputArray dst, enum COLORID colorid, int ddepth)
{
  INSTRUMENT_REGION("");

  if ( src.channels() != 4 ) {
    CF_ERROR("Invalid number of channel=%d in input image. Must be 4", src.channels());
    return false;
  }

  if ( dst.fixedType() ) {
    ddepth = dst.depth();
    if ( dst.channels() != 3 ) {
      CF_ERROR("Invalid argument: output number of channels must be 3");
      return false;
    }
  }
  else if ( ddepth < 0 ) {
    ddepth = src.depth();
  }

  CV_DISPATCH2(src.depth(), ddepth, _interpolate_bayer_planes_ea, src, dst, colorid);

  CF_ERROR("Not supported combination of src.depth()=%d and ddepth=%d",
      src.depth(), ddepth);
  return false;
}
