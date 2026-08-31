/*
 * debayer.cc
 *
 *  Created on: Jul 31, 2020
 *      Author: amyznikov
 */

#include "debayer.h"
#include <core/proc/run-loop.h>
#include <core/proc/pixtype.h>
#include <core/proc/reduce_channels.h>
#include <core/ssprintf.h>
#include <core/debug.h>

template<>
const c_enum_member* members_of<COLORID>()
{
  static const c_enum_member members[] = {
      { COLORID_MONO, "MONO", "" },
      { COLORID_BAYER_RGGB, "BAYER_RGGB", "" },
      { COLORID_BAYER_GRBG, "BAYER_GRBG", "" },
      { COLORID_BAYER_GBRG, "BAYER_GBRG", "" },
      { COLORID_BAYER_BGGR, "BAYER_BGGR", "" },
      { COLORID_BAYER_CYYM, "BAYER_CYYM", "" },
      { COLORID_BAYER_YCMY, "BAYER_YCMY", "" },
      { COLORID_BAYER_YMCY, "BAYER_YMCY", "" },
      { COLORID_BAYER_MYYC, "BAYER_MYYC", "" },
      { COLORID_RGB, "RGB", "" },
      { COLORID_BGR, "BGR", "" },
      { COLORID_BGRA, "BGRA", "BGR + Binary Mask" },
      // { COLORID_BGRW, "BGRW", "BGR + Analog Mask" },
      { COLORID_OPTFLOW, "OPTFLOW", "" },
      { COLORID_UNKNOWN },
  };
  return members;
}

template<>
const c_enum_member * members_of<DEBAYER_ALGORITHM>()
{
  static const c_enum_member members[] = {
      {DEBAYER_DISABLE, "DISABLE", "DEBAYER_DISABLE: Don't debayer"},
      {DEBAYER_NN,    "NN",     "DEBAYER_NN: OpenCV nearest-neighboor interpolation with cv::demosaicing()"},
      {DEBAYER_NN2,   "NN2",    "DEBAYER_NN2: SerStacker nearest-neighboor interpolation with nninterpolate()"},
      {DEBAYER_EA,    "EA",     "DEBAYER_EA: OpenCV EA (edge aware) interpolation with cv::demosaicing()"},
      {DEBAYER_VNG,   "VNG",    "DEBAYER_VNG: OpenCV VNG interpolation with cv::demosaicing()"},
      {DEBAYER_SP,    "SP",     "2x2 super-pixel pixel binning"},
      {DEBAYER_MATRIX,"MATRIX", "DEBAYER_MATRIX: Don't debayer, create colored bayer matrix image instead"},
      {DEBAYER_PLANE_0,"PLANE_0", "Extract bayer plane 0 "},
      {DEBAYER_PLANE_1,"PLANE_1", "Extract bayer plane 1 "},
      {DEBAYER_PLANE_2,"PLANE_2", "Extract bayer plane 2 "},
      {DEBAYER_PLANE_3,"PLANE_3", "Extract bayer plane 3 "},
      {DEBAYER_AVGBP,  "AVGBP", "Average 4 bayer planes into single-chanel image of twice less resolution"},


      {DEBAYER_NN, } // must  be last
  };
  return members;
}

static DEBAYER_ALGORITHM g_default_debayer_algorithm = DEBAYER_NN2;

void set_default_debayer_algorithm(DEBAYER_ALGORITHM algo)
{
  if( algo == DEBAYER_DEFAULT ) {
    algo = DEBAYER_NN2;
  }
  g_default_debayer_algorithm = algo;
}

DEBAYER_ALGORITHM default_debayer_algorithm()
{
  return g_default_debayer_algorithm;
}

/** @brief
 * Return true if colorid is one from
 * known bayer patterns (COLORID_BAYER_XXXX)
 */
bool is_bayer_pattern(enum COLORID colorid)
{
  switch ( colorid ) {
  case COLORID_BAYER_RGGB :
  case COLORID_BAYER_GRBG :
  case COLORID_BAYER_GBRG :
  case COLORID_BAYER_BGGR :
  case COLORID_BAYER_CYYM :
  case COLORID_BAYER_YCMY :
  case COLORID_BAYER_YMCY :
  case COLORID_BAYER_MYYC :
    return true;
  }
  return false;
}

template<typename _Tp1, typename _Tp2>
static bool _extract_bayer_planes(cv::InputArray _src, cv::OutputArray _dst)
{
  if( (_src.cols() & 0x1) || (_src.rows() & 0x1) || _src.channels() != 1 ) {
    CF_ERROR("Can not make debayer for uneven image size %dx%dx%d",
        _src.cols(), _src.rows(), _src.channels());
    return false;
  }

  const int rows4 = _src.rows() / 2;
  const int cols4 = _src.cols() / 2;
  const int ddepth = cv::DataType<_Tp2>::depth;

  const cv::Mat bayer_image = _src.getMat();
  const uint8_t * bayer_base = (const uint8_t * )bayer_image.ptr();
  const size_t bayer_stride = bayer_image.step;

  _dst.create(rows4, cols4, CV_MAKETYPE(ddepth, 4));
  cv::Mat & planes = _dst.getMatRef();
  uint8_t * planes_base = (uint8_t * )planes.ptr();
  const size_t planes_stride = planes.step;

  parallel_for(0, rows4, [=](const auto & range) {

    for( int y = rbegin(range); y < rend(range); ++y ) {

      const _Tp1 * src0 = (const _Tp1 *)(bayer_base + (2 * y + 0) * bayer_stride);
      const _Tp1 * src1 = (const _Tp1 *)(bayer_base + (2 * y + 1) * bayer_stride);
      _Tp2 * __restrict dstp = (_Tp2 * )(planes_base + y * planes_stride);

      for( int x = 0; x < cols4; ++x, src0 += 2, src1 += 2, dstp += 4 ) {
        dstp[0] = cv::saturate_cast<_Tp2>(src0[0]);
        dstp[1] = cv::saturate_cast<_Tp2>(src0[1]);
        dstp[2] = cv::saturate_cast<_Tp2>(src1[0]);
        dstp[3] = cv::saturate_cast<_Tp2>(src1[1]);
      }
    }
  });

  return true;
}

/** @brief
 * Extract src into 4-channel dst matrix with 4 bayer planes ordered the same as src bayer pattern
 * The output size of dst is twice smaller than src
 */
bool extract_bayer_planes(cv::InputArray src, cv::OutputArray dst)
{
  INSTRUMENT_REGION("");

  const int ddepth = dst.fixedType() ? dst.depth() : src.depth();
  CV_DISPATCH2(src.depth(),ddepth, _extract_bayer_planes, src, dst);

  return false;
}


/** @brief
 * Extract single bayer plane with given index [0..3] into 1-channel dst matrix
 * The output size of dst is roi size or twice smaller than src if roi is empty
 */
template<typename _Tp1, typename _Tp2>
static bool _extract_bayer_plane(cv::InputArray _src, cv::OutputArray _dst, int index,
    const cv::Rect& _roi)
{
  if( _src.channels() != 1 ) {
    CF_ERROR("Source image must have 1 channel, got %d", _src.channels());
    return false;
  }

  // Coordinates ROI must be even to not destroy Bayer pattern
  if ( !_roi.empty() ) {
    if( (_roi.x & 0x1) || (_roi.y & 0x1) || (_roi.width & 0x1) || (_roi.height & 0x1) ) {
      CF_ERROR("ROI bounds and sizes must be even integers. Got x:%d, y:%d, w:%d, h:%d",
               _roi.x, _roi.y, _roi.width, _roi.height);
      return false;
    }
  }

  const cv::Rect img_rect(0, 0, _src.cols(), _src.rows());
  const cv::Rect roi = _roi.empty() ? img_rect : _roi & img_rect;
  if ( roi.empty() ) {
    CF_ERROR("Requested ROI is out of image bounds");
    return false;
  }

  const cv::Mat bayer_image = _src.getMat();

  const int dst_rows = roi.height / 2;
  const int dst_cols = roi.width / 2;
  const int ddepth = cv::DataType<_Tp2>::depth;

  _dst.create(dst_rows, dst_cols, CV_MAKETYPE(ddepth, 1));
  cv::Mat & plane = _dst.getMatRef();

  const uint8_t * bayer_base = (const uint8_t * )bayer_image.ptr() + roi.y * bayer_image.step + roi.x * sizeof(_Tp1);
  const size_t bayer_stride = bayer_image.step;

  uint8_t * plane_base = (uint8_t * )plane.ptr();
  const size_t plane_stride = plane.step;

  // 0 -> row:0, col:0 | 1 -> row:0, col:1 | 2 -> row:1, col:0 | 3 -> row:1, col:1
  const int row_offset = (index >> 1) & 1;
  const int col_offset = index & 1;

  parallel_for(0, dst_rows, [=](const auto & range) {
    for( int y = rbegin(range); y < rend(range); ++y ) {
      const _Tp1 * src = (const _Tp1 *)(bayer_base + (2 * y + row_offset) * bayer_stride) + col_offset;
      _Tp2 * __restrict dstp = (_Tp2 * )(plane_base + y * plane_stride);
      for( int x = 0; x < dst_cols; ++x, src += 2, ++dstp ) {
        *dstp = cv::saturate_cast<_Tp2>(*src);
      }
    }
  });

  return true;
}

/** @brief
 * Extract single bayer plane with given index [0..3] into 1-channel dst matrix
 * The output size of dst is roi size or twice smaller than src if roi is empty
 */
bool extract_bayer_plane(cv::InputArray src, cv::OutputArray dst, int index, const cv::Rect &roi)
{
  INSTRUMENT_REGION("");
  const int ddepth = dst.fixedType() ? dst.depth() : src.depth();
  CV_DISPATCH2(src.depth(), ddepth, _extract_bayer_plane, src, dst, index, roi);
  return false;
}


//template<typename _Tp1, typename _Tp2>
//static bool _average_bayer_planes(cv::InputArray _src, cv::OutputArray _dst)
//{
//  using _Tpmax = std::common_type_t<_Tp1, _Tp2>;
//  using _Tps = std::conditional_t<std::is_floating_point_v<_Tpmax>, _Tpmax, int>;
//  constexpr _Tps c1 = std::is_integral_v<_Tps> ? 2 : 0;
//
//  if( (_src.cols() & 0x1) || (_src.rows() & 0x1) || _src.channels() != 1 ) {
//    CF_ERROR("Can not average bayer planes for uneven or multi-channel image size %dx%dx%d",
//        _src.cols(), _src.rows(), _src.channels());
//    return false;
//  }
//
//  const int dst_rows = _src.rows() / 2;
//  const int dst_cols = _src.cols() / 2;
//  const int ddepth = cv::DataType<_Tp2>::depth;
//
//  const cv::Mat bayer_image = _src.getMat();
//  const uint8_t * bayer_base = (const uint8_t * )bayer_image.ptr();
//  const size_t bayer_stride = bayer_image.step;
//
//  _dst.create(dst_rows, dst_cols, CV_MAKETYPE(ddepth, 1));
//  cv::Mat & dst = _dst.getMatRef();
//  uint8_t * dst_base = (uint8_t * )dst.ptr();
//  const size_t dst_stride = dst.step;
//
//  // 2x2 block average
//  parallel_for(0, dst_rows, [=](const auto & range) {
//    const int y0 = rbegin(range);
//    const uint8_t * src0_base = bayer_base + (2 * y0 + 0) * bayer_stride;
//    const uint8_t * src1_base = bayer_base + (2 * y0 + 1) * bayer_stride;
//    uint8_t * dstp_base = dst_base + y0 * dst_stride;
//
//    for( int y = y0; y < rend(range); ++y, dstp_base += dst_stride,
//         src0_base += 2 * bayer_stride, src1_base += 2 * bayer_stride) {
//
//      const _Tp1 * src0 = (const _Tp1 *)(src0_base);
//      const _Tp1 * src1 = (const _Tp1 *)(src1_base);
//      _Tp2 * __restrict dstp = (_Tp2 * )(dstp_base);
//
//      for( int x = 0; x < dst_cols; ++x, src0 += 2, src1 += 2, ++dstp ) {
//        const _Tps sum = (c1 + src0[0] + src0[1] + src1[0] + src1[1]) / 4;
//        *dstp = cv::saturate_cast<_Tp2>(sum);
//      }
//    }
//  });
//
//  return true;
//}

template<typename _Tp1, typename _Tp2>
static bool _average_bayer_planes(cv::InputArray _src, cv::OutputArray _dst)
{
  using _Tpmax = std::common_type_t<_Tp1, _Tp2>;
  using _Tps = std::conditional_t<std::is_floating_point_v<_Tpmax>, _Tpmax, int>;
  constexpr _Tps c1 = std::is_integral_v<_Tps> ? 2 : 0;

  const int src_channels = _src.channels();

  if (src_channels == 1) {
    if ((_src.cols() & 0x1) || (_src.rows() & 0x1)) {
      CF_ERROR("Can not average raw bayer planes for uneven image size %dx%d", _src.cols(), _src.rows());
      return false;
    }
  }
  else if (src_channels != 4) {
    CF_ERROR("Unsupported channel count %d. Must be 1 (Raw Bayer) or 4 (Split Bayer planes)", src_channels);
    return false;
  }

  const int ddepth = cv::DataType<_Tp2>::depth;
  const cv::Mat bayer_image = _src.getMat();
  const uint8_t * bayer_base = (const uint8_t * )bayer_image.ptr();
  const size_t bayer_stride = bayer_image.step;

  if (src_channels == 1) {

    const int dst_rows = _src.rows() / 2;
    const int dst_cols = _src.cols() / 2;

    _dst.create(dst_rows, dst_cols, CV_MAKETYPE(ddepth, 1));
    cv::Mat & dst = _dst.getMatRef();
    uint8_t * dst_base = (uint8_t * )dst.ptr();
    const size_t dst_stride = dst.step;

    parallel_for(0, dst_rows, [=](const auto & range) {
      const int y0 = rbegin(range);
      const int ye = rend(range);

      const uint8_t * src0_base = bayer_base + (2 * y0 + 0) * bayer_stride;
      const uint8_t * src1_base = bayer_base + (2 * y0 + 1) * bayer_stride;
      uint8_t * dstp_base = dst_base + y0 * dst_stride;

      for( int y = y0; y < ye; ++y, dstp_base += dst_stride,
           src0_base += 2 * bayer_stride, src1_base += 2 * bayer_stride) {

        const _Tp1 * src0 = (const _Tp1 *)(src0_base);
        const _Tp1 * src1 = (const _Tp1 *)(src1_base);
        _Tp2 * __restrict dstp = (_Tp2 * )(dstp_base);

        for( int x = 0; x < dst_cols; ++x, src0 += 2, src1 += 2, ++dstp ) {
          const _Tps sum = (c1 + src0[0] + src0[1] + src1[0] + src1[1]) / 4;
          *dstp = cv::saturate_cast<_Tp2>(sum);
        }
      }
    });

  }
  else { // 4 channel input image

    const int dst_rows = _src.rows();
    const int dst_cols = _src.cols();

    _dst.create(dst_rows, dst_cols, CV_MAKETYPE(ddepth, 1));
    cv::Mat & dst = _dst.getMatRef();
    uint8_t * dst_base = (uint8_t * )dst.ptr();
    const size_t dst_stride = dst.step;

    parallel_for(0, dst_rows, [=](const auto & range) {
      const int y0 = rbegin(range);
      const int ye = rend(range);

      const uint8_t * src_base = bayer_base + y0 * bayer_stride;
      uint8_t * dstp_base = dst_base + y0 * dst_stride;
      for( int y = y0; y < ye; ++y, dstp_base += dst_stride, src_base += bayer_stride) {

        const _Tp1 * src = (const _Tp1 *)(src_base);
        _Tp2 * __restrict dstp = (_Tp2 * )(dstp_base);

        for( int x = 0; x < dst_cols; ++x, src += 4, ++dstp ) {
          const _Tps sum = (c1 + src[0] + src[1] + src[2] + src[3]) / 4;
          *dstp = cv::saturate_cast<_Tp2>(sum);
        }
      }
    });
  }

  return true;
}

/** @brief
 * Averages 4 pixels of each Bayer cell (2x2) into a single grayscale pixel.
 * Output size is exactly twice smaller than former bayer image
 */
bool average_bayer_planes(cv::InputArray src, cv::OutputArray dst)
{
  INSTRUMENT_REGION("");
  const int ddepth = dst.fixedType() ? dst.depth() : src.depth();
  CV_DISPATCH2(src.depth(), ddepth, _average_bayer_planes, src, dst);
  return true;
}



/** @brief
 * Combine input 4-channel src ordered as [ R G1 B G2 ] into 3-channel BGR dst matrix.
 * The output size of dst is the same as src
 */
template<class _Tp1, class _Tp2>
static bool _bayer_planes_to_bgr(cv::InputArray _src, cv::OutputArray _dst)
{
  using Vec4T = cv::Vec<_Tp1, 4>;
  using Vec3T = cv::Vec<_Tp2, 3>;
  constexpr _Tp1 c1 = std::is_integral_v<_Tp1> ? 1 : 0;

  const int w = _src.cols();
  const int h = _src.rows();

  const cv::Mat_<Vec4T> src = _src.getMat();
  cv::Mat_<Vec3T> dst(h, w);

  parallel_for(0, h, [&, w](const auto & range) {
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
      const _Tp1 * __restrict srcp = (const _Tp1 *)(src[y]);
      _Tp2 * __restrict dstp = (_Tp2 * )(dst[y]);
      for ( int x = 0; x < w; ++x, dstp += 3, srcp += 4 ) {
        dstp[0] = _Tp2(srcp[2]);
        dstp[1] = _Tp2((c1 + srcp[1] + srcp[3]) / 2);
        dstp[2] = _Tp2(srcp[0]);
      }
    }
  });

  _dst.move(dst);

  return true;
}

/** @brief
 * Combine input 4-channel src ordered as [ R G1 B G2 ] into 3-channel BGR dst matrix.
 * The output size of dst is the same as src
 */
bool bayer_planes_to_bgr(cv::InputArray src, cv::OutputArray dst, int ddepth)
{
  INSTRUMENT_REGION("");

  if( src.channels() != 4 ) {
    CF_ERROR("Invalid argument: 4-channel input image expected but src.channels=%d",
        src.channels());
    return false;
  }

  if( dst.fixedType() && dst.channels() != 3 ) {
    CF_ERROR("Invalid argument: 3-channel output destination image expected but dst.channels=%d",
        dst.channels());
    return false;
  }

  if( dst.fixedType() ) {
    ddepth = dst.depth();
  }
  else if( ddepth < 0 ) {
    ddepth = src.depth();
  }

  CV_DISPATCH2(src.depth(), ddepth, _bayer_planes_to_bgr, src, dst);

  CF_ERROR("Not supported combination of src.depth()=%d and ddepth=%d",
      src.depth(), ddepth);

  return false;
}

/** @brief
 * Extract bayer src into dense 3-channel BGR dst matrix with .
 * The output size of dst is the same as src
 */

template<class _Tp>
static bool _debayer_matrix(cv::InputArray _src, cv::OutputArray _dst, enum COLORID colorid)
{
  if ( (_src.cols() & 0x1) || (_src.rows() & 0x1) || _src.channels() != 1 )  {
    CF_ERROR("Can not make extract_bayer_matrix for Uneven image size %dx%dx%d",
        _src.cols(), _src.rows(), _src.channels());
    return false;
  }

  using Vec3T = cv::Vec<_Tp,3>;
  using Mat3T = cv::Mat_<Vec3T>;
  using Mat1T = cv::Mat_<_Tp>;

  const Mat1T src = _src.getMat();
  Mat3T dst(src.size());

  static const auto R =
      [](const _Tp & v) {
        return Vec3T(0, 0, v);
      };

  static const auto G =
      [](const _Tp & v) {
        return Vec3T(0, v, 0);
      };

  static const auto B =
      [](const _Tp & v) {
        return Vec3T(v, 0, 0);
      };

  switch (colorid) {
    case COLORID_BAYER_MYYC:
    case COLORID_BAYER_RGGB:
    //  [ R  G1 ]
    //  [ G2 B  ]
    parallel_for(0, src.rows / 2, [&, xmax = src.cols / 2](const auto & range) {
      for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
        const auto * __restrict src0 = src[2 * y + 0], * __restrict src1 = src[2 * y + 1];
        auto * __restrict dst0 = dst[2 * y + 0], * __restrict dst1 = dst[2 * y + 1];
        for ( int x = 0; x < xmax; ++x ) {
          dst0[2 * x + 0] = R(src0[2 * x + 0]);
          dst0[2 * x + 1] = G(src0[2 * x + 1]);
          dst1[2 * x + 0] = G(src1[2 * x + 0]);
          dst1[2 * x + 1] = B(src1[2 * x + 1]);
        }
      }
    });
    break;

    case COLORID_BAYER_YMCY:
    case COLORID_BAYER_GRBG:
    //  [ G R ]
    //  [ B G ]
    parallel_for(0, src.rows / 2, [&, xmax = src.cols / 2](const auto & range) {
      for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
        const auto * __restrict src0 = src[2 * y + 0], * __restrict src1 = src[2 * y + 1];
        auto * __restrict dst0 = dst[2 * y + 0], * __restrict dst1 = dst[2 * y + 1];
        for ( int x = 0; x < xmax; ++x ) {
          dst0[2 * x + 0] = G(src0[2 * x + 0]);
          dst0[2 * x + 1] = R(src0[2 * x + 1]);
          dst1[2 * x + 0] = B(src1[2 * x + 0]);
          dst1[2 * x + 1] = G(src1[2 * x + 1]);
        }
      }
    });
    break;

    case COLORID_BAYER_YCMY:
    case COLORID_BAYER_GBRG:
    //  [ G B ]
    //  [ R G ]
    parallel_for(0, src.rows / 2, [&, xmax = src.cols / 2](const auto & range) {
      for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
        const auto * __restrict src0 = src[2 * y + 0], * __restrict src1 = src[2 * y + 1];
        auto * __restrict dst0 = dst[2 * y + 0], * __restrict dst1 = dst[2 * y + 1];
        for ( int x = 0; x < xmax; ++x ) {
          dst0[2 * x + 0] = G(src0[2 * x + 0]);
          dst0[2 * x + 1] = B(src0[2 * x + 1]);
          dst1[2 * x + 0] = R(src1[2 * x + 0]);
          dst1[2 * x + 1] = G(src1[2 * x + 1]);
        }
      }
    });
    break;

    case COLORID_BAYER_CYYM:
    case COLORID_BAYER_BGGR:
    //  [ B G ]
    //  [ G R ]
    parallel_for(0, src.rows / 2, [&, xmax = src.cols / 2](const auto & range) {
      for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
        const auto * __restrict src0 = src[2 * y + 0], * __restrict src1 = src[2 * y + 1];
        auto * __restrict dst0 = dst[2 * y + 0], * __restrict dst1 = dst[2 * y + 1];
        for ( int x = 0; x < xmax; ++x ) {
          dst0[2 * x + 0] = B(src0[2 * x + 0]);
          dst0[2 * x + 1] = G(src0[2 * x + 1]);
          dst1[2 * x + 0] = G(src1[2 * x + 0]);
          dst1[2 * x + 1] = R(src1[2 * x + 1]);
        }
      }
    });
    break;

  default :
    return false;
  }

  if ( _dst.fixedType() ) {
    dst.convertTo(_dst, _dst.depth());
  }
  else {
    _dst.move(dst);
  }

  return true;
}

/** @brief
 * Extract bayer src into dense 3-channel BGR dst matrix with
 * The output size of dst is the same as src
 */
bool debayer_matrix(cv::InputArray src, cv::OutputArray dst, enum COLORID colorid)
{
  CV_DISPATCH(src.depth(), _debayer_matrix, src, dst, colorid);
  return false;
}

/** @brief
 * Bayer Demosaicing by 2x2 super-pixel pixel binning
 * Output dst image size is twice smaller than input src image size
 */
template<typename _Tp1, typename _Tp2>
static bool _debayer_sp(cv::InputArray _src, cv::OutputArray _dst, COLORID colorid)
{
  using Vec4T = cv::Vec<_Tp1, 4>;
  using Vec3T = cv::Vec<_Tp2, 3>;

  constexpr int c1 = (std::is_integral_v<_Tp1> && std::is_integral_v<_Tp2>) ? 1 : 0;

  const int src_channels = _src.channels();

  if ( src_channels == 1 ) {

    const int h = _src.rows() / 2;
    const int w = _src.cols() / 2;

    const cv::Mat src = _src.getMat();
    const uint8_t * const src_base = (const uint8_t*) src.data;
    const size_t src_stride = src.step;

    _dst.create(h, w, CV_MAKETYPE(cv::DataType<_Tp2>::depth, 3));
    cv::Mat & dst = _dst.getMatRef();
    uint8_t * const dst_base = (uint8_t*) dst.data;
    const size_t dst_stride = dst.step;

    switch (colorid) {
      case COLORID_BAYER_MYYC:
      case COLORID_BAYER_RGGB:
        // RGGB -> [B G R]
        parallel_for(0, h, [=](const auto & range) {
          for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
            const _Tp1 * __restrict s0 = (const _Tp1*)(src_base + (y * 2 + 0) * src_stride);
            const _Tp1 * __restrict s1 = (const _Tp1*)(src_base + (y * 2 + 1) * src_stride);
            _Tp2 * __restrict dstp = (_Tp2*)(dst_base + y * dst_stride);

            for (int x = 0; x < w; ++x, s0 += 2, s1 += 2,  dstp += 3) {
              dstp[0] = _Tp2(s1[1]);
              dstp[1] = _Tp2((s0[1] + s1[0] + c1) / 2);
              dstp[2] = _Tp2(s0[0]);
            }
          }
        });
        break;

      case COLORID_BAYER_YMCY:
      case COLORID_BAYER_GRBG:
        // GRBG -> [B G R]
        parallel_for(0, h, [=](const auto & range) {
          for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
            const _Tp1 * __restrict s0 = (const _Tp1*)(src_base + (y * 2 + 0) * src_stride);
            const _Tp1 * __restrict s1 = (const _Tp1*)(src_base + (y * 2 + 1) * src_stride);
            _Tp2 * __restrict dstp = (_Tp2*)(dst_base + y * dst_stride);

            for (int x = 0; x < w; ++x, s0 += 2, s1 += 2,  dstp += 3) {
              dstp[0] = _Tp2(s1[0]);
              dstp[1] = _Tp2((c1 + s0[0] + s1[1]) / 2);
              dstp[2] = _Tp2(s0[1]);
            }
          }
        });
        break;

      case COLORID_BAYER_YCMY:
      case COLORID_BAYER_GBRG:
        // GBRG -> [B G R]
        parallel_for(0, h, [=](const auto & range) {
          for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
            const _Tp1 * __restrict s0 = (const _Tp1*)(src_base + (y * 2 + 0) * src_stride);
            const _Tp1 * __restrict s1 = (const _Tp1*)(src_base + (y * 2 + 1) * src_stride);
            _Tp2 * __restrict dstp = (_Tp2*)(dst_base + y * dst_stride);

            for (int x = 0; x < w; ++x, s0 += 2, s1 += 2,  dstp += 3) {
              dstp[0] = _Tp2(s0[1]);
              dstp[1] = _Tp2((c1 + s0[0] + s1[1]) / 2);
              dstp[2] = _Tp2(s1[0]);
            }
          }
        });
        break;

      case COLORID_BAYER_CYYM:
      case COLORID_BAYER_BGGR:
        // BGGR -> [B G R]
        parallel_for(0, h, [=](const auto & range) {
          for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
            const _Tp1 * __restrict s0 = (const _Tp1*)(src_base + (y * 2 + 0) * src_stride);
            const _Tp1 * __restrict s1 = (const _Tp1*)(src_base + (y * 2 + 1) * src_stride);
            _Tp2 * __restrict dstp = (_Tp2*)(dst_base + y * dst_stride);

            for (int x = 0; x < w; ++x, s0 += 2, s1 += 2,  dstp += 3) {
              dstp[0] = _Tp2(s0[0]);
              dstp[1] = _Tp2((c1 + s0[1] + s1[0]) / 2);
              dstp[2] = _Tp2(s1[1]);
            }
          }
        });
        break;

      default: // Not supported
        CF_ERROR("Not supported colorid = %d", colorid);
        return false;
    }
  }
  else if (src_channels == 4 ) {

    const int h = _src.rows();
    const int w = _src.cols();

    const cv::Mat src = _src.getMat();
    const uint8_t * const src_base = (const uint8_t*) src.data;
    const size_t src_stride = src.step;

    _dst.create(h, w, CV_MAKETYPE(cv::DataType<_Tp2>::depth, 3));
    cv::Mat & dst = _dst.getMatRef();
    uint8_t * const dst_base = (uint8_t*) dst.data;
    const size_t dst_stride = dst.step;

    switch (colorid) {
      case COLORID_BAYER_MYYC:
      case COLORID_BAYER_RGGB:
        // RGGB -> [B G R]
        parallel_for(0, h, [=](const auto & range) {
          for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
            const _Tp1 * __restrict sp = (const _Tp1*)(src_base + y * src_stride);
            _Tp2 * __restrict dp = (_Tp2*)(dst_base + y * dst_stride);

            for (int x = 0; x < w; ++x, sp += 4, dp += 3) {
              dp[0] = _Tp2(sp[3]); // B
              dp[1] = _Tp2((c1 + sp[1] + sp[2]) / 2);// G
              dp[2] = _Tp2(sp[0]);// R
            }
          }
        });
        break;

      case COLORID_BAYER_YMCY:
      case COLORID_BAYER_GRBG:
        // GRBG -> [B G R]
        parallel_for(0, h, [=](const auto & range) {
          for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
            const _Tp1 * __restrict sp = (const _Tp1*)(src_base + y * src_stride);
            _Tp2 * __restrict dp = (_Tp2*)(dst_base + y * dst_stride);

            for (int x = 0; x < w; ++x, sp += 4, dp += 3) {
              dp[0] = _Tp2(sp[2]); // B
              dp[1] = _Tp2((c1 + sp[0] + sp[3]) / 2);// G
              dp[2] = _Tp2(sp[1]);// R
            }
          }
        });
        break;

      case COLORID_BAYER_YCMY:
      case COLORID_BAYER_GBRG:
        // GBRG -> [B G R]
        parallel_for(0, h, [=](const auto & range) {
          for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
            const _Tp1 * __restrict sp = (const _Tp1*)(src_base + y * src_stride);
            _Tp2 * __restrict dp = (_Tp2*)(dst_base + y * dst_stride);

            for (int x = 0; x < w; ++x, sp += 4, dp += 3) {
              dp[0] = _Tp2(sp[1]); // B
              dp[1] = _Tp2((c1 + sp[0] + sp[3]) / 2);// G
              dp[2] = _Tp2(sp[0]);// R
            }
          }
        });
        break;

      case COLORID_BAYER_CYYM:
      case COLORID_BAYER_BGGR:
        // BGGR -> [B G R]
        parallel_for(0, h, [=](const auto & range) {
          for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
            const _Tp1 * __restrict sp = (const _Tp1*)(src_base + y * src_stride);
            _Tp2 * __restrict dp = (_Tp2*)(dst_base + y * dst_stride);

            for (int x = 0; x < w; ++x, sp += 4, dp += 3) {
              dp[0] = _Tp2(sp[0]); // B
              dp[1] = _Tp2((c1 + sp[1] + sp[2]) / 2);// G
              dp[2] = _Tp2(sp[3]);// R
            }
          }
        });
        break;

      default: // Not supported
        CF_ERROR("Not supported colorid = %d", colorid);
        return false;
    }
  }
  else {
    CF_ERROR("Bad number of src channels=%d", _src.channels());
    return false;
  }

  return true;
}

/** @brief
 * Bayer Demosaicing by 2x2 super-pixel pixel binning
 * Output dst image size is twice smaller than input src image size
 */
bool debayer_sp(cv::InputArray src, cv::OutputArray dst, COLORID colorid, int ddepth)
{
  INSTRUMENT_REGION("");

  if( dst.fixedType() && dst.channels() != 3 ) {
    CF_ERROR("Invalid argument: 3-channel output destination image expected but dst.channels=%d",
        dst.channels());
    return false;
  }

  if ( dst.fixedType() ) {
    ddepth = dst.depth();
  }
  else if ( ddepth < 0 ) {
    ddepth = src.depth();
  }

  if ( src.channels() == 1 ) {
    if( (src.cols() & 0x1) || (src.rows() & 0x1) ) {
      CF_ERROR("Can not make debayer for uneven image size %dx%dx%d",
          src.cols(), src.rows(), src.channels());
      return false;
    }
  }
  else if ( src.channels() != 4 ) {
    CF_ERROR("Not supported number of channels=%d in input image of size %dx%d",
        src.channels(), src.cols(), src.rows());
    return false;
  }


  CV_DISPATCH2(src.depth(), ddepth, _debayer_sp, src, dst, colorid);

  CF_ERROR("Not supported combination of src.depth()=%d and ddepth=%d",
      src.depth(), ddepth);

  return false;
}

template<class _Tp1, class _Tp2>
bool _debayer_nn2_interpolation(cv::InputArray _src, cv::OutputArray _dst, enum COLORID colorid)
{
  if ( (_src.cols() & 0x1) || (_src.rows() & 0x1) || _src.channels() != 1 )  {
    CF_ERROR("Can not make debayer for uneven image size %dx%dx%d",
        _src.cols(), _src.rows(), _src.channels());
    return false;
  }

  constexpr int c1 = (std::is_integral_v<_Tp1> && std::is_integral_v<_Tp2>) ? 1 : 0;
  constexpr int c2 = (std::is_integral_v<_Tp1> && std::is_integral_v<_Tp2>) ? 2 : 0;

  const cv::Size size = _src.size();

 const cv::Mat src = _src.getMat();

  _dst.create(size, CV_MAKETYPE(cv::DataType<_Tp2>::depth, 3));
  cv::Mat & dst = _dst.getMatRef();

  #define CAPTURE_PARAMS \
    bayer_base = src.data, \
    dst_base = dst.data, \
    src_stride = src.step, \
    dst_stride = dst.step, \
    size, c1, c2

  switch (colorid) {
    case COLORID_BAYER_MYYC:
    case COLORID_BAYER_RGGB: {
      parallel_for(0, size.height, [CAPTURE_PARAMS](const auto & range) {
        for( int y1 = rbegin(range), ymax = rend(range); y1 < ymax; ++y1 ) {
          const int y0 = (y1 == 0) ? 1 : y1 - 1;
          const int y2 = (y1 == size.height - 1) ? size.height - 2 : y1 + 1;

          const _Tp1 * __restrict s0 = (const _Tp1 *)(bayer_base + y0 * src_stride);
          const _Tp1 * __restrict s1 = (const _Tp1 *)(bayer_base + y1 * src_stride);
          const _Tp1 * __restrict s2 = (const _Tp1 *)(bayer_base + y2 * src_stride);
          _Tp2 * __restrict dstp = (_Tp2 *)(dst_base + y1 * dst_stride);

          if ( !(y1 & 0x1) ) { // R G
            dstp[0] = cv::saturate_cast<_Tp2>((c2 + s0[1] + s0[1] + s2[1] + s2[1]) / 4);
            dstp[1] = cv::saturate_cast<_Tp2>((c2 + s0[0] + s1[1] + s1[1] + s2[0]) / 4);
            dstp[2] = cv::saturate_cast<_Tp2>(s1[0]);
            dstp[3] = cv::saturate_cast<_Tp2>((c1 + s0[1] + s2[1]) / 2);
            dstp[4] = cv::saturate_cast<_Tp2>(s1[1]);
            dstp[5] = cv::saturate_cast<_Tp2>((c1 + s1[0] + s1[2 < size.width ? 2 : 0]) / 2);
            dstp += 6;

            for( int x = 2; x < size.width - 2; x += 2, dstp += 6 ) {
              dstp[0] = cv::saturate_cast<_Tp2>((c2 + s0[x-1] + s0[x+1] + s2[x-1] + s2[x+1]) / 4);
              dstp[1] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s1[x-1] + s1[x+1] + s2[x]) / 4);
              dstp[2] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[3] = cv::saturate_cast<_Tp2>((c1 + s0[x+1] + s2[x+1]) / 2);
              dstp[4] = cv::saturate_cast<_Tp2>(s1[x+1]);
              dstp[5] = cv::saturate_cast<_Tp2>((c1 + s1[x] + s1[x+2]) / 2);
            }

            if (size.width > 2) {
              const int x = size.width - 2;
              dstp[0] = cv::saturate_cast<_Tp2>((c2 + s0[x-1] + s0[x+1] + s2[x-1] + s2[x+1]) / 4);
              dstp[1] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s1[x-1] + s1[x+1] + s2[x]) / 4);
              dstp[2] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[3] = cv::saturate_cast<_Tp2>((c1 + s0[x+1] + s2[x+1]) / 2);
              dstp[4] = cv::saturate_cast<_Tp2>(s1[x+1]);
              dstp[5] = cv::saturate_cast<_Tp2>((c1 + s1[x] + s1[x]) / 2);
            }
          }
          else { // G B
            dstp[0] = cv::saturate_cast<_Tp2>((c1 + s1[1] + s1[1]) / 2);
            dstp[1] = cv::saturate_cast<_Tp2>(s1[0]);
            dstp[2] = cv::saturate_cast<_Tp2>((c1 + s0[0] + s2[0]) / 2);
            dstp[3] = cv::saturate_cast<_Tp2>(s1[1]);
            dstp[4] = cv::saturate_cast<_Tp2>((c2 + s0[1] + s1[0] + s1[2 < size.width ? 2 : 0] + s2[1]) / 4);
            dstp[5] = cv::saturate_cast<_Tp2>((c2 + s0[0] + s0[2 < size.width ? 2 : 0] + s2[0] + s2[2 < size.width ? 2 : 0]) / 4);
            dstp += 6;

            for( int x = 2; x < size.width - 2; x += 2, dstp += 6 ) {
              dstp[0] = cv::saturate_cast<_Tp2>((c1 + s1[x-1] + s1[x+1]) / 2);
              dstp[1] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[2] = cv::saturate_cast<_Tp2>((c1 + s0[x] + s2[x]) / 2);
              dstp[3] = cv::saturate_cast<_Tp2>(s1[x+1]);
              dstp[4] = cv::saturate_cast<_Tp2>((c2 + s0[x+1] + s1[x] + s1[x+2] + s2[x+1]) / 4);
              dstp[5] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s0[x+2] + s2[x] + s2[x+2]) / 4);
            }

            if (size.width > 2) {
              const int x = size.width - 2;
              dstp[0] = cv::saturate_cast<_Tp2>((c1 + s1[x-1] + s1[x+1]) / 2);
              dstp[1] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[2] = cv::saturate_cast<_Tp2>((c1 + s0[x] + s2[x]) / 2);
              dstp[3] = cv::saturate_cast<_Tp2>(s1[x+1]);
              dstp[4] = cv::saturate_cast<_Tp2>((c2 + s0[x+1] + s1[x] + s1[x] + s2[x+1]) / 4);
              dstp[5] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s0[x] + s2[x] + s2[x]) / 4);
            }
          }
        }
      });
      break;
    }

    case COLORID_BAYER_YMCY:
    case COLORID_BAYER_GRBG: {
      parallel_for(0, size.height, [CAPTURE_PARAMS](const auto & range) {
        for( int y1 = rbegin(range), ymax = rend(range); y1 < ymax; ++y1 ) {
          const int y0 = (y1 == 0) ? 1 : y1 - 1;
          const int y2 = (y1 == size.height - 1) ? size.height - 2 : y1 + 1;

          const _Tp1 * __restrict s0 = (const _Tp1 *)(bayer_base + y0 * src_stride);
          const _Tp1 * __restrict s1 = (const _Tp1 *)(bayer_base + y1 * src_stride);
          const _Tp1 * __restrict s2 = (const _Tp1 *)(bayer_base + y2 * src_stride);
          _Tp2 * __restrict dstp = (_Tp2 *)(dst_base + y1 * dst_stride);

          if ( !(y1 & 0x1) ) { // G R
            dstp[0] = cv::saturate_cast<_Tp2>((c1 + s0[0] + s2[0]) / 2);
            dstp[1] = cv::saturate_cast<_Tp2>(s1[0]);
            dstp[2] = cv::saturate_cast<_Tp2>((c1 + s1[1] + s1[1]) / 2);
            dstp[3] = cv::saturate_cast<_Tp2>((c2 + s0[0] + s0[2 < size.width ? 2 : 0] + s2[0] + s2[2 < size.width ? 2 : 0]) / 4);
            dstp[4] = cv::saturate_cast<_Tp2>((c2 + s0[1] + s1[0] + s1[2 < size.width ? 2 : 0] + s2[1]) / 4);
            dstp[5] = cv::saturate_cast<_Tp2>(s1[1]);
            dstp += 6;

            for( int x = 2; x < size.width - 2; x += 2, dstp += 6 ) {
              dstp[0] = cv::saturate_cast<_Tp2>((c1 + s0[x] + s2[x]) / 2);
              dstp[1] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[2] = cv::saturate_cast<_Tp2>((c1 + s1[x-1] + s1[x+1]) / 2);
              dstp[3] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s0[x+2] + s2[x] + s2[x+2]) / 4);
              dstp[4] = cv::saturate_cast<_Tp2>((c2 + s0[x+1] + s1[x] + s1[x+2] + s2[x+1]) / 4);
              dstp[5] = cv::saturate_cast<_Tp2>(s1[x+1]);
            }

            if (size.width > 2) {
              const int x = size.width - 2;
              dstp[0] = cv::saturate_cast<_Tp2>((c1 + s0[x] + s2[x]) / 2);
              dstp[1] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[2] = cv::saturate_cast<_Tp2>((c1 + s1[x-1] + s1[x+1]) / 2);
              dstp[3] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s0[x] + s2[x] + s2[x]) / 4);
              dstp[4] = cv::saturate_cast<_Tp2>((c2 + s0[x+1] + s1[x] + s1[x] + s2[x+1]) / 4);
              dstp[5] = cv::saturate_cast<_Tp2>(s1[x+1]);
            }
          }
          else { // B G
            dstp[0] = cv::saturate_cast<_Tp2>(s1[0]);
            dstp[1] = cv::saturate_cast<_Tp2>((c2 + s0[0] + s1[1] + s1[1] + s2[0]) / 4);
            dstp[2] = cv::saturate_cast<_Tp2>((c2 + s0[1] + s0[1] + s2[1] + s2[1]) / 4);
            dstp[3] = cv::saturate_cast<_Tp2>((c1 + s1[0] + s1[2 < size.width ? 2 : 0]) / 2);
            dstp[4] = cv::saturate_cast<_Tp2>(s1[1]);
            dstp[5] = cv::saturate_cast<_Tp2>((c1 + s0[1] + s2[1]) / 2);
            dstp += 6;

            for( int x = 2; x < size.width - 2; x += 2, dstp += 6 ) {
              dstp[0] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[1] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s1[x-1] + s1[x+1] + s2[x]) / 4);
              dstp[2] = cv::saturate_cast<_Tp2>((c2 + s0[x-1] + s0[x+1] + s2[x-1] + s2[x+1]) / 4);
              dstp[3] = cv::saturate_cast<_Tp2>((c1 + s1[x] + s1[x+2]) / 2);
              dstp[4] = cv::saturate_cast<_Tp2>(s1[x+1]);
              dstp[5] = cv::saturate_cast<_Tp2>((c1 + s0[x+1] + s2[x+1]) / 2);
            }

            if (size.width > 2) {
              const int x = size.width - 2;
              dstp[0] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[1] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s1[x-1] + s1[x+1] + s2[x]) / 4);
              dstp[2] = cv::saturate_cast<_Tp2>((c2 + s0[x-1] + s0[x+1] + s2[x-1] + s2[x+1]) / 4);
              dstp[3] = cv::saturate_cast<_Tp2>((c1 + s1[x] + s1[x]) / 2);
              dstp[4] = cv::saturate_cast<_Tp2>(s1[x+1]);
              dstp[5] = cv::saturate_cast<_Tp2>((c1 + s0[x+1] + s2[x+1]) / 2);
            }
          }
        }
      });
      break;
    }

    case COLORID_BAYER_YCMY:
    case COLORID_BAYER_GBRG: {
      parallel_for(0, size.height, [CAPTURE_PARAMS](const auto & range) {
        for( int y1 = rbegin(range), ymax = rend(range); y1 < ymax; ++y1 ) {
          const int y0 = (y1 == 0) ? 1 : y1 - 1;
          const int y2 = (y1 == size.height - 1) ? size.height - 2 : y1 + 1;

          const _Tp1 * __restrict s0 = (const _Tp1 *)(bayer_base + y0 * src_stride);
          const _Tp1 * __restrict s1 = (const _Tp1 *)(bayer_base + y1 * src_stride);
          const _Tp1 * __restrict s2 = (const _Tp1 *)(bayer_base + y2 * src_stride);
          _Tp2 * __restrict dstp = (_Tp2 *)(dst_base + y1 * dst_stride);

          if ( !(y1 & 0x1) ) { // G B
            dstp[0] = cv::saturate_cast<_Tp2>((c1 + s1[1] + s1[1]) / 2);
            dstp[1] = cv::saturate_cast<_Tp2>(s1[0]);
            dstp[2] = cv::saturate_cast<_Tp2>((c1 + s0[0] + s2[0]) / 2);
            dstp[3] = cv::saturate_cast<_Tp2>(s1[1]);
            dstp[4] = cv::saturate_cast<_Tp2>((c2 + s0[1] + s1[0] + s1[2 < size.width ? 2 : 0] + s2[1]) / 4);
            dstp[5] = cv::saturate_cast<_Tp2>((c2 + s0[0] + s0[2 < size.width ? 2 : 0] + s2[0] + s2[2 < size.width ? 2 : 0]) / 4);
            dstp += 6;

            for( int x = 2; x < size.width - 2; x += 2, dstp += 6 ) {
              dstp[0] = cv::saturate_cast<_Tp2>((c1 + s1[x-1] + s1[x+1]) / 2);
              dstp[1] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[2] = cv::saturate_cast<_Tp2>((c1 + s0[x] + s2[x]) / 2);
              dstp[3] = cv::saturate_cast<_Tp2>(s1[x+1]);
              dstp[4] = cv::saturate_cast<_Tp2>((c2 + s0[x+1] + s1[x] + s1[x+2] + s2[x+1]) / 4);
              dstp[5] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s0[x+2] + s2[x] + s2[x+2]) / 4);
            }

            if (size.width > 2) {
              const int x = size.width - 2;
              dstp[0] = cv::saturate_cast<_Tp2>((c1 + s1[x-1] + s1[x+1]) / 2);
              dstp[1] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[2] = cv::saturate_cast<_Tp2>((c1 + s0[x] + s2[x]) / 2);
              dstp[3] = cv::saturate_cast<_Tp2>(s1[x+1]);
              dstp[4] = cv::saturate_cast<_Tp2>((c2 + s0[x+1] + s1[x] + s1[x] + s2[x+1]) / 4);
              dstp[5] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s0[x] + s2[x] + s2[x]) / 4);
            }
          }
          else { // R G
            dstp[0] = cv::saturate_cast<_Tp2>((c2 + s0[1] + s0[1] + s2[1] + s2[1]) / 4);
            dstp[1] = cv::saturate_cast<_Tp2>((c2 + s0[0] + s1[1] + s1[1] + s2[0]) / 4);
            dstp[2] = cv::saturate_cast<_Tp2>(s1[0]);
            dstp[3] = cv::saturate_cast<_Tp2>((c1 + s0[1] + s2[1]) / 2);
            dstp[4] = cv::saturate_cast<_Tp2>(s1[1]);
            dstp[5] = cv::saturate_cast<_Tp2>((c1 + s1[0] + s1[2 < size.width ? 2 : 0]) / 2);
            dstp += 6;

            for( int x = 2; x < size.width - 2; x += 2, dstp += 6 ) {
              dstp[0] = cv::saturate_cast<_Tp2>((c2 + s0[x-1] + s0[x+1] + s2[x-1] + s2[x+1]) / 4);
              dstp[1] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s1[x-1] + s1[x+1] + s2[x]) / 4);
              dstp[2] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[3] = cv::saturate_cast<_Tp2>((c1 + s0[x+1] + s2[x+1]) / 2);
              dstp[4] = cv::saturate_cast<_Tp2>(s1[x+1]);
              dstp[5] = cv::saturate_cast<_Tp2>((c1 + s1[x] + s1[x+2]) / 2);
            }

            if (size.width > 2) {
              const int x = size.width - 2;
              dstp[0] = cv::saturate_cast<_Tp2>((c2 + s0[x-1] + s0[x+1] + s2[x-1] + s2[x+1]) / 4);
              dstp[1] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s1[x-1] + s1[x+1] + s2[x]) / 4);
              dstp[2] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[3] = cv::saturate_cast<_Tp2>((c1 + s0[x+1] + s2[x+1]) / 2);
              dstp[4] = cv::saturate_cast<_Tp2>(s1[x+1]);
              dstp[5] = cv::saturate_cast<_Tp2>((c1 + s1[x] + s1[x]) / 2);
            }
          }
        }
      });
      break;
    }

    case COLORID_BAYER_CYYM:
    case COLORID_BAYER_BGGR: {
      parallel_for(0, size.height, [CAPTURE_PARAMS](const auto & range) {
        for( int y1 = rbegin(range), ymax = rend(range); y1 < ymax; ++y1 ) {
          const int y0 = (y1 == 0) ? 1 : y1 - 1;
          const int y2 = (y1 == size.height - 1) ? size.height - 2 : y1 + 1;

          const _Tp1 * __restrict s0 = (const _Tp1 *)(bayer_base + y0 * src_stride);
          const _Tp1 * __restrict s1 = (const _Tp1 *)(bayer_base + y1 * src_stride);
          const _Tp1 * __restrict s2 = (const _Tp1 *)(bayer_base + y2 * src_stride);
          _Tp2 * __restrict dstp = (_Tp2 *)(dst_base + y1 * dst_stride);

          if ( !(y1 & 0x1) ) { // B G
            dstp[0] = cv::saturate_cast<_Tp2>(s1[0]);
            dstp[1] = cv::saturate_cast<_Tp2>((c2 + s0[0] + s1[1] + s1[1] + s2[0]) / 4);
            dstp[2] = cv::saturate_cast<_Tp2>((c2 + s0[1] + s0[1] + s2[1] + s2[1]) / 4);
            dstp[3] = cv::saturate_cast<_Tp2>((c1 + s1[0] + s1[2 < size.width ? 2 : 0]) / 2);
            dstp[4] = cv::saturate_cast<_Tp2>(s1[1]);
            dstp[5] = cv::saturate_cast<_Tp2>((c1 + s0[1] + s2[1]) / 2);
            dstp += 6;

            for( int x = 2; x < size.width - 2; x += 2, dstp += 6 ) {
              dstp[0] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[1] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s1[x-1] + s1[x+1] + s2[x]) / 4);
              dstp[2] = cv::saturate_cast<_Tp2>((c2 + s0[x-1] + s0[x+1] + s2[x-1] + s2[x+1]) / 4);
              dstp[3] = cv::saturate_cast<_Tp2>((c1 + s1[x] + s1[x+2]) / 2);
              dstp[4] = cv::saturate_cast<_Tp2>(s1[x+1]);
              dstp[5] = cv::saturate_cast<_Tp2>((c1 + s0[x+1] + s2[x+1]) / 2);
            }

            if (size.width > 2) {
              const int x = size.width - 2;
              dstp[0] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[1] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s1[x-1] + s1[x+1] + s2[x]) / 4);
              dstp[2] = cv::saturate_cast<_Tp2>((c2 + s0[x-1] + s0[x+1] + s2[x-1] + s2[x+1]) / 4);
              dstp[3] = cv::saturate_cast<_Tp2>((c1 + s1[x] + s1[x]) / 2);
              dstp[4] = cv::saturate_cast<_Tp2>(s1[x+1]);
              dstp[5] = cv::saturate_cast<_Tp2>((c1 + s0[x+1] + s2[x+1]) / 2);
            }
          }
          else { // G R
            dstp[0] = cv::saturate_cast<_Tp2>((c1 + s0[0] + s2[0]) / 2);
            dstp[1] = cv::saturate_cast<_Tp2>(s1[0]);
            dstp[2] = cv::saturate_cast<_Tp2>((c1 + s1[1] + s1[1]) / 2);
            dstp[3] = cv::saturate_cast<_Tp2>((c2 + s0[0] + s0[2 < size.width ? 2 : 0] + s2[0] + s2[2 < size.width ? 2 : 0]) / 4);
            dstp[4] = cv::saturate_cast<_Tp2>((c2 + s0[1] + s1[0] + s1[2 < size.width ? 2 : 0] + s2[1]) / 4);
            dstp[5] = cv::saturate_cast<_Tp2>(s1[1]);
            dstp += 6;

            for( int x = 2; x < size.width - 2; x += 2, dstp += 6 ) {
              dstp[0] = cv::saturate_cast<_Tp2>((c1 + s0[x] + s2[x]) / 2);
              dstp[1] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[2] = cv::saturate_cast<_Tp2>((c1 + s1[x-1] + s1[x+1]) / 2);
              dstp[3] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s0[x+2] + s2[x] + s2[x+2]) / 4);
              dstp[4] = cv::saturate_cast<_Tp2>((c2 + s0[x+1] + s1[x] + s1[x+2] + s2[x+1]) / 4);
              dstp[5] = cv::saturate_cast<_Tp2>(s1[x+1]);
            }

            if (size.width > 2) {
              const int x = size.width - 2;
              dstp[0] = cv::saturate_cast<_Tp2>((c1 + s0[x] + s2[x]) / 2);
              dstp[1] = cv::saturate_cast<_Tp2>(s1[x]);
              dstp[2] = cv::saturate_cast<_Tp2>((c1 + s1[x-1] + s1[x+1]) / 2);
              dstp[3] = cv::saturate_cast<_Tp2>((c2 + s0[x] + s0[x] + s2[x] + s2[x]) / 4);
              dstp[4] = cv::saturate_cast<_Tp2>((c2 + s0[x+1] + s1[x] + s1[x] + s2[x+1]) / 4);
              dstp[5] = cv::saturate_cast<_Tp2>(s1[x+1]);
            }
          }
        }
      });
      break;
    }

    default:
      CF_ERROR("Not supported colorid=%d (%s) for debayer",(int)(colorid), toCString(colorid));
      return false;
  }

  #undef CAPTURE_PARAMS

//  _dst.move(dst);
  return true;
}





bool debayer_nn2(cv::InputArray src, cv::OutputArray dst, enum COLORID colorid, int ddepth)
{
  INSTRUMENT_REGION("");

  if ( !is_bayer_pattern(colorid) ) {
    CF_ERROR("colorid=%d (%s) is not bayer pattern", (int)colorid, toCString(colorid));
    return false;
  }

  if ( src.channels() != 1 ) {
    CF_ERROR("Input image channels=%d. Must be 1", src.channels());
    return false;
  }

  if( dst.fixedType() ) {
    ddepth = dst.depth();
    if (dst.channels() != 3 ) {
      CF_ERROR("Output image channels=%d. Must be 3", dst.channels());
      return false;
    }
  }
  else if( ddepth < 0 ) {
    ddepth = src.depth();
  }

  CV_DISPATCH2(src.depth(), ddepth, _debayer_nn2_interpolation, src, dst, colorid);

  CF_ERROR("Not supported combination of src.depth()=%d and ddepth=%d",
      src.depth(), ddepth);

  return false;
}


/** @brief Bayer demosaicing
 */
bool debayer(cv::InputArray src, cv::OutputArray dst, enum COLORID colorid, enum DEBAYER_ALGORITHM algo)
{
  if( algo == DEBAYER_DEFAULT ) {
    algo = default_debayer_algorithm();
  }

  if ( algo == DEBAYER_AVGBP ) {
    return average_bayer_planes(src, dst);
  }

  if( src.channels() == 4 ) {

    switch (algo)
    {
      case DEBAYER_NN:
      case DEBAYER_NN2:
      case DEBAYER_VNG:
      case DEBAYER_EA:
        return interpolate_bayer_planes(src, dst, colorid);
      case DEBAYER_SP:
        return debayer_sp(src, dst, colorid);
      case DEBAYER_PLANE_0:
      case DEBAYER_PLANE_1:
      case DEBAYER_PLANE_2:
      case DEBAYER_PLANE_3:
        cv::extractChannel(src, dst, (int(algo) - DEBAYER_PLANE_0));
        return true;
    }

    CF_ERROR("Not supported debayer algorithm %d (%s) requested from 4-plane bayer image",
        algo, toCString(algo));
    return false;
  }

  if (src.channels() != 1 ) {
    CF_ERROR("Invalid number of channels=%d in bayer image. Must be 1",
        src.channels());
    return false;
  }

  if ( algo >= DEBAYER_PLANE_0 && algo <= DEBAYER_PLANE_3 ) {
    extract_bayer_plane(src, dst, (int(algo) - DEBAYER_PLANE_0));
    return true;
  }

  switch (algo) {
    case DEBAYER_DISABLE:
      src.copyTo(dst);
      return true;
    case DEBAYER_NN:
      if( src.depth() != CV_8U && src.depth() != CV_16U ) {
        return debayer_nn2(src, dst, colorid);
      }
      break;
    case DEBAYER_EA:
      if ( src.depth() != CV_8U && src.depth() != CV_16U) { // fall back to NN2
        return debayer_nn2(src, dst, colorid);
      }
      break;
    case DEBAYER_VNG:
      if ( src.depth() != CV_8U ) { // fall back to NN2
        return debayer_nn2(src, dst, colorid);
      }
      break;
    case DEBAYER_NN2:
      return debayer_nn2(src, dst, colorid);
    case DEBAYER_SP:
      return debayer_sp(src, dst, colorid);
    case DEBAYER_MATRIX:
      return debayer_matrix(src, dst, colorid);
    default:
      break;
  }

  switch ( colorid ) {
  case COLORID_BAYER_MYYC:
  case COLORID_BAYER_RGGB:
    switch ( algo ) {
    case DEBAYER_NN :
      cv::demosaicing(src, dst, cv::COLOR_BayerRGGB2BGR);
      break;
    case DEBAYER_VNG :
      cv::demosaicing(src, dst, cv::COLOR_BayerRGGB2BGR_VNG);
      break;
    case DEBAYER_EA :
      cv::demosaicing(src, dst, cv::COLOR_BayerRGGB2BGR_EA);
      break;
    default :
      CF_DEBUG("Unknown debayer algorithm=%d requested", algo);
      return false;
    }
    break;

  case COLORID_BAYER_YMCY:
  case COLORID_BAYER_GRBG:
    switch ( algo ) {
    case DEBAYER_NN :
      cv::demosaicing(src, dst, cv::COLOR_BayerGRBG2BGR);
      break;
    case DEBAYER_VNG :
      cv::demosaicing(src, dst, cv::COLOR_BayerGRBG2BGR_VNG);
      break;
    case DEBAYER_EA :
      cv::demosaicing(src, dst, cv::COLOR_BayerGRBG2BGR_EA);
      break;
    default :
      CF_DEBUG("Unknown debayer algorithm=%d requested", algo);
      return false;
    }
    break;

  case COLORID_BAYER_YCMY:
  case COLORID_BAYER_GBRG:
    switch ( algo ) {
    case DEBAYER_NN :
      cv::demosaicing(src, dst, cv::COLOR_BayerGBRG2BGR);
      break;
    case DEBAYER_VNG :
      cv::demosaicing(src, dst, cv::COLOR_BayerGBRG2BGR_VNG);
      break;
    case DEBAYER_EA :
      cv::demosaicing(src, dst, cv::COLOR_BayerGBRG2BGR_EA);
      break;
    default :
      CF_DEBUG("Unknown debayer algorithm=%d requested", algo);
      return false;
    }
    break;

  case COLORID_BAYER_CYYM:
  case COLORID_BAYER_BGGR:
    switch ( algo ) {
    case DEBAYER_NN :
      cv::demosaicing(src, dst, cv::COLOR_BayerBGGR2BGR);
      break;
    case DEBAYER_VNG :
      cv::demosaicing(src, dst, cv::COLOR_BayerBGGR2BGR_VNG);
      break;
    case DEBAYER_EA :
      cv::demosaicing(src, dst, cv::COLOR_BayerBGGR2BGR_EA);
      break;
    default :
      CF_DEBUG("Unknown debayer algorithm=%d requested", algo);
      return false;
    }
    break;
  default :
    CF_DEBUG("Unknown colorid=%d requested", colorid);
    return false;
  }

  return true;
}

/** @brief
 * Check for ZWO ASI specific horizontal stripe artifact
 * on the 4-channel Bayer image.
 * The input 4-plane Bayer image can be created
 * from raw 1-channel Bayer frame using extract_bayer_planes()
 */
bool is_corrupted_asi_frame(const cv::Mat & image)
{
  if( image.channels() == 4 ) {

    // Bayer planes image, check only two green planes

    cv::Mat plane;
    double minVal, maxVal;
    cv::Scalar avgVal, stdVal;

    //
    for( int i = 1; i <= 2; ++i ) {

      static float K[2] = {
          -1, 1
      };

      cv::extractChannel(image, plane, 1);
      cv::filter2D(plane, plane, CV_32F, cv::Mat1f(2, 1, K));
      cv::reduce(plane, plane, cv::REDUCE_AVG, 1, CV_32F);

      cv::minMaxLoc(plane, &minVal, &maxVal);
      cv::meanStdDev(plane, avgVal, stdVal);

      if( stdVal[0] > 0 && std::max(fabs(minVal), fabs(maxVal)) / stdVal[0] > 10 ) {
        return true;
      }
    }
  }
  else if( image.channels() == 3 ) {

    // assume BGR image, check only B plane

    cv::Mat plane;
    double minVal, maxVal;
    cv::Scalar avgVal, stdVal;

    static float K[2] = {
        -1, 1
    };

    cv::extractChannel(image, plane, 0);
    cv::filter2D(plane, plane, CV_32F, cv::Mat1f(2, 1, K));
    cv::reduce(plane, plane, cv::REDUCE_AVG, 1, CV_32F);

    cv::minMaxLoc(plane, &minVal, &maxVal);
    cv::meanStdDev(plane, avgVal, stdVal);

    if( stdVal[0] > 0 && std::max(fabs(minVal), fabs(maxVal)) / stdVal[0] > 10 ) {
      return true;
    }
  }
  else if( image.channels() == 1 ) {
    // assume monochrome image

    cv::Mat plane;
    double minVal, maxVal;
    cv::Scalar avgVal, stdVal;

    static float K[2] = {
        -1, 1
    };

    cv::filter2D(image, plane, CV_32F, cv::Mat1f(2, 1, K));
    cv::reduce(plane, plane, cv::REDUCE_AVG, 1, CV_32F);

    cv::minMaxLoc(plane, &minVal, &maxVal);
    cv::meanStdDev(plane, avgVal, stdVal);

    if( stdVal[0] > 0 && std::max(fabs(minVal), fabs(maxVal)) / stdVal[0] > 10 ) {
      return true;
    }
  }

  return false;
}

/** @brief
 * Check for ZWO ASI specific horizontal stripe artifact
 * on the 1-channel Bayer image.
 */
bool is_corrupted_asi_bayer_frame(const cv::Mat & bayer_image, enum COLORID colorid,
    double median_hat_threshold)
{
  cv::Mat tmp;
  cv::Mat mb;

  if ( !extract_bayer_planes(bayer_image, tmp) ) {
    CF_ERROR("extract_bayer_planes() fails");
    return false;
  }

  cv::absdiff(tmp(cv::Rect(0, 0, tmp.cols, tmp.rows - 1)),
      tmp(cv::Rect(0, 1, tmp.cols, tmp.rows - 1)),
      tmp);

  const cv::Size size = tmp.size();
  cv::reduce(tmp, tmp, 1, cv::REDUCE_AVG, CV_32F);

  cv::medianBlur(tmp, mb, 5);
  cv::absdiff(tmp, mb, tmp);
  cv::reduce(tmp.reshape(1, tmp.total()), tmp, 1, cv::REDUCE_MAX);
  tmp = tmp.reshape(0, size.height);

  cv::compare(tmp, median_hat_threshold, tmp, cv::CMP_GE);

  return cv::countNonZero(tmp) > 0;
}


/**
 * bayer_image: Must be single-channel bayer pattern image, or 4-channel bayer planes image
 */
template<class _Tp>
static bool _debayer_denoise(cv::Mat & _bayer_image, double _k, COLORID color_id,
    bool returnBayerPlanes)
{
  INSTRUMENT_REGION("");

  using Vec4T = cv::Vec<_Tp, 4>;

  const int cn = _bayer_image.channels();

  switch (cn) {
    case 1: // assume monochrome bayer patter
      if( (_bayer_image.cols & 0x1) || (_bayer_image.rows & 0x1) ) {
        CF_ERROR("Can not make debayer for uneven image size %dx%dx%d",
            _bayer_image.cols, _bayer_image.rows, _bayer_image.channels());
        return false;
      }
      break;

    case 4: // assume already bayer planes
      break;

    default:
      CF_ERROR("Invalid number of bayer_image channels = %d (image size is %dx%dx%d)",
          _bayer_image.channels(), _bayer_image.cols, _bayer_image.rows);
      return false;
  }

  cv::Mat_<Vec4T> planes, median, mad;
  int rows4, cols4;

  if ( cn == 4 ) {
    rows4 = _bayer_image.rows;
    cols4 = _bayer_image.cols;
    planes = _bayer_image;
  }
  else {
    rows4 = _bayer_image.rows / 2;
    cols4 = _bayer_image.cols / 2;
    _extract_bayer_planes<_Tp, _Tp>(_bayer_image, planes);
  }

  cv::medianBlur(planes, median, 3);
  cv::absdiff(planes, median, mad);
  cv::boxFilter(mad, mad, -1, cv::Size(3, 3), cv::Point(-1, -1), true, cv::BORDER_DEFAULT);

  const float minvar = _bayer_image.depth() < CV_32F ? 1.f : 1.f / 256.f;

  if ( cn == 4 || returnBayerPlanes ) {

    uint8_t * const planes_base = (uint8_t*)planes.ptr();
    const size_t planes_stride = planes.step;

    const uint8_t * const median_base = (const uint8_t*)median.ptr();
    const size_t median_stride = median.step;

    const uint8_t * const mad_base = (const uint8_t*)mad.ptr();
    const size_t mad_stride = mad.step;

    parallel_for(0, rows4, [=](const auto & range) {
      const float k = _k;
      const int xmax = cols4 * 4;
      for( int y = rbegin(range); y < rend(range); ++y ) {

        _Tp * __restrict plane = (_Tp *) (planes_base + y * planes_stride);
        const _Tp * __restrict med = (const _Tp*)(median_base + y * median_stride);
        const _Tp * __restrict mad = (const _Tp*)(mad_base + y * mad_stride);

        for( int x = 0; x < xmax; ++x, ++plane, ++med, ++mad) {
          const float p = *plane, m = *med;
          if ( std::abs(m - p) > k * (*mad) + minvar ) {
            *plane = m;
          }
        }
      }
    });

    if (cn == 1) {
      _bayer_image = std::move(planes);
    }
  }
  else {

    uint8_t * const bayer_base = (uint8_t*)_bayer_image.ptr();
    const size_t bayer_stride = _bayer_image.step;

    uint8_t * const planes_base = (uint8_t*)planes.ptr();
    const size_t planes_stride = planes.step;

    const uint8_t * const median_base = (const uint8_t*)median.ptr();
    const size_t median_stride = median.step;

    const uint8_t * const mad_base = (const uint8_t*)mad.ptr();
    const size_t mad_stride = mad.step;

    parallel_for(0, rows4, [=](const auto & range) {
      const float k = _k;
      for( int y = rbegin(range); y < rend(range); ++y ) {

        const _Tp * __restrict plane = (_Tp *) (planes_base + y * planes_stride);
        const _Tp * __restrict med = (const _Tp*)(median_base + y * median_stride);
        const _Tp * __restrict mad = (const _Tp*)(mad_base + y * mad_stride);

        _Tp * __restrict bayer0 = (_Tp * )(bayer_base + (2 * y + 0) * bayer_stride);
        _Tp * __restrict bayer1 = (_Tp * )(bayer_base + (2 * y + 1) * bayer_stride);

        for( int x = 0; x < cols4; ++x, plane += 4, med += 4, mad += 4 ) {
          const float p0 = plane[0], p1 = plane[1], p2 = plane[2], p3 = plane[3];
          const float m0 = med[0], m1 = med[1], m2 = med[2], m3 = med[3];
          if ( std::abs(m0 - p0) > k * mad[0] + minvar ) {
            bayer0[2 * x + 0] = m0;
          }
          if ( std::abs(m1 - p1) > k * mad[1] + minvar ) {
            bayer0[2 * x + 1] = m1;
          }
          if ( std::abs(m2 - p2) > k * mad[2] + minvar ) {
            bayer1[2 * x+ 0] = m2;
          }
          if ( std::abs(m3 - p3) > k * mad[3] + minvar ) {
            bayer1[2 * x + 1] = m3;
          }
        }
      }
    });
  }

  return true;
}

bool bayer_denoise(cv::Mat & image, double variation_threshold,
    COLORID color_id, bool returnBayerPlanes)
{
  INSTRUMENT_REGION("");

  if( is_bayer_pattern(color_id) ) {
    CV_DISPATCH(image.depth(), _debayer_denoise, image, variation_threshold, color_id,
        returnBayerPlanes);
  }

  CF_ERROR("Not a valid bayer pattern color_id=%d (%s) or not supported pixel depth=%d",
      color_id, toCString(color_id), image.depth());

  return false;
}

/** @brief
 * Combine input 4-channel bayer planes src image into 3-channel BGR dst matrix using NN interpolaton.
 * The output size of dst is twce large to src size
 */
template<typename _Tp1, typename _Tp2>
static bool _interpolate_bayer_planes(cv::InputArray _src, cv::OutputArray _dst, enum COLORID colorid,
    int ddepth /*= -1*/)
{
  INSTRUMENT_REGION("");

  using Vec4T = cv::Vec<_Tp1, 4>;
  using Vec3T = cv::Vec<_Tp2, 3>;

  constexpr int c1 = (std::is_integral_v<_Tp1> && std::is_integral_v<_Tp2>) ? 1 : 0;
  constexpr int c2 = (std::is_integral_v<_Tp1> && std::is_integral_v<_Tp2>) ? 2 : 0;

  const int src_rows = _src.rows();
  const int src_cols = _src.cols();

  if( src_rows <= 0 || src_cols <= 0 ) {
    CF_ERROR("Invalid source image size %dx%d", src_cols, src_rows);
    return false;
  }

  const cv::Mat_<Vec4T> src = _src.getMat();
  cv::Mat_<Vec3T> dst(2 * src_rows, 2 * src_cols);

  const uint8_t * const src_base = (const uint8_t*) src.data;
  uint8_t * const dst_base = (uint8_t*) dst.data;

  const size_t src_stride = src.step;
  const size_t dst_stride = dst.step;

  switch (colorid) {
    case COLORID_BAYER_MYYC:
    case COLORID_BAYER_RGGB: {
      // R=0, G1=1, G2=2, B=3
      parallel_for(0, src_rows, [=](const auto & range) {
        const uint8_t * const bayer_base = src_base;
        uint8_t * const planes_base = dst_base;

        for (int y1 = rbegin(range), ymax = rend(range); y1 < ymax; ++y1) {
          const int y0 = std::max(0, y1 - 1);
          const int y2 = std::min(src_rows - 1, y1 + 1);

          const Vec4T* __restrict s0 = (const Vec4T*)(bayer_base + y0 * src_stride);
          const Vec4T* __restrict s1 = (const Vec4T*)(bayer_base + y1 * src_stride);
          const Vec4T* __restrict s2 = (const Vec4T*)(bayer_base + y2 * src_stride);

          _Tp2* __restrict d0 = (_Tp2*)(planes_base + (2 * y1 + 0) * dst_stride);
          _Tp2* __restrict d1 = (_Tp2*)(planes_base + (2 * y1 + 1) * dst_stride);

          for (int x1 = 0; x1 < src_cols; ++x1, d0 += 6, d1 += 6) {
            const int x0 = std::max(0, x1 - 1);
            const int x2 = std::min(src_cols - 1, x1 + 1);

            const _Tp1 * s00 = s0[x0].val;
            const _Tp1 * s01 = s0[x1].val;
            const _Tp1 * s02 = s0[x2].val;

            const _Tp1 * s10 = s1[x0].val;
            const _Tp1 * s11 = s1[x1].val;
            const _Tp1 * s12 = s1[x2].val;

            const _Tp1 * s20 = s2[x0].val;
            const _Tp1 * s21 = s2[x1].val;
            const _Tp1 * s22 = s2[x2].val;

            // left top (B, G, R)
            d0[0] = cv::saturate_cast<_Tp2>((s11[3] + s01[3] + s10[3] + s00[3] + c2) / 4);
            d0[1] = cv::saturate_cast<_Tp2>((s11[1] + s11[2] + s10[1] + s01[2] + c2) / 4);
            d0[2] = cv::saturate_cast<_Tp2>(s11[0]);

            // right top (B, G, R)
            d0[3] = cv::saturate_cast<_Tp2>((s11[3] + s01[3] + c1) / 2);
            d0[4] = cv::saturate_cast<_Tp2>(s11[1]);
            d0[5] = cv::saturate_cast<_Tp2>((s11[0] + s12[0] + c1) / 2);

            // left bottom (B, G, R)
            d1[0] = cv::saturate_cast<_Tp2>((s11[3] + s10[3] + c1) / 2);
            d1[1] = cv::saturate_cast<_Tp2>(s11[2]);
            d1[2] = cv::saturate_cast<_Tp2>((s11[0] + s21[0] + c1) / 2);

            // right bottom (B, G, R)
            d1[3] = cv::saturate_cast<_Tp2>(s11[3]);
            d1[4] = cv::saturate_cast<_Tp2>((s11[1] + s11[2] + s21[1] + s12[2] + c2) / 4);
            d1[5] = cv::saturate_cast<_Tp2>((s11[0] + s21[0] + s12[0] + s22[0] + c2) / 4);
          }
        }
      });
      break;
    }

    case COLORID_BAYER_YMCY:
    case COLORID_BAYER_GRBG: {
      // R=1, G1=0, G2=3, B=2
      parallel_for(0, src_rows, [=](const auto & range) {
        const uint8_t * const bayer_base = src_base;
        uint8_t * const planes_base = dst_base;

        for (int y1 = rbegin(range), ymax = rend(range); y1 < ymax; ++y1) {
          const int y0 = std::max(0, y1 - 1);
          const int y2 = std::min(src_rows - 1, y1 + 1);

          const Vec4T* __restrict s0 = (const Vec4T*)(bayer_base + y0 * src_stride);
          const Vec4T* __restrict s1 = (const Vec4T*)(bayer_base + y1 * src_stride);
          const Vec4T* __restrict s2 = (const Vec4T*)(bayer_base + y2 * src_stride);

          _Tp2* __restrict d0 = (_Tp2*)(planes_base + (2 * y1 + 0) * dst_stride);
          _Tp2* __restrict d1 = (_Tp2*)(planes_base + (2 * y1 + 1) * dst_stride);

          for (int x1 = 0; x1 < src_cols; ++x1, d0 += 6, d1 += 6) {
            const int x0 = std::max(0, x1 - 1);
            const int x2 = std::min(src_cols - 1, x1 + 1);

            const Vec4T& s00 = s0[x0];
            const Vec4T& s01 = s0[x1];
            const Vec4T& s02 = s0[x2];

            const Vec4T& s10 = s1[x0];
            const Vec4T& s11 = s1[x1];
            const Vec4T& s12 = s1[x2];

            const Vec4T& s20 = s2[x0];
            const Vec4T& s21 = s2[x1];
            const Vec4T& s22 = s2[x2];

            // left top (B, G, R)
            d0[0] = cv::saturate_cast<_Tp2>((s11[2] + s01[2] + s10[2] + s00[2] + c2) / 4);
            d0[1] = cv::saturate_cast<_Tp2>((s11[0] + s11[3] + s10[0] + s01[3] + c2) / 4);
            d0[2] = cv::saturate_cast<_Tp2>(s11[1]);

            // right top (B, G, R)
            d0[3] = cv::saturate_cast<_Tp2>((s11[2] + s01[2] + c1) / 2);
            d0[4] = cv::saturate_cast<_Tp2>(s11[0]);
            d0[5] = cv::saturate_cast<_Tp2>((s11[1] + s12[1] + c1) / 2);

            // left bottom (B, G, R)
            d1[0] = cv::saturate_cast<_Tp2>((s11[2] + s10[2] + c1) / 2);
            d1[1] = cv::saturate_cast<_Tp2>(s11[3]);
            d1[2] = cv::saturate_cast<_Tp2>((s11[1] + s21[1] + c1) / 2);

            // right bottom (B, G, R)
            d1[3] = cv::saturate_cast<_Tp2>(s11[2]);
            d1[4] = cv::saturate_cast<_Tp2>((s11[0] + s11[3] + s21[0] + s12[3] + c2) / 4);
            d1[5] = cv::saturate_cast<_Tp2>((s11[1] + s21[1] + s12[1] + s22[1] + c2) / 4);
          }
        }
      });
      break;
    }

    case COLORID_BAYER_YCMY:
    case COLORID_BAYER_GBRG: {
      // R=2, G1=0, G2=3, B=1
      parallel_for(0, src_rows, [=](const auto & range) {
        const uint8_t * const bayer_base = src_base;
        uint8_t * const planes_base = dst_base;

        for (int y1 = rbegin(range), ymax = rend(range); y1 < ymax; ++y1) {
          const int y0 = std::max(0, y1 - 1);
          const int y2 = std::min(src_rows - 1, y1 + 1);

          const Vec4T* __restrict s0 = (const Vec4T*)(bayer_base + y0 * src_stride);
          const Vec4T* __restrict s1 = (const Vec4T*)(bayer_base + y1 * src_stride);
          const Vec4T* __restrict s2 = (const Vec4T*)(bayer_base + y2 * src_stride);

          _Tp2* __restrict d0 = (_Tp2*)(planes_base + (2 * y1 + 0) * dst_stride);
          _Tp2* __restrict d1 = (_Tp2*)(planes_base + (2 * y1 + 1) * dst_stride);

          for (int x1 = 0; x1 < src_cols; ++x1, d0 += 6, d1 += 6) {
            const int x0 = std::max(0, x1 - 1);
            const int x2 = std::min(src_cols - 1, x1 + 1);

            const Vec4T& s00 = s0[x0];
            const Vec4T& s01 = s0[x1];
            const Vec4T& s02 = s0[x2];

            const Vec4T& s10 = s1[x0];
            const Vec4T& s11 = s1[x1];
            const Vec4T& s12 = s1[x2];

            const Vec4T& s20 = s2[x0];
            const Vec4T& s21 = s2[x1];
            const Vec4T& s22 = s2[x2];

            // left top (B, G, R)
            d0[0] = cv::saturate_cast<_Tp2>((s11[1] + s01[1] + s10[1] + s00[1] + c2) / 4);
            d0[1] = cv::saturate_cast<_Tp2>((s11[0] + s11[3] + s10[0] + s01[3] + c2) / 4);
            d0[2] = cv::saturate_cast<_Tp2>(s11[2]);

            // right top (B, G, R)
            d0[3] = cv::saturate_cast<_Tp2>((s11[1] + s01[1] + c1) / 2);
            d0[4] = cv::saturate_cast<_Tp2>(s11[0]);
            d0[5] = cv::saturate_cast<_Tp2>((s11[2] + s12[2] + c1) / 2);

            // left bottom (B, G, R)
            d1[0] = cv::saturate_cast<_Tp2>((s11[1] + s10[1] + c1) / 2);
            d1[1] = cv::saturate_cast<_Tp2>(s11[3]);
            d1[2] = cv::saturate_cast<_Tp2>((s11[2] + s21[2] + c1) / 2);

            // right bottom (B, G, R)
            d1[3] = cv::saturate_cast<_Tp2>(s11[1]);
            d1[4] = cv::saturate_cast<_Tp2>((s11[0] + s11[3] + s21[0] + s12[3] + c2) / 4);
            d1[5] = cv::saturate_cast<_Tp2>((s11[2] + s21[2] + s12[2] + s22[2] + c2) / 4);
          }
        }
      });
      break;
    }

    case COLORID_BAYER_CYYM:
    case COLORID_BAYER_BGGR: {
      // R=3, G1=1, G2=2, B=0
      parallel_for(0, src_rows, [=](const auto & range) {
        const uint8_t * const bayer_base = src_base;
        uint8_t * const planes_base = dst_base;

        for (int y1 = rbegin(range), ymax = rend(range); y1 < ymax; ++y1) {
          const int y0 = std::max(0, y1 - 1);
          const int y2 = std::min(src_rows - 1, y1 + 1);

          const Vec4T* __restrict s0 = (const Vec4T*)(bayer_base + y0 * src_stride);
          const Vec4T* __restrict s1 = (const Vec4T*)(bayer_base + y1 * src_stride);
          const Vec4T* __restrict s2 = (const Vec4T*)(bayer_base + y2 * src_stride);

          _Tp2* __restrict d0 = (_Tp2*)(planes_base + (2 * y1 + 0) * dst_stride);
          _Tp2* __restrict d1 = (_Tp2*)(planes_base + (2 * y1 + 1) * dst_stride);

          for (int x1 = 0; x1 < src_cols; ++x1, d0 += 6, d1 += 6) {
            const int x0 = std::max(0, x1 - 1);
            const int x2 = std::min(src_cols - 1, x1 + 1);

            const Vec4T& s00 = s0[x0];
            const Vec4T& s01 = s0[x1];
            const Vec4T& s02 = s0[x2];

            const Vec4T& s10 = s1[x0];
            const Vec4T& s11 = s1[x1];
            const Vec4T& s12 = s1[x2];

            const Vec4T& s20 = s2[x0];
            const Vec4T& s21 = s2[x1];
            const Vec4T& s22 = s2[x2];

            // left top (B, G, R)
            d0[0] = cv::saturate_cast<_Tp2>((s11[0] + s01[0] + s10[0] + s00[0] + c2) / 4);
            d0[1] = cv::saturate_cast<_Tp2>((s11[1] + s11[2] + s10[1] + s01[2] + c2) / 4);
            d0[2] = cv::saturate_cast<_Tp2>(s11[3]);
            // right top (B, G, R)
            d0[3] = cv::saturate_cast<_Tp2>((s11[0] + s01[0] + c1) / 2);
            d0[4] = cv::saturate_cast<_Tp2>(s11[1]);
            d0[5] = cv::saturate_cast<_Tp2>((s11[3] + s12[3] + c1) / 2);
            // left bottom (B, G, R)
            d1[0] = cv::saturate_cast<_Tp2>((s11[0] + s10[0] + c1) / 2);
            d1[1] = cv::saturate_cast<_Tp2>(s11[2]);
            d1[2] = cv::saturate_cast<_Tp2>((s11[3] + s21[3] + c1) / 2);
            // right bottom (B, G, R)
            d1[3] = cv::saturate_cast<_Tp2>(s11[0]);
            d1[4] = cv::saturate_cast<_Tp2>((s11[1] + s11[2] + s21[1] + s12[2] + c2) / 4);
            d1[5] = cv::saturate_cast<_Tp2>((s11[3] + s21[3] + s12[3] + s22[3] + c2) / 4);
          }
        }
      });
      break;
    }
    default:
      CF_ERROR("Invalid or not supported colorid = %d (%s)", colorid, toCString(colorid));
      return false;
  }
  _dst.move(dst);
  return true;
}

/** @brief
 * Combine input 4-channel bayer planes src image into 3-channel BGR dst matrix using NN interpolaton.
 * The output size of dst is twce large to src size
 */
bool interpolate_bayer_planes(cv::InputArray src, cv::OutputArray dst, enum COLORID colorid, int ddepth /*= -1*/)
{
  if ( src.channels() != 4 ) {
    CF_ERROR("Invalid number of channel=%d in input image. Must be 1", src.channels());
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

  CV_DISPATCH2(src.depth(), ddepth, _interpolate_bayer_planes, src, dst, colorid, ddepth);
  return false;
}
