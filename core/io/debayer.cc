/*
 * debayer.cc
 *
 *  Created on: Jul 31, 2020
 *      Author: amyznikov
 */

#include "debayer.h"
#include <core/proc/run-loop.h>
#include <core/proc/pixtype.h>
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
      { COLORID_BGRA, "BGRA", "" },
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
      {DEBAYER_AVGC,  "AVGC",    "DEBAYER_AVGC: 2x2 pixel binning"},
      {DEBAYER_MATRIX,"MATRIX", "DEBAYER_MATRIX: Don't debayer but create colored bayer matrix image"},
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

template<class _Tp>
static bool _extract_bayer_planes(cv::InputArray _src, cv::OutputArray _dst, enum COLORID colorid)
{
  if( (_src.cols() & 0x1) || (_src.rows() & 0x1) || _src.channels() != 1 ) {
    CF_ERROR("Can not make debayer for uneven image size %dx%dx%d",
        _src.cols(), _src.rows(), _src.channels());
    return false;
  }

  using Vec4T = cv::Vec<_Tp, 4>;
  const cv::Mat_<_Tp> src = _src.getMat();
  cv::Mat_<Vec4T> dst(_src.size() / 2);

  switch (colorid) {
    case COLORID_BAYER_MYYC:
    case COLORID_BAYER_RGGB: {
      // RGGB -> [R G1 B G2]
      parallel_for(0, dst.rows, [&, cols = dst.cols](const auto & range) {
        for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
          const _Tp * r0 = src[y * 2 + 0];
          const _Tp * r1 = src[y * 2 + 1];
          Vec4T * dstp = dst[y];
          for (int x = 0; x < cols; ++x) {
            dstp[x][0] = r0[2 * x + 0];
            dstp[x][1] = r0[2 * x + 1];
            dstp[x][3] = r1[2 * x + 0];
            dstp[x][2] = r1[2 * x + 1];
          }
        }
      });
      break;
    }

    case COLORID_BAYER_YMCY:
    case COLORID_BAYER_GRBG:{
      // GRBG -> [R G1 B G2]
      parallel_for(0, dst.rows, [&, cols = dst.cols](const auto & range) {
        for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
          const _Tp * r0 = src[y * 2 + 0];
          const _Tp * r1 = src[y * 2 + 1];
          Vec4T * dstp = dst[y];
          for (int x = 0; x < cols; ++x) {
            dstp[x][1] = r0[2 * x + 0];
            dstp[x][0] = r0[2 * x + 1];
            dstp[x][2] = r1[2 * x + 0];
            dstp[x][3] = r1[2 * x + 1];
          }
        }
      });
      break;
    }

    case COLORID_BAYER_YCMY:
    case COLORID_BAYER_GBRG: {
      // GBRG -> [R G1 B G2]
      parallel_for(0, dst.rows, [&, cols = dst.cols](const auto & range) {
        for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
          const _Tp * r0 = src[y * 2 + 0];
          const _Tp * r1 = src[y * 2 + 1];
          Vec4T * dstp = dst[y];
          for (int x = 0; x < cols; ++x) {
            dstp[x][1] = r0[2 * x + 0];
            dstp[x][2] = r0[2 * x + 1];
            dstp[x][0] = r1[2 * x + 0];
            dstp[x][3] = r1[2 * x + 1];
          }
        }
      });
      break;
    }

    case COLORID_BAYER_CYYM:
    case COLORID_BAYER_BGGR: {
      // BGGR -> [R G1 B G2]
      parallel_for(0, dst.rows, [&, cols = dst.cols](const auto & range) {
        for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
          const _Tp * r0 = src[y * 2 + 0];
          const _Tp * r1 = src[y * 2 + 1];
          Vec4T * dstp = dst[y];
          for (int x = 0; x < cols; ++x) {
            dstp[x][2] = r0[2 * x + 0];
            dstp[x][1] = r0[2 * x + 1];
            dstp[x][3] = r1[2 * x + 0];
            dstp[x][0] = r1[2 * x + 1];
          }
        }
      });
      break;
    }

    default:
      CF_ERROR("Not supported colorid = %d", colorid);
      return false;
  }

  if( _dst.fixedType() ) {
    dst.convertTo(_dst, _dst.depth());
  }
  else {
    _dst.move(dst);
  }

  return true;
}

/** @brief
 * Extract src into 4-channel dst matrix with 4 bayer planes ordered as[ R G1 B G2 ]
 * The output size of dst is twice smaller than src
 */
bool extract_bayer_planes(cv::InputArray src, cv::OutputArray dst, enum COLORID colorid)
{
  INSTRUMENT_REGION("");

  if ( !is_bayer_pattern(colorid) ) {
    CF_ERROR("Invalid argument: colorid=%d is no a bayer pattern",
        colorid);
    return false;
  }

  CV_DISPATCH(src.depth(), _extract_bayer_planes, src, dst, colorid);

  return false;
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

  const int w = _src.cols();
  const int h = _src.rows();

  const cv::Mat_<Vec4T> src = _src.getMat();
  cv::Mat_<Vec3T> dst(h, w);

  parallel_for(0, h, [&, w](const auto & range) {
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
      const Vec4T * __restrict srcp = src[y];
      Vec3T * __restrict dstp = dst[y];
      for ( int x = 0; x < w; ++x ) {
        dstp[x][0] = _Tp2(srcp[x][2]);
        dstp[x][1] = _Tp2((1 + srcp[x][1] + srcp[x][3]) / 2);
        dstp[x][2] = _Tp2(srcp[x][0]);
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

template<typename _Tp1, typename _Tp2>
static bool _debayer_avgc(cv::InputArray _src, cv::OutputArray _dst, COLORID colorid)
{
  const int h = _src.rows() / 2;
  const int w = _src.cols() / 2;

  const cv::Mat_<_Tp1> src = _src.getMat();

  cv::Mat_<cv::Vec<_Tp2, 3>> dst(h, w);

  switch (colorid) {
    case COLORID_BAYER_MYYC:
    case COLORID_BAYER_RGGB:
      // RGGB -> [B G R]
      parallel_for(0, h, [&, w](const auto & range) {
        for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
          const _Tp1 * __restrict s0 = src[y * 2 + 0], * __restrict s1 = src[y * 2 + 1];
          _Tp2 * __restrict dstp = (_Tp2 * )(dst[y]);
          for (int x = 0; x < w; ++x, dstp += 3) {
            dstp[0] = _Tp2(s1[2 * x + 1]);
            dstp[1] = _Tp2((1 + s0[2 * x + 1] + s1[2 * x + 0]) / 2);
            dstp[2] = _Tp2(s0[2 * x + 0]);
          }
        }
      });
      break;

    case COLORID_BAYER_YMCY:
    case COLORID_BAYER_GRBG:
      // GRBG -> [B G R]
      parallel_for(0, h, [&, w](const auto & range) {
        for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
          const _Tp1 * __restrict s0 = src[y * 2 + 0], * __restrict s1 = src[y * 2 + 1];
          _Tp2 * __restrict dstp = (_Tp2 * )(dst[y]);
          for (int x = 0; x < w; ++x, dstp += 3) {
            dstp[0] = _Tp2(s1[2 * x + 0]);
            dstp[1] = _Tp2((1 + s0[2 * x + 0] + s1[2 * x + 1]) / 2);
            dstp[2] = _Tp2(s0[2 * x + 1]);
          }
        }
      });
      break;

    case COLORID_BAYER_YCMY:
    case COLORID_BAYER_GBRG:
      // GBRG -> [B G R]
      parallel_for(0, h, [&, w](const auto & range) {
        for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
          const _Tp1 * __restrict s0 = src[y * 2 + 0], * __restrict s1 = src[y * 2 + 1];
          _Tp2 * __restrict dstp = (_Tp2 * )(dst[y]);
          for (int x = 0; x < w; ++x, dstp += 3) {
            dstp[0] = _Tp2(s0[2 * x + 1]);
            dstp[1] = _Tp2((1 + s0[2 * x + 0] + s1[2 * x + 1]) / 2);
            dstp[2] = _Tp2(s1[2 * x + 0]);
          }
        }
      });
      break;

    case COLORID_BAYER_CYYM:
    case COLORID_BAYER_BGGR:
      // BGGR -> [B G R]
      parallel_for(0, h, [&, w](const auto & range) {
        for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
          const _Tp1 * __restrict s0 = src[y * 2 + 0], * __restrict s1 = src[y * 2 + 1];
          _Tp2 * __restrict dstp = (_Tp2 * )(dst[y]);
          for (int x = 0; x < w; ++x, dstp += 3) {
            dstp[0] = _Tp2(s0[2 * x + 0]);
            dstp[1] = _Tp2((1 + s0[2 * x + 1] + s1[2 * x + 0]) / 2);
            dstp[2] = _Tp2(s1[2 * x + 1]);
          }
        }
      });
      break;

    default: // Not supported
      CF_ERROR("Not supported colorid = %d", colorid);
      return false;
  }

  _dst.move(dst);
  return true;
}

bool debayer_avgc(cv::InputArray src, cv::OutputArray dst, COLORID colorid, int ddepth)
{
  INSTRUMENT_REGION("");

  if( (src.cols() & 0x1) || (src.rows() & 0x1) || src.channels() != 1 ) {
    CF_ERROR("Can not make debayer for uneven image size %dx%dx%d",
        src.cols(), src.rows(), src.channels());
    return false;
  }

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

  CV_DISPATCH2(src.depth(), ddepth, _debayer_avgc, src, dst, colorid);

  CF_ERROR("Not supported combination of src.depth()=%d and ddepth=%d",
      src.depth(), ddepth);

  return false;
}

template<class _Tp1, class _Tp2>
bool _debayer_nn_interpolation(cv::InputArray _src, cv::OutputArray _dst, enum COLORID colorid)
{
  if ( (_src.cols() & 0x1) || (_src.rows() & 0x1) || _src.channels() != 1 )  {
    CF_ERROR("Can not make debayer for uneven image size %dx%dx%d",
        _src.cols(), _src.rows(), _src.channels());
    return false;
  }

  using Vec3T = cv::Vec<_Tp2, 3>;
  using Mat3T = cv::Mat_<Vec3T>;

  const cv::Size size = _src.size();

  Mat3T dst(_src.size());

  cv::Mat_<_Tp1> src;
  cv::copyMakeBorder(_src, src, 1, 1, 1, 1, cv::BORDER_REFLECT101);

  switch (colorid) {
    case COLORID_BAYER_MYYC:
    case COLORID_BAYER_RGGB: {
      // R G R G R G
      // G B G B G B
      // R G R G R G
      // G B G B G B

      parallel_for(0, size.height, [=, &dst](const auto & range) {
        for( int y = rbegin(range), ymax = rend(range); y < ymax; ++y ) {
          const _Tp1 * __restrict s0 = src[y + 0] + 1;
          const _Tp1 * __restrict s1 = src[y + 1] + 1;
          const _Tp1 * __restrict s2 = src[y + 2] + 1;
          _Tp2 * __restrict dstp = (_Tp2 * )(dst[y]);

          if ( !(y & 0x1) ) { // R G
            for( int x = 0; x < size.width; x += 2, dstp += 6 ) {
              dstp[0] = _Tp2((2 + s0[x-1] + s0[x+1] + s2[x-1] + s2[x+1]) / 4);
              dstp[1] = _Tp2((2 + s0[x] + s1[x-1] + s1[x+1] + s2[x]) / 4);
              dstp[2] = _Tp2(s1[x]);
              dstp[3] = _Tp2((1 + s0[x+1] + s2[x+1]) / 2);
              dstp[4] = _Tp2(s1[x+1]);
              dstp[5] = _Tp2((1 + s1[x-1+1] + s1[x+1+1]) / 2);
            }
          }
          else { // G B
            for( int x = 0; x < size.width; x += 2, dstp += 6  ) {
              dstp[0] = _Tp2((1 + s1[x-1] + s1[x+1]) / 2);
              dstp[1] = _Tp2(s1[x]);
              dstp[2] = _Tp2((1 + s0[x] + s2[x]) / 2);
              dstp[3] = _Tp2(s1[x+1]);
              dstp[4] = _Tp2((2 + s0[x+1] + s1[x-1+1] + s1[x+1+1] + s2[x+1]) / 4);
              dstp[5] = _Tp2((2 + s0[x-1+1] + s0[x+1+1] + s2[x-1+1] + s2[x+1+1]) / 4);
            }
          }
        }
      });

      break;
    }

    case COLORID_BAYER_YMCY:
    case COLORID_BAYER_GRBG: {
      parallel_for(0, size.height, [=, &dst](const auto & range) {
        for( int y = rbegin(range), ymax = rend(range); y < ymax; ++y ) {
          const _Tp1 * __restrict s0 = src[y + 0] + 1;
          const _Tp1 * __restrict s1 = src[y + 1] + 1;
          const _Tp1 * __restrict s2 = src[y + 2] + 1;
          _Tp2 * __restrict dstp = (_Tp2 * )(dst[y]);

          if ( !(y & 0x1) ) { // G R
            for( int x = 0; x < size.width; x += 2, dstp += 6 ) {
              dstp[0] = _Tp2((1 + s0[x] + s2[x]) / 2); // B
              dstp[1] = _Tp2(s1[x]); // G
              dstp[2] = _Tp2((1 + s1[x-1] + s1[x+1]) / 2); // R
              dstp[3] = _Tp2((2 + s0[x] + s0[x+2] + s2[x] + s2[x+2]) / 4); // B
              dstp[4] = _Tp2((2 + s0[x+1] + s1[x] + s1[x+2] + s2[x+1]) / 4); // G
              dstp[5] = _Tp2(s1[x+1]); // R
            }
          }
          else { // B G
            for( int x = 0; x < size.width; x += 2, dstp += 6 ) {
              dstp[0] = _Tp2(s1[x]); // B
              dstp[1] = _Tp2((2 + s0[x] + s1[x-1] + s1[x+1] + s2[x]) / 4); // G
              dstp[2] = _Tp2((2 + s0[x-1] + s0[x+1] + s2[x-1] + s2[x+1]) / 4); // R
              dstp[3] = _Tp2((1 + s1[x] + s1[x+2]) / 2); // B
              dstp[4] = _Tp2(s1[x+1]); // G
              dstp[5] = _Tp2((1 + s0[x+1] + s2[x+1]) / 2); // R
            }
          }
        }
      });
      break;
    }

    case COLORID_BAYER_YCMY:
    case COLORID_BAYER_GBRG: {
      parallel_for(0, size.height, [=, &dst](const auto & range) {
        for( int y = rbegin(range), ymax = rend(range); y < ymax; ++y ) {
          const _Tp1 * __restrict s0 = src[y + 0] + 1;
          const _Tp1 * __restrict s1 = src[y + 1] + 1;
          const _Tp1 * __restrict s2 = src[y + 2] + 1;
          _Tp2 * __restrict dstp = (_Tp2 * )(dst[y]);

          if ( !(y & 0x1) ) { // G B
            for( int x = 0; x < size.width; x += 2, dstp += 6 ) {
              dstp[0] = _Tp2((1 + s1[x-1] + s1[x+1]) / 2); // B
              dstp[1] = _Tp2(s1[x]); // G
              dstp[2] = _Tp2((1 + s0[x] + s2[x]) / 2); // R
              dstp[3] = _Tp2(s1[x+1]); // B
              dstp[4] = _Tp2((2 + s0[x+1] + s1[x] + s1[x+2] + s2[x+1]) / 4); // G
              dstp[5] = _Tp2((2 + s0[x] + s0[x+2] + s2[x] + s2[x+2]) / 4); // R
            }
          }
          else { // R G
            for( int x = 0; x < size.width; x += 2, dstp += 6 ) {
              dstp[0] = _Tp2((2 + s0[x-1] + s0[x+1] + s2[x-1] + s2[x+1]) / 4); // B
              dstp[1] = _Tp2((2 + s0[x] + s1[x-1] + s1[x+1] + s2[x]) / 4); // G
              dstp[2] = _Tp2(s1[x]); // R
              dstp[3] = _Tp2((1 + s0[x+1] + s2[x+1]) / 2); // B
              dstp[4] = _Tp2(s1[x+1]); // G
              dstp[5] = _Tp2((1 + s1[x] + s1[x+2]) / 2); // R
            }
          }
        }
      });
      break;
    }

    case COLORID_BAYER_CYYM:
    case COLORID_BAYER_BGGR: {
      parallel_for(0, size.height, [=, &dst](const auto & range) {
        for( int y = rbegin(range), ymax = rend(range); y < ymax; ++y ) {
          const _Tp1 * __restrict s0 = src[y + 0] + 1;
          const _Tp1 * __restrict s1 = src[y + 1] + 1;
          const _Tp1 * __restrict s2 = src[y + 2] + 1;
          _Tp2 * __restrict dstp = (_Tp2 * )(dst[y]);

          if ( !(y & 0x1) ) { // B G
            for( int x = 0; x < size.width; x += 2, dstp += 6 ) {
              dstp[0] = _Tp2(s1[x]); // B
              dstp[1] = _Tp2((2 + s0[x] + s1[x-1] + s1[x+1] + s2[x]) / 4); // G
              dstp[2] = _Tp2((2 + s0[x-1] + s0[x+1] + s2[x-1] + s2[x+1]) / 4); // R
              dstp[3] = _Tp2((1 + s1[x] + s1[x+2]) / 2); // B
              dstp[4] = _Tp2(s1[x+1]); // G
              dstp[5] = _Tp2((1 + s0[x+1] + s2[x+1]) / 2); // R
            }
          }
          else { // G R
            for( int x = 0; x < size.width; x += 2, dstp += 6 ) {
              dstp[0] = _Tp2((1 + s0[x] + s2[x]) / 2); // B
              dstp[1] = _Tp2(s1[x]); // G
              dstp[2] = _Tp2((1 + s1[x-1] + s1[x+1]) / 2); // R
              dstp[3] = _Tp2((2 + s0[x] + s1[x] + s1[x+2] + s2[x]) / 4); // B
              dstp[4] = _Tp2((2 + s0[x+1] + s1[x] + s1[x+2] + s2[x+1]) / 4); // G
              dstp[5] = _Tp2(s1[x+1]); // R
            }
          }
        }
      });
      break;
    }

    default:
      CF_ERROR("Not supported colorid=%d (%s) for debayer",
          (int)(colorid), toCString(colorid));
      return false;
  }

  _dst.move(dst);

  return true;
}

bool debayer_nn(cv::InputArray src, cv::OutputArray dst, enum COLORID colorid, int ddepth)
{
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

  CV_DISPATCH2(src.depth(), ddepth, _debayer_nn_interpolation, src, dst, colorid);

  CF_ERROR("Not supported combination of src.depth()=%d and ddepth=%d",
      src.depth(), ddepth);

  return false;
}


/** @brief Bayer demosaicing
 */
bool debayer(cv::InputArray src, cv::OutputArray dst, enum COLORID colorid, enum DEBAYER_ALGORITHM algo)
{
  if ( algo == DEBAYER_DEFAULT ) {
    algo = default_debayer_algorithm();
  }

  switch (algo) {
    case DEBAYER_DISABLE:
      src.copyTo(dst);
      return true;
    case DEBAYER_NN:
      if( src.depth() != CV_8U && src.depth() != CV_16U ) {
        return debayer_nn(src, dst, colorid);
      }
      break;
    case DEBAYER_NN2:
      return debayer_nn(src, dst, colorid);
    case DEBAYER_AVGC:
      return debayer_avgc(src, dst, colorid);
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
bool is_corrupted_asi_bayer_frame(const cv::Mat & bayer_image, COLORID bayer_pattern,
    double median_hat_threshold)
{
  cv::Mat tmp;
  cv::Mat mb;

  if ( !extract_bayer_planes(bayer_image, tmp, bayer_pattern) ) {
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

