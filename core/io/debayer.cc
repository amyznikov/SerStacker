/*
 * debayer.cc
 *
 *  Created on: Jul 31, 2020
 *      Author: amyznikov
 */

#include "debayer.h"
#include <core/proc/run-loop.h>
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
      {DEBAYER_NNR,   "NNR",    "DEBAYER_NNR: SerStacker nearest-neighboor interpolation with nninterpolate() and midian-bease noise reduction"},
      {DEBAYER_EA,    "EA",     "DEBAYER_EA: OpenCV EA (edge aware) interpolation with cv::demosaicing()"},
      {DEBAYER_VNG,   "VNG",    "DEBAYER_VNG: OpenCV VNG interpolation with cv::demosaicing()"},
      {DEBAYER_AVGC,  "AVGC",    "DEBAYER_AVGC: 2x2 pixel binning"},
      {DEBAYER_MATRIX,"MATRIX", "DEBAYER_MATRIX: Don't debayer but create colored bayer matrix image"},
      {DEBAYER_NN, } // must  be last
  };
  return members;
}


namespace {


static DEBAYER_ALGORITHM g_default_debayer_algorithm = DEBAYER_NN2;

template<class T> inline constexpr T cvmax() { return 1; };
template<> inline constexpr uint8_t cvmax<uint8_t>() { return UINT8_MAX; }
template<> inline constexpr int8_t cvmax<int8_t>() { return INT8_MAX; }
template<> inline constexpr uint16_t cvmax<uint16_t>() { return UINT16_MAX; }
template<> inline constexpr int16_t cvmax<int16_t>() { return INT16_MAX; }
template<> inline constexpr uint32_t cvmax<uint32_t>() { return UINT32_MAX; }
template<> inline constexpr int32_t cvmax<int32_t>() { return INT32_MAX; }


/** @brief
 * Max native value for given depth
 */
inline cv::Scalar depthmax(int depth)
{
  switch ( depth ) {
  case CV_8U :
    return cv::Scalar::all(UINT8_MAX);
  case CV_8S :
    return cv::Scalar::all(INT8_MAX);
  case CV_16U :
    return cv::Scalar::all(UINT16_MAX);
  case CV_16S :
    return cv::Scalar::all(INT16_MAX);
  case CV_32S :
    return cv::Scalar::all(INT32_MAX);
  case CV_32F :
    return cv::Scalar::all(1);
  case CV_64F :
    return cv::Scalar::all(1);
  }

  return cv::Scalar::all(0);
}

} // namespace


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
      parallel_for(0, dst.rows, [&, cols = dst.cols](const auto & range) {
        for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
          const _Tp * r0 = src[y * 2 + 0];
          const _Tp * r1 = src[y * 2 + 1];
          Vec4T * dstp = dst[y];
          // RGGB -> [R G1 B G2]
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
      parallel_for(0, dst.rows, [&, cols = dst.cols](const auto & range) {
        for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
          const _Tp * r0 = src[y * 2 + 0];
          const _Tp * r1 = src[y * 2 + 1];
          Vec4T * dstp = dst[y];
          // GRBG -> [R G1 B G2]
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
      parallel_for(0, dst.rows, [&, cols = dst.cols](const auto & range) {
        for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
          const _Tp * r0 = src[y * 2 + 0];
          const _Tp * r1 = src[y * 2 + 1];
          Vec4T * dstp = dst[y];
          // GBRG -> [R G1 B G2]
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
      parallel_for(0, dst.rows, [&, cols = dst.cols](const auto & range) {
        for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
          const _Tp * r0 = src[y * 2 + 0];
          const _Tp * r1 = src[y * 2 + 1];
          Vec4T * dstp = dst[y];
          // BGGR -> [R G1 B G2]
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

  switch ( src.depth() ) {
  case CV_8U :
    return _extract_bayer_planes<uint8_t>(src, dst, colorid);
  case CV_8S :
    return _extract_bayer_planes<int8_t>(src, dst, colorid);
  case CV_16U :
    return _extract_bayer_planes<uint16_t>(src, dst, colorid);
  case CV_16S :
    return _extract_bayer_planes<int16_t>(src, dst, colorid);
  case CV_32S :
    return _extract_bayer_planes<int32_t>(src, dst, colorid);
  case CV_32F :
    return _extract_bayer_planes<float>(src, dst, colorid);
  case CV_64F :
    return _extract_bayer_planes<double>(src, dst, colorid);
  }

  return false;
}




/** @brief
 * Combine input 4-channel src ordered as [ R G1 B G2 ] into 3-channel BGR dst matrix.
 * The output size of dst is the same as src
 */
template<class _Tp1, class _Tp2>
static bool _bayer_planes_to_bgr(cv::InputArray _src, cv::OutputArray _dst)
{
  using CalcType = typename std::conditional_t<std::is_floating_point<_Tp2>::value, _Tp2, int32_t>;

  using Vec4T = cv::Vec<_Tp1, 4>;
  using Vec3T = cv::Vec<_Tp2, 3>;

  const int w = _src.cols();
  const int h = _src.rows();

  const cv::Mat_<Vec4T> src = _src.getMat();
  cv::Mat_<Vec3T> dst(h, w);

  parallel_for(0, h, [&, w](const auto & range) {
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
      const Vec4T * srcp = src[y];
      Vec3T * __restrict dstp = dst[y];
      for ( int x = 0; x < w; ++x ) {
        dstp[x][0] = srcp[x][2];
        dstp[x][1] = _Tp2((CalcType(srcp[x][1]) + CalcType(srcp[x][3])) / 2);
        dstp[x][2] = srcp[x][0];
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

  switch (src.depth()) {
    case CV_8U:
      switch (ddepth) {
        case CV_8U:
          return _bayer_planes_to_bgr<uint8_t, uint8_t>(src, dst);
        case CV_8S:
          return _bayer_planes_to_bgr<uint8_t, int8_t>(src, dst);
        case CV_16U:
          return _bayer_planes_to_bgr<uint8_t, uint16_t>(src, dst);
        case CV_16S:
          return _bayer_planes_to_bgr<uint8_t, int16_t>(src, dst);
        case CV_32S:
          return _bayer_planes_to_bgr<uint8_t, int32_t>(src, dst);
        case CV_32F:
          return _bayer_planes_to_bgr<uint8_t, float>(src, dst);
        case CV_64F:
          return _bayer_planes_to_bgr<uint8_t, double>(src, dst);
      }
      break;

    case CV_8S:
      switch (ddepth) {
        case CV_8U:
          return _bayer_planes_to_bgr<int8_t, uint8_t>(src, dst);
        case CV_8S:
          return _bayer_planes_to_bgr<int8_t, int8_t>(src, dst);
        case CV_16U:
          return _bayer_planes_to_bgr<int8_t, uint16_t>(src, dst);
        case CV_16S:
          return _bayer_planes_to_bgr<int8_t, int16_t>(src, dst);
        case CV_32S:
          return _bayer_planes_to_bgr<int8_t, int32_t>(src, dst);
        case CV_32F:
          return _bayer_planes_to_bgr<int8_t, float>(src, dst);
        case CV_64F:
          return _bayer_planes_to_bgr<int8_t, double>(src, dst);
      }
      break;
    case CV_16U:
      switch (ddepth) {
        case CV_8U:
          return _bayer_planes_to_bgr<uint16_t, uint8_t>(src, dst);
        case CV_8S:
          return _bayer_planes_to_bgr<uint16_t, int8_t>(src, dst);
        case CV_16U:
          return _bayer_planes_to_bgr<uint16_t, uint16_t>(src, dst);
        case CV_16S:
          return _bayer_planes_to_bgr<uint16_t, int16_t>(src, dst);
        case CV_32S:
          return _bayer_planes_to_bgr<uint16_t, int32_t>(src, dst);
        case CV_32F:
          return _bayer_planes_to_bgr<uint16_t, float>(src, dst);
        case CV_64F:
          return _bayer_planes_to_bgr<uint16_t, double>(src, dst);
      }
      break;
    case CV_16S:
      switch (ddepth) {
        case CV_8U:
          return _bayer_planes_to_bgr<int16_t, uint8_t>(src, dst);
        case CV_8S:
          return _bayer_planes_to_bgr<int16_t, int8_t>(src, dst);
        case CV_16U:
          return _bayer_planes_to_bgr<int16_t, uint16_t>(src, dst);
        case CV_16S:
          return _bayer_planes_to_bgr<int16_t, int16_t>(src, dst);
        case CV_32S:
          return _bayer_planes_to_bgr<int16_t, int32_t>(src, dst);
        case CV_32F:
          return _bayer_planes_to_bgr<int16_t, float>(src, dst);
        case CV_64F:
          return _bayer_planes_to_bgr<int16_t, double>(src, dst);
      }
      break;
    case CV_32S:
      switch (ddepth) {
        case CV_8U:
          return _bayer_planes_to_bgr<int32_t, uint8_t>(src, dst);
        case CV_8S:
          return _bayer_planes_to_bgr<int32_t, int8_t>(src, dst);
        case CV_16U:
          return _bayer_planes_to_bgr<int32_t, uint16_t>(src, dst);
        case CV_16S:
          return _bayer_planes_to_bgr<int32_t, int16_t>(src, dst);
        case CV_32S:
          return _bayer_planes_to_bgr<int32_t, int32_t>(src, dst);
        case CV_32F:
          return _bayer_planes_to_bgr<int32_t, float>(src, dst);
        case CV_64F:
          return _bayer_planes_to_bgr<int32_t, double>(src, dst);
      }
      break;
    case CV_32F:
      switch (ddepth) {
        case CV_8U:
          return _bayer_planes_to_bgr<float, uint8_t>(src, dst);
        case CV_8S:
          return _bayer_planes_to_bgr<float, int8_t>(src, dst);
        case CV_16U:
          return _bayer_planes_to_bgr<float, uint16_t>(src, dst);
        case CV_16S:
          return _bayer_planes_to_bgr<float, int16_t>(src, dst);
        case CV_32S:
          return _bayer_planes_to_bgr<float, int32_t>(src, dst);
        case CV_32F:
          return _bayer_planes_to_bgr<float, float>(src, dst);
        case CV_64F:
          return _bayer_planes_to_bgr<float, double>(src, dst);
      }
      break;
    case CV_64F:
      switch (ddepth) {
        case CV_8U:
          return _bayer_planes_to_bgr<double, uint8_t>(src, dst);
        case CV_8S:
          return _bayer_planes_to_bgr<double, int8_t>(src, dst);
        case CV_16U:
          return _bayer_planes_to_bgr<double, uint16_t>(src, dst);
        case CV_16S:
          return _bayer_planes_to_bgr<double, int16_t>(src, dst);
        case CV_32S:
          return _bayer_planes_to_bgr<double, int32_t>(src, dst);
        case CV_32F:
          return _bayer_planes_to_bgr<double, float>(src, dst);
        case CV_64F:
          return _bayer_planes_to_bgr<double, double>(src, dst);
      }
      break;
  }

  return false;
}

/** @brief
 * Extract bayer src into dense 3-channel BGR dst matrix with .
 * The output size of dst is the same as src
 */

template<class _Tp>
static bool _extract_bayer_matrix(cv::InputArray _src, cv::OutputArray _dst, enum COLORID colorid)
{
  typedef tbb::blocked_range<int> range;

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
        const auto * src0 = src[2 * y + 0], * src1 = src[2 * y + 1];
        auto * dst0 = dst[2 * y + 0], * dst1 = dst[2 * y + 1];
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
        const auto * src0 = src[2 * y + 0], * src1 = src[2 * y + 1];
        auto * dst0 = dst[2 * y + 0], * dst1 = dst[2 * y + 1];
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
        const auto * src0 = src[2 * y + 0], * src1 = src[2 * y + 1];
        auto * dst0 = dst[2 * y + 0], * dst1 = dst[2 * y + 1];
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
        const auto * src0 = src[2 * y + 0], * src1 = src[2 * y + 1];
        auto * dst0 = dst[2 * y + 0], * dst1 = dst[2 * y + 1];
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
bool extract_bayer_matrix(cv::InputArray src, cv::OutputArray dst, enum COLORID colorid)
{
  switch (src.depth()) {
    case CV_8U:
      return _extract_bayer_matrix<uint8_t>(src, dst, colorid);
    case CV_8S:
      return _extract_bayer_matrix<int8_t>(src, dst, colorid);
    case CV_16U:
      return _extract_bayer_matrix<uint16_t>(src, dst, colorid);
    case CV_16S:
      return _extract_bayer_matrix<int16_t>(src, dst, colorid);
    case CV_32S:
      return _extract_bayer_matrix<int32_t>(src, dst, colorid);
    case CV_32F:
      return _extract_bayer_matrix<float>(src, dst, colorid);
    case CV_64F:
      return _extract_bayer_matrix<double>(src, dst, colorid);
  }
  return false;
}

static bool demosaic_nnr(cv::InputArray src, cv::OutputArray dst, enum COLORID colorid, enum DEBAYER_ALGORITHM algo)
{
  static const auto removeBadPixels =
      [](cv::Mat & image) {

        cv::Mat medianImage, variationImage, meanVariationImage;

        static float K[3*3] = {
          1./8, 1./8, 1./8,
          1./8, 0.0, 1./8,
          1./8, 1./8, 1./8,
        };

        cv::medianBlur(image,
            medianImage,
            3);

        cv::absdiff(image,
            medianImage,
            variationImage);

        cv::filter2D(variationImage,
            meanVariationImage,
            -1,
            cv::Mat1f(3, 3, K));

        medianImage.copyTo(image,
            variationImage > 10 * meanVariationImage);
      };

  cv::Mat tmp;

  if( !extract_bayer_planes(src, tmp, colorid) ) {
    CF_DEBUG("extract_bayer_planes() fails");
    return false;
  }

  if ( algo == DEBAYER_NNR ) {
    removeBadPixels(tmp);
  }

  if( !nninterpolation(tmp, dst, colorid) ) {
    CF_DEBUG("nninterpolation() fails");
    return false;
  }

  return true;
}

static bool demosaic(cv::InputArray src, cv::OutputArray dst, enum COLORID colorid, enum DEBAYER_ALGORITHM algo)
{
  if( algo == DEBAYER_DEFAULT && (algo = default_debayer_algorithm()) == DEBAYER_DEFAULT ) {
    algo = DEBAYER_NN2;
  }

  if ( algo == DEBAYER_VNG && src.depth() != CV_8U ) {
    //  OpenCV(4.6.0) /modules/imgproc/src/demosaicing.cpp:1740:
    //    error: (-215:Assertion failed) depth == CV_8U in function 'demosaicing'
    algo = DEBAYER_EA;
  }

  switch (algo) {
    case DEBAYER_NN2:
      return demosaic_nnr(src, dst, colorid, algo);
    case DEBAYER_NNR:
      return demosaic_nnr(src, dst, colorid, algo);
    case DEBAYER_AVGC:
        return demosaic_avgc(src, dst, colorid);
    case DEBAYER_MATRIX:
      return extract_bayer_matrix(src, dst, colorid);
    default:
      break;
  }


  switch ( colorid ) {

  case COLORID_BAYER_RGGB :
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

  case COLORID_BAYER_GRBG :
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

  case COLORID_BAYER_GBRG :
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

  case COLORID_BAYER_BGGR :
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

template<typename _Tp1, typename _Tp2>
static bool _debayer_avgc(cv::InputArray _src, cv::OutputArray _dst, COLORID colorid)
{
  using T = typename std::conditional_t<std::is_floating_point_v<_Tp2>, _Tp2,
    std::conditional_t<sizeof(_Tp2) == 1, int16_t,
    std::conditional_t<sizeof(_Tp2) == 2, int32_t,
    std::conditional_t<sizeof(_Tp2) == 4, float,
    double >>>>;

  using Vec3T = cv::Vec<_Tp2, 3>;
  using Mat3T = cv::Mat_<Vec3T>;

  const int h = _src.rows() / 2;
  const int w = _src.cols() / 2;

  const cv::Mat_<_Tp1> src = _src.getMat();

  Mat3T dst(h, w);

  switch (colorid) {
    case COLORID_BAYER_MYYC:
    case COLORID_BAYER_RGGB:
      parallel_for(0, dst.rows, [&, cols = dst.cols](const auto & range) {
        for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
          const _Tp1 * r0 = src[y * 2 + 0];
          const _Tp1 * r1 = src[y * 2 + 1];
          Vec3T * dstp = dst[y];
          // RGGB -> [B G R]
          for (int x = 0; x < cols; ++x) {
            dstp[x][0] = cv::saturate_cast<_Tp2>(r1[2 * x + 1]);
            dstp[x][1] = cv::saturate_cast<_Tp2>((T(r0[2 * x + 1]) + T(r1[2 * x + 0])) / 2);
            dstp[x][2] = cv::saturate_cast<_Tp2>(r0[2 * x + 0]);
          }
        }
      });
      break;

    case COLORID_BAYER_YMCY:
    case COLORID_BAYER_GRBG:
      parallel_for(0, dst.rows, [&, cols = dst.cols](const auto & range) {
        for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
          const _Tp1 * r0 = src[y * 2 + 0];
          const _Tp1 * r1 = src[y * 2 + 1];
          Vec3T * dstp = dst[y];
          // GRBG -> [B G R]
          for (int x = 0; x < cols; ++x) {
            dstp[x][0] = cv::saturate_cast<_Tp2>(r1[2 * x + 0]);
            dstp[x][1] = cv::saturate_cast<_Tp2>((T(r0[2 * x + 0]) + T(r1[2 * x + 1])) / 2);
            dstp[x][2] = cv::saturate_cast<_Tp2>(r0[2 * x + 1]);
          }
        }
      });
      break;

    case COLORID_BAYER_YCMY:
    case COLORID_BAYER_GBRG:
      parallel_for(0, dst.rows, [&, cols = dst.cols](const auto & range) {
        for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
          const _Tp1 * r0 = src[y * 2 + 0];
          const _Tp1 * r1 = src[y * 2 + 1];
          Vec3T * dstp = dst[y];
          // GBRG -> [B G R]
          for (int x = 0; x < cols; ++x) {
            dstp[x][0] = cv::saturate_cast<_Tp2>(r0[2 * x + 1]);
            dstp[x][1] = cv::saturate_cast<_Tp2>((T(r0[2 * x + 0]) + T(r1[2 * x + 1])) / 2);
            dstp[x][2] = cv::saturate_cast<_Tp2>(r1[2 * x + 0]);
          }
        }
      });
      break;

    case COLORID_BAYER_CYYM:
    case COLORID_BAYER_BGGR:
      parallel_for(0, dst.rows, [&, cols = dst.cols](const auto & range) {
        for (int y = rbegin(range), ny = rend(range); y < ny; ++y) {
          const _Tp1 * r0 = src[y * 2 + 0];
          const _Tp1 * r1 = src[y * 2 + 1];
          Vec3T * dstp = dst[y];
          // BGGR -> [B G R]
          for (int x = 0; x < cols; ++x) {
            dstp[x][0] = cv::saturate_cast<_Tp2>(r0[2 * x + 0]);
            dstp[x][1] = cv::saturate_cast<_Tp2>((T(r0[2 * x + 1]) + T(r1[2 * x + 0])) / 2);
            dstp[x][2] = cv::saturate_cast<_Tp2>(r1[2 * x + 1]);
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

bool demosaic_avgc(cv::InputArray src, cv::OutputArray dst, COLORID colorid, int ddepth)
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

  switch (src.depth()) {
    case CV_8U:
      switch (ddepth) {
        case CV_8U:
          return _debayer_avgc<uint8_t, uint8_t>(src, dst, colorid);
        case CV_8S:
          return _debayer_avgc<uint8_t, int8_t>(src, dst, colorid);
        case CV_16U:
          return _debayer_avgc<uint8_t, uint16_t>(src, dst, colorid);
        case CV_16S:
          return _debayer_avgc<uint8_t, int16_t>(src, dst, colorid);
        case CV_32S:
          return _debayer_avgc<uint8_t, int32_t>(src, dst, colorid);
        case CV_32F:
          return _debayer_avgc<uint8_t, float>(src, dst, colorid);
        case CV_64F:
          return _debayer_avgc<uint8_t, double>(src, dst, colorid);
      }
      break;

    case CV_8S:
      switch (ddepth) {
        case CV_8U:
          return _debayer_avgc<int8_t, uint8_t>(src, dst, colorid);
        case CV_8S:
          return _debayer_avgc<int8_t, int8_t>(src, dst, colorid);
        case CV_16U:
          return _debayer_avgc<int8_t, uint16_t>(src, dst, colorid);
        case CV_16S:
          return _debayer_avgc<int8_t, int16_t>(src, dst, colorid);
        case CV_32S:
          return _debayer_avgc<int8_t, int32_t>(src, dst, colorid);
        case CV_32F:
          return _debayer_avgc<int8_t, float>(src, dst, colorid);
        case CV_64F:
          return _debayer_avgc<int8_t, double>(src, dst, colorid);
      }
      break;
    case CV_16U:
      switch (ddepth) {
        case CV_8U:
          return _debayer_avgc<uint16_t, uint8_t>(src, dst, colorid);
        case CV_8S:
          return _debayer_avgc<uint16_t, int8_t>(src, dst, colorid);
        case CV_16U:
          return _debayer_avgc<uint16_t, uint16_t>(src, dst, colorid);
        case CV_16S:
          return _debayer_avgc<uint16_t, int16_t>(src, dst, colorid);
        case CV_32S:
          return _debayer_avgc<uint16_t, int32_t>(src, dst, colorid);
        case CV_32F:
          return _debayer_avgc<uint16_t, float>(src, dst, colorid);
        case CV_64F:
          return _debayer_avgc<uint16_t, double>(src, dst, colorid);
      }
      break;
    case CV_16S:
      switch (ddepth) {
        case CV_8U:
          return _debayer_avgc<int16_t, uint8_t>(src, dst, colorid);
        case CV_8S:
          return _debayer_avgc<int16_t, int8_t>(src, dst, colorid);
        case CV_16U:
          return _debayer_avgc<int16_t, uint16_t>(src, dst, colorid);
        case CV_16S:
          return _debayer_avgc<int16_t, int16_t>(src, dst, colorid);
        case CV_32S:
          return _debayer_avgc<int16_t, int32_t>(src, dst, colorid);
        case CV_32F:
          return _debayer_avgc<int16_t, float>(src, dst, colorid);
        case CV_64F:
          return _debayer_avgc<int16_t, double>(src, dst, colorid);
      }
      break;
    case CV_32S:
      switch (ddepth) {
        case CV_8U:
          return _debayer_avgc<int32_t, uint8_t>(src, dst, colorid);
        case CV_8S:
          return _debayer_avgc<int32_t, int8_t>(src, dst, colorid);
        case CV_16U:
          return _debayer_avgc<int32_t, uint16_t>(src, dst, colorid);
        case CV_16S:
          return _debayer_avgc<int32_t, int16_t>(src, dst, colorid);
        case CV_32S:
          return _debayer_avgc<int32_t, int32_t>(src, dst, colorid);
        case CV_32F:
          return _debayer_avgc<int32_t, float>(src, dst, colorid);
        case CV_64F:
          return _debayer_avgc<int32_t, double>(src, dst, colorid);
      }
      break;
    case CV_32F:
      switch (ddepth) {
        case CV_8U:
          return _debayer_avgc<float, uint8_t>(src, dst, colorid);
        case CV_8S:
          return _debayer_avgc<float, int8_t>(src, dst, colorid);
        case CV_16U:
          return _debayer_avgc<float, uint16_t>(src, dst, colorid);
        case CV_16S:
          return _debayer_avgc<float, int16_t>(src, dst, colorid);
        case CV_32S:
          return _debayer_avgc<float, int32_t>(src, dst, colorid);
        case CV_32F:
          return _debayer_avgc<float, float>(src, dst, colorid);
        case CV_64F:
          return _debayer_avgc<float, double>(src, dst, colorid);
      }
      break;
    case CV_64F:
      switch (ddepth) {
        case CV_8U:
          return _debayer_avgc<double, uint8_t>(src, dst, colorid);
        case CV_8S:
          return _debayer_avgc<double, int8_t>(src, dst, colorid);
        case CV_16U:
          return _debayer_avgc<double, uint16_t>(src, dst, colorid);
        case CV_16S:
          return _debayer_avgc<double, int16_t>(src, dst, colorid);
        case CV_32S:
          return _debayer_avgc<double, int32_t>(src, dst, colorid);
        case CV_32F:
          return _debayer_avgc<double, float>(src, dst, colorid);
        case CV_64F:
          return _debayer_avgc<double, double>(src, dst, colorid);
      }
      break;
  }

  return false;
}


/** @brief Bayer demosaicing
 */
bool debayer(cv::InputArray src, cv::OutputArray dst, enum COLORID colorid, enum DEBAYER_ALGORITHM algo)
{
  if ( algo == DEBAYER_DEFAULT ) {
    algo = default_debayer_algorithm();
  }

  if ( algo == DEBAYER_DISABLE ) {
    if ( dst.getMatRef().data != src.getMat().data ) {
      src.getMat().copyTo(dst);
    }
    return true;
  }

  if ( algo == DEBAYER_AVGC ) {
    return demosaic_avgc(src, dst, colorid);
  }

  switch ( colorid ) {

  case COLORID_BAYER_RGGB :
  case COLORID_BAYER_GRBG :
  case COLORID_BAYER_GBRG :
  case COLORID_BAYER_BGGR :
    return demosaic(src, dst, colorid, algo);

    // From /ser-player/src/image.cpp c_image::debayer_image_bilinear_int() :
    // Start by inverting all the pixels to make them RGB;
    // Debayer;
    // Convert from GBR order to BGR:
    //    B    0 1 0   G
    //    G  = 1 0 0 * B
    //    R    0 0 1   R

  case COLORID_BAYER_CYYM : // Inverted RBBG - which is RGGB with swapped G and B
    cv::subtract(depthmax(src.depth()), src, dst);
    if ( demosaic(dst, dst, COLORID_BAYER_RGGB, algo) ) {
      cv::transform(dst, dst, cv::Matx33f(0, 1, 0, 1, 0, 0, 0, 0, 1));
      return true;
    }
    break;

  case COLORID_BAYER_YCMY :  // Inverted BRGB with swapped G and B
    cv::subtract(depthmax(src.depth()), src, dst);
    if ( demosaic(dst, dst, COLORID_BAYER_GRBG, algo) ) {
      cv::transform(dst, dst, cv::Matx33f(0, 1, 0, 1, 0, 0, 0, 0, 1));
      return true;
    }
    break;

  case COLORID_BAYER_YMCY :  // Inverted BGRB with swapped  G and B
    cv::subtract(depthmax(src.depth()), src, dst);
    if ( demosaic(dst, dst, COLORID_BAYER_GBRG, algo) ) {
      cv::transform(dst, dst, cv::Matx33f(0, 1, 0, 1, 0, 0, 0, 0, 1));
      return true;
    }
    break;

  case COLORID_BAYER_MYYC :  // Inverted GBBR with swapped  G and B
    cv::subtract(depthmax(src.depth()), src, dst);
    if ( demosaic(dst, dst, COLORID_BAYER_BGGR, algo) ) {
      cv::transform(dst, dst, cv::Matx33f(0, 1, 0, 1, 0, 0, 0, 0, 1));
      return true;
    }
    break;

  default:
    break;
  }

  return false;
}



/////////////////

/** @brief
 * Bayer interpolation of 4 bayer planes using nearest neighbors.
 * Expected order of input channels are [ R G1 B G2 ]
 */
template<class _Tp1, class _Tp2>
static bool __nninterpolation(cv::InputArray _src, cv::OutputArray _dst, enum COLORID colorid)
{
  if ( _src.channels() != 4 ) {
    CF_ERROR("Invalid arg: 4-channel input image expected, but src.channels=%d",
        _src.channels());
    return false;
  }

  if ( _dst.fixedType() && _dst.channels() != 3 ) {
    CF_ERROR("Invalid argument: 3-channel output destination image expected but dst.channels=%d",
        _dst.channels());
    return false;
  }

  typedef cv::Vec<_Tp1, 4> SrcVec;
  typedef cv::Vec<_Tp2, 3> DstVec;

  const cv::Mat_<SrcVec> src = _src.getMat();
  const cv::Size src_size = src.size();
  const cv::Size dst_size = src_size * 2;

  cv::Mat_<DstVec> dst(dst_size); // , DstVec::all(0)

  enum {
    SR = 0,
    SG1 = 1,
    SB = 2,
    SG2 = 3,
  };

  enum {
    DB = 0,
    DG = 1,
    DR = 2,
  };

#define S(yy, xx, cc) src[y+(yy)][x+(xx)][(cc)]
#define D(yy, xx, cc) dst[2 * y + (yy)][2 * x + (xx)][(cc)]

  switch (colorid) {
    case COLORID_BAYER_MYYC:
    case COLORID_BAYER_RGGB:
    // [R G   R G   R G   R G   R G   R G ]
    // [G B   G B   G B   G B   G B   G B ]

    // [R G   R G   R G   R G   R G   R G ]
    // [G B   G B   G B   G B   G B   G B ]

    // [R G   R G   R G   R G   R G   R G ]
    // [G B   G B   G B   G B   G B   G B ]
    parallel_for(0, src_size.height, [&, xmax = src_size.width, ymax = src_size.height](const auto & range) {
      for( int y = rbegin(range), ymax = rend(range); y < ymax; ++y ) {
        int x = 0;

        if( y == 0 ) {
          D(0, 0, DB)= (S(0, 0, SB));
          D(0, 1, DB)= (S(0, 0, SB));
          D(1, 0, DB)= (S(0, 0, SB));
          D(1, 1, DB)= (S(0, 0, SB));

          D(0, 0, DG)= (S(0, 0, SG1) + S(0,0,SG2)) / 2;
          D(0, 1, DG)= (S(0, 0, SG1));
          D(1, 0, DG)= (S(0, 0, SG2));
          D(1, 1, DG)= (S(0, 0, SG1) + S(0,0,SG2) + S(0,1,SG2) + S(1, 0, SG1)) / 4;

          D(0, 0, DR)= (S(0, 0, SR));
          D(0, 1, DR)= (S(0, 0, SR) + S(0,1,SR)) / 2;
          D(1, 0, DR)= (S(0, 0, SR) + S(1,0,SR)) / 2;
          D(1, 1, DR)= (S(0, 0, SR) + S(0,1,SR) + S(1,0,SR) + S(1,1,SR)) / 4;

          //
          for( x = 1; x < xmax - 1; ++x ) {

            D(0,0,DB) = (S(0,-1, SB) + S(0,0,SB)) / 2;
            D(0,1,DB) = (S(0, 0, SB));
            D(1,0,DB) = (S(0,-1, SB) + S(0,0,SB)) / 2;
            D(1,1,DB) = (S(0, 0, SB));

            D(0,0,DG) = (S(0,-1, SG1) + S(0,0,SG1) + S(1,0,SG2)) / 3;
            D(0,1,DG) = (S(0, 0, SG1));
            D(1,0,DG) = (S(0, 0, SG2));
            D(1,1,DG) = (S(0, 0, SG1) + S(0,0,SG2) + S(0,1,SG2)) / 3;

            D(0,0,DR) = (S(0, 0, SR));
            D(0,1,DR) = (S(0, 0, SR) + S(0,1,SR)) / 2;
            D(1,0,DR) = (S(0, 0, SR) + S(1,0,SR)) / 2;
            D(1,1,DR) = (S(0, 0, SR) + S(0,1,SR) + S(1,0,SR) + S(1,1,SR)) / 4;
          }

          D(0,0,DB)= (S(0, 0, SB) + S(0,-1,SB)) / 2;
          D(0,1,DB)= (S(0, 0, SB));
          D(1,0,DB)= (S(0,-1, SB)+ S(1,0,SB) + S(0,0,SB)) / 3;
          D(1,1,DB)= (S(0, 0, SB));

          D(0,0,DG)= (S(0,-1, SG1) + S(0,0,SG1) + S(0,0,SG2)) / 3;
          D(0,1,DG)= (S(0, 0, SG1));
          D(1,0,DG)= (S(0, 0, SG2));
          D(1,1,DG)= (S(0, 0, SG1)+ S(0,0,SG2) + S(1,0,SG1)) / 3;

          D(0,0,DR)= (S(0, 0, SR));
          D(0,1,DR)= (S(0, 0, SR));
          D(1,0,DR)= (S(0, 0, SR)+ S(1,0,SR)) / 2;
          D(1,1,DR)= (S(0, 0, SR)+ S(1,0,SR)) / 2;

          continue;
        }

        if( y == ymax - 1 ) {

          D(0,0,DB)= (S( 0, 0, SB) + S(-1, 0, SB)) / 2;
          D(0,1,DB)= (S( 0, 0, SB)+ S(-1, 0, SB)) / 2;
          D(1,0,DB)= (S( 0, 0, SB));
          D(1,1,DB)= (S( 0, 0, SB));

          D(0,0,DG)= (S(-1, 0, SG2) + S(0,0,SG1) + S(0,0,SG2)) / 3;
          D(0,1,DG)= (S( 0, 0, SG1));
          D(1,0,DG)= (S( 0, 0, SG2));
          D(1,1,DG)= (S( 0, 0, SG1)+ S(0,0,SG2) + S(0,1,SG2)) / 3;

          D(0,0,DR)= (S( 0, 0, SR));
          D(0,1,DR)= (S( 0, 0, SR)+ S(0,1,SR)) / 2;
          D(1,0,DR)= (S( 0, 0, SR));
          D(1,1,DR)= (S( 0, 0, SR)+ S(0,1,SR)) / 2;

          for( x = 1; x < xmax - 1; ++x ) {

            D(0,0,DB) = (S(-1, -1, SB) + S(0,-1,SB) + S(0,0,SB) + S(-1, 0, SB)) / 4;
            D(0,1,DB) = (S(-1, 0, SB) + S(0,0,SB)) / 2;
            D(1,0,DB) = (S(0,-1,SB) + S(0,0,SB)) / 2;
            D(1,1,DB) = (S(0,0,SB));

            D(0,0,DG) = (S(0,0,SG1) + S(0,0,SG2) + S(-1,0,SG2) + S(0,-1,SG1)) / 4;
            D(0,1,DG) = (S(0,0,SG1));
            D(1,0,DG) = (S(0,0,SG2));
            D(1,1,DG) = (S(0,0,SG1) + S(0,0,SG2) + S(0,1,SG2) ) / 3;

            D(0,0,DR) = (S(0,0,SR));
            D(0,1,DR) = (S(0,0,SR) + S(0,1,SR)) /2;
            D(1,0,DR) = (S(0,0,SR));
            D(1,1,DR) = (S(0,0,SR) + S(0,1,SR)) /2;
          }

          D(0,0,DB)= (S(-1, -1, SB) + S(-1, 0, SB) + S(0,-1,SB) + S(0,0,SB)) / 4;
          D(0,1,DB)= (S(-1, 0, SB) + S(0, 0, SB)) / 2;
          D(1,0,DB)= (S(0,-1,SB)+ S(0,0,SB)) / 2;
          D(1,1,DB)= (S(0, 0, SB));

          D(0,0,DG)= (S(0,-1,SG1) + S(0,0,SG2) + S(0,0,SG1) + S(-1, 0, SG2)) / 4;
          D(0,1,DG)= (S(0, 0, SG1));
          D(1,0,DG)= (S(0, 0, SG2));
          D(1,1,DG)= (S(0,0,SG1)+ S(0,0,SG2)) / 2;

          D(0,0,DR)= (S(0,0,SR));
          D(0,1,DR)= (S(0, 0, SR));
          D(1,0,DR)= (S(0, 0, SR));
          D(1,1,DR)= (S(0, 0, SR));

          continue;
        }

        D(0, 0, DB)= (S(-1,0, SB) + S(0,0, SB) ) / 2;
        D(0, 1, DB)= (S(-1,0, SB) + S(0,0, SB) ) / 2;
        D(1, 0, DB)= (S(0,0, SB));
        D(1, 1, DB)= (S(0,0, SB));

        D(0, 0, DG)= (S(-1,0, SG2) + S(0,0, SG1) + S(0,0, SG2)) / 3;
        D(0, 1, DG)= (S(0,0, SG1));
        D(1, 0, DG)= (S(0,0, SG2));
        D(1, 1, DG)= (S(0,0, SG1) + S(0,0, SG2) + S(1,0, SG1) + S(0,1, SG2)) / 4;

        D(0, 0, DR)= (S(0,0, SR));
        D(0, 1, DR)= (S(0,0, SR) + S(0,1, SR)) / 2;
        D(1, 0, DR)= (S(0,0, SR) + S(1,0, SR) ) /2;
        D(1, 1, DR)= (S(0,0, SR) + S(1,0, SR) + S(1,1, SR) + S(0,1, SR)) / 4;

        for( x = 1; x < xmax - 1; ++x ) {

          D(0, 0, DB) = (S(-1,-1, SB) + S(-1,0, SB) + S(0, -1, SB) + S(0,0, SB)) / 4;
          D(0, 1, DB) = (S(-1,0, SB) + S(0,0, SB)) / 2;
          D(1, 0, DB) = (S(0, -1, SB) + S(0,0, SB)) / 2;
          D(1, 1, DB) = (S(0,0, SB));

          D(0, 0, DG) = (S(-1,0, SG2) + S(0, -1, SG1) + S(0,0, SG1) + S(0,0, SG2)) / 4;
          D(0, 1, DG) = S(0,0, SG1);
          D(1, 0, DG) = S(0,0, SG2);
          D(1, 1, DG) = (S(0,0, SG1) + S(0,0, SG2) + S(0, 1, SG2) + +S(1, 0, SG1)) / 4;

          D(0, 0, DR) = S(0,0, SR);
          D(0, 1, DR) = (S(0,0, SR) + S(0, 1, SR)) / 2;
          D(1, 0, DR) = (S(0,0, SR) + S(1, 0, SR)) / 2;
          D(1, 1, DR) = (S(0,0, SR) + S(0, 1, SR) + S(1, 0, SR) + S(1,1,SR)) / 4;
        }


        D(0, 0, DB)= (S(-1,-1, SB) + S(-1,0, SB) + S(0,-1, SB) + S(0,0, SB)) / 4;
        D(0, 1, DB)= (S(-1,0, SB) + S(0,0, SB)) / 2;
        D(1, 0, DB)= (S(0,-1, SB) + S(0,0, SB)) / 2;
        D(1, 1, DB)= (S(0,0, SB));

        D(0, 0, DG)= (S(1,0, SG2) + S(0,-1, SG1) + S(0,0, SG1) + S(0,0, SG2)) / 4;
        D(0, 1, DG)= (S(0,0, SG1));
        D(1, 0, DG)= (S(0,0, SG2));
        D(1, 1, DG)= (S(0,0, SG1) + S(0,0, SG2) + S(1,0, SG1) ) / 3;

        D(0, 0, DR)= (S(0,0, SR));
        D(0, 1, DR)= (S(0,0, SR));
        D(1, 0, DR)= (S(0,0, SR) + S(0,-1, SR) + S(1,0, SR)) / 3;
        D(1, 1, DR)= (S(0,0, SR) + S(1,0, SR)) / 2;
      }
    });
    break;

    case COLORID_BAYER_YMCY:
    case COLORID_BAYER_GRBG:
    //  [ G R    G R    G R    G R    G R   ]
    //  [ B G    B G    B G    B G    B G   ]

    //  [ G R    G R    G R    G R    G R   ]
    //  [ B G    B G    B G    B G    B G   ]

    //  [ G R    G R    G R    G R    G R   ]
    //  [ B G    B G    B G    B G    B G   ]

    //  [ G R    G R    G R    G R    G R   ]
    //  [ B G    B G    B G    B G    B G   ]
    parallel_for(0, src_size.height, [&, xmax = src_size.width, ymax = src_size.height](const auto & range) {
      for( int y = rbegin(range), ymax = rend(range); y < ymax; ++y ) {

        int x = 0;

        if ( y == 0 ) {

          D(0, 0, DB) = (S( 0, 0, SB));
          D(0, 1, DB) = (S( 0, 0, SB) + S(0, 1, SB)) / 2;
          D(1, 0, DB) = (S( 0, 0, SB));
          D(1, 1, DB) = (S( 0, 0, SB) + S(0, 1, SB)) / 2;

          D(0, 0, DG) = (S( 0, 0, SG1));
          D(0, 1, DG) = (S( 0, 0, SG1) + S(0, 0, SG2) + S( 0, 1, SG1)) / 3;
          D(1, 0, DG) = (S( 0, 0, SG1) + S(1, 0, SG1) + S( 0, 0, SG2)) / 3;
          D(1, 1, DG) = (S( 0, 0, SG2));

          D(0, 0, DR) = (S( 0, 0, SR));
          D(0, 1, DR) = (S( 0, 0, SR));
          D(1, 0, DR) = (S( 1, 0, SR) + S( 0, 0, SR)) / 2;
          D(1, 1, DR) = (S( 0, 0, SR) + S( 1, 0, SR)) / 2;


          for( x = 1; x < xmax - 1; ++x ) {

            D(0, 0, DB) = (S( 0, 0, SB));
            D(0, 1, DB) = (S( 0, 0, SB) + S(0, 1, SB)) / 2;
            D(1, 0, DB) = (S( 0, 0, SB));
            D(1, 1, DB) = (S( 0, 0, SB) + S(0, 1, SB)) / 2;

            D(0, 0, DG) = (S( 0, 0, SG1));
            D(0, 1, DG) = (S( 0, 0, SG1) + S( 0, 0, SG2) + S( 0, 1, SG1)) / 3;
            D(1, 0, DG) = (S( 0, 0, SG1) + S( 0,-1, SG2) + S( 1, 0, SG1) + S( 0, 0, SG2)) / 4;
            D(1, 1, DG) = (S( 0, 0, SG2));

            D(0, 0, DR) = (S( 0,-1, SR) + S( 0, 0, SR)) / 2;
            D(0, 1, DR) = (S( 0, 0, SR));
            D(1, 0, DR) = (S( 0,-1, SR) + S( 1, -1, SR) + S( 1, 0, SR) + S( 0, 0, SR)) / 4;
            D(1, 1, DR) = (S( 0, 0, SR) + S( 1, 0, SR)) / 2;
          }

          D(0, 0, DB) = (S( 0, 0, SB));
          D(0, 1, DB) = (S( 0, 0, SB));
          D(1, 0, DB) = (S( 0, 0, SB));
          D(1, 1, DB) = (S( 0, 0, SB));

          D(0, 0, DG) = (S( 0, 0, SG1));
          D(0, 1, DG) = (S( 0, 0, SG1) + S( 0, 0, SG2)) / 2;
          D(1, 0, DG) = (S( 0, 0, SG1) + S( 0,-1, SG2) + S( 0, 0, SG2)) / 3;
          D(1, 1, DG) = (S( 0, 0, SG2));

          D(0, 0, DR) = (S( 0,-1, SR) + S( 0, 0, SR)) / 2;
          D(0, 1, DR) = (S( 0, 0, SR));
          D(1, 0, DR) = (S( 0,-1, SR) + S( 1, -1, SR) + S( 1, 0, SR) + S( 0, 0, SR)) / 4;
          D(1, 1, DR) = (S( 0, 0, SR) + S( 1, 0, SR)) / 2;

          continue;
        }

        if ( y == ymax - 1 ) {

          D(0, 0, DB) = (S(-1, 0, SB) + S( 0, 0, SB)) / 2;
          D(0, 1, DB) = (S(-1, 0, SB) + S( 0, 0, SB) + S(0, 1, SB) + S(-1, 1, SB)) / 4;
          D(1, 0, DB) = (S( 0, 0, SB));
          D(1, 1, DB) = (S( 0, 0, SB) + S(0, 1, SB)) / 2;

          D(0, 0, DG) = (S( 0, 0, SG1));
          D(0, 1, DG) = (S(-1, 0, SG2) + S( 0, 0, SG1) + S( 0, 0, SG2) + S( 0, 1, SG1)) / 4;
          D(1, 0, DG) = (S( 0, 0, SG1) + S( 0, 0, SG2)) / 2;
          D(1, 1, DG) = (S( 0 ,0, SG2));

          D(0, 0, DR) = (S( 0, 0, SR));
          D(0, 1, DR) = (S( 0, 0, SR));
          D(1, 0, DR) = (S( 0, 0, SR));
          D(1, 1, DR) = (S( 0, 0, SR));

          for( x = 1; x < xmax - 1; ++x ) {

            D(0, 0, DB) = (S(-1, 0, SB) + S( 0, 0, SB)) / 2;
            D(0, 1, DB) = (S(-1, 0, SB) + S( 0, 0, SB) + S(0, 1, SB) + S(-1, 1, SB)) / 4;
            D(1, 0, DB) = (S( 0, 0, SB));
            D(1, 1, DB) = (S( 0, 0, SB) + S( 0, 1, SB)) / 2;

            D(0, 0, DG) = (S( 0, 0, SG1));
            D(0, 1, DG) = (S(-1, 0, SG2) + S( 0, 0, SG1) + S( 0, 0, SG2) + S( 0, 1, SG1)) / 4;
            D(1, 0, DG) = (S( 0, 0, SG1) + S( 0,-1, SG2) + S( 0, 0, SG2)) / 3;
            D(1, 1, DG) = (S( 0 ,0, SG2));

            D(0, 0, DR) = (S( 0,-1, SR) + S( 0, 0, SR)) / 2;
            D(0, 1, DR) = (S( 0, 0, SR));
            D(1, 0, DR) = (S( 0,-1, SR) + S( 0, 0, SR)) / 2;
            D(1, 1, DR) = (S( 0, 0, SR));
          }

          D(0, 0, DB) = (S(-1, 0, SB) + S( 0, 0, SB)) / 2;
          D(0, 1, DB) = (S(-1, 0, SB) + S( 0, 0, SB)) / 2;
          D(1, 0, DB) = (S( 0, 0, SB));
          D(1, 1, DB) = (S( 0, 0, SB));

          D(0, 0, DG) = (S( 0, 0, SG1));
          D(0, 1, DG) = (S(-1, 0, SG2) + S( 0, 0, SG1) + S( 0, 0, SG2)) / 3;
          D(1, 0, DG) = (S( 0, 0, SG1) + S( 0,-1, SG2) + S( 0, 0, SG2)) / 3;
          D(1, 1, DG) = (S( 0 ,0, SG2));

          D(0, 0, DR) = (S( 0,-1, SR) + S( 0, 0, SR)) / 2;
          D(0, 1, DR) = (S( 0, 0, SR));
          D(1, 0, DR) = (S( 0,-1, SR) + S( 0, 0, SR)) / 2;
          D(1, 1, DR) = (S( 0, 0, SR));

          continue;
        }

        D(0, 0, DB) = (S(-1, 0, SB) + S( 0, 0, SB)) / 2;
        D(0, 1, DB) = (S(-1, 0, SB) + S( 0, 0, SB) + S(0, 1, SB) + S(-1, 1, SB)) / 4;
        D(1, 0, DB) = (S( 0, 0, SB));
        D(1, 1, DB) = (S( 0, 0, SB) + S(0, 1, SB)) / 2;

        D(0, 0, DG) = (S( 0, 0, SG1));
        D(0, 1, DG) = (S(-1, 0, SG2) + S( 0, 0, SG1) + S( 0, 0, SG2) + S( 0, 1, SG1)) / 4;
        D(1, 0, DG) = (S( 0, 0, SG1) + S( 1, 0, SG1) + S( 0, 0, SG2)) / 3;
        D(1, 1, DG) = (S( 0 ,0, SG2));

        D(0, 0, DR) = (S( 0, 0, SR));
        D(0, 1, DR) = (S( 0, 0, SR));
        D(1, 0, DR) = (S( 0, 0, SR) + S( 1, 0, SR)) / 2;
        D(1, 1, DR) = (S( 0, 0, SR) + S( 1, 0, SR)) / 2;

        for( x = 1; x < xmax - 1; ++x ) {

          D(0, 0, DB) = (S(-1, 0, SB) + S( 0, 0, SB)) / 2;
          D(0, 1, DB) = (S(-1, 0, SB) + S( 0, 0, SB) + S(0, 1, SB) + S(-1, 1, SB)) / 4;
          D(1, 0, DB) = (S( 0, 0, SB));
          D(1, 1, DB) = (S( 0, 0, SB) + S(0, 1, SB)) / 2;

          D(0, 0, DG) = (S( 0, 0, SG1));
          D(0, 1, DG) = (S(-1, 0, SG2) + S( 0, 0, SG1) + S( 0, 0, SG2) + S( 0, 1, SG1)) / 4;
          D(1, 0, DG) = (S( 0, 0, SG1) + S( 0,-1, SG2) + S( 1, 0, SG1) + S( 0, 0, SG2)) / 4;
          D(1, 1, DG) = (S( 0 ,0, SG2));

          D(0, 0, DR) = (S( 0,-1, SR) + S( 0, 0, SR)) / 2;
          D(0, 1, DR) = (S( 0, 0, SR));
          D(1, 0, DR) = (S( 0, 0, SR) + S( 0,-1, SR) + S( 1,-1, SR) + S( 1, 0, SR)) / 4;
          D(1, 1, DR) = (S( 0, 0, SR) + S( 1, 0, SR)) / 2;
        }

        D(0, 0, DB) = (S(-1, 0, SB) + S( 0, 0, SB)) / 2;
        D(0, 1, DB) = (S(-1, 0, SB) + S( 0, 0, SB)) / 2;
        D(1, 0, DB) = (S( 0, 0, SB));
        D(1, 1, DB) = (S( 0, 0, SB));

        D(0, 0, DG) = (S( 0, 0, SG1));
        D(0, 1, DG) = (S( 0, 0, SG1) + S( 0, 0, SG2) + S(-1, 0, SG2) ) / 3;
        D(1, 0, DG) = (S( 0, 0, SG1) + S( 0, 0, SG2) + S( 0,-1, SG2) + S( 1, 0, SG1)) / 4;
        D(1, 1, DG) = (S( 0 ,0, SG2));

        D(0, 0, DR) = (S( 0, 0, SR) + S( 0,-1, SR)) / 2;
        D(0, 1, DR) = (S( 0, 0, SR));
        D(1, 0, DR) = (S( 0, 0, SR) + S( 0,-1, SR) + S( 1,-1, SR) + S( 1, 0, SR)) / 4;
        D(1, 1, DR) = (S( 0, 0, SR) + S( 1, 0, SR)) / 2;
      }
    });
    break;

    case COLORID_BAYER_YCMY:
    case COLORID_BAYER_GBRG:
    //  [ G B   G B   G B   G B   G B   G B]
    //  [ R G   R G   R G   R G   R G   R G]

    //  [ G B   G B   G B   G B   G B   G B]
    //  [ R G   R G   R G   R G   R G   R G]

    //  [ G B   G B   G B   G B   G B   G B]
    //  [ R G   R G   R G   R G   R G   R G]

    //  [ G B   G B   G B   G B   G B   G B]
    //  [ R G   R G   R G   R G   R G   R G]
    parallel_for(0, src_size.height, [&, xmax = src_size.width, ymax = src_size.height](const auto & range) {
      for( int y = rbegin(range), ymax = rend(range); y < ymax; ++y ) {

        int x = 0;

        if ( y == 0 ) {
          D(0, 0, DR) = (S( 0, 0, SR));
          D(0, 1, DR) = (S( 0, 0, SR) + S(0, 1, SR)) / 2;
          D(1, 0, DR) = (S( 0, 0, SR));
          D(1, 1, DR) = (S( 0, 0, SR) + S(0, 1, SR)) / 2;

          D(0, 0, DG) = (S( 0, 0, SG1));
          D(0, 1, DG) = (S( 0, 0, SG1) + S( 0, 0, SG2) + S( 0, 1, SG1)) / 3;
          D(1, 0, DG) = (S( 0, 0, SG1) + S( 0, 0, SG2) + S( 1, 0, SG1)) / 3;
          D(1, 1, DG) = (S( 0, 0, SG2));

          D(0, 0, DB) = (S( 0, 0, SB));
          D(0, 1, DB) = (S( 0, 0, SB));
          D(1, 0, DB) = (S( 0, 0, SB) + S( 1, 0, SB)) / 2;
          D(1, 1, DB) = (S( 0, 0, SB) + S( 1, 0, SB)) / 2;

          for( x = 1; x < xmax - 1; ++x ) {

            D(0, 0, DR) = (S( 0, 0, SR));
            D(0, 1, DR) = (S( 0, 0, SR) + S(0, 1, SR)) / 2;
            D(1, 0, DR) = (S( 0, 0, SR));
            D(1, 1, DR) = (S( 0, 0, SR) + S(0, 1, SR)) / 2;

            D(0, 0, DG) = (S( 0, 0, SG1));
            D(0, 1, DG) = (S( 0, 0, SG1) + S( 0, 0, SG2) + S( 0, 1, SG1)) / 3;
            D(1, 0, DG) = (S( 0, 0, SG1) + S( 0, 0, SG2) + S( 0,-1, SG2) + S( 1, 0, SG1)) / 4;
            D(1, 1, DG) = (S( 0, 0, SG2));

            D(0, 0, DB) = (S( 0, 0, SB) + S( 0,-1, SB)) / 2;
            D(0, 1, DB) = (S( 0, 0, SB));
            D(1, 0, DB) = (S( 0, 0, SB) + S( 1, 0, SB) + S( 0,-1, SB) + S( 1,-1, SB)) / 4;
            D(1, 1, DB) = (S( 0, 0, SB) + S( 1, 0, SB)) / 2;
          }

          D(0, 0, DR) = (S( 0, 0, SR));
          D(0, 1, DR) = (S( 0, 0, SR));
          D(1, 0, DR) = (S( 0, 0, SR));
          D(1, 1, DR) = (S( 0, 0, SR));

          D(0, 0, DG) = (S( 0, 0, SG1));
          D(0, 1, DG) = (S( 0, 0, SG1) + S( 0, 0, SG2)) / 2;
          D(1, 0, DG) = (S( 0, 0, SG1) + S( 0, 0, SG2) + S( 0,-1, SG2) + S( 1, 0, SG1)) / 4;
          D(1, 1, DG) = (S( 0, 0, SG2));

          D(0, 0, DB) = (S( 0, 0, SB) + S( 0,-1, SB)) / 2;
          D(0, 1, DB) = (S( 0, 0, SB));
          D(1, 0, DB) = (S( 0, 0, SB) + S( 1, 0, SB) + S( 0,-1, SB) + S( 1,-1, SB)) / 4;
          D(1, 1, DB) = (S( 0, 0, SB) + S( 1, 0, SB)) / 2;

          continue;
        }

        if ( y == ymax - 1 ) {

          D(0, 0, DR) = (S( 0, 0, SR));
          D(0, 1, DR) = (S( 0, 0, SR) + S(-1, 0, SR) + S(0, 1, SR) + S(-1, 1, SR)) / 4;
          D(1, 0, DR) = (S( 0, 0, SR));
          D(1, 1, DR) = (S( 0, 0, SR) + S(0, 1, SR)) / 2;

          D(0, 0, DG) = (S( 0, 0, SG1));
          D(0, 1, DG) = (S( 0, 0, SG1) + S( 0, 0, SG2) + S(-1, 0, SG1) + S( 0, 1, SG1)) / 4;
          D(1, 0, DG) = (S( 0, 0, SG1) + S( 0, 0, SG2)) / 2;
          D(1, 1, DG) = (S( 0, 0, SG2));

          D(0, 0, DB) = (S( 0, 0, SB));
          D(0, 1, DB) = (S( 0, 0, SB));
          D(1, 0, DB) = (S( 0, 0, SB));
          D(1, 1, DB) = (S( 0, 0, SB));

          for( x = 1; x < xmax - 1; ++x ) {

            D(0, 0, DR) = (S( 0, 0, SR) + S(-1, 0, SR)) / 2;
            D(0, 1, DR) = (S( 0, 0, SR) + S(-1, 0, SR) + S(0, 1, SR) + S(-1, 1, SR)) / 4;
            D(1, 0, DR) = (S( 0, 0, SR));
            D(1, 1, DR) = (S( 0, 0, SR) + S(0, 1, SR)) / 2;

            D(0, 0, DG) = (S( 0, 0, SG1));
            D(0, 1, DG) = (S( 0, 0, SG1) + S( 0, 0, SG2) + S(-1, 0, SG1) + S( 0, 1, SG1)) / 4;
            D(1, 0, DG) = (S( 0, 0, SG1) + S( 0, 0, SG2) + S( 0,-1, SG2)) / 3;
            D(1, 1, DG) = (S( 0, 0, SG2));

            D(0, 0, DB) = (S( 0, 0, SB) + S( 0,-1, SB)) / 2;
            D(0, 1, DB) = (S( 0, 0, SB));
            D(1, 0, DB) = (S( 0, 0, SB) + S( 0,-1, SB)) / 2;
            D(1, 1, DB) = (S( 0, 0, SB));
          }

          D(0, 0, DR) = (S( 0, 0, SR) + S(-1, 0, SR)) / 2;
          D(0, 1, DR) = (S( 0, 0, SR) + S(-1, 0, SR)) / 2;
          D(1, 0, DR) = (S( 0, 0, SR));
          D(1, 1, DR) = (S( 0, 0, SR));

          D(0, 0, DG) = (S( 0, 0, SG1));
          D(0, 1, DG) = (S( 0, 0, SG1) + S( 0, 0, SG2) + S(-1, 0, SG1)) / 3;
          D(1, 0, DG) = (S( 0, 0, SG1) + S( 0, 0, SG2) + S( 0,-1, SG2)) / 3;
          D(1, 1, DG) = (S( 0, 0, SG2));

          D(0, 0, DB) = (S( 0, 0, SB) + S( 0,-1, SB)) / 2;
          D(0, 1, DB) = (S( 0, 0, SB));
          D(1, 0, DB) = (S( 0, 0, SB) + S( 0,-1, SB)) / 2;
          D(1, 1, DB) = (S( 0, 0, SB));

          continue;
        }


        D(0, 0, DR) = (S( 0, 0, SR) + S(-1, 0, SR)) / 2;
        D(0, 1, DR) = (S( 0, 0, SR) + S(-1, 0, SR) + S(0, 1, SR) + S(-1, 1, SR)) / 4;
        D(1, 0, DR) = (S( 0, 0, SR));
        D(1, 1, DR) = (S( 0, 0, SR) + S(0, 1, SR)) / 2;

        D(0, 0, DG) = (S( 0, 0, SG1));
        D(0, 1, DG) = (S( 0, 0, SG1) + S( 0, 0, SG2) + S(-1, 0, SG1) + S( 0, 1, SG1)) / 4;
        D(1, 0, DG) = (S( 0, 0, SG1) + S( 0, 0, SG2) + S( 1, 0, SG1)) / 3;
        D(1, 1, DG) = (S( 0, 0, SG2));

        D(0, 0, DB) = (S( 0, 0, SB));
        D(0, 1, DB) = (S( 0, 0, SB));
        D(1, 0, DB) = (S( 0, 0, SB) + S( 1, 0, SB)) / 2;
        D(1, 1, DB) = (S( 0, 0, SB) + S( 1, 0, SB)) / 2;

        for( x = 1; x < xmax - 1; ++x ) {

          D(0, 0, DR) = (S( 0, 0, SR) + S(-1, 0, SR)) / 2;
          D(0, 1, DR) = (S( 0, 0, SR) + S(-1, 0, SR) + S(0, 1, SR) + S(-1, 1, SR)) / 4;
          D(1, 0, DR) = (S( 0, 0, SR));
          D(1, 1, DR) = (S( 0, 0, SR) + S(0, 1, SR)) / 2;

          D(0, 0, DG) = (S( 0, 0, SG1));
          D(0, 1, DG) = (S( 0, 0, SG1) + S( 0, 0, SG2) + S(-1, 0, SG1) + S( 0, 1, SG1)) / 4;
          D(1, 0, DG) = (S( 0, 0, SG1) + S( 0, 0, SG2) + S( 0,-1, SG2) + S( 1, 0, SG1)) / 4;
          D(1, 1, DG) = (S( 0, 0, SG2));

          D(0, 0, DB) = (S( 0, 0, SB) + S( 0,-1, SB)) / 2;
          D(0, 1, DB) = (S( 0, 0, SB));
          D(1, 0, DB) = (S( 0, 0, SB) + S( 1, 0, SB) + S( 0,-1, SB) + S( 1,-1, SB)) / 4;
          D(1, 1, DB) = (S( 0, 0, SB) + S( 1, 0, SB)) / 2;
        }

        D(0, 0, DR) = (S( 0, 0, SR) + S(-1, 0, SR)) / 2;
        D(0, 1, DR) = (S( 0, 0, SR) + S(-1, 0, SR)) / 2;
        D(1, 0, DR) = (S( 0, 0, SR));
        D(1, 1, DR) = (S( 0, 0, SR));

        D(0, 0, DG) = (S( 0, 0, SG1));
        D(0, 1, DG) = (S( 0, 0, SG1) + S( 0, 0, SG2) + S(-1, 0, SG1)) / 3;
        D(1, 0, DG) = (S( 0, 0, SG1) + S( 0, 0, SG2) + S( 0,-1, SG2) + S( 1, 0, SG1)) / 4;
        D(1, 1, DG) = (S( 0, 0, SG2));

        D(0, 0, DB) = (S( 0, 0, SB) + S( 0,-1, SB)) / 2;
        D(0, 1, DB) = (S( 0, 0, SB));
        D(1, 0, DB) = (S( 0, 0, SB) + S( 1, 0, SB) + S( 0,-1, SB) + S( 1,-1, SB)) / 4;
        D(1, 1, DB) = (S( 0, 0, SB) + S( 1, 0, SB)) / 2;
      }
    });
    break;

    case COLORID_BAYER_CYYM:
    case COLORID_BAYER_BGGR:
    //  [ B G   B G   B G   B G   B G]
    //  [ G R   G R   G R   G R   G R]

    //  [ B G   B G   B G   B G   B G]
    //  [ G R   G R   G R   G R   G R]

    //  [ B G   B G   B G   B G   B G]
    //  [ G R   G R   G R   G R   G R]

    //  [ B G   B G   B G   B G   B G]
    //  [ G R   G R   G R   G R   G R]
    parallel_for(0, src_size.height, [&, xmax = src_size.width, ymax = src_size.height](const auto & range) {
      for( int y = rbegin(range), ymax = rend(range); y < ymax; ++y ) {

        int x = 0;

        if( y == 0 ) {
          D(0, 0, DR) = (S(0, 0, SR));
          D(0, 1, DR) = (S(0, 0, SR));
          D(1, 0, DR) = (S(0, 0, SR));
          D(1, 1, DR) = (S(0, 1, SR));

          D(0, 0, DG) = (S(0, 0, SG1) + S(0, 0, SG2) ) / 2;
          D(0, 1, DG) = (S(0, 0, SG1));
          D(1, 0, DG) = (S(0, 0, SG2));
          D(1, 1, DG) = (S(0, 0, SG1) + S(0, 0, SG2) + S(1, 0, SG1) + S(0, 1, SG2)) / 4;

          D(0, 0, DB) = (S(0, 0, SB));
          D(0, 1, DB) = (S(0, 0, SB) + S(0, 1, SB)) / 2;
          D(1, 0, DB) = (S(0, 0, SB) + S(1, 0, SB)) / 2;
          D(1, 1, DB) = (S(0, 0, SB) + S(1, 0, SB) + S(1, 1, SB) + S(0, 1, SB)) / 4;

          for( x = 1; x < xmax - 1; ++x ) {

            D(0, 0, DR) = (S(0,-1, SR) + S(0, 0, SR)) / 2;
            D(0, 1, DR) = (S(0, 0, SR));
            D(1, 0, DR) = (S(0,-1, SR) + S(0, 0, SR)) / 2;
            D(1, 1, DR) = (S(0, 0, SR));

            D(0, 0, DG) = (S(0,-1, SG1) + S(0, 0, SG1) + S(0, 0, SG2)) / 3;
            D(0, 1, DG) = (S(0, 0, SG1));
            D(1, 0, DG) = (S(0, 0, SG2));
            D(1, 1, DG) = (S(0, 0, SG1) + S(0, 0, SG2) + S(0, 1, SG2) + +S(1, 0, SG1)) / 4;

            D(0, 0, DB) = (S(0, 0, SB));
            D(0, 1, DB) = (S(0, 0, SB) + S(0, 1, SB)) / 2;
            D(1, 0, DB) = (S(0, 0, SB) + S(1, 0, SB)) / 2;
            D(1, 1, DB) = (S(0, 0, SB) + S(0, 1, SB) + S(1, 0, SB) + S(1, 1, SB)) / 4;
          }


          D(0, 0, DR) = (S(0,-1, SR) + S(0, 0, SR)) / 2;
          D(0, 1, DR) = (S(0, 0, SR)) / 2;
          D(1, 0, DR) = (S(0,-1, SR) + S(0, 0, SR)) / 2;
          D(1, 1, DR) = (S(0, 0, SR));

          D(0, 0, DG) = (S(0,-1, SG1) + S(0, 0, SG1) + S(0, 0, SG2)) / 3;
          D(0, 1, DG) = (S(0, 0, SG1));
          D(1, 0, DG) = (S(0, 0, SG2));
          D(1, 1, DG) = (S(0, 0, SG1) + S(0, 0, SG2)  + S(1, 0, SG1)) / 3;

          D(0, 0, DB) = (S(0, 0, SB));
          D(0, 1, DB) = (S(0, 0, SB));
          D(1, 0, DB) = (S(0, 0, SB) + S(1, 0, SB)) / 2;
          D(1, 1, DB) = (S(0, 0, SB) + S(1, 0, SB)) / 2;

          continue;
        }

        if( y == ymax - 1 ) {

          D(0, 0, DR) = (S(-1, 0, SR) + S(0, 0, SR)) / 2;
          D(0, 1, DR) = (S(-1, 0, SR) + S(0, 0, SR)) / 2;
          D(1, 0, DR) = (S( 0, 0, SR));
          D(1, 1, DR) = (S( 0, 0, SR));

          D(0, 0, DG) = (S(-1, 0, SG2) + S(0, 0, SG1) + S(0, 0, SG2)) / 3;
          D(0, 1, DG) = (S( 0, 0, SG1));
          D(1, 0, DG) = (S( 0, 0, SG2));
          D(1, 1, DG) = (S( 0, 0, SG1) + S(0, 0, SG2) + S(0, 1, SG2)) / 3;

          D(0, 0, DB) = (S( 0, 0, SB));
          D(0, 1, DB) = (S( 0, 0, SB) + S(0, 1, SB)) / 2;
          D(1, 0, DB) = (S( 0, 0, SB));
          D(1, 1, DB) = (S( 0, 0, SB) + S(0, 1, SB)) / 2;

          for( x = 1; x < xmax - 1; ++x ) {

            D(0, 0, DR) = (S(-1,-1, SR) + S(-1, 0, SR) + S(0, -1, SR) + S(0, 0, SR)) / 4;
            D(0, 1, DR) = (S(-1, 0, SR) + S( 0, 0, SR)) / 2;
            D(1, 0, DR) = (S( 0,-1, SR) + S( 0, 0, SR)) / 2;
            D(1, 1, DR) = (S( 0, 0, SR));

            D(0, 0, DG) = (S(-1, 0, SG2) + S(0, -1, SG1) + S(0, 0, SG1) + S(0, 0, SG2)) / 4;
            D(0, 1, DG) = (S( 0, 0, SG1));
            D(1, 0, DG) = (S( 0, 0, SG2));
            D(1, 1, DG) = (S( 0, 0, SG1) + S(0, 0, SG2) + S(0, 1, SG2) ) / 3;

            D(0, 0, DB) = (S( 0, 0, SB));
            D(0, 1, DB) = (S( 0, 0, SB) + S(0, 1, SB)) / 2;
            D(1, 0, DB) = (S( 0, 0, SB));
            D(1, 1, DB) = (S( 0, 0, SB) + S(0, 1, SB)) / 2;
          }

          D(0, 0, DR) = (S(-1,-1, SR) + S(-1, 0, SR) + S(0, -1, SR) + S(0, 0, SR)) / 4;
          D(0, 1, DR) = (S(-1, 0, SR) + S( 0, 0, SR)) / 2;
          D(1, 0, DR) = (S( 0,-1, SR) + S( 0, 0, SR)) / 2;
          D(1, 1, DR) = (S( 0, 0, SR));

          D(0, 0, DG) = (S(-1, 0, SG2) + S(0, -1, SG1) + S(0, 0, SG1) + S(0, 0, SG2)) / 4;
          D(0, 1, DG) = (S( 0, 0, SG1));
          D(1, 0, DG) = (S( 0, 0, SG2));
          D(1, 1, DG) = (S( 0, 0, SG1) + S(0, 0, SG2) ) / 2;

          D(0, 0, DB) = (S( 0, 0, SB));
          D(0, 1, DB) = (S( 0, 0, SB));
          D(1, 0, DB) = (S( 0, 0, SB));
          D(1, 1, DB) = (S( 0, 0, SB));

          continue;
        }

        D(0, 0, DR) = (S(-1, 0, SR) + S( 0, 0, SR)) / 2;
        D(0, 1, DR) = (S(-1, 0, SR) + S( 0, 0, SR)) / 2;
        D(1, 0, DR) = (S( 0, 0, SR));
        D(1, 1, DR) = (S( 0, 0, SR));

        D(0, 0, DG) = (S(-1, 0, SG2) + S( 0, 0, SG1) + S( 0, 0, SG2)) / 3;
        D(0, 1, DG) = (S( 0, 0, SG1));
        D(1, 0, DG) = (S( 0, 0, SG2));
        D(1, 1, DG) = (S( 0, 0, SG1) + S(0, 0, SG2) + S(0, 1, SG2) + S(1, 0, SG1)) / 4;

        D(0, 0, DB) = (S( 0, 0, SB));
        D(0, 1, DB) = (S( 0, 0, SB) + S(0, 1, SB)) / 2;
        D(1, 0, DB) = (S( 0, 0, SB) + S(1, 0, SB)) / 2;
        D(1, 1, DB) = (S( 0, 0, SB) + S(0, 1, SB) + S(1, 0, SB) + S(1, 1, SB)) / 4;

        for( x = 1; x < xmax - 1; ++x ) {

          D(0, 0, DR) = (S(-1,-1, SR) + S(-1, 0, SR) + S(0, -1, SR) + S(0, 0, SR)) / 4;
          D(0, 1, DR) = (S(-1, 0, SR) + S( 0, 0, SR)) / 2;
          D(1, 0, DR) = (S( 0,-1, SR) + S(0, 0, SR)) / 2;
          D(1, 1, DR) = (S( 0, 0, SR));

          D(0, 0, DG) = (S(-1, 0, SG2) + S(0, -1, SG1) + S(0,0, SG1) + S(0,0, SG2)) / 4;
          D(0, 1, DG) = (S( 0, 0, SG1));
          D(1, 0, DG) = (S( 0, 0, SG2));
          D(1, 1, DG) = (S( 0, 0, SG1) + S(0,0, SG2) + S(0, 1, SG2) + S(1, 0, SG1)) / 4;

          D(0, 0, DB) = (S( 0, 0, SB));
          D(0, 1, DB) = (S( 0, 0, SB) + S(0, 1, SB)) / 2;
          D(1, 0, DB) = (S( 0, 0, SB) + S(1, 0, SB)) / 2;
          D(1, 1, DB) = (S( 0, 0, SB) + S(0, 1, SB) + S(1, 0, SB) + S(1, 1, SB)) / 4;
        }

        D(0, 0, DR) = (S(-1,-1, SR) + S(-1, 0, SR) + S(0, -1, SR) + S(0, 0, SR)) / 4;
        D(0, 1, DR) = (S(-1, 0, SR) + S( 0, 0, SR)) / 2;
        D(1, 0, DR) = (S( 0,-1, SR) + S( 0, 0, SR)) / 2;
        D(1, 1, DR) = (S( 0, 0, SR));

        D(0, 0, DG) = (S(-1, 0, SG2) + S(0, -1, SG1) + S(0, 0, SG1) + S(0, 0, SG2)) / 4;
        D(0, 1, DG) = (S( 0, 0, SG1));
        D(1, 0, DG) = (S( 0, 0, SG2));
        D(1, 1, DG) = (S( 0, 0, SG1) + S(0, 0, SG2) + S(1, 0, SG1)) / 3;

        D(0, 0, DB) = (S( 0, 0, SB));
        D(0, 1, DB) = (S( 0, 0, SB));
        D(1, 0, DB) = (S( 0, 0, SB) + S(1, 0, SB)) / 2;
        D(1, 1, DB) = (S( 0, 0, SB) + S(1, 0, SB) ) / 2;

      }
    });
    break;

    default:
      CF_ERROR("Unsupported colorid=%d requested", colorid);
      return false;
  }

#undef D
#undef S

  if( _dst.fixedType() ) {
    if( _dst.depth() == dst.depth() ) {
      _dst.move(dst);
    }
    else {
      dst.convertTo(_dst, dst.depth());
    }
  }
  else {
    if( _src.depth() == dst.depth() ) {
      _dst.move(dst);
    }
    else {
      dst.convertTo(_dst, src.depth());
    }
  }

  return true;
}


template<class T>
static bool _nninterpolation(cv::InputArray src, cv::OutputArray dst, enum COLORID colorid)
{
  const int ddepth = dst.fixedType() ? dst.depth() : src.depth();

  switch ( ddepth ) {
  case CV_8U :
    return __nninterpolation<T, uint8_t>(src, dst, colorid);
  case CV_8S :
    return __nninterpolation<T, int8_t>(src, dst, colorid);
  case CV_16U :
    return __nninterpolation<T, uint16_t>(src, dst, colorid);
  case CV_16S :
    return __nninterpolation<T, int16_t>(src, dst, colorid);
  case CV_32S :
    return __nninterpolation<T, int32_t>(src, dst, colorid);
  case CV_32F :
    return __nninterpolation<T, float>(src, dst, colorid);
  case CV_64F :
    return __nninterpolation<T, double>(src, dst, colorid);
  }

  CF_ERROR("Invalid argument: Unsuppoted ddepth=%d requested", ddepth);
  return false;
}


bool nninterpolation(cv::InputArray src, cv::OutputArray dst, enum COLORID colorid)
{
  INSTRUMENT_REGION("");

  switch ( src.depth() ) {
  case CV_8U :
    return _nninterpolation<uint8_t>(src, dst, colorid);
  case CV_8S :
    return _nninterpolation<int8_t>(src, dst, colorid);
  case CV_16U :
    return _nninterpolation<uint16_t>(src, dst, colorid);
  case CV_16S :
    return _nninterpolation<int16_t>(src, dst, colorid);
  case CV_32S :
    return _nninterpolation<int32_t>(src, dst, colorid);
  case CV_32F :
    return _nninterpolation<float>(src, dst, colorid);
  case CV_64F :
    return _nninterpolation<double>(src, dst, colorid);
  }

  CF_ERROR("Invalid argument: Unsuppoted src.depth()=%d requested", src.depth());
  return false;
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

  const cv::Size size =
      tmp.size();

  cv::reduce(tmp, tmp, 1, cv::REDUCE_AVG,
      CV_32F);

  cv::medianBlur(tmp, mb, 5);
  cv::absdiff(tmp, mb, tmp);
  cv::reduce(tmp.reshape(1, tmp.total()), tmp, 1, cv::REDUCE_MAX);
  tmp = tmp.reshape(0, size.height);

  cv::compare(tmp, median_hat_threshold, tmp,
      cv::CMP_GE);

  return cv::countNonZero(tmp) > 0;
}

