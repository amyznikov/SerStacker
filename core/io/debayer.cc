/*
 * debayer.cc
 *
 *  Created on: Jul 31, 2020
 *      Author: amyznikov
 */

#include "debayer.h"
#include <tbb/tbb.h>
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

static DEBAYER_ALGORITHM g_default_debayer_algorithm =
    DEBAYER_NN2;

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

/** @brief
 * Extract src into 4-channel dst matrix with 4 bayer planes ordered as[ R G1 B G2 ]
 * The output size of dst is twice smaller than src
 */
template<class T>
static bool extract_bayer_planes_(cv::InputArray __src, cv::OutputArray __dst, enum COLORID colorid)
{
  typedef tbb::blocked_range<int> range;

  if( (__src.cols() & 0x1) || (__src.rows() & 0x1) || __src.channels() != 1 ) {
    CF_ERROR("Can not make debayer4planes_ for Uneven image size %dx%dx%d",
        __src.cols(), __src.rows(), __src.channels());
    return false;
  }

  const cv::Mat_<T> src = __src.getMat();
  cv::Mat_<cv::Vec<T, 4>> dst(src.size() / 2);

  switch (colorid) {
    case COLORID_BAYER_RGGB: {
      //  [ R  G1 ]
      //  [ G2 B  ]
      tbb::parallel_for(range(0, dst.rows, 256),
          [&](const range & rrange) {
            for ( int y = rrange.begin(), ny = rrange.end(); y < ny; ++y ) {
              const int src_y = y * 2;
              for ( int x = 0; x < dst.cols; ++x ) {
                const int src_x = x * 2;
                const T src_r = src[src_y][src_x];
                const T src_g1 = src[src_y][src_x + 1];
                const T src_g2 = src[src_y + 1][src_x];
                const T src_b = src[src_y + 1][src_x + 1];
                dst[y][x][0] = src_r;
                dst[y][x][1] = src_g1;
                dst[y][x][2] = src_b;
                dst[y][x][3] = src_g2;
              }
            }
          });
      break;
    }

    case COLORID_BAYER_GRBG: {
      //  [ G R ]
      //  [ B G ]
      tbb::parallel_for(range(0, dst.rows, 256),
          [&](const range & rrange) {
            for ( int y = rrange.begin(), ny = rrange.end(); y < ny; ++y ) {
              const int src_y = y * 2;
              for ( int x = 0; x < dst.cols; ++x ) {
                const int src_x = x * 2;
                const T src_r = src[src_y][src_x + 1];
                const T src_g1 = src[src_y][src_x];
                const T src_g2 = src[src_y + 1][src_x + 1];
                const T src_b = src[src_y + 1][src_x];
                dst[y][x][0] = src_r;
                dst[y][x][1] = src_g1;
                dst[y][x][2] = src_b;
                dst[y][x][3] = src_g2;
              }
            }
          });
      break;
    }

    case COLORID_BAYER_GBRG: {
      //  [ G B ]
      //  [ R G ]
      tbb::parallel_for(range(0, dst.rows, 256),
          [&](const range & rrange) {
            for ( int y = rrange.begin(), ny = rrange.end(); y < ny; ++y ) {
              const int src_y = y * 2;
              for ( int x = 0; x < dst.cols; ++x ) {
                const int src_x = x * 2;
                const T src_r = src[src_y+1][src_x];
                const T src_g1 = src[src_y][src_x];
                const T src_g2 = src[src_y + 1][src_x + 1];
                const T src_b = src[src_y][src_x+1];
                dst[y][x][0] = src_r;
                dst[y][x][1] = src_g1;
                dst[y][x][2] = src_b;
                dst[y][x][3] = src_g2;
              }
            }
          });
      break;
    }

    case COLORID_BAYER_BGGR: {
      //  [ B G ]
      //  [ G R ]
      tbb::parallel_for(range(0, dst.rows, 256),
          [&](const range & rrange) {
            for ( int y = rrange.begin(), ny = rrange.end(); y < ny; ++y ) {
              const int src_y = y * 2;
              for ( int x = 0; x < dst.cols; ++x ) {
                const int src_x = x * 2;
                const T src_r = src[src_y+1][src_x+1];
                const T src_g1 = src[src_y][src_x+1];
                const T src_g2 = src[src_y + 1][src_x];
                const T src_b = src[src_y][src_x];
                dst[y][x][0] = src_r;
                dst[y][x][1] = src_g1;
                dst[y][x][2] = src_b;
                dst[y][x][3] = src_g2;
              }
            }
          });
      break;
    }

    case COLORID_BAYER_CYYM: {
      //  [ R G ]
      //  [ G B ]
      //  cv::subtract(depthmax(src.depth()), src, dst);
      //  cv::cvtColor(dst, dst, cv::COLOR_BayerRG2RGB);
      //  B' =  0, 1, 0   B
      //  G' =  1, 0, 0 * G
      //  R' =  0, 0, 1   R

      tbb::parallel_for(range(0, dst.rows, 256),
          [&](const range & rrange) {

            constexpr T dmax = cvmax<T>();

            for ( int y = rrange.begin(), ny = rrange.end(); y < ny; ++y ) {

              const int src_y = y * 2;

              for ( int x = 0; x < dst.cols; ++x ) {
                const int src_x = x * 2;
                const T src_r = dmax - src[src_y][src_x];
                const T src_g1 = dmax - src[src_y][src_x + 1];
                const T src_g2 = dmax - src[src_y + 1][src_x];
                const T src_b = dmax - src[src_y + 1][src_x + 1];
                dst[y][x][0] = src_r;
                dst[y][x][1] = src_g1;
                dst[y][x][2] = src_b;
                dst[y][x][3] = src_g2;
              }
            }
          });

      break;
    }

    case COLORID_BAYER_YCMY: {
      //  [ G R ]
      //  [ B G ]
      //  cv::subtract(depthmax(src.depth()), src, dst);
      //  cv::cvtColor(dst, dst, cv::COLOR_BayerGR2RGB);
      //  B' = 0, 1, 0  B
      //  G' = 1, 0, 0  G
      //  R' = 0, 0, 1  R

      tbb::parallel_for(range(0, dst.rows, 256),
          [&](const range & rrange) {

            constexpr T dmax = cvmax<T>();

            for ( int y = rrange.begin(), ny = rrange.end(); y < ny; ++y ) {

              const int src_y = y * 2;

              for ( int x = 0; x < dst.cols; ++x ) {
                const int src_x = x * 2;
                const T src_r = dmax - src[src_y][src_x + 1];
                const T src_g1 = dmax - src[src_y][src_x];
                const T src_g2 = dmax - src[src_y + 1][src_x + 1];
                const T src_b = dmax - src[src_y + 1][src_x];
                dst[y][x][0] = src_r;
                dst[y][x][1] = src_g1;
                dst[y][x][2] = src_b;
                dst[y][x][3] = src_g2;
              }
            }
          });

      break;
    }

    case COLORID_BAYER_YMCY: {
      //  [ G B ]
      //  [ R G ]
      //  cv::subtract(depthmax(src.depth()), src, dst);
      //  cv::cvtColor(dst, dst, cv::COLOR_BayerGR2RGB);
      //  B' = 0, 1, 0  B
      //  G' = 1, 0, 0  G
      //  R' = 0, 0, 1  R
      tbb::parallel_for(range(0, dst.rows, 256),
          [&](const range & rrange) {

            constexpr T dmax = cvmax<T>();

            for ( int y = rrange.begin(), ny = rrange.end(); y < ny; ++y ) {

              const int src_y = y * 2;

              for ( int x = 0; x < dst.cols; ++x ) {
                const int src_x = x * 2;
                const T src_r = dmax - src[src_y+1][src_x];
                const T src_g1 = dmax - src[src_y][src_x];
                const T src_g2 = dmax - src[src_y + 1][src_x + 1];
                const T src_b = dmax - src[src_y][src_x+1];
                dst[y][x][0] = src_r;
                dst[y][x][1] = src_g1;
                dst[y][x][2] = src_b;
                dst[y][x][3] = src_g2;
              }
            }
          });
      break;
    }

    case COLORID_BAYER_MYYC: {
      //  [ B G ]
      //  [ G R ]
      //   cv::subtract(depthmax(src.depth()), src, dst);
      //   cv::cvtColor(dst, dst, cv::COLOR_BayerBG2RGB);
      //  B' = 0, 1, 0  B
      //  G' = 1, 0, 0  G
      //  R' = 0, 0, 1  R
      tbb::parallel_for(range(0, dst.rows, 256),
          [&](const range & rrange) {

            constexpr T dmax = cvmax<T>();
            for ( int y = rrange.begin(), ny = rrange.end(); y < ny; ++y ) {

              const int src_y = y * 2;

              for ( int x = 0; x < dst.cols; ++x ) {
                const int src_x = x * 2;
                const T src_r = dmax - src[src_y+1][src_x+1];
                const T src_g1 = dmax - src[src_y][src_x+1];
                const T src_g2 = dmax - src[src_y + 1][src_x];
                const T src_b = dmax - src[src_y][src_x];
                dst[y][x][0] = src_r;
                dst[y][x][1] = src_g1;
                dst[y][x][2] = src_b;
                dst[y][x][3] = src_g2;
              }
            }
          });
      break;
    }

    default:
      return false;
  }

  if( __dst.fixedType() ) {
    dst.convertTo(__dst, __dst.depth());
  }
  else {
    __dst.move(dst);
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
    return extract_bayer_planes_<uint8_t>(src, dst, colorid);
  case CV_8S :
    return extract_bayer_planes_<int8_t>(src, dst, colorid);
  case CV_16U :
    return extract_bayer_planes_<uint16_t>(src, dst, colorid);
  case CV_16S :
    return extract_bayer_planes_<int16_t>(src, dst, colorid);
  case CV_32S :
    return extract_bayer_planes_<int32_t>(src, dst, colorid);
  case CV_32F :
    return extract_bayer_planes_<float>(src, dst, colorid);
  case CV_64F :
    return extract_bayer_planes_<double>(src, dst, colorid);
  }

  return false;
}





/** @brief
 * Combine input 4-channel src ordered as [ R G1 B G2 ] into 3-channel BGR dst matrix.
 * The output size of dst is the same as src
 */
template<class SrcType, class DstType>
static bool bayer_planes_to_bgr__(cv::InputArray __src, cv::OutputArray __dst)
{
  const cv::Mat_<cv::Vec<SrcType, 4>> src = __src.getMat();
  cv::Mat_<cv::Vec<DstType, 3>> dst;

  if ( __dst.getMatRef().data == src.data ) {
    dst.create(src.size());
  }
  else {
    __dst.create(src.size(), CV_MAKETYPE(cv::DataType<DstType>::depth, 3));
    dst = __dst.getMatRef();
  }


  typedef tbb::blocked_range<int> range;

  tbb::parallel_for(range(0, dst.rows),
      [&](const range & rrange) {
        for ( int y = rrange.begin(), ny = rrange.end(); y < ny; ++y ) {
          for ( int x = 0; x < dst.cols; ++x ) {

            const SrcType & r   = src[y][x][0];
            const SrcType & g1  = src[y][x][1];
            const SrcType & b   = src[y][x][2];
            const SrcType & g2  = src[y][x][3];

            dst[y][x][0] = b;
            dst[y][x][1] = 0.5 * ((double)g1 + (double)g2);
            dst[y][x][2] = r;
          }
        }
      }, tbb::static_partitioner());


  if ( dst.data != __dst.getMatRef().data ) {
    if ( __dst.fixedType() ) {
      dst.convertTo(__dst, __dst.depth());
    }
    else {
      __dst.move(dst);
    }
  }

  return true;
}



/** @brief
 * Combine input 4-channel src ordered as [ R G1 B G2 ] into 3-channel BGR dst matrix.
 * The output size of dst is the same as src
 */
template<class T>
static bool bayer_planes_to_bgr_(cv::InputArray src, cv::OutputArray dst, int ddepth)
{
  if ( src.channels() != 4 ) {
    CF_ERROR("Invalid argument: 4-channel input image expected but src.channels=%d",
        src.channels());
    return false;
  }

  if ( dst.fixedType() && dst.channels() != 3 ) {
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

  switch ( ddepth ) {
  case CV_8U :
    return bayer_planes_to_bgr__<T, uint8_t>(src, dst);
  case CV_8S :
    return bayer_planes_to_bgr__<T, int8_t>(src, dst);
  case CV_16U :
    return bayer_planes_to_bgr__<T, uint16_t>(src, dst);
  case CV_16S :
    return bayer_planes_to_bgr__<T, int16_t>(src, dst);
  case CV_32S :
    return bayer_planes_to_bgr__<T, int32_t>(src, dst);
  case CV_32F :
    return bayer_planes_to_bgr__<T, float>(src, dst);
  case CV_64F :
    return bayer_planes_to_bgr__<T, double>(src, dst);
  }

  return false;
}

/** @brief
 * Combine input 4-channel src ordered as [ R G1 B G2 ] into 3-channel BGR dst matrix.
 * The output size of dst is the same as src
 */
bool bayer_planes_to_bgr(cv::InputArray src, cv::OutputArray dst, int ddepth)
{
  switch ( src.depth() ) {
  case CV_8U :
    return bayer_planes_to_bgr_<uint8_t>(src, dst, ddepth);
  case CV_8S :
    return bayer_planes_to_bgr_<int8_t>(src, dst, ddepth);
  case CV_16U :
    return bayer_planes_to_bgr_<uint16_t>(src, dst, ddepth);
  case CV_16S :
    return bayer_planes_to_bgr_<int16_t>(src, dst, ddepth);
  case CV_32S :
    return bayer_planes_to_bgr_<int32_t>(src, dst, ddepth);
  case CV_32F :
    return bayer_planes_to_bgr_<float>(src, dst, ddepth);
  case CV_64F :
    return bayer_planes_to_bgr_<double>(src, dst, ddepth);
  }

  return false;
}



/** @brief
 * Extract bayer src into dense 3-channel BGR dst matrix with .
 * The output size of dst is the same as src
 */

template<class T>
static bool extract_bayer_matrix_(cv::InputArray __src, cv::OutputArray __dst, enum COLORID colorid)
{
  typedef tbb::blocked_range<int> range;

  if ( (__src.cols() & 0x1) || (__src.rows() & 0x1) || __src.channels() != 1 )  {
    CF_ERROR("Can not make debayer4planes_ for Uneven image size %dx%dx%d",
        __src.cols(), __src.rows(), __src.channels());
    return false;
  }

  const cv::Mat_<T> src =
      __src.getMat();

  cv::Mat_<cv::Vec<T,3>>
      dst(src.size());

  static const auto R =
      [](const T & v) {
        return cv::Vec<T, 3>(0, 0, v);
      };

  static const auto G =
      [](const T & v) {
        return cv::Vec<T, 3>(0, v, 0);
      };

  static const auto B =
      [](const T & v) {
        return cv::Vec<T, 3>(v, 0, 0);
      };

  switch (colorid) {
    case COLORID_BAYER_RGGB: {
      //  [ R  G1 ]
      //  [ G2 B  ]
      tbb::parallel_for(range(0, src.rows / 2),
          [&](const range & rrange) {
            for ( int y = rrange.begin(), ny = rrange.end(); y < ny; ++y ) {
              for ( int x = 0; x < src.cols / 2; ++x ) {
                dst[2*y + 0][2*x + 0] = R(src[2*y + 0][2*x + 0]);
                dst[2*y + 0][2*x + 1] = G(src[2*y + 0][2*x + 1]);
                dst[2*y + 1][2*x + 0] = G(src[2*y + 1][2*x + 0]);
                dst[2*y + 1][2*x + 1] = B(src[2*y + 1][2*x + 1]);
              }
            }
          }, tbb::static_partitioner());
      break;
    }

  case COLORID_BAYER_GRBG : {
    //  [ G R ]
    //  [ B G ]
    tbb::parallel_for(range(0, src.rows / 2),
        [&](const range & rrange) {
          for ( int y = rrange.begin(), ny = rrange.end(); y < ny; ++y ) {
            for ( int x = 0; x < src.cols / 2; ++x ) {
              dst[2*y + 0][2*x + 0] = G(src[2*y + 0][2*x + 0]);
              dst[2*y + 0][2*x + 1] = R(src[2*y + 0][2*x + 1]);
              dst[2*y + 1][2*x + 0] = B(src[2*y + 1][2*x + 0]);
              dst[2*y + 1][2*x + 1] = G(src[2*y + 1][2*x + 1]);
            }
          }
        }, tbb::static_partitioner());
    break;
  }


  case COLORID_BAYER_GBRG : {
    //  [ G B ]
    //  [ R G ]
    tbb::parallel_for(range(0, src.rows / 2),
        [&](const range & rrange) {
          for ( int y = rrange.begin(), ny = rrange.end(); y < ny; ++y ) {
            for ( int x = 0; x < src.cols / 2; ++x ) {
              dst[2*y + 0][2*x + 0] = G(src[2*y + 0][2*x + 0]);
              dst[2*y + 0][2*x + 1] = B(src[2*y + 0][2*x + 1]);
              dst[2*y + 1][2*x + 0] = R(src[2*y + 1][2*x + 0]);
              dst[2*y + 1][2*x + 1] = G(src[2*y + 1][2*x + 1]);
            }
          }
        }, tbb::static_partitioner());
    break;
  }

  case COLORID_BAYER_BGGR : {
    //  [ B G ]
    //  [ G R ]
    tbb::parallel_for(range(0, src.rows / 2),
        [&](const range & rrange) {
          for ( int y = rrange.begin(), ny = rrange.end(); y < ny; ++y ) {
            for ( int x = 0; x < src.cols / 2; ++x ) {
              dst[2*y + 0][2*x + 0] = B(src[2*y + 0][2*x + 0]);
              dst[2*y + 0][2*x + 1] = G(src[2*y + 0][2*x + 1]);
              dst[2*y + 1][2*x + 0] = G(src[2*y + 1][2*x + 0]);
              dst[2*y + 1][2*x + 1] = R(src[2*y + 1][2*x + 1]);
            }
          }
        }, tbb::static_partitioner());
    break;
  }

  case COLORID_BAYER_CYYM : {
    //  [ R G ]
    //  [ G B ]
    //  cv::subtract(depthmax(src.depth()), src, dst);
    //  cv::cvtColor(dst, dst, cv::COLOR_BayerRG2RGB);
    //  B' =  0, 1, 0   B
    //  G' =  1, 0, 0 * G
    //  R' =  0, 0, 1   R
    tbb::parallel_for(range(0, src.rows / 2),
        [&](const range & rrange) {

          constexpr T M = cvmax<T>();

          for ( int y = rrange.begin(), ny = rrange.end(); y < ny; ++y ) {
            for ( int x = 0; x < src.cols / 2; ++x ) {
              dst[2*y + 0][2*x + 0] = R(M - src[2*y + 0][2*x + 0]);
              dst[2*y + 0][2*x + 1] = G(M - src[2*y + 0][2*x + 1]);
              dst[2*y + 1][2*x + 0] = G(M - src[2*y + 1][2*x + 0]);
              dst[2*y + 1][2*x + 1] = B(M - src[2*y + 1][2*x + 1]);
            }
          }
        }, tbb::static_partitioner());
    break;
  }


  case COLORID_BAYER_YCMY : {
    //  [ G R ]
    //  [ B G ]
    //  cv::subtract(depthmax(src.depth()), src, dst);
    //  cv::cvtColor(dst, dst, cv::COLOR_BayerGR2RGB);
    //  B' = 0, 1, 0  B
    //  G' = 1, 0, 0  G
    //  R' = 0, 0, 1  R
    tbb::parallel_for(range(0, src.rows / 2),
        [&](const range & rrange) {

          constexpr T M = cvmax<T>();

          for ( int y = rrange.begin(), ny = rrange.end(); y < ny; ++y ) {
            for ( int x = 0; x < src.cols / 2; ++x ) {
              dst[2*y + 0][2*x + 0] = G(M - src[2*y + 0][2*x + 0]);
              dst[2*y + 0][2*x + 1] = R(M - src[2*y + 0][2*x + 1]);
              dst[2*y + 1][2*x + 0] = B(M - src[2*y + 1][2*x + 0]);
              dst[2*y + 1][2*x + 1] = G(M - src[2*y + 1][2*x + 1]);
            }
          }
        }, tbb::static_partitioner());

    break;
  }


  case COLORID_BAYER_YMCY : {
    //  [ G B ]
    //  [ R G ]
    //  cv::subtract(depthmax(src.depth()), src, dst);
    //  cv::cvtColor(dst, dst, cv::COLOR_BayerGR2RGB);
    //  B' = 0, 1, 0  B
    //  G' = 1, 0, 0  G
    //  R' = 0, 0, 1  R
    tbb::parallel_for(range(0, src.rows / 2),
        [&](const range & rrange) {

          constexpr T M = cvmax<T>();

          for ( int y = rrange.begin(), ny = rrange.end(); y < ny; ++y ) {
            for ( int x = 0; x < src.cols / 2; ++x ) {
              dst[2*y + 0][2*x + 0] = G(M - src[2*y + 0][2*x + 0]);
              dst[2*y + 0][2*x + 1] = B(M - src[2*y + 0][2*x + 1]);
              dst[2*y + 1][2*x + 0] = R(M - src[2*y + 1][2*x + 0]);
              dst[2*y + 1][2*x + 1] = G(M - src[2*y + 1][2*x + 1]);
            }
          }
        }, tbb::static_partitioner());

    break;
  }


  case COLORID_BAYER_MYYC : {
    //  [ B G ]
    //  [ G R ]
    //   cv::subtract(depthmax(src.depth()), src, dst);
    //   cv::cvtColor(dst, dst, cv::COLOR_BayerBG2RGB);
    //  B' = 0, 1, 0  B
    //  G' = 1, 0, 0  G
    //  R' = 0, 0, 1  R
    tbb::parallel_for(range(0, src.rows / 2),
        [&](const range & rrange) {

          constexpr T M = cvmax<T>();

          for ( int y = rrange.begin(), ny = rrange.end(); y < ny; ++y ) {
            for ( int x = 0; x < src.cols / 2; ++x ) {
              dst[2*y + 0][2*x + 0] = B(M - src[2*y + 0][2*x + 0]);
              dst[2*y + 0][2*x + 1] = G(M - src[2*y + 0][2*x + 1]);
              dst[2*y + 1][2*x + 0] = G(M - src[2*y + 1][2*x + 0]);
              dst[2*y + 1][2*x + 1] = R(M - src[2*y + 1][2*x + 1]);
            }
          }
        }, tbb::static_partitioner());

    break;
  }

  default :
    return false;
  }

  if ( __dst.fixedType() ) {
    dst.convertTo(__dst, __dst.depth());
  }
  else {
    __dst.move(dst);
  }

  return true;
}

/** @brief
 * Extract bayer src into dense 3-channel BGR dst matrix with .
 * The output size of dst is the same as src
 */
bool extract_bayer_matrix(cv::InputArray src, cv::OutputArray dst, enum COLORID colorid)
{
  switch (src.depth()) {
    case CV_8U:
      return extract_bayer_matrix_<uint8_t>(src, dst, colorid);
    case CV_8S:
      return extract_bayer_matrix_<int8_t>(src, dst, colorid);
    case CV_16U:
      return extract_bayer_matrix_<uint16_t>(src, dst, colorid);
    case CV_16S:
      return extract_bayer_matrix_<int16_t>(src, dst, colorid);
    case CV_32S:
      return extract_bayer_matrix_<int32_t>(src, dst, colorid);
    case CV_32F:
      return extract_bayer_matrix_<float>(src, dst, colorid);
    case CV_64F:
      return extract_bayer_matrix_<double>(src, dst, colorid);
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

template<typename T>
static bool debayer_avgc_tbb(const cv::Mat_<T> & src, cv::Mat_<cv::Vec<T, 3>> & dst, COLORID colorid)
{
  using CalcType =
      typename std::conditional_t<
        std::is_floating_point<T>::value,
        T,
        int64_t>;

  const int outHeight = src.rows / 2;
  const int outWidth = src.cols / 2;
  dst.create(outHeight, outWidth);

  // Channel indexes inside of 2x2 binnig blocks
  // p00 p01
  // p10 p11
  int ri, g1i, g2i, bi; // индексы для (row, col)

  switch (colorid) {
    case COLORID_BAYER_RGGB: ri = 0; g1i = 1; g2i = 2; bi = 3; break; // R G1 / G2 B
    case COLORID_BAYER_GRBG: g1i = 0; ri = 1; bi = 2; g2i = 3; break; // G1 R / B G2
    case COLORID_BAYER_GBRG: g1i = 0; bi = 1; ri = 2; g2i = 3; break; // G1 B / R G2
    case COLORID_BAYER_BGGR: bi = 0; g1i = 1; g2i = 2; ri = 3; break; // B G1 / G2 R
    case COLORID_BAYER_CYYM: bi = 0; g1i = 1; g2i = 2; ri = 3; break;
    case COLORID_BAYER_YCMY: g1i = 0; bi = 1; ri = 2; g2i = 3; break;
    case COLORID_BAYER_YMCY: g1i = 0; ri = 1; bi = 2; g2i = 3; break;
    case COLORID_BAYER_MYYC: ri = 0; g1i = 1; g2i = 2; bi = 3; break;
    default: // Not supported
      CF_ERROR("Not supported colorid = %d", colorid);
      return false;
  }

  tbb::parallel_for(tbb::blocked_range<int>(0, outHeight),
      [&](const tbb::blocked_range<int> & range) {
        for (int y = range.begin(); y != range.end(); ++y) {
          const T * r1 = src[y * 2];
          const T * r2 = src[y * 2 + 1];
          cv::Vec<T, 3>* dstRow = dst[y];

          for (int x = 0; x < outWidth; ++x) {
            const T p[] = {r1[x*2], r1[x*2+1], r2[x*2], r2[x*2+1]};
            const T r = p[ri];
            const T b = p[bi];
            const T g = static_cast<T>((static_cast<CalcType>(p[g1i]) + static_cast<CalcType>(p[g2i])) / 2);
            dstRow[x] = cv::Vec<T, 3>(b, g, r);
        }
      }
    }, tbb::static_partitioner());

  return true;
}


bool demosaic_avgc(cv::InputArray src, cv::OutputArray dst, COLORID colorid)
{
  if( src.empty() || src.channels() != 1 ) {
    CF_ERROR("Single-channel input image is expected");
    return false;
  }

  const cv::Size srcSize = src.size();
  if( srcSize.width % 2 != 0 || srcSize.height % 2 != 0 ) {
    CF_ERROR("Single-channel even-sized input image is expected");
    return false;
  }

  const int depth = src.depth();
  const int outHeight = srcSize.height / 2;
  const int outWidth = srcSize.width / 2;

  if( dst.fixedSize() && dst.size() != cv::Size(outWidth, outHeight) ) {
    CF_ERROR("Fixed-size destination image has incorrect size");
    return false;
  }

  if( dst.fixedType() && dst.type() != CV_MAKETYPE(depth, 3) ) {
    CF_ERROR("Fixed-type destination image has incorrect type");
    return false;
  }

  const cv::Mat srcMat = src.getMat();

  dst.create(outHeight, outWidth, CV_MAKETYPE(depth, 3));
  cv::Mat& dstMat = dst.getMatRef();

  switch (depth) {
    case CV_8U:  debayer_avgc_tbb<uint8_t>(srcMat, (cv::Mat_<cv::Vec<uint8_t, 3>>&)dstMat, colorid); break;
    case CV_8S:  debayer_avgc_tbb<int8_t>(srcMat, (cv::Mat_<cv::Vec<int8_t, 3>>&)dstMat, colorid); break;
    case CV_16U: debayer_avgc_tbb<uint16_t>(srcMat, (cv::Mat_<cv::Vec<uint16_t, 3>>&)dstMat, colorid); break;
    case CV_16S: debayer_avgc_tbb<int16_t>(srcMat, (cv::Mat_<cv::Vec<int16_t, 3>>&)dstMat, colorid); break;
    case CV_32S: debayer_avgc_tbb<int32_t>(srcMat, (cv::Mat_<cv::Vec<int32_t, 3>>&)dstMat, colorid); break;
    case CV_32F: debayer_avgc_tbb<float>(srcMat, (cv::Mat_<cv::Vec<float, 3>>&)dstMat, colorid); break;
    case CV_64F: debayer_avgc_tbb<double>(srcMat, (cv::Mat_<cv::Vec<double, 3>>&)dstMat, colorid); break;
    default:
      CF_ERROR("Not supported input image depth %d", depth);
      return false;
  }
  return true;
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
 * Bayer interpolation using nearest neighbors.
 * Expected input channels are [ R G1 B G2 ]
 */
template<class SrcType, class DstType>
static bool nninterpolation__(cv::InputArray __src, cv::OutputArray __dst, enum COLORID colorid)
{
  if ( __src.channels() != 4 ) {
    CF_ERROR("Invalid arg: 4-channel input image expected, but src.channels=%d",
        __src.channels());
    return false;
  }

  if ( __dst.fixedType() && __dst.channels() != 3 ) {
    CF_ERROR("Invalid argument: 3-channel output destination image expected but dst.channels=%d",
        __dst.channels());
    return false;
  }

  typedef cv::Vec<SrcType, 4> SrcVec;
  typedef cv::Vec<DstType, 3> DstVec;

  const cv::Mat_<SrcVec> src =
      __src.getMat();

  const cv::Size src_size =
      src.size();

  const cv::Size dst_size =
      src_size * 2;

  cv::Mat_<DstVec> dst(dst_size, DstVec::all(0));

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

  int x, y;

#define D(yy, xx, cc) dst[2 * y + (yy)][2 * x + (xx)][(cc)]
#define S(yy, xx, cc) src[y+(yy)][x+(xx)][(cc)]

  switch (colorid) {
    case COLORID_BAYER_RGGB:
      // [R G   R G   R G   R G   R G   R G ]
      // [G B   G B   G B   G B   G B   G B ]

      // [R G   R G   R G   R G   R G   R G ]
      // [G B   G B   G B   G B   G B   G B ]

      // [R G   R G   R G   R G   R G   R G ]
      // [G B   G B   G B   G B   G B   G B ]

      for( y = 0; y < src_size.height; ++y ) {

        x = 0;

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
          for( x = 1; x < src_size.width - 1; ++x ) {

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

        if( y == src_size.height - 1 ) {

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

          for( x = 1; x < src_size.width - 1; ++x ) {

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

        for( x = 1; x < src_size.width - 1; ++x ) {

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
      break;

    case COLORID_BAYER_GRBG:
      //  [ G R    G R    G R    G R    G R   ]
      //  [ B G    B G    B G    B G    B G   ]

      //  [ G R    G R    G R    G R    G R   ]
      //  [ B G    B G    B G    B G    B G   ]

      //  [ G R    G R    G R    G R    G R   ]
      //  [ B G    B G    B G    B G    B G   ]

      //  [ G R    G R    G R    G R    G R   ]
      //  [ B G    B G    B G    B G    B G   ]

      for( y = 0; y < src_size.height - 1; ++y ) {

        x = 0;

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


          for( x = 1; x < src_size.width - 1; ++x ) {

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

        if ( y == src_size.height - 1 ) {

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

          for( x = 1; x < src_size.width - 1; ++x ) {

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

        for( x = 1; x < src_size.width - 1; ++x ) {

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
      break;

    case COLORID_BAYER_GBRG:
      //  [ G B   G B   G B   G B   G B   G B]
      //  [ R G   R G   R G   R G   R G   R G]

      //  [ G B   G B   G B   G B   G B   G B]
      //  [ R G   R G   R G   R G   R G   R G]

      //  [ G B   G B   G B   G B   G B   G B]
      //  [ R G   R G   R G   R G   R G   R G]

      //  [ G B   G B   G B   G B   G B   G B]
      //  [ R G   R G   R G   R G   R G   R G]

      for( y = 0; y < src_size.height; ++y ) {

        x = 0;

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

          for( x = 1; x < src_size.width - 1; ++x ) {

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

        if ( y == src_size.height - 1 ) {

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

          for( x = 1; x < src_size.width - 1; ++x ) {

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

        for( x = 1; x < src_size.width - 1; ++x ) {

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
      break;

    case COLORID_BAYER_BGGR:
      //  [ B G   B G   B G   B G   B G]
      //  [ G R   G R   G R   G R   G R]

      //  [ B G   B G   B G   B G   B G]
      //  [ G R   G R   G R   G R   G R]

      //  [ B G   B G   B G   B G   B G]
      //  [ G R   G R   G R   G R   G R]

      //  [ B G   B G   B G   B G   B G]
      //  [ G R   G R   G R   G R   G R]

      for( y = 0; y < src_size.height; ++y ) {

        x = 0;

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

          for( x = 1; x < src_size.width - 1; ++x ) {

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

        if( y == src_size.height - 1 ) {

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

          for( x = 1; x < src_size.width - 1; ++x ) {

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

        for( x = 1; x < src_size.width - 1; ++x ) {

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
      break;

    default:
      CF_ERROR("Unsupported colorid=%d requested", colorid);
      return false;
  }

#undef D
#undef S

  if( __dst.fixedType() ) {
    if( __dst.depth() == dst.depth() ) {
      __dst.move(dst);
    }
    else {
      dst.convertTo(__dst, dst.depth());
    }
  }
  else {
    if( __src.depth() == dst.depth() ) {
      __dst.move(dst);
    }
    else {
      dst.convertTo(__dst, src.depth());
    }
  }

  return true;
}


template<class T>
static bool nninterpolation_(cv::InputArray src, cv::OutputArray dst, enum COLORID colorid)
{
  const int ddepth = dst.fixedType() ? dst.depth() : src.depth();

  switch ( ddepth ) {
  case CV_8U :
    return nninterpolation__<T, uint8_t>(src, dst, colorid);
  case CV_8S :
    return nninterpolation__<T, int8_t>(src, dst, colorid);
  case CV_16U :
    return nninterpolation__<T, uint16_t>(src, dst, colorid);
  case CV_16S :
    return nninterpolation__<T, int16_t>(src, dst, colorid);
  case CV_32S :
    return nninterpolation__<T, int32_t>(src, dst, colorid);
  case CV_32F :
    return nninterpolation__<T, float>(src, dst, colorid);
  case CV_64F :
    return nninterpolation__<T, double>(src, dst, colorid);
  }

  CF_ERROR("Invalid argument: Unsuppoted ddepth=%d requested", ddepth);
  return false;
}


bool nninterpolation(cv::InputArray src, cv::OutputArray dst, enum COLORID colorid)
{
  INSTRUMENT_REGION("");

  switch ( src.depth() ) {
  case CV_8U :
    return nninterpolation_<uint8_t>(src, dst, colorid);
  case CV_8S :
    return nninterpolation_<int8_t>(src, dst, colorid);
  case CV_16U :
    return nninterpolation_<uint16_t>(src, dst, colorid);
  case CV_16S :
    return nninterpolation_<int16_t>(src, dst, colorid);
  case CV_32S :
    return nninterpolation_<int32_t>(src, dst, colorid);
  case CV_32F :
    return nninterpolation_<float>(src, dst, colorid);
  case CV_64F :
    return nninterpolation_<double>(src, dst, colorid);
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


template<class T>
static bool bayer_denoise_(cv::Mat & bayer_image, double k)
{
  cv::Mat_<T> src_image = bayer_image;

  cv::Mat_<cv::Vec<T, 4>> planes_image;
  cv::Mat_<cv::Vec<T, 4>> median_image;
  cv::Mat_<cv::Vec<T, 4>> variation_image;
  cv::Mat_<cv::Vec<T, 4>> mean_variation_image;

  static const cv::Matx33f SE(
      1. / 8, 1. / 8, 1. / 8,
      1. / 8, 0.0, 1. / 8,
      1. / 8, 1. / 8, 1. / 8);

  const double minimal_mean_variation_for_very_smooth_images =
      src_image.depth() >= CV_32F ? 1e-3 : 1;

  planes_image.create(src_image.rows / 2, src_image.cols / 2);

  for( int y = 0; y < src_image.rows / 2; ++y ) {
    for( int x = 0; x < src_image.cols / 2; ++x ) {
      planes_image[y][x][0] = src_image[2 * y + 0][2 * x + 0];
      planes_image[y][x][1] = src_image[2 * y + 0][2 * x + 1];
      planes_image[y][x][2] = src_image[2 * y + 1][2 * x + 0];
      planes_image[y][x][3] = src_image[2 * y + 1][2 * x + 1];
    }
  }

  cv::medianBlur(planes_image, median_image, 3);

  cv::absdiff(planes_image, median_image, variation_image);
  cv::filter2D(variation_image, mean_variation_image, -1, SE * k, cv::Point(-1, -1),
      minimal_mean_variation_for_very_smooth_images);

  for( int y = 0; y < src_image.rows / 2; ++y ) {
    for( int x = 0; x < src_image.cols / 2; ++x ) {

      if( variation_image[y][x][0] > mean_variation_image[y][x][0] ) {
        src_image[2 * y + 0][2 * x + 0] = median_image[y][x][0];
      }
      if( variation_image[y][x][1] > mean_variation_image[y][x][1] ) {
        src_image[2 * y + 0][2 * x + 1] = median_image[y][x][1];
      }
      if( variation_image[y][x][2] > mean_variation_image[y][x][2] ) {
        src_image[2 * y + 1][2 * x + 0] = median_image[y][x][2];
      }
      if( variation_image[y][x][3] > mean_variation_image[y][x][3] ) {
        src_image[2 * y + 1][2 * x + 1] = median_image[y][x][3];
      }
    }
  }

  return true;
}

/**
 *
 */
bool bayer_denoise(cv::Mat & bayer_image, double k)
{
  switch (bayer_image.depth()) {
    case CV_8U:
      return bayer_denoise_<uint8_t>(bayer_image, k);
    case CV_8S:
      return bayer_denoise_<int8_t>(bayer_image, k);
    case CV_16U:
      return bayer_denoise_<uint16_t>(bayer_image, k);
    case CV_16S:
      return bayer_denoise_<int16_t>(bayer_image, k);
    case CV_32S:
      return bayer_denoise_<int32_t>(bayer_image, k);
    case CV_32F:
      return bayer_denoise_<float>(bayer_image, k);
    case CV_64F:
      return bayer_denoise_<double>(bayer_image, k);
  }

  CF_ERROR("APP BUG: Unexpected bayer_image.depth()=%d", bayer_image.depth());

  return false;
}

