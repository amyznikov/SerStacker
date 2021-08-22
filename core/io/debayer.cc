/*
 * debayer.cc
 *
 *  Created on: Jul 31, 2020
 *      Author: amyznikov
 */

#include "debayer.h"
#include <tbb/tbb.h>
#include <core/debug.h>



const struct DEBAYER_ALGORITHM_desc debayer_algorithms[] = {
    {"DISABLE", DEBAYER_DISABLE},
    {"GB", DEBAYER_GB},
    {"EA", DEBAYER_EA},
    {"VNG", DEBAYER_VNG},
    {"NN", DEBAYER_NN},
    {"GBNR", DEBAYER_GBNR},
    {nullptr, DEBAYER_GB} // must  be last
};


std::string toStdString(enum DEBAYER_ALGORITHM v)
{
  for ( uint i = 0; debayer_algorithms[i].name; ++i ) {
    if ( debayer_algorithms[i].value == v ) {
      return debayer_algorithms[i].name;
    }
  }
  return "";
}

enum DEBAYER_ALGORITHM fromStdString(const std::string & s, enum DEBAYER_ALGORITHM defval)
{
  const char * cstr = s.c_str();

  for ( uint i = 0; debayer_algorithms[i].name; ++i ) {
    if ( strcasecmp(debayer_algorithms[i].name, cstr) == 0 ) {
      return debayer_algorithms[i].value;
    }
  }
  return defval;
}


/** @brief
 * Max native value for given depth
 */
static inline cv::Scalar depthmax(int depth)
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

  if ( (__src.cols() & 0x1) || (__src.rows() & 0x1) || __src.channels() != 1 )  {
    CF_ERROR("Can not make debayer4planes_ for Uneven image size %dx%dx%d",
        __src.cols(), __src.rows(), __src.channels());
    return false;
  }

  const cv::Mat_<T> src = __src.getMat();
  cv::Mat_<cv::Vec<T,4>> dst(src.size() / 2);

  switch ( colorid ) {
  case COLORID_BAYER_RGGB : {
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

  case COLORID_BAYER_GRBG : {
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


  case COLORID_BAYER_GBRG : {
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

  case COLORID_BAYER_BGGR : {
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

  case COLORID_BAYER_CYYM : {
    //  [ R G ]
    //  [ G B ]
    //  cv::subtract(depthmax(src.depth()), src, dst);
    //  cv::cvtColor(dst, dst, cv::COLOR_BayerRG2RGB);
    //  B' =  0, 1, 0   B
    //  G' =  1, 0, 0 * G
    //  R' =  0, 0, 1   R

    tbb::parallel_for(range(0, dst.rows, 256),
        [&](const range & rrange) {
          for ( int y = rrange.begin(), ny = rrange.end(); y < ny; ++y ) {

            const int src_y = y * 2;
            const T dmax = depthmax(src.depth())[0];

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


  case COLORID_BAYER_YCMY : {
    //  [ G R ]
    //  [ B G ]
    //  cv::subtract(depthmax(src.depth()), src, dst);
    //  cv::cvtColor(dst, dst, cv::COLOR_BayerGR2RGB);
    //  B' = 0, 1, 0  B
    //  G' = 1, 0, 0  G
    //  R' = 0, 0, 1  R

    tbb::parallel_for(range(0, dst.rows, 256),
        [&](const range & rrange) {
          for ( int y = rrange.begin(), ny = rrange.end(); y < ny; ++y ) {

            const int src_y = y * 2;
            const T dmax = depthmax(src.depth())[0];

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


  case COLORID_BAYER_YMCY : {
    //  [ G B ]
    //  [ R G ]
    //  cv::subtract(depthmax(src.depth()), src, dst);
    //  cv::cvtColor(dst, dst, cv::COLOR_BayerGR2RGB);
    //  B' = 0, 1, 0  B
    //  G' = 1, 0, 0  G
    //  R' = 0, 0, 1  R
    tbb::parallel_for(range(0, dst.rows, 256),
        [&](const range & rrange) {
          for ( int y = rrange.begin(), ny = rrange.end(); y < ny; ++y ) {

            const int src_y = y * 2;
            const T dmax = depthmax(src.depth())[0];

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


  case COLORID_BAYER_MYYC : {
    //  [ B G ]
    //  [ G R ]
    //   cv::subtract(depthmax(src.depth()), src, dst);
    //   cv::cvtColor(dst, dst, cv::COLOR_BayerBG2RGB);
    //  B' = 0, 1, 0  B
    //  G' = 1, 0, 0  G
    //  R' = 0, 0, 1  R
    tbb::parallel_for(range(0, dst.rows, 256),
        [&](const range & rrange) {
          for ( int y = rrange.begin(), ny = rrange.end(); y < ny; ++y ) {

            const int src_y = y * 2;
            const T dmax = depthmax(src.depth())[0];

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
 * Extract src into 4-channel dst matrix with 4 bayer planes ordered as[ R G1 B G2 ]
 * The output size of dst is twice smaller than src
 */
bool extract_bayer_planes(cv::InputArray src, cv::OutputArray dst, enum COLORID colorid)
{
  if ( !is_bayer_pattern(colorid) ) {
    CF_ERROR("Invalid argument: colorid=%d is no at bayer pattern",
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

  tbb::parallel_for(range(0, dst.rows, 256),
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
      });


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
 * Use of gaussian blur for bayer interpolation.
 * Expected input channels are [ R  G1 B G2 ]
 */
template<class SrcType, class DstType>
static bool gbinterpolation__(cv::InputArray __src, cv::OutputArray __dst, enum COLORID colorid)
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

  // fill channel ofsets data
  struct {
    int x, y;
  } channel_offsets[4];

  switch (colorid) {
    case COLORID_BAYER_RGGB:
      // R
      channel_offsets[0].x = 0;
      channel_offsets[0].y = 0;
      // G1
      channel_offsets[1].x = 1;
      channel_offsets[1].y = 0;
      // B
      channel_offsets[2].x = 1;
      channel_offsets[2].y = 1;
      // G2
      channel_offsets[3].x = 0;
      channel_offsets[3].y = 1;
      break;

    case COLORID_BAYER_GRBG:
      // R
      channel_offsets[0].x = 1;
      channel_offsets[0].y = 0;
      // G1
      channel_offsets[1].x = 0;
      channel_offsets[1].y = 0;
      // B
      channel_offsets[2].x = 0;
      channel_offsets[2].y = 1;
      // G2
      channel_offsets[3].x = 1;
      channel_offsets[3].y = 1;
      break;

    case COLORID_BAYER_GBRG:
      // R
      channel_offsets[0].x = 0;
      channel_offsets[0].y = 1;
      // G1
      channel_offsets[1].x = 0;
      channel_offsets[1].y = 0;
      // B
      channel_offsets[2].x = 1;
      channel_offsets[2].y = 0;
      // G2
      channel_offsets[3].x = 1;
      channel_offsets[3].y = 1;
      break;

    case COLORID_BAYER_BGGR:
      // R
      channel_offsets[0].x = 1;
      channel_offsets[0].y = 1;
      // G1
      channel_offsets[1].x = 1;
      channel_offsets[1].y = 0;
      // B
      channel_offsets[2].x = 0;
      channel_offsets[2].y = 0;
      // G2
      channel_offsets[3].x = 0;
      channel_offsets[3].y = 1;
      break;
    default:
      CF_ERROR("Unsupported colorid=%d requested", colorid);
      return false;
  }


  cv::Mat_<cv::Vec<SrcType, 4>> src = __src.getMat();
  cv::Mat_<DstType> bayer_channels[4];
  cv::Mat1b bayer_masks[4];
  cv::Mat tmp;


  constexpr int src_depth = cv::DataType<SrcType>::depth;
  constexpr int dst_depth = cv::DataType<DstType>::depth;
  constexpr int kernel_depth = cv::max(CV_32F, cv::max(src_depth, dst_depth));

  // Initialize planes and channel masks
  for ( int i = 0; i < 4; ++i ) {
    bayer_channels[i].create(src.size() * 2);
    bayer_channels[i].setTo(0);
    bayer_masks[i].create(src.size() * 2);
    bayer_masks[i].setTo(255);
  }

  // Fill planes and channel masks
  typedef tbb::blocked_range<int> range;
  tbb::parallel_for(range(0, src.rows, 256),
      [&](const range & rrange) {
        for ( int y = rrange.begin(), ny = rrange.end(); y < ny; ++y ) {
          for ( int x = 0; x < src.cols; ++x ) {
            for ( int c = 0; c < 4; ++c ) {
              bayer_channels[c][2*y+channel_offsets[c].y][2*x+channel_offsets[c].x] = src[y][x][c];
              bayer_masks[c][2*y+channel_offsets[c].y][2*x+channel_offsets[c].x] = 0;
            }
          }
        }
  });

  // Intrepolate
  static const thread_local cv::Mat G = 2 * cv::getGaussianKernel(5, 0, kernel_depth);
  for ( int i = 0; i < 4; ++i ) {
    cv::sepFilter2D(bayer_channels[i], tmp, -1, G, G, cv::Point(-1, -1), 0, cv::BORDER_REFLECT101);
    tmp.copyTo(bayer_channels[i], bayer_masks[i]);
  }


  // Average greens and merge

  cv::addWeighted(bayer_channels[1], 0.5,
      bayer_channels[3], 0.5,
      0,
      bayer_channels[1]);

  cv::Mat bgr_channels[3] = {
      bayer_channels[2],  // B
      bayer_channels[1],  // G
      bayer_channels[0]   // R
      };

  cv::merge(bgr_channels, 3, __dst);

  return true;
}


template<class T>
static bool gbinterpolation_(cv::InputArray src, cv::OutputArray dst, enum COLORID colorid)
{
  const int ddepth = dst.fixedType() ? dst.depth() : src.depth();

  switch ( ddepth ) {
  case CV_8U :
    return gbinterpolation__<T, uint8_t>(src, dst, colorid);
  case CV_8S :
    return gbinterpolation__<T, int8_t>(src, dst, colorid);
  case CV_16U :
    return gbinterpolation__<T, uint16_t>(src, dst, colorid);
  case CV_16S :
    return gbinterpolation__<T, int16_t>(src, dst, colorid);
  case CV_32S :
    return gbinterpolation__<T, int32_t>(src, dst, colorid);
  case CV_32F :
    return gbinterpolation__<T, float>(src, dst, colorid);
  case CV_64F :
    return gbinterpolation__<T, double>(src, dst, colorid);
  }

  CF_ERROR("Invalid argument: Unsuppoted ddepth=%d requested", ddepth);
  return false;
}


bool gbinterpolation(cv::InputArray src, cv::OutputArray dst, enum COLORID colorid)
{
  switch ( src.depth() ) {
  case CV_8U :
    return gbinterpolation_<uint8_t>(src, dst, colorid);
  case CV_8S :
    return gbinterpolation_<int8_t>(src, dst, colorid);
  case CV_16U :
    return gbinterpolation_<uint16_t>(src, dst, colorid);
  case CV_16S :
    return gbinterpolation_<int16_t>(src, dst, colorid);
  case CV_32S :
    return gbinterpolation_<int32_t>(src, dst, colorid);
  case CV_32F :
    return gbinterpolation_<float>(src, dst, colorid);
  case CV_64F :
    return gbinterpolation_<double>(src, dst, colorid);
  }

  CF_ERROR("Invalid argument: Unsuppoted src.depth()=%d requested", src.depth());
  return false;
}

static bool gbdemosaic(cv::InputArray src, cv::OutputArray dst, enum COLORID colorid, bool remove_bad_pixels)
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

  if ( !extract_bayer_planes(src, tmp, colorid) ) {
    return false;
  }

  if ( remove_bad_pixels ) {
    removeBadPixels(tmp);
  }

  if ( !gbinterpolation(tmp, dst, colorid) ) {
    return false;
  }

  return true;
}



static bool demosaic(cv::InputArray src, cv::OutputArray dst, enum COLORID colorid, enum DEBAYER_ALGORITHM algo)
{
  if ( algo == DEBAYER_GB || algo == DEBAYER_GBNR ) {
    return gbdemosaic(src, dst, colorid, algo == DEBAYER_GBNR);
  }

  switch ( colorid ) {

  case COLORID_BAYER_RGGB :
    switch ( algo ) {
    case DEBAYER_NN :
      cv::demosaicing(src, dst, cv::COLOR_BayerBG2BGR);
      break;
    case DEBAYER_VNG :
      cv::demosaicing(src, dst, cv::COLOR_BayerBG2BGR_VNG);
      break;
    case DEBAYER_EA :
      cv::demosaicing(src, dst, cv::COLOR_BayerBG2BGR_EA);
      break;
    default :
      CF_DEBUG("Unknown  debayer algorithm=%d requested", algo);
      return false;
    }
    break;

  case COLORID_BAYER_GRBG :
    switch ( algo ) {
    case DEBAYER_NN :
      cv::demosaicing(src, dst, cv::COLOR_BayerGR2BGR);
      break;
    case DEBAYER_VNG :
      cv::demosaicing(src, dst, cv::COLOR_BayerGR2BGR_VNG);
      break;
    case DEBAYER_EA :
      cv::demosaicing(src, dst, cv::COLOR_BayerGR2BGR_EA);
      break;
    default :
      CF_DEBUG("Unknown  debayer algorithm=%d requested", algo);
      return false;
    }
    break;

  case COLORID_BAYER_GBRG :
    switch ( algo ) {
    case DEBAYER_NN :
      cv::demosaicing(src, dst, cv::COLOR_BayerGB2BGR);
      break;
    case DEBAYER_VNG :
      cv::demosaicing(src, dst, cv::COLOR_BayerGB2BGR_VNG);
      break;
    case DEBAYER_EA :
      cv::demosaicing(src, dst, cv::COLOR_BayerGB2BGR_EA);
      break;
    default :
      CF_DEBUG("Unknown  debayer algorithm=%d requested", algo);
      return false;
    }
    break;

  case COLORID_BAYER_BGGR :
    switch ( algo ) {
    case DEBAYER_NN :
      cv::demosaicing(src, dst, cv::COLOR_BayerBG2BGR);
      break;
    case DEBAYER_VNG :
      cv::demosaicing(src, dst, cv::COLOR_BayerBG2BGR_VNG);
      break;
    case DEBAYER_EA :
      cv::demosaicing(src, dst, cv::COLOR_BayerBG2BGR_EA);
      break;
    default :
      CF_DEBUG("Unknown  debayer algorithm=%d requested", algo);
      return false;
    }
    break;

  default :
    CF_DEBUG("Unknown colorid=%d requested", colorid);
    return false;
  }

  return true;
}


/** @brief Bayer demosaicing
 */
bool debayer(cv::InputArray src, cv::OutputArray dst, enum COLORID colorid, enum DEBAYER_ALGORITHM algo)
{
  if ( algo == DEBAYER_DISABLE ) {
    if ( dst.getMatRef().data != src.getMat().data ) {
      src.getMat().copyTo(dst);
    }
    return true;
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

