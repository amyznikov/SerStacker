/*
 * eccalign.cc
 *
 *  Created on: Jan 2, 2020
 *      Author: amyznikov
 */

#include "eccalign.h"
#include <tbb/tbb.h>
#include <core/debug.h>

#include <core/io/save_image.h>
#include <core/ssprintf.h>

// OpenCV version macro
#ifndef CV_VERSION_INT
# define CV_VERSION_INT(a,b,c) ((a)<<16 | (b)<<8 | (c))
#endif

#ifndef CV_VERSION_CURRRENT
# define CV_VERSION_CURRRENT CV_VERSION_INT(CV_VERSION_MAJOR, CV_VERSION_MINOR, CV_VERSION_REVISION)
#endif

// Debug logging macros

#ifndef CF_DEBUG
  #define CF_DEBUG(...) \
    fprintf(stderr, "%s() : %d ", __FUNCTION__, __LINE__), \
    fprintf(stderr, __VA_ARGS__), \
    fprintf(stderr, "\n")
#endif

#ifndef CF_ERROR
# define CF_ERROR  CF_DEBUG
#endif

#ifndef CF_FATAL
# define CF_FATAL CF_ERROR
#endif

// Prevent conflicts with MS VS min() and max() macros
#ifdef _WIN32
# ifdef min
#   undef min
# endif
# ifdef max
#  undef max
# endif
#endif



#if ( CV_VERSION_CURRRENT >= CV_VERSION_INT(3,4,0) )
namespace cv {
namespace hal {
void filter2D(int stype, int dtype, int kernel_type,
              uchar * src_data, size_t src_step,
              uchar * dst_data, size_t dst_step,
              int width, int height,
              int full_width, int full_height,
              int offset_x, int offset_y,
              uchar * kernel_data, size_t kernel_step,
              int kernel_width, int kernel_height,
              int anchor_x, int anchor_y,
              double delta, int borderType,
              bool isSubmatrix);
}}
#endif


///////////////////////////////////////////////////////////////////////////////

const struct ecc_motion_type_desc ecc_motion_types[] = {
    {"TRANSLATION", ECC_MOTION_TRANSLATION },
    {"EUCLIDEAN", ECC_MOTION_EUCLIDEAN},
    {"SCALED_EUCLIDEAN", ECC_MOTION_EUCLIDEAN_SCALED},
    {"AFFINE", ECC_MOTION_AFFINE },
    {"HOMOGRAPHY", ECC_MOTION_HOMOGRAPHY},
    {"QUADRATIC", ECC_MOTION_QUADRATIC },
    {nullptr, ECC_MOTION_NONE}
};


std::string toStdString(enum ECC_MOTION_TYPE v)
{
  for ( uint i = 0; ecc_motion_types[i].name; ++i ) {
    if ( ecc_motion_types[i].value == v ) {
      return ecc_motion_types[i].name;
    }
  }
  return "";
}

enum ECC_MOTION_TYPE fromStdString(const std::string & s, enum ECC_MOTION_TYPE defval)
{
  const char * cstr = s.c_str();

  for ( uint i = 0; ecc_motion_types[i].name; ++i ) {
    if ( strcasecmp(ecc_motion_types[i].name, cstr) == 0 ) {
      return ecc_motion_types[i].value;
    }
  }
  return defval;
}


///////////////////////////////////////////////////////////////////////////////


const extern struct ecc_interpolation_flags_desc ecc_interpolation_flags[] = {
    { "AREA", cv::INTER_AREA },
    { "LINEAR", cv::INTER_LINEAR },
    { "CUBIC", cv::INTER_CUBIC },
    { "LANCZOS4", cv::INTER_LANCZOS4 },
    { "NEAREST", cv::INTER_NEAREST },
    { nullptr, cv::INTER_NEAREST },
};

std::string toStdString(enum cv::InterpolationFlags v)
{
  for ( uint i = 0; ecc_interpolation_flags[i].name; ++i ) {
    if ( (ecc_interpolation_flags[i].value & v) == ecc_motion_types[i].value ) {
      return ecc_interpolation_flags[i].name;
    }
  }
  return "";
}

enum cv::InterpolationFlags fromStdString(const std::string  & s, enum cv::InterpolationFlags defval )
{
  const char * cstr = s.c_str();

  for ( uint i = 0; ecc_interpolation_flags[i].name; ++i ) {
    if ( strcasecmp(ecc_interpolation_flags[i].name, cstr) == 0 ) {
      return ecc_interpolation_flags[i].value;
    }
  }
  return defval;
}


///////////////////////////////////////////////////////////////////////////////

static inline double square(double x)
{
  return x * x;
}

static inline void doFilter2D(const cv::Mat & src, cv::Mat & dst, int ddepth,
    cv::InputArray kernel, const cv::Point & anchor,
    double delta = 0, int borderType = cv::BORDER_REFLECT)
{
#if ( CV_VERSION_CURRRENT < CV_VERSION_INT(3,4,0) )
  cv::filter2D(src, dst, ddepth, kernel, anchor, delta, borderType);
#else

  if ( src.data == dst.data ) { // in-place filtering is not supported by this routine
    CF_FATAL("APP BUG: Inplace doFilter2D() is not supported by this function. "
        "Fix your source code.");
    exit (1);
  }


  if ( ddepth < 0 ) {
    ddepth = src.depth();
  }

  dst.create(src.size(), CV_MAKETYPE(ddepth, src.channels()));

  const cv::Point ofs(0, 0);
  const cv::Size wsz(src.cols, src.rows);
  const cv::Mat K = kernel.getMat();

  cv::hal::filter2D(src.type(), dst.type(), K.type(),
      src.data, src.step, dst.data, dst.step,
      dst.cols, dst.rows, wsz.width, wsz.height, ofs.x, ofs.y,
      K.data, K.step, K.cols, K.rows,
      anchor.x, anchor.y,
      delta, borderType, src.isSubmatrix());

#endif
}


/*
 * Estimate the standard deviation of the noise in a gray-scale image.
 *  J. Immerkr, Fast Noise Variance Estimation,
 *    Computer Vision and Image Understanding,
 *    Vol. 64, No. 2, pp. 300-302, Sep. 1996
 *
 * Matlab code:
 *  https://www.mathworks.com/matlabcentral/fileexchange/36941-fast-noise-estimation-in-images
 */
static void create_noise_map(cv::InputArray src, cv::OutputArray dst, cv::InputArray mask)
{
  /* Compute sum of absolute values of special laplacian */

  // sqrt(M_PI_2) / 6.0
  constexpr double S = 0.20888568955258338;

  static thread_local float C[3 * 3] = {
      +1 * S, -2 * S, +1 * S,
      -2 * S, +4 * S, -2 * S,
      +1 * S, -2 * S, +1 * S
  };

  static thread_local cv::Mat1f K(3,3, C);

  cv::filter2D(src, dst,
      dst.fixedType() ? dst.type() : src.depth() == CV_64F ? CV_64F : CV_32F,
      K,
      cv::Point(-1,-1),
      0,
      cv::BORDER_REPLICATE);

  if ( !mask.empty() ) {
    cv::Mat mtmp;
    cv::dilate(~mask.getMat(), mtmp, cv::Mat1b(3, 3, 255));
    dst.setTo(0, mtmp);
  }
}


/*
 * Estimate the standard deviation of the noise in a gray-scale image.
 *  J. Immerkr, Fast Noise Variance Estimation,
 *    Computer Vision and Image Understanding,
 *    Vol. 64, No. 2, pp. 300-302, Sep. 1996
 *
 * Matlab code:
 *  https://www.mathworks.com/matlabcentral/fileexchange/36941-fast-noise-estimation-in-images
 */
static cv::Scalar estimate_noise(cv::InputArray src,
    cv::OutputArray dst  = cv::noArray(),
    cv::InputArray mask  = cv::noArray())
{
  cv::Mat H;
  cv::Scalar sigma;

  create_noise_map(src, H, cv::noArray());

  if ( dst.needed() ) {
    if ( dst.fixedType() ) {
      H.convertTo(dst, dst.depth());
    }
    else {
      H.copyTo(dst);
    }
  }

  if ( H.depth() != CV_8U && H.depth() != CV_16U ) {
    absdiff(H, 0, H);
  }

  if ( mask.empty() ) {
    // sigma = cv::sum(H) / ((H.cols - 2) * (H.rows - 2));
    sigma = cv::mean(H);
  }
  else {
    cv::Mat mtmp;

    cv::dilate(~mask.getMat(), mtmp, cv::Mat1b(3, 3, 255));
    sigma = cv::mean(H, mtmp);

    if ( dst.needed() ) {
      dst.setTo(0, mtmp);
    }
  }

  return sigma;
}




/*
 * 2x downsampling step by rejecting each UNEVEN (ODD) row and column,
 * for single-channel images
 *  REMOVE each UNEVEN (ODD) row and column,
 *    keep only even
 * */
template<class T>
static void downstrike_uneven_1c(const cv::Mat & src, cv::Mat & _dst)
{
  cv::Mat tmp;

  const cv::Size dst_size((src.cols + 1) / 2, (src.rows + 1) / 2);

  cv::Mat & dst = src.data == _dst.data ? tmp : _dst;
  dst.create(dst_size, src.type());
  dst.setTo(0);

  const int ymax = dst.rows;
  const int xmax = dst.cols;

  typedef tbb::blocked_range<int> range;
  tbb::parallel_for(range(0, ymax, 64),
      [&src, &dst, xmax] (const range & r) {
        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
          const T * srcp = src.ptr<const T>(2 * y);
          T * dstp = dst.ptr<T>(y);
          for ( int x = 0; x < xmax; ++x ) {
            dstp[x] = srcp[2 * x];
          }
        }
      });

  if ( dst.data != _dst.data ) {
    _dst = std::move(dst);
  }
}

/*
 * 2x downsampling step by rejecting each UNEVEN (ODD) row and column,
 * for multi-channel images
 *  REMOVE each UNEVEN (ODD) row and column,
 *    keep only even
 * */
static void downstrike_uneven_mc(const cv::Mat & src, cv::Mat & _dst)
{
  cv::Mat tmp;

  const cv::Size dst_size((src.cols + 1) / 2, (src.rows + 1) / 2);

  cv::Mat & dst = src.data == _dst.data ? tmp : _dst;

  dst.create(dst_size, src.type());
  dst.setTo(0);

  const int ymax = dst.rows;
  const int xmax = dst.cols;
  const size_t elem_size = src.elemSize();

  for ( int y = 0; y < ymax; ++y ) {
    const uint8_t * s = src.ptr(2 * y);
    uint8_t * d = dst.ptr(y);
    for ( int x = 0; x < xmax; ++x ) {
      memcpy(d + x * elem_size, s + 2 * x * elem_size, elem_size);
    }
  }

  if ( dst.data != _dst.data ) {
    _dst = std::move(dst);
  }
}



/*
 * 2x downsampling step by rejecting each UNEVEN (ODD) row and column,
 *    keep only even
 * */
static void downstrike_uneven(cv::InputArray _src, cv::Mat & dst)
{
  const cv::Mat src = _src.getMat();

  if ( src.channels() > 1 ) {
    downstrike_uneven_mc(src, dst);
  }
  else {
    switch ( src.type() ) {
    case CV_8U:
      return downstrike_uneven_1c<uint8_t>(src, dst);
    case CV_8S:
      return downstrike_uneven_1c<int8_t>(src, dst);
    case CV_16U:
      return downstrike_uneven_1c<uint16_t>(src, dst);
    case CV_16S:
      return downstrike_uneven_1c<int16_t>(src, dst);
    case CV_32F:
      return downstrike_uneven_1c<float>(src, dst);
    case CV_64F:
      return downstrike_uneven_1c<double>(src, dst);
    default:
      return downstrike_uneven_mc(src, dst);
    }
  }
}



/*
 * Five-point approximation to first order image derivative.
 *  <https://en.wikipedia.org/wiki/Numerical_differentiation>
 * */
void ecc_differentiate(cv::InputArray _src, cv::Mat & gx, cv::Mat & gy, int ddepth)
{
  static thread_local const cv::Matx<float, 1, 5> K(
      (+1.f / 12),
      (-8.f / 12),
        0.f,
      (+8.f / 12),
      (-1.f / 12));

  if ( ddepth < 0 ) {
    ddepth = std::max(_src.depth(), CV_32F);
  }

  const cv::Mat & src = _src.getMat();
  doFilter2D(src, gx, ddepth, K, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  doFilter2D(src, gy, ddepth, K.t(), cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
}



/*
 * Pyramid down to specific level
 */
bool ecc_downscale(cv::InputArray src, cv::Mat & dst, int level, int border_mode)
{
  cv::pyrDown(src, dst, cv::Size(), border_mode);
  for ( int l = 1; l < level; ++l ) {
    cv::pyrDown(dst, dst, cv::Size(), border_mode);
  }
  return true;
}

/*
 * Pyramid up to specific size
 */
bool ecc_upscale(cv::Mat & image, cv::Size dstSize)
{
  const cv::Size inputSize = image.size();

  if ( inputSize != dstSize ) {

    std::vector<cv::Size> spyramid;

    spyramid.emplace_back(dstSize);

    while ( 42 ) {
      const cv::Size nextSize((spyramid.back().width + 1) / 2, (spyramid.back().height + 1) / 2);
      if ( nextSize == inputSize ) {
        break;
      }
      if ( nextSize.width < inputSize.width || nextSize.height < inputSize.height ) {
        CF_DEBUG("FATAL: invalid next size : nextSize=%dx%d inputSize=%dx%d",
            nextSize.width, nextSize.height,
            inputSize.width, inputSize.height);
        return false;
      }
      spyramid.emplace_back(nextSize);
    }

    for ( int i = spyramid.size() - 1; i >= 0; --i ) {
      cv::pyrUp(image, image, spyramid[i]);
    }
  }

  return true;
}

/*
 * Image normalizxation with zero mean and unity stdev,
 * use of pyramidal scaling for efficiency
 */
bool ecc_normalize(cv::Mat1f & image, cv::InputArray _mask, int level, double regularization_term)
{
  // stdev = sqrt( mean(image ^2) - mean(image)^2 )

  cv::Mat mean, stdev;
  cv::Mat mask;

  const int pyramid_border_mode = cv::BORDER_REPLICATE;

  ecc_downscale(image, mean, level, pyramid_border_mode);
  ecc_downscale(image.mul(image), stdev, level, pyramid_border_mode);
  cv::absdiff(stdev, mean.mul(mean), stdev);
  cv::sqrt(stdev, stdev);

  ecc_upscale(mean, image.size());
  ecc_upscale(stdev, image.size());

  cv::subtract(image, mean, image);
  cv::divide(image, stdev + regularization_term, image);
  cv::normalize(image, image, 0, 1, cv::NORM_MINMAX);

  return true;
}


///////////////////////////////////////////////////////////////////////////////


/*
 * Create identity remap
 */
template<class T>
static void fillIdentityRemap_(cv::Mat & rmap)
{
  typedef tbb::blocked_range<int> range;

  cv::Mat_<cv::Vec<T, 2>> map = rmap;

  tbb::parallel_for(range(0, map.rows, 256),
      [&map](const range & r ) {
        for ( int y = r.begin(); y < r.end(); ++y ) {
          cv::Vec<T,2> * m = map[y];
          for ( int x = 0; x < map.cols; ++x ) {
            m[x][0] = x;
            m[x][1] = y;
          }
        }
      });
}


/*
 * Create translation remap
 *   x' =  x + a0
 *   y' =  y + a1
 */
template<class T1, class T2>
static bool fillTranslateRemap_(const cv::Mat_<T1> & a, cv::Mat_<cv::Vec<T2,2>> & map)
{
  typedef tbb::blocked_range<int> range;

  float tx, ty;

  if ( a.rows == 1 && a.cols == 2 ) {
    tx = a[0][0], ty = a[0][1];
  }
  else if ( a.rows == 2 && a.cols == 1 ) {
    tx = a[0][0], ty = a[1][0];
  }
  else if ( a.rows == 2 && a.cols >= 3 ) {
    tx = a[0][2];
    ty = a[1][2];
  }
  else {
    CF_ERROR("Invalid warp matrix size specified : %dx%d, can not extract translation components", a.rows, a.cols);
    return false;
  }

  tbb::parallel_for(range(0, map.rows, 256),
      [tx, ty, &map](const range & r ) {
        for ( int y = r.begin(); y < r.end(); ++y ) {
          cv::Vec<T2,2> * m = map[y];
          for ( int x = 0; x < map.cols; ++x ) {
            m[x][0] = x + tx;
            m[x][1] = y + ty;
          }
        }
      });

  return true;
}

/*
 * Create affine remap
 *   x' =  a00 * x + a01 * y + a02
 *   y' =  a10 * x + a11 * y + a12
 */
template<class T1, class T2>
static bool fillAffineRemap_(const cv::Mat_<T1> & a, cv::Mat_<cv::Vec<T2,2>> & map)
{
  typedef tbb::blocked_range<int> range;

  tbb::parallel_for(range(0, map.rows, 256),
      [&](const range & r ) {
        const float a00 = a[0][0], a01 = a[0][1], a02 = a[0][2];
        const float a10 = a[1][0], a11 = a[1][1], a12 = a[1][2];
        for ( int y = r.begin(); y < r.end(); ++y ) {
          cv::Vec<T2,2> * m = map[y];
          for ( int x = 0; x < map.cols; ++x ) {

            const float xx = x;
            const float yy = y;

            m[x][0] = a00 * xx + a01 * yy + a02;
            m[x][1] = a10 * xx + a11 * yy + a12;
          }
        }
      });

  return true;
}


/*
 * Create Quadratic remap
 *   x' =  a00 * x + a01 * y + a02 + a03 * x * y + a04 * x * x + a05 * y * y
 *   y' =  a10 * x + a11 * y + a12 + a13 * x * y + a14 * x * x + a15 * y * y
 */
template<class T1, class T2>
static bool fillQuadraticRemap_(const cv::Mat_<T1> & a, cv::Mat_<cv::Vec<T2,2>> & map)
{
  for ( int y = 0; y < map.rows; ++y ) {
    cv::Vec<T2,2> * m = map[y];
    for ( int x = 0; x < map.cols; ++x ) {
      const float xx = x;
      const float yy = y;
      m[x][0] = a[0][0] * xx + a[0][1] * yy + a[0][2] + a[0][3] * xx * yy + a[0][4] * xx * xx + a[0][5] * yy * yy;
      m[x][1] = a[1][0] * xx + a[1][1] * yy + a[1][2] + a[1][3] * xx * yy + a[1][4] * xx * xx + a[1][5] * yy * yy;
    }
  }
  return true;
}

/*
 * Create Homoghraphy remap
 *   w  =  (x * a20 + y * a21 + a22)
 *   x' =  (x * a00 + y * a01 + a02) / w
 *   y' =  (x * a10 + y * a11 + a12) / w
 */
template<class T1, class T2>
static bool fillHomoghraphyRemap_(const cv::Mat_<T1> & a, cv::Mat_<cv::Vec<T2, 2>> & map)
{
  T2 w;
  for ( int y = 0; y < map.rows; ++y ) {
    cv::Vec<T2, 2> * m = map[y];
    for ( int x = 0; x < map.cols; ++x ) {

      const float xx = x;
      const float yy = y;

      if ( !(w = xx * a[2][0] + yy * a[2][1] + a[2][2]) ) {
        w = 1.0;
      }

      m[x][0] = (a[0][0] * xx + a[0][1] * yy + a[0][2]) / w;
      m[x][1] = (a[1][0] * xx + a[1][1] * yy + a[1][2]) / w;
    }
  }
  return true;
}


/*
 * Create remap of requested type
 * */
template<class T1, class T2>
static bool fillRemap(int motionType, const cv::Mat_<T1> & a, cv::Mat_<cv::Vec<T2,2>> & map)
{
  switch ( motionType ) {
  case ECC_MOTION_TRANSLATION :
    return fillTranslateRemap_(a, map);
  case ECC_MOTION_EUCLIDEAN :
  case ECC_MOTION_EUCLIDEAN_SCALED :
    case ECC_MOTION_AFFINE :
    return fillAffineRemap_(a, map);
    break;
  case ECC_MOTION_HOMOGRAPHY :
    return fillHomoghraphyRemap_(a, map);
  case ECC_MOTION_QUADRATIC :
    return fillQuadraticRemap_(a, map);
  }
  return false;
}

/*
 * Create remap of requested type for given parameters
 * */
bool createRemap(int motionType, cv::InputArray T, cv::OutputArray map, const cv::Size & size, int ddepth)
{
  if ( (T.depth() != CV_32F && T.depth() != CV_64F) || T.channels() != 1 ) {
    CF_ERROR("Invalid argument: warp matrix must be single channel floating point");
    return false;
  }

  switch ( motionType ) {
  case ECC_MOTION_TRANSLATION :
    if ( T.size() == cv::Size(1, 2) || T.size() == cv::Size(2, 1) || (T.rows() == 2 && T.cols() >= 3) ) {
      break;
    }
    CF_ERROR("Invalid argument: warp matrix size must be 2x1, 1x2 or 2x3");
    return false;


    case ECC_MOTION_EUCLIDEAN :
    case ECC_MOTION_EUCLIDEAN_SCALED:
    case ECC_MOTION_AFFINE :
    if ( T.size() != cv::Size(3, 2) ) {
      CF_ERROR("Invalid argument: warp matrix size must be 2x3");
      return false;
    }
    break;
  case ECC_MOTION_HOMOGRAPHY :
    if ( T.size() != cv::Size(3, 3) ) {
      CF_ERROR("Invalid argument: warp matrix size must be 3x3");
      return false;
    }
    break;

  case ECC_MOTION_QUADRATIC :
    if ( T.size() != cv::Size(6, 2) ) {
      CF_ERROR("Invalid argument: warp matrix size must be 2x6");
      return false;
    }
    break;

  default :
    CF_ERROR("Invalid motion remap type %d specified", motionType);
    return false;
  }

  if ( !map.needed() ) {
    CF_ERROR("Invalid argument: m is not needed");
    return false;
  }

  if ( map.fixedType() ) {
    if ( map.channels() != 2 ) {
      CF_ERROR("Output map must be two channel floating point matrix");
      return false;
    }
    ddepth = map.depth();
  }
  else if ( ddepth < 0 ) {
    ddepth = CV_32F;
  }

  switch ( T.depth() ) {
  case CV_32F : {
    const cv::Mat_<float> a = T.getMat();
    switch ( ddepth ) {
    case CV_32F : {
      cv::Mat_<cv::Vec<float, 2>> m = (map.create(size, CV_MAKETYPE(ddepth,2)), map.getMatRef());
      return fillRemap(motionType, a, m);
    }
    case CV_64F : {
      cv::Mat_<cv::Vec<double, 2>> m = (map.create(size, CV_MAKETYPE(ddepth,2)), map.getMatRef());
      return fillRemap(motionType, a, m);
    }
    }
    break;
  }
  case CV_64F : {
    const cv::Mat_<double> a = T.getMat();
    switch ( ddepth ) {
    case CV_32F : {
      cv::Mat_<cv::Vec<float, 2>> m = (map.create(size, CV_MAKETYPE(ddepth,2)), map.getMatRef());
      return fillRemap(motionType, a, m);
    }
    case CV_64F : {
      cv::Mat_<cv::Vec<double, 2>> m = (map.create(size, CV_MAKETYPE(ddepth,2)), map.getMatRef());
      return fillRemap(motionType, a, m);
    }
    }
    break;
  }
  }

  return false;
}

cv::Mat createRemap(int motionType, cv::InputArray T, const cv::Size & size, int ddepth)
{
  cv::Mat m;
  if ( createRemap(motionType, T, m, size, ddepth) ) {
    return m;
  }
  return cv::Mat();
}

/*
 * Create identity remap of requested type
 * */
void createIdentityRemap(cv::OutputArray rmap, const cv::Size & size, int ddepth)
{
  if ( rmap.fixedType() ) {
    ddepth = rmap.type();
  }
  else if ( ddepth < 0 ) {
    ddepth = CV_32FC2;
  }

  rmap.create(size, ddepth);

  cv::Mat & matref = rmap.getMatRef();

  switch ( matref.depth() ) {
  case CV_8U :
    fillIdentityRemap_<uint8_t>(matref);
    break;
  case CV_8S :
    fillIdentityRemap_<int8_t>(matref);
    break;
  case CV_16U :
    fillIdentityRemap_<uint16_t>(matref);
    break;
  case CV_16S :
    fillIdentityRemap_<int16_t>(matref);
    break;
  case CV_32S :
    fillIdentityRemap_<int32_t>(matref);
    break;
  case CV_32F :
    fillIdentityRemap_<float>(matref);
    break;
  case CV_64F :
    fillIdentityRemap_<double>(matref);
    break;
    break;
  }
}


/*
 * Create identity matrix for given motion type
 * */
cv::Mat1f createEyeTransform(int ecc_motion_type)
{
  switch ( ecc_motion_type ) {
  case ECC_MOTION_TRANSLATION :
    return cv::Mat1f(2, 1, 0.f);
  case ECC_MOTION_EUCLIDEAN :
  case ECC_MOTION_EUCLIDEAN_SCALED:
  case ECC_MOTION_AFFINE :
    return cv::Mat1f::eye(2, 3);
  case ECC_MOTION_HOMOGRAPHY :
    return cv::Mat1f::eye(3, 3);
  case ECC_MOTION_QUADRATIC :
    return cv::Mat1f::eye(2, 6);
  default :
    break;
  }
  return cv::Mat1f();
}

/*
 * Initialize translation transform matrix of appropriate size for given motion type
 */
template<class T>
static cv::Mat createTranslationTransform_(double Tx, double Ty)
{
  cv::Mat_<T> M(2, 1);

  M[0][0] = (T) Tx;
  M[1][0] = (T) Ty;

  return M;
}

cv::Mat createTranslationTransform(double Tx, double Ty, int ddepth /*= CV_32F*/)
{
  switch ( ddepth ) {
  case CV_32F :
    return createTranslationTransform_<float>(Tx, Ty);
  case CV_64F :
    return createTranslationTransform_<double>(Tx, Ty);
  }

  CF_DEBUG("Invalid ddepth=%d specified. Only CV_32F and CV_64F supported", ddepth);
  return cv::Mat();
}

/*
*
*  x' =  scale * (cos(a) * (x-Cx) + sin(a) * (y-Cy)) + Tx
*  y' =  scale * (-sin(a) * (x-Cx) + cos(a)* (y-Cy)) + Ty
*
*  For simple rotation around of a point C call
*   createEuclideanTransform(C.x, C.y, C.x, C.y, scale, angle);
*
*/

template<class T>
static cv::Mat createEuclideanTransform_(double Cx, double Cy, double Tx, double Ty, double scale, double angle)
{
  cv::Mat_<T> M(2, 3);
  double sa, ca;

  sincos(angle, &sa, &ca);

  M[0][0] = scale * ca;
  M[0][1] = scale * sa;
  M[0][2] = Tx - scale * (ca * Cx + sa * Cy);

  M[1][0] = -scale * sa;
  M[1][1] = scale * ca;
  M[1][2] = Ty + scale * (sa * Cx - ca * Cy);

  return M;
}

cv::Mat createEuclideanTransform(double Cx, double Cy, double Tx, double Ty, double scale, double angle,
    int ddepth /*= CV_32F*/)
{
  switch ( ddepth ) {
  case CV_32F :
    return createEuclideanTransform_<float>(Cx, Cy, Tx, Ty, scale, angle);
  case CV_64F :
    return createEuclideanTransform_<double>(Cx, Cy, Tx, Ty, scale, angle);
  }

  CF_DEBUG("Invalid ddepth=%d specified. Only CV_32F and CV_64F supported", ddepth);
  return cv::Mat();
}

template<class T>
static bool getEuclideanComponents_(const cv::Mat & _M,
    double * outTx, double * outTy,
    double * outScale,
    double * outAngle)
{
  const cv::Mat_<T> M = _M;

  // check if this is equcledian transform matrix

  if ( M.rows != 2 || M.cols != 3 ) {
    CF_ERROR("Invalid matrix size for euclidean motion: must be 2x3");
    return false;
  }

  if ( abs(abs(M(0, 0)) - abs(M(1, 1))) > 1e-5 || abs(abs(M(0, 1)) - abs(M(1, 0))) > 1e-5 ) {
    CF_ERROR("Not an eucledian matrix: diagonal components not match");
    return false;
  }


  const double scale =
      0.5 * (hypot(M(0, 0), M(1, 0)) + hypot(M(0, 1), M(1, 1)));

  if ( outScale ) {
    *outScale = scale;
  }

  if ( outAngle ) {
    *outAngle = -asin(M(1, 0) / scale);
  }

  if ( outTx ) {
    *outTx = M(0, 2);
  }

  if ( outTy ) {
    *outTy = M(1, 2);
  }

  return true;
}

// Extract Euclidean components from (scaled) Euclidean transfom matrix M of size 2x3
bool getEuclideanComponents(cv::InputArray M,
    double * outTx, double * outTy,
    double * outScale,
    double * outAngle)
{
  switch ( M.type() ) {
  case CV_32FC1 :
    return getEuclideanComponents_<float>(M.getMat(),
        outTx, outTy,
        outScale,
        outAngle);
  case CV_64FC1 :
    return getEuclideanComponents_<double>(M.getMat(),
        outTx, outTy,
        outScale,
        outAngle);
  }

  CF_ERROR("Invalid matrix type=%d specified: must be CV_32FC1 or CV_64FC1", M.type());
  return false;
}

/*
 * Scale remap to account image size change
 */
void scaleTransform(int motion_type, cv::Mat1f & T, double scale)
{
  switch ( motion_type ) {
  case ECC_MOTION_TRANSLATION :
    if ( T.rows == 1 && T.cols == 2 ) {
      T[0][0] *= scale;
      T[0][1] *= scale;
    }
    else if ( T.rows == 2 && T.cols == 1 ) {
      T[0][0] *= scale;
      T[1][0] *= scale;
    }
    else if ( T.rows == 2 && T.cols >= 3 ) {
      T[0][2] *= scale;
      T[1][2] *= scale;
    }
    else {
      CF_ERROR("Invalid warp matrix size specified : %dx%d, can not extract translation components",
          T.rows, T.cols);
    }
    break;

  case ECC_MOTION_EUCLIDEAN :
    case ECC_MOTION_EUCLIDEAN_SCALED :
    case ECC_MOTION_AFFINE :
    T[0][2] *= scale;
    T[1][2] *= scale;
    break;

  case ECC_MOTION_QUADRATIC :
    T[0][2] *= scale;
    T[0][3] /= scale;
    T[0][4] /= scale;
    T[0][5] /= scale;
    T[1][2] *= scale;
    T[1][3] /= scale;
    T[1][4] /= scale;
    T[1][5] /= scale;
    break;

  case ECC_MOTION_HOMOGRAPHY :
    T[0][2] *= scale;
    T[1][2] *= scale;
    T[2][0] /= scale;
    T[2][1] /= scale;
    break;

  default :
    break;
  }
}

/*
 * Scale remap to account image size change
 */
void scaleTransform(int motion_type, const cv::Mat1f & src, cv::Mat1f & dst, double scale)
{
  if ( src.data != dst.data ) {
    src.copyTo(dst);
  }
  scaleTransform(motion_type, dst, scale);
}


/*
 * Convert the matrix of affine transform into other compatibe motion type,
 * filling new added elements as appropriate
 */
cv::Mat1f expandAffineTransform(const cv::Mat1f T, int target_motion_type)
{
  cv::Mat1f TT;

  if ( (T.rows == 1 && T.cols == 2) ) { // T is translation matrix

    switch ( target_motion_type ) {
    case ECC_MOTION_TRANSLATION :
      TT.create(2, 1);
      TT[0][0] = T[0][0];
      TT[1][0] = T[0][1];
      break;

    case ECC_MOTION_AFFINE :
    case ECC_MOTION_EUCLIDEAN :
    case ECC_MOTION_EUCLIDEAN_SCALED:
      TT = cv::Mat1f::eye(2, 3);
      TT[0][2] = T[0][0];
      TT[1][2] = T[0][1];
      break;

    case ECC_MOTION_QUADRATIC :
      TT = cv::Mat1f::eye(2, 6);
      TT[0][2] = T[0][0];
      TT[1][2] = T[0][1];
      break;

    case ECC_MOTION_HOMOGRAPHY :
      TT = cv::Mat1f::eye(3, 3);
      TT[0][2] = T[0][0];
      TT[1][2] = T[0][1];
      break;

    default:
      CF_FATAL("App BUG: invalid target_motion_type=%d", target_motion_type);
      return TT;
    }

  }
  else if ( T.rows == 2 && T.cols == 1 ) {


    switch ( target_motion_type ) {
    case ECC_MOTION_TRANSLATION :
      TT.create(2, 1);
      TT[0][0] = T[0][0];
      TT[1][0] = T[1][0];
      break;

    case ECC_MOTION_AFFINE :
    case ECC_MOTION_EUCLIDEAN :
    case ECC_MOTION_EUCLIDEAN_SCALED:
      TT = cv::Mat1f::eye(2, 3);
      TT[0][2] = T[0][0];
      TT[1][2] = T[1][0];
      break;

    case ECC_MOTION_QUADRATIC :
      TT = cv::Mat1f::eye(2, 6);
      TT[0][2] = T[0][0];
      TT[1][2] = T[1][0];
      break;

    case ECC_MOTION_HOMOGRAPHY :
      TT = cv::Mat1f::eye(3, 3);
      TT[0][2] = T[0][0];
      TT[1][2] = T[1][0];
      break;

    default:
      CF_FATAL("App BUG: invalid target_motion_type=%d", target_motion_type);
      return TT;
    }
  }

  else if ( T.rows == 2 && T.cols == 3 ) { // T is affine matrix

    switch ( target_motion_type ) {

    case ECC_MOTION_TRANSLATION :
      TT.create(2, 1);
      TT[0][0] = T[0][2];
      TT[1][0] = T[1][2];
      break;


    case ECC_MOTION_AFFINE :
    case ECC_MOTION_EUCLIDEAN :
    case ECC_MOTION_EUCLIDEAN_SCALED:
      T.copyTo(TT);
      break;

    case ECC_MOTION_QUADRATIC :
      TT = cv::Mat1f::eye(2, 6);
      for ( int i = 0; i < 2; ++i ) {
        for ( int j = 0; j < 3; ++j ) {
          TT[i][j] = T[i][j];
        }
      }
      break;

    case ECC_MOTION_HOMOGRAPHY :
      TT = cv::Mat1f::eye(3, 3);
      for ( int i = 0; i < 2; ++i ) {
        for ( int j = 0; j < 3; ++j ) {
          TT[i][j] = T[i][j];
        }
      }
      break;

    default:
      CF_FATAL("App BUG: invalid target_motion_type=%d", target_motion_type);
      return TT;
    }

  }

  return TT;
}






// Extract translation component from current transform
bool getTranslationComponents(int motion_type, const cv::Mat1f & T,
    double * tx, double * ty)
{
  switch ( motion_type ) {
  case ECC_MOTION_TRANSLATION :
    if ( T.rows == 1 && T.cols == 2 ) {
      *tx = T[0][0], *ty = T[0][1];
    }
    else if ( T.rows == 2 && T.cols == 1 ) {
      *tx = T[0][0], *ty = T[1][0];
    }
    else {
      CF_ERROR("Invalid warp matrix size specified : %dx%d, can not extract translation components", T.rows, T.cols);
      *tx = *ty = 0;
      return false;
    }
    break;

  case ECC_MOTION_EUCLIDEAN :
  case ECC_MOTION_EUCLIDEAN_SCALED:
  case ECC_MOTION_AFFINE :
  case ECC_MOTION_QUADRATIC :
    if ( T.rows == 2 && T.cols >= 3 ) {
      *tx = T[0][2];
      *ty = T[1][2];
    }
    else {
      CF_ERROR("Invalid warp matrix size specified : %dx%d, can not extract translation components", T.rows, T.cols);
      *tx = *ty = 0;
      return false;
    }
    break;

  case ECC_MOTION_HOMOGRAPHY :
    if ( T.rows == 3 && T.cols == 3 ) {
      // FIXME: check if this is correct for ECC_MOTION_HOMOGRAPHY
      *tx = T[0][2];
      *ty = T[1][2];
    }
    else {
      CF_ERROR("Invalid warp matrix size specified : %dx%d, can not extract translation components", T.rows, T.cols);
      *tx = *ty = 0;
      return false;
    }
    break;

  default :
    CF_ERROR("Invalid ecc_motion_type=%d", motion_type);
    *tx = *ty = 0;
    return false;
  }

  return true;
}



/////////////////////////////////////////////////////////////////////////////////


static int init_warp_matrix(int motion_type, cv::InputOutputArray warpMatrix)
{
  int numberOfParameters = -1;

 if ( warpMatrix.empty() ) {
    createEyeTransform(motion_type).copyTo(warpMatrix);
  }
  else if ( warpMatrix.type() != CV_32FC1 && warpMatrix.type() != CV_64FC1 ) {
    CF_FATAL("Warp matrix must be single-channel floating-point matrix");
    return -1;
  }

  if ( warpMatrix.channels() != 1 ) {
    CF_FATAL("Warp matrix must be single-channel matrix");
    return -1;
  }

  switch ( motion_type ) {
  case ECC_MOTION_TRANSLATION : {
    const int r =  warpMatrix.rows();
    const int c =  warpMatrix.cols();

    if ( !(r == 1 && c == 2) && !(r == 2 && c == 1) && !(r == 2 && c == 3) ) {
      CF_FATAL("Warp matrix must be single-channel floating-point 2x1, 1x2 or 2x3 matrix");
      return -1;
    }

    numberOfParameters = 2;
  }
    break;

  case ECC_MOTION_EUCLIDEAN :
    if ( warpMatrix.size() != cv::Size(3, 2) ) {
      CF_FATAL("Warp matrix must be single-channel floating-point 2x3 matrix");
      return -1;
    }
    numberOfParameters = 3;
    break;

  case ECC_MOTION_EUCLIDEAN_SCALED:
    if ( warpMatrix.size() != cv::Size(3, 2) ) {
      CF_FATAL("Warp matrix must be single-channel floating-point 2x3 matrix");
      return -1;
    }
    numberOfParameters = 4;
    break;

  case ECC_MOTION_AFFINE :
    if ( warpMatrix.size() != cv::Size(3, 2) ) {
      CF_FATAL("Warp matrix must be single-channel floating-point 2x3 matrix");
      return -1;
    }
    numberOfParameters = 6;
    break;

  case ECC_MOTION_QUADRATIC :
    if ( warpMatrix.size() != cv::Size(6, 2) ) {
      CF_FATAL("Warp Matrix must be single-channel floating-point 2x6 matrix");
      return -1;
    }
    numberOfParameters = 12;
    break;

  case ECC_MOTION_HOMOGRAPHY :
    if ( warpMatrix.size() != cv::Size(3, 3) ) {
      CF_FATAL("Warp matrix must be single-channel floating-point 3x3 matrix");
      return -1;
    }
    numberOfParameters = 8;
    break;

  default :
    CF_FATAL("Unsupported motion type %d", motion_type);
    return -1;
  }

  return numberOfParameters;
}

static bool compute_jacobian(int motion_type, const cv::Mat1f & gx, const cv::Mat1f & gy, const cv::Mat1f & W,
    cv::Mat1f & jac)
{
  const int w = gx.cols;
  const int h = gx.rows;

  typedef tbb::blocked_range<int> range;
  constexpr int block_size = 256;

  switch ( motion_type ) {
  case ECC_MOTION_TRANSLATION : {
    // x' =  x + a0
    // y' =  y + a1
    jac.create(h * 2, w);
    tbb::parallel_for(range(0, h, block_size),
        [w, h, &gx, &gy, &jac](const range & r) {
          for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
            for ( int x = 0; x < w; ++x ) {
              jac[y + 0 * h][x] = gx[y][x];
              jac[y + 1 * h][x] = gy[y][x];
            }
          }
        });


    break;
  }

  case ECC_MOTION_EUCLIDEAN : {
    // For the signs of angles see opencv doc
    //  <https://docs.opencv.org/master/dd/d52/tutorial_js_geometric_transformations.html>
    // x' =  cos(theta) * x - sin(theta) * y + a02
    // y' =  sin(theta) * x + cos(theta) * y + a12

    const float ca = W(0, 0);  //  cos(theta)
    const float sa = -W(1, 0);  // sin(theta)

    // jac (theta):
    // -sin(theta)*X - cos(theta)*Y
    //  cos(theta)*X - sin(theta)*Y

    jac.create(h * 3, w);
    tbb::parallel_for(range(0, h, block_size),
        [ca, sa, w, h, &gx, &gy, &jac](const range & r) {
          for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
            for ( int x = 0; x < w; ++x ) {
              jac[y + 0 * h][x] = gx[y][x] * (sa * x - ca * y) + gy[y][x]* (ca * x + sa * y);
              jac[y + 1 * h][x] = gx[y][x];
              jac[y + 2 * h][x] = gy[y][x];
            }
          }
        });
    break;
  }


  case ECC_MOTION_EUCLIDEAN_SCALED : {
    // For the signs of angles see opencv doc
    //  <https://docs.opencv.org/master/dd/d52/tutorial_js_geometric_transformations.html>
    // x' =  scale * cos(theta) * x - scale * sin(theta) * y + a02
    // y' =  scale * sin(theta) * x + scale * cos(theta) * y + a12

    const double scale = hypot(W(0, 0), W(1, 0));
    const double ca = W(0, 0) / scale;  // cos(theta)
    const double sa = W(1, 0) / scale;  // sin(theta)

    // jac (theta):
    // -sin(theta)*X - cos(theta)*Y
    //  cos(theta)*X - sin(theta)*Y

    jac.create(h * 4, w);

    tbb::parallel_for(range(0, h, block_size),
        [ca, sa, scale, w, h, &gx, &gy, &jac](const range & r) {
          for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
            for ( int x = 0; x < w; ++x ) {
              jac[y + 0 * h][x] = (-gx[y][x] * (sa * x + ca * y) + gy[y][x]* (ca * x - sa * y)) * scale;
              jac[y + 1 * h][x] = gx[y][x];
              jac[y + 2 * h][x] = gy[y][x];
              jac[y + 3 * h][x] = gx[y][x] * (ca * x - sa * y) + gy[y][x] * (sa * x + ca * y);
            }
          }
        });
    break;
  }


  case ECC_MOTION_AFFINE : {
    // x' =  a00 * x + a01 * y + a02
    // y' =  a10 * x + a11 * y + a12

    jac.create(h * 6, w);
    tbb::parallel_for(range(0, h, block_size),
        [w, h, &gx, &gy, &jac](const range & r) {
          for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
            for ( int x = 0; x < w; ++x ) {
              jac[y + 0 * h][x] = gx[y][x] * x;
              jac[y + 1 * h][x] = gy[y][x] * x;
              jac[y + 2 * h][x] = gx[y][x] * y;
              jac[y + 3 * h][x] = gy[y][x] * y;
              jac[y + 4 * h][x] = gx[y][x];
              jac[y + 5 * h][x] = gy[y][x];
            }
          }
        });
    break;
  }

  case ECC_MOTION_HOMOGRAPHY : {
    // w  =  (x * a20 + y * a21 + a22)
    // x' =  (x * a00 + y * a01 + a02) / w
    // y' =  (x * a10 + y * a11 + a12) / w

    jac.create(h * 8, w);
    tbb::parallel_for(range(0, h, block_size),
        [w, h, &W, &gx, &gy, &jac](const range & r) {
          for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
            for ( int x = 0; x < w; ++x ) {
              //  create projected points
              const float den = 1.f / (x * W[2][0] + y * W[2][1] + 1.f );
              const float hatX = -(x * W[0][0] + y * W[0][1] + W[0][2]) * den;
              const float hatY = -(x * W[1][0] + y * W[1][1] + W[1][2]) * den;

              const float ggx = gx[y][x] * den;
              const float ggy = gy[y][x] * den;
              const float gg = hatX * ggx + hatY * ggy;

              jac[y + 0 * h ][x] = ggx * x;
              jac[y + 1 * h ][x] = ggy * x;
              jac[y + 2 * h ][x] = gg * x;
              jac[y + 3 * h ][x] = ggx * y;
              jac[y + 4 * h ][x] = ggy * y;
              jac[y + 5 * h ][x] = gg * y;
              jac[y + 6 * h ][x] = ggx;
              jac[y + 7 * h ][x] = ggy;
            }
          }
        });
    break;
  }

  case ECC_MOTION_QUADRATIC : {
    // x' =  a00 * x + a01 * y + a02 + a03 * x * y + a04 * x * x + a05 * y * y
    // y' =  a10 * x + a11 * y + a12 + a13 * x * y + a14 * x * x + a15 * y * y
    jac.create(h * 12, w);
    tbb::parallel_for(range(0, h, block_size),
        [w, h, &gx, &gy, &jac](const range & r) {
          for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
            for ( int x = 0; x < w; ++x ) {
              jac[y + 0 * h][x] = gx[y][x] * x;
              jac[y + 1 * h][x] = gy[y][x] * x;
              jac[y + 2 * h][x] = gx[y][x] * y;
              jac[y + 3 * h][x] = gy[y][x] * y;
              jac[y + 4 * h][x] = gx[y][x];
              jac[y + 5 * h][x] = gy[y][x];
              jac[y + 6 * h][x] = gx[y][x] * x * y;
              jac[y + 7 * h][x] = gy[y][x] * x * y;
              jac[y + 8 * h][x] = gx[y][x] * x * x;
              jac[y + 9 * h][x] = gy[y][x] * x * x;
              jac[y + 10 * h][x] = gx[y][x] * y * y;
              jac[y + 11 * h][x] = gy[y][x] * y * y;
            }
          }
        });
    break;
  }

  default : {
    return false;
  }
  }

  return true;

}

static bool update_warp_matrix_forward_additive(int motion_type, cv::Mat1f & W, const cv::Mat1f & deltaP, double * e, double xmax, double ymax)
{
  switch ( motion_type ) {
  case ECC_MOTION_TRANSLATION :
    if ( W.rows == 1 && W.cols == 2 ) {
      W(0, 0) += deltaP(0, 0);
      W(0, 1) += deltaP(1, 0);
    }
    else if ( W.rows == 2 && W.cols == 1 ) {
      W(0, 0) += deltaP(0, 0);
      W(1, 0) += deltaP(1, 0);
    }
    else {
      W(0, 2) += deltaP(0, 0);
      W(1, 2) += deltaP(1, 0);
    }
    if ( e ) {
      *e = sqrt(square(deltaP(0, 0)) + square(deltaP(1, 0)));
    }
    break;

  case ECC_MOTION_EUCLIDEAN : {
    // For the signs of angles see opencv doc
    //  <https://docs.opencv.org/master/dd/d52/tutorial_js_geometric_transformations.html>
    // x' =  cos(theta) * x - sin(theta) * y + a02
    // y' =  sin(theta) * x + cos(theta) * y + a12

    // 0 1 2
    // 3 4 5

    double sa, ca;
    sincos(deltaP(0, 0) + asin(W(1, 0)), &sa, &ca);

    W(0, 0) = ca;
    W(0, 1) = -sa;
    W(0, 2) += deltaP(1, 0);

    W(1, 0) = sa;
    W(1, 1) = ca;
    W(1, 2) += deltaP(2, 0);

    if ( e ) {
      const double a = sin(deltaP(0, 0));
      *e = sqrt(square(deltaP(1, 0)) + square(deltaP(2, 0)) +
          square(xmax * a) + square(ymax * a));
    }

    break;
  }

  case ECC_MOTION_EUCLIDEAN_SCALED : {
    // For the signs of angles see opencv doc
    //  <https://docs.opencv.org/master/dd/d52/tutorial_js_geometric_transformations.html>
    // x' =  cos(theta) * x - sin(theta) * y + a02
    // y' =  sin(theta) * x + cos(theta) * y + a12

    // 0 1 2
    // 3 4 5

    double sa, ca, scale;

    scale = hypot(W(0, 0), W(1, 0));

    sincos(deltaP(0, 0) + asin(W(1, 0) / scale), &sa, &ca);
    scale += deltaP(3, 0);

    W(0, 0) = scale * ca;
    W(0, 1) = -scale * sa;
    W(0, 2) += deltaP(1, 0);

    W(1, 0) = scale * sa;
    W(1, 1) = scale * ca;
    W(1, 2) += deltaP(2, 0);

    if ( e ) {
      const double a = sin(deltaP(0, 0));
      *e = sqrt(square(deltaP(1, 0)) + square(deltaP(2, 0)) +
          square(xmax * a) + square(ymax * a));
    }

    break;
  }



  case ECC_MOTION_AFFINE : {

    W(0, 0) += deltaP(0, 0);
    W(0, 1) += deltaP(2, 0);
    W(0, 2) += deltaP(4, 0);
    W(1, 0) += deltaP(1, 0);
    W(1, 1) += deltaP(3, 0);
    W(1, 2) += deltaP(5, 0);

    if ( e ) {
      *e = sqrt(square(deltaP(4, 0)) + square(deltaP(5, 0)) +
          square(xmax * deltaP(0, 0)) + square(xmax * deltaP(1, 0)) +
          square(ymax * deltaP(2, 0)) + square(ymax * deltaP(3, 0)));
    }

    break;
  }

  case ECC_MOTION_HOMOGRAPHY : {
    // w  =  (x * a20 + y * a21 + a22)
    // x' =  (x * a00 + y * a01 + a02) / w
    // y' =  (x * a10 + y * a11 + a12) / w

    W(0, 0) += deltaP(0, 0);
    W(0, 1) += deltaP(3, 0);
    W(0, 2) += deltaP(6, 0);
    W(1, 0) += deltaP(1, 0);
    W(1, 1) += deltaP(4, 0);
    W(1, 2) += deltaP(7, 0);
    W(2, 0) += deltaP(2, 0);
    W(2, 1) += deltaP(5, 0);

    if ( e ) {
      // fixme: this estimate does not account for w
      *e = sqrt(square(deltaP(6, 0)) + square(deltaP(7, 0)) +
          square(xmax * deltaP(0, 0)) + square(ymax * deltaP(3, 0)) +
          square(xmax * deltaP(1, 0)) + square(ymax * deltaP(4, 0)));
    }

    break;
  }
  case ECC_MOTION_QUADRATIC : {
    // x' =  a00 * x + a01 * y + a02 + a03 * x * y + a04 * x * x + a05 * y * y
    // y' =  a10 * x + a11 * y + a12 + a13 * x * y + a14 * x * x + a15 * y * y

    W(0, 0) += deltaP(0, 0);
    W(0, 1) += deltaP(2, 0);
    W(0, 2) += deltaP(4, 0);
    W(0, 3) += deltaP(6, 0);
    W(0, 4) += deltaP(8, 0);
    W(0, 5) += deltaP(10, 0);
    W(1, 0) += deltaP(1, 0);
    W(1, 1) += deltaP(3, 0);
    W(1, 2) += deltaP(5, 0);
    W(1, 3) += deltaP(7, 0);
    W(1, 4) += deltaP(9, 0);
    W(1, 5) += deltaP(11, 0);

    if ( e ) {
      *e = sqrt(square(deltaP(4, 0)) + square(deltaP(5, 0)) +
          square(xmax * deltaP(0, 0)) + square(ymax * deltaP(2, 0)) +
          square(xmax * deltaP(1, 0)) + square(ymax * deltaP(3, 0)) +
          square(xmax * ymax * deltaP(6, 0)) + square(xmax * ymax * deltaP(7, 0)) +
          square(xmax * xmax * deltaP(8, 0)) + square(xmax * xmax * deltaP(9, 0)) +
          square(ymax * ymax * deltaP(10, 0)) + square(ymax * ymax * deltaP(11, 0)));
    }

    break;
  }

  default : {
    return false;
  }
  }

  return true;
}



static bool update_warp_matrix_inverse_composite(int motion_type, cv::Mat1f & W, const cv::Mat1f & deltaP, double * e, double xmax, double ymax)
{
  switch ( motion_type ) {
  case ECC_MOTION_TRANSLATION :
    if ( W.rows == 1 && W.cols == 2 ) {
      W(0, 0) -= deltaP(0, 0);
      W(0, 1) -= deltaP(1, 0);
    }
    else if ( W.rows == 2 && W.cols == 1 ) {
      W(0, 0) -= deltaP(0, 0);
      W(1, 0) -= deltaP(1, 0);
    }
    else {
      W(0, 2) -= deltaP(0, 0);
      W(1, 2) -= deltaP(1, 0);
    }
    if ( e ) {
      *e = sqrt(deltaP(0, 0) * deltaP(0, 0) + deltaP(1, 0) * deltaP(1, 0));
    }
    return true;

  case ECC_MOTION_EUCLIDEAN : {
    // https://docs.opencv.org/master/dd/d52/tutorial_js_geometric_transformations.html
    // x' =  cos(theta) * x - sin(theta) * y + a02
    // y' =  sin(theta) * x + cos(theta) * y + a12

    // 0 1 2
    // 3 4 5

    double sa, ca;
    bool isok = false;

    sincos(deltaP(0, 0), &sa, &ca);


    cv::Matx33f P(
        W(0, 0), W(0, 1), W(0, 2),
        W(1, 0), W(1, 1), W(1, 2),
        0,        0,        1   );

    cv::Matx33f dP(
        ca, -sa, deltaP(1, 0),
        sa, ca, deltaP(2, 0),
        0,   0,     1     );


    dP = dP.inv(cv::DECOMP_LU, &isok);
    if ( !isok ) {
      CF_DEBUG("dP.inv() fails");
    }
    else {
      P = P * dP;

      sincos(P(1, 0), &sa, &ca);

      W(0,0) = ca;
      W(0,1) = -sa;
      W(0,2) = P(0,2);

      W(1,0) = sa;
      W(1,1) = ca;
      W(1,2) = P(1,2);

      if ( e ) {
        const double a = sin(deltaP(0, 0));
        *e = sqrt(square(deltaP(1, 0)) + square(deltaP(2, 0)) +
            square(xmax * a) + square(ymax * a));
      }
    }

    return isok;
  }


  case ECC_MOTION_EUCLIDEAN_SCALED : {
    // https://docs.opencv.org/master/dd/d52/tutorial_js_geometric_transformations.html
    // x' =  cos(theta) * x - sin(theta) * y + a02
    // y' =  sin(theta) * x + cos(theta) * y + a12

    // 0 1 2
    // 3 4 5

    double sa, ca, scale;
    bool isok = false;

    sincos(deltaP(0, 0), &sa, &ca);

    cv::Matx33f P(
        W(0, 0), W(0, 1), W(0, 2),
        W(1, 0), W(1, 1), W(1, 2),
        0,        0,        1   );

    cv::Matx33f dP(
        ca, -sa, deltaP(1, 0),
        sa, ca, deltaP(2, 0),
        0,   0,     1     );


    dP = dP.inv(cv::DECOMP_LU, &isok);
    if ( !isok ) {
      CF_DEBUG("dP.inv() fails");
    }
    else {
      P = P * dP;

      scale = hypot(P(0, 0), P(1, 0));
      sincos(P(1, 0) / scale, &sa, &ca);

      // FIXME: Check if this is corect
      scale = hypot(W(0, 0), W(1, 0)) - deltaP(3, 0);

      W(0,0) = scale * ca;
      W(0,1) = -scale * sa;
      W(0,2) = P(0,2);

      W(1,0) = scale * sa;
      W(1,1) = scale * ca;
      W(1,2) = P(1,2);

      if ( e ) {
        const double a = sin(deltaP(0, 0));
        *e = sqrt(square(deltaP(1, 0)) + square(deltaP(2, 0)) +
            square(xmax * a) + square(ymax * a));
      }
    }

    return isok;
  }

  case ECC_MOTION_AFFINE : {

    //  W(0, 0) => deltaP(0, 0);
    //  W(0, 1) => deltaP(2, 0);
    //  W(0, 2) => deltaP(4, 0);
    //  W(1, 0) => deltaP(1, 0);
    //  W(1, 1) => deltaP(3, 0);
    //  W(1, 2) => deltaP(5, 0);

    cv::Matx33f P(
        W(0,0), W(0,1), W(0,2),
        W(1,0), W(1,1), W(1,2),
          0,     0,      1);

    cv::Matx33f dP(
        1 + deltaP(0, 0), deltaP(2, 0), deltaP(4, 0),
        deltaP(1, 0), 1 + deltaP(3, 0), deltaP(5, 0),
        0, 0, 1);

    bool isok = false;

    dP = dP.inv(cv::DECOMP_LU, &isok);
    if ( !isok ) {
      CF_DEBUG("dP.inv() fails");
    }
    else {

      P = P * dP;

      W(0,0) = P(0,0);
      W(0,1) = P(0,1);
      W(0,2) = P(0,2);

      W(1,0) = P(1,0);
      W(1,1) = P(1,1);
      W(1,2) = P(1,2);


      if ( e ) {
        *e = sqrt(square(deltaP(4, 0)) + square(deltaP(5, 0)) +
            square(xmax * deltaP(0, 0)) + square(xmax * deltaP(1, 0)) +
            square(ymax * deltaP(2, 0)) + square(ymax * deltaP(3, 0)));
      }
    }

    return isok;
  }

  case ECC_MOTION_HOMOGRAPHY : {
    //  x' =  (x * a00 + y * a01 + a02) / w
    //  y' =  (x * a10 + y * a11 + a12) / w
    //  w  =  (x * a20 + y * a21 + a22)

    //  W(0, 0) => deltaP(0, 0);
    //  W(0, 1) => deltaP(3, 0);
    //  W(0, 2) => deltaP(6, 0);
    //  W(1, 0) => deltaP(1, 0);
    //  W(1, 1) => deltaP(4, 0);
    //  W(1, 2) => deltaP(7, 0);
    //  W(2, 0) => deltaP(2, 0);
    //  W(2, 1) => deltaP(5, 0);
    //  W(2, 2) => 1;


    cv::Matx33f P(
        W(0,0), W(0,1), W(0,2),
        W(1,0), W(1,1), W(1,2),
        W(2,0), W(2,1),  1.0);

    cv::Matx33f dP(
        1 + deltaP(0, 0),   deltaP(3, 0),   deltaP(6, 0),
        deltaP(1, 0),     1 + deltaP(4, 0), deltaP(7, 0),
        deltaP(2, 0),      deltaP(5, 0),       1.);

    bool isok = false;

    dP = dP.inv(cv::DECOMP_LU, &isok);
    if ( !isok ) {
      CF_DEBUG("dP.inv() fails");
    }
    else {

      P = P * dP;

      W(0, 0) = P(0, 0);
      W(0, 1) = P(0, 1);
      W(0, 2) = P(0, 2);

      W(1, 0) = P(1, 0);
      W(1, 1) = P(1, 1);
      W(1, 2) = P(1, 2);

      W(2, 0) = P(2, 0);
      W(2, 1) = P(2, 1);
      W(2, 2) = 1;

      if ( e ) {
        *e = sqrt(square(deltaP(6, 0)) + square(deltaP(7, 0)) +
            square(xmax * deltaP(0, 0)) + square(ymax * deltaP(3, 0)) +
            square(xmax * deltaP(1, 0)) + square(ymax * deltaP(4, 0)));
      }
    }

    return isok;
  }
  case ECC_MOTION_QUADRATIC : {
    return false;
  }

  default : {
    return false;
  }
  }

  return true;
}



/* this functions is used for two types of projections. If src1.cols ==src.cols
 it does a blockwise multiplication (like in the outer product of vectors)
 of the blocks in matrices src1 and src2 and dst
 has size (number_of_blcks x number_of_blocks), otherwise dst is a vector of size
 (number_of_blocks x 1) since src2 is "multiplied"(dot) with each block of src1.

 The number_of_blocks is equal to the number of parameters we are lloking for
 (i.e. rtanslation:2, euclidean: 3, affine: 6, homography: 8)
 */
static void project_onto_jacobian(const cv::Mat1f & jac, const cv::Mat & src, cv::Mat1f & dst, int numberOfParameters)
{
  if ( src.data == dst.data ) {
    CF_FATAL("APP BUG: SRC and DST must be different objects, inplace not supported");
    exit(1);
  }

  if ( jac.rows != src.rows ) {

    // Projection to jacobian is requested

    dst.create(numberOfParameters, 1);

    const int h = jac.rows / numberOfParameters;
    for ( int i = 0; i < numberOfParameters; ++i ) {
      dst[i][0] = src.dot(jac.rowRange(i * h, (i + 1) * h));
    }
  }
  else {

    // Hessian matrix requested

    dst.create(numberOfParameters, numberOfParameters);

    const int h = jac.rows / numberOfParameters;

    for ( int i = 0; i < numberOfParameters; i++ ) {
      const cv::Mat m(jac.rowRange(i * h, (i + 1) * h));
      const float nn = cv::norm(m);
      dst[i][i] = nn * nn;  //diagonal elements
      for ( int j = i + 1; j < numberOfParameters; j++ ) {
        dst[i][j] = m.dot(src.rowRange(j * h, (j + 1) * h));
        dst[j][i] = dst[i][j];
      }
    }
  }
}




///////////////////////////////////////////////////////////////////////////////

c_ecc_align::~c_ecc_align()
{
}

void c_ecc_align::set_motion_type(int v)
{
  this->ecc_motion_type_ = v;
}

int c_ecc_align::motion_type() const
{
  return this->ecc_motion_type_;
}

void c_ecc_align::set_max_iterations(int v)
{
  this->max_iterations_ = v;
}

int c_ecc_align::max_iterations() const
{
  return this->max_iterations_;
}

void c_ecc_align::set_eps(double v)
{
  this->eps_ = v;
}

double c_ecc_align::eps() const
{
  return this->eps_;
}

void c_ecc_align::set_min_rho(double v)
{
  min_rho_ = v;
}

double c_ecc_align::min_rho() const
{
  return min_rho_;
}


void c_ecc_align::set_image_interpolation_flags(int v)
{
  image_interpolation_flags_ = v;
}

int c_ecc_align::image_interpolation_flags() const
{
  return image_interpolation_flags_;
}

void c_ecc_align::set_input_smooth_sigma(double v)
{
  input_smooth_sigma_ = v;
}

double c_ecc_align::input_smooth_sigma() const
{
  return input_smooth_sigma_;
}

void c_ecc_align::set_reference_smooth_sigma(double v)
{
  reference_smooth_sigma_ = v;
}

double c_ecc_align::reference_smooth_sigma() const
{
  return reference_smooth_sigma_;
}

void c_ecc_align::set_update_step_scale(double v)
{
  update_step_scale_ = v;
}

double c_ecc_align::update_step_scale() const
{
  return update_step_scale_;
}


//void c_ecc_align::set_pyramid_normalization_level(int v)
//{
//  pyramid_normalization_level_ = v;
//}
//
//int c_ecc_align::pyramid_normalization_level() const
//{
//  return pyramid_normalization_level_;
//}
//
//void c_ecc_align::set_pyramid_normalization_regularization_term(double v)
//{
//  pyramid_normalization_regularization_term_ = v;
//}
//
//double c_ecc_align::pyramid_normalization_regularization_term() const
//{
//  return pyramid_normalization_regularization_term_;
//}

double c_ecc_align::rho() const
{
  return this->rho_;
}

int c_ecc_align::num_iterations() const
{
  return this->num_iterations_;
}

double c_ecc_align::current_eps() const
{
  return current_eps_;
}

const cv::Mat2f & c_ecc_align::current_remap() const
{
  return current_remap_;
}


// convert image to float with optional gaussian smoothing
void c_ecc_align::prepare_input_image(cv::InputArray src, cv::InputArray src_mask,
    double smooth_sigma, bool force_create_mask,
    cv::Mat1f & dst, cv::Mat1b & dst_mask)
{
  if ( smooth_sigma <= 0 ) {
    src.getMat().convertTo(dst, dst.depth());
  }
  else {
    const cv::Mat1f G = cv::getGaussianKernel(2 * ((int) (4 * smooth_sigma)) + 1, smooth_sigma);

    if ( src_mask.empty() ) {
      cv::sepFilter2D(src, dst, dst.depth(), G, G, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
    }
    else {
      const cv::Mat1b mask = src_mask.getMat();
      cv::Mat1f fmask;
      mask.convertTo(fmask, fmask.type(), 1. / 255);
      cv::sepFilter2D(src, dst, dst.depth(), G, G, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
      cv::sepFilter2D(fmask, fmask, fmask.depth(), G, G, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
      cv::divide(dst, fmask, dst);
      dst.setTo(0, ~mask);
    }
  }


  if ( !src_mask.empty() ) {
    if ( cv::countNonZero(src_mask) == src_mask.size().area() ) {
      src_mask.copyTo(dst_mask);
    }
    else { // may need to protect some border near mask edges because of differentiation
      cv::erode(src_mask, dst_mask, cv::Mat1b(5, 5, 255), cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
    }
  }
  else if ( force_create_mask ) {
    dst_mask.create(src.size());
    dst_mask.setTo(255);
  }
  else {
    // ensure it is empty
    dst_mask.release();
  }
}


///////////////////////////////////////////////////////////////////////////////

c_ecc_forward_additive::c_ecc_forward_additive(int ecc_motion_type)
{
  set_motion_type(ecc_motion_type);
}


bool c_ecc_forward_additive::set_reference_image(cv::InputArray referenceImage, cv::InputArray referenceMask)
{
  if ( referenceImage.channels() != 1 ) {
    CF_ERROR("reference image must be single-channel (grayscale)");
    return false;
  }

  if ( !referenceMask.empty() ) {
    if ( referenceMask.type() != CV_8UC1 || referenceMask.size() != referenceImage.size() ) {
      CF_ERROR("reference mask must CV_8UC1 type of the same size as input image");
      return false;
    }
  }

  // convert reference image to float
  prepare_input_image(referenceImage, referenceMask, reference_smooth_sigma_, false, f, fmask);
//  if ( pyramid_normalization_level_ > 0 ) {
//    pnormalize(f, fmask, pyramid_normalization_level_, pyramid_normalization_regularization_term_);
//  }
  return true;
}

const cv::Mat1f & c_ecc_forward_additive::reference_image() const
{
  return f;
}

const cv::Mat1b & c_ecc_forward_additive::reference_mask() const
{
  return fmask;
}

const cv::Mat1f & c_ecc_forward_additive::input_image() const
{
  return g;
}

const cv::Mat1b & c_ecc_forward_additive::input_mask() const
{
  return gmask;
}

bool c_ecc_forward_additive::align_to_reference(cv::InputArray inputImage,
    cv::InputOutputArray inputOutputWarpMatrix,
    cv::InputArray inputMask)
{
  if ( inputImage.channels() != 1 ) {
    CF_ERROR("input image must be single-channel (grayscale)");
    return false;
  }

  if ( !inputMask.empty() ) {
    if ( inputMask.type() != CV_8UC1 || inputMask.size() != inputImage.size() ) {
      CF_ERROR("input mask must CV_8UC1 type of the same size as input image");
      return false;
    }
  }


  num_iterations_ = -1;
  rho_ = -1;
  if ( eps_ <= 0 ) {
    eps_ = 1e-3;
  }

  if ( (number_of_parameters_ = init_warp_matrix(ecc_motion_type_, inputOutputWarpMatrix)) < 1 ) {
    CF_FATAL("c_ecc_align: initWarpMatrix() fails");
    return false;
  }
  inputOutputWarpMatrix.getMat().convertTo(p, p.type());

  //
  // Precompute
  //

  // convert input image (to be aligned) to float and create the mask
  prepare_input_image(inputImage, inputMask, input_smooth_sigma_, true, g, gmask);
  //  if ( pyramid_normalization_level_ > 0 ) {
  //    pnormalize(g, gmask, pyramid_normalization_level_, pyramid_normalization_regularization_term_);
  //  }

  // Evaluate the gradient ∇G of the input image G(x)
  ecc_differentiate(g, gx, gy);
  if ( !inputMask.empty() ) {
    cv::bitwise_not(gmask, iwmask);
    gx.setTo(0, iwmask);
    gy.setTo(0, iwmask);
  }

  //
  // Iterate
  //

  cv::Scalar gMean, gStd, fMean, fStd;
  double stdev_ratio;
  bool failed = false;

  for ( num_iterations_ = 0; num_iterations_ <= max_iterations_; ++num_iterations_ ) {

    // Warp g, gx and gy with W(x; p) to compute warped input image g(W(x; p)) and it's gradients
    createRemap(ecc_motion_type_, p, current_remap_, f.size(), -1);

    cv::remap(g, gw, current_remap_, cv::noArray(), image_interpolation_flags_, cv::BORDER_REPLICATE);
    cv::remap(gx, gxw, current_remap_, cv::noArray(), image_interpolation_flags_, cv::BORDER_REPLICATE);
    cv::remap(gy, gyw, current_remap_, cv::noArray(), image_interpolation_flags_, cv::BORDER_REPLICATE);
    cv::remap(gmask, wmask, current_remap_, cv::noArray(), cv::INTER_AREA, cv::BORDER_CONSTANT, 0);

    if ( !fmask.empty() ) {
      bitwise_and(wmask, fmask, wmask);
    }

    cv::bitwise_not(wmask, iwmask);

    gxw.setTo(0, iwmask);
    gyw.setTo(0, iwmask);

    // compute stdev ratio stdev(g)/stdev(f) and mean values
    cv::meanStdDev(f, fMean, fStd, wmask);
    cv::meanStdDev(gw, gMean, gStd, wmask);
    stdev_ratio = gStd[0] / fStd[0];

    // calculate jacobian of image wrt parameters
    compute_jacobian(ecc_motion_type_, gxw, gyw, p, jac);

    // calculate Hessian and its inverse
    project_onto_jacobian(jac, jac, Hinv, number_of_parameters_);
    cv::invert(Hinv, Hinv, cv::DECOMP_CHOLESKY);

    // compute update parameters
    // e = -(gwzm - stdev_ratio * fzm);
    cv::scaleAdd(f, -stdev_ratio, gw, e);
    cv::subtract(e, gMean - stdev_ratio * fMean, e);
    e.setTo(0, iwmask);

    project_onto_jacobian(jac, e, ep, number_of_parameters_);
    dp = (Hinv * ep) * (-update_step_scale_);

    // update warping matrix
    if ( !update_warp_matrix_forward_additive(ecc_motion_type_, p, dp, &current_eps_, f.cols, f.rows) ) {
      CF_ERROR("update_warp_matrix() fails");
      failed = true;
    }

    //    rho_ = fzm.dot(gwzm) / (cv::countNonZero(wmask) * fStd[0] * gStd[0]);
    //    CF_DEBUG("[%d] rho_=%g stdev_ratio=%g e=%g\n"
    //        "dp %dx%d: {\n"
    //        "%+12f %+12f %+12f\n"
    //        "%+12f %+12f %+12f\n"
    //        "}\n"
    //        "\n",
    //        num_iterations_,
    //        rho_,
    //        stdev_ratio,
    //        current_eps_,
    //        dp.rows, dp.cols,
    //        dp(0, 0), dp(0, 1), dp(0, 2),
    //        dp(1, 0), dp(1, 1), dp(1, 2)
    //    );

    if ( current_eps_ < eps_ || num_iterations_ == max_iterations_ || failed ) {
      rho_ = cv::computeECC(f, gw, wmask);
      cv::subtract(f, fMean, e), e.setTo(0, iwmask);
      cv::subtract(gw, gMean, gw), gw.setTo(0, iwmask);
      rho_ = e.dot(gw) / (cv::countNonZero(wmask) * fStd[0] * gStd[0]);

      //      cv::Scalar m1, s1, m2, s2;
      //      cv::meanStdDev(f, m1, s1, wmask);
      //      cv::subtract(f, m1, e), e.setTo(0, iwmask);
      //
      //      cv::meanStdDev(gw, m2, s2, wmask);
      //      cv::subtract(gw, m2, gw), gw.setTo(0, iwmask);
      //      rho_ = e.dot(gw) / (cv::countNonZero(wmask) * s1[0] * s2[0]);

      break;
    }

  }

  p.convertTo(inputOutputWarpMatrix, inputOutputWarpMatrix.depth());

  return !failed && rho_ >= min_rho_;
}

bool c_ecc_forward_additive::align(cv::InputArray inputImage, cv::InputArray referenceImage,
    cv::InputOutputArray inputOutputWarpMatrix,
    cv::InputArray inputMask, cv::InputArray referenceMask)
{
  num_iterations_ = -1, rho_ = -1;
  if ( !set_reference_image(referenceImage, referenceMask) ) {
    CF_FATAL("c_ecc_forward_additive: set_reference_image() fails");
    return false;
  }
  return align_to_reference(inputImage, inputOutputWarpMatrix, inputMask);
}


///////////////////////////////////////////////////////////////////////////////

bool c_ecc_inverse_compositional::set_reference_image(cv::InputArray referenceImage, cv::InputArray referenceMask)
{
  // Convert reference image to float,
  // Pre-Evaluate the gradient ∇T of reference image T(x),
  // Pre-Compute Jacobian ∂W/∂p at (x,0) and steepest descent images ▽f*∂W/∂p
  prepare_input_image(referenceImage, referenceMask, reference_smooth_sigma_, false, f, fmask);
  //  if ( pyramid_normalization_level_ > 0 ) {
  //    pnormalize(f, fmask, pyramid_normalization_level_, pyramid_normalization_regularization_term_);
  //  }

  ecc_differentiate(f, fx, fy);
  compute_jacobian(ecc_motion_type_, fx, fy, createEyeTransform(ecc_motion_type_), jac_);
  return true;
}

const cv::Mat1f & c_ecc_inverse_compositional::reference_image() const
{
  return f;
}

const cv::Mat1f & c_ecc_inverse_compositional::input_image() const
{
  return g;
}


bool c_ecc_inverse_compositional::align_to_reference(cv::InputArray inputImage, cv::InputOutputArray inputOutputWarpMatrix,
    cv::InputArray inputMask)
{
  cv::Mat1f jac;

  num_iterations_ = -1;
  rho_ = -1;
  if ( eps_ <= 0 ) {
    eps_ = 1e-3;
  }

  if ( (number_of_parameters_ = init_warp_matrix(ecc_motion_type_, inputOutputWarpMatrix)) < 1 ) {
    CF_FATAL("c_ecc_align: initWarpMatrix() fails");
    return false;
  }
  inputOutputWarpMatrix.getMat().convertTo(p, p.type());

  // Convert input image (to be aligned) to float and create the mask
  prepare_input_image(inputImage, inputMask, input_smooth_sigma_, true, g, gmask);
  //  if ( pyramid_normalization_level_ > 0 ) {
  //    pnormalize(g, gmask, pyramid_normalization_level_, pyramid_normalization_regularization_term_);
  //  }

  // Assume the inital warp matrix p is good enough to avoid the hessian matrix evaluation on each iteration
  createRemap(ecc_motion_type_, p, current_remap_, f.size());
  cv::remap(gmask, wmask, current_remap_, cv::noArray(),
      cv::INTER_AREA,
      cv::BORDER_CONSTANT,
      cv::Scalar(0));

  if ( !fmask.empty() ) {
    bitwise_and(wmask, fmask, wmask);
  }
  wmask.copyTo(fmask);
  cv::bitwise_not(wmask, iwmask);
  const double nnz0 = cv::countNonZero(wmask);

  jac_.copyTo(jac);
  for ( int i = 0; i < number_of_parameters_; ++i ) {
    jac.rowRange(i * f.rows, (i + 1) * f.rows).setTo(0, iwmask);
  }


  // Compute the Hessian matrix H = ∑x[▽f*∂W/∂p]^T*[▽f*∂W/∂p] and it's inverse
  project_onto_jacobian(jac, jac, Hinv, number_of_parameters_);
  cv::invert(Hinv, Hinv, cv::DECOMP_CHOLESKY);
  Hinv *= -update_step_scale_;

  //
  // Iterate
  //

  cv::Scalar gMean, gStd, fMean, fStd;
  double stdev_ratio, nnz1;
  bool failed = false;

  for ( num_iterations_ = 0; num_iterations_ <= max_iterations_; ++num_iterations_ ) {

    //
    // Warp g with W(x; p) to compute g(W(x; p))
    //
    if ( num_iterations_ < 1 ) {
      nnz1 = nnz0;
    }
    else {

      createRemap(ecc_motion_type_, p, current_remap_, f.size());

      cv::remap(gmask, wmask, current_remap_, cv::noArray(),
          cv::INTER_AREA,
          cv::BORDER_CONSTANT,
          cv::Scalar(0));

      cv::bitwise_not(wmask, iwmask);

      nnz1 = cv::countNonZero(wmask);
    }

    cv::remap(g, gw, current_remap_, cv::noArray(),
        image_interpolation_flags_,
        cv::BORDER_REPLICATE);  // The cv::BORDER_REPLICATE is to avoid some annoying edge effects caused by interpolation

    //
    // compute stdev ratio stdev(f)/stdev(gw) and mean values
    //
    cv::meanStdDev(f, fMean, fStd, wmask);
    cv::meanStdDev(gw, gMean, gStd, wmask);
    stdev_ratio = fStd[0] / gStd[0];

    //
    // compute the error image e = fzm - stdev_ratio * gwzm
    //
    cv::scaleAdd(gw, -stdev_ratio, f, e);  // e now contains the error image
    cv::subtract(e, fMean - stdev_ratio * gMean, e), e.setTo(0, iwmask);

    //
    // compute update parameters
    //
    project_onto_jacobian(jac, e, ep, number_of_parameters_);  // ep now contains projected error
    dp = (Hinv * ep) * square(nnz0 / nnz1);

    //
    // update warping parameters
    //
    if ( !update_warp_matrix_inverse_composite(ecc_motion_type_, p, dp, &current_eps_, f.cols, f.rows) ) {
      CF_ERROR("update_warp_matrix_inverse_composite() fails");
      failed = true;
    }

    //    CF_DEBUG("[%d] rho_=%g lambda=%g eps=%g mean(e)=%g\n"
    //        "p: %dx%d dp %dx%d: {\n"
    //        "%+12f %+12f %+12f\n"
    //        "%+12f %+12f %+12f\n"
    //        "}\n"
    //        "\n",
    //        num_iterations_,
    //        rho_,
    //        stdev_ratio,
    //        current_eps_,
    //        mean(e, wmask)[0],
    //        p.rows, p.cols,
    //        dp.rows, dp.cols,
    //        dp(0,0),dp(0,1),dp(0,2),
    //        dp(1,0),dp(1,1),dp(1,2)
    //      );

    if ( current_eps_ < eps_ || num_iterations_ == max_iterations_ || failed ) {
      //  rho_ = cv::computeECC(f, gw, wmask);
      cv::subtract(f, fMean, e), e.setTo(0, iwmask);
      cv::subtract(gw, gMean, gw), gw.setTo(0, iwmask);
      rho_ = e.dot(gw) / (nnz1 * fStd[0] * gStd[0]);
      break;
    }

  }

  p.convertTo(inputOutputWarpMatrix, inputOutputWarpMatrix.depth());

  return !failed && rho_ > min_rho_;
}

bool c_ecc_inverse_compositional::align(cv::InputArray inputImage, cv::InputArray referenceImage,
    cv::InputOutputArray inputOutputWarpMatrix,
    cv::InputArray inputMask, cv::InputArray referenceMask)
{
  num_iterations_ = -1, rho_ = -1;
  if ( !set_reference_image(referenceImage, referenceMask) ) {
    CF_FATAL("c_ecc_inverse_compositional: set_reference_image() fails");
    return false;
  }
  return align_to_reference(inputImage, inputOutputWarpMatrix, inputMask);
}

///////////////////////////////////////////////////////////////////////////////

c_ecc_pyramide_align::c_ecc_pyramide_align(c_ecc_align * method)
  : method_(method)
{
}

c_ecc_align * c_ecc_pyramide_align::method() const
{
  return this->method_;
}


void c_ecc_pyramide_align::set_minimum_image_size(int v)
{
  minimum_image_size_ = v;
}
int c_ecc_pyramide_align::minimum_image_size() const
{
  return minimum_image_size_;
}


const std::vector<cv::Mat> & c_ecc_pyramide_align::image_pyramid(int index) const
{
  return image_pyramids_[index];
}

const std::vector<cv::Mat> & c_ecc_pyramide_align::mask_pyramid( int index) const
{
  return mask_pyramids_[index];
}

const std::vector<cv::Mat1f> & c_ecc_pyramide_align::transform_pyramid() const
{
  return transform_pyramid_;
}



bool c_ecc_pyramide_align::align(cv::InputArray inputImage, cv::InputArray referenceImage,
    cv::InputOutputArray inputOutputWarpMatrix,
    cv::InputArray inputMask, cv::InputArray referenceMask)
{
  return align(this->method_, inputImage, referenceImage,
      inputOutputWarpMatrix,
      inputMask, referenceMask);
}

bool c_ecc_pyramide_align::align(c_ecc_align * method,
    cv::InputArray inputImage, cv::InputArray referenceImage,
    cv::InputOutputArray inputOutputWarpMatrix,
    cv::InputArray inputMask, cv::InputArray referenceMask)
{
  cv::Mat1f T;

  if ( !method ) {
    CF_ERROR("Invalid call: No underlying align method specified");
    return false;
  }

  if ( inputImage.channels() != 1 || referenceImage.channels() != 1 ) {
    CF_ERROR("Both input and reference images must be single-channel (grayscale)");
    return false;
  }


  if ( !inputOutputWarpMatrix.empty() ) {
    inputOutputWarpMatrix.getMat().convertTo(T, T.type());
  }
  else if ( (T = createEyeTransform(method->motion_type())).empty() ) {
    CF_ERROR("createTransform(motion_type=%d) fails", method->motion_type() );
    return false;
  }

  // Build image / initial transform pyramid

  for ( int i = 0; i < 2; ++i ) {
    image_pyramids_[i].clear();
    image_pyramids_[i].reserve(10);
    mask_pyramids_[i].clear();
    mask_pyramids_[i].reserve(10);
  }
  image_pyramids_[0].emplace_back(inputImage.getMat());
  image_pyramids_[1].emplace_back(referenceImage.getMat());
  mask_pyramids_[0].emplace_back(inputMask.getMat());
  mask_pyramids_[1].emplace_back(referenceMask.getMat());

  transform_pyramid_.clear();
  transform_pyramid_.reserve(10);
  transform_pyramid_.emplace_back(T);

  // Build pyramid
  while ( 42 ) {

    const int currentMinSize = std::min(std::min(std::min(
        image_pyramids_[0].back().cols, image_pyramids_[0].back().rows),
            image_pyramids_[1].back().cols), image_pyramids_[1].back().rows);

    const int nextMinSize = currentMinSize / 2;

    if ( nextMinSize < minimum_image_size_) {
      break;
    }

    for ( int i = 0; i < 2; ++i ) {
      image_pyramids_[i].emplace_back();
      // fixme: GaussianBlur with mask will produce edge effects
      cv::GaussianBlur(image_pyramids_[i][image_pyramids_[i].size() - 2], image_pyramids_[i].back(), cv::Size(5, 5), 0, 0, cv::BORDER_REPLICATE);
      downstrike_uneven(image_pyramids_[i].back(), image_pyramids_[i].back());

      mask_pyramids_[i].emplace_back();
      if ( !mask_pyramids_[i][mask_pyramids_[i].size() - 2].empty() ) {
        downstrike_uneven(mask_pyramids_[i][mask_pyramids_[i].size() - 2], mask_pyramids_[i].back());
      }
    }

    transform_pyramid_.emplace_back();
    scaleTransform(method->motion_type(), transform_pyramid_[transform_pyramid_.size() - 2],
        transform_pyramid_[transform_pyramid_.size() - 1], 0.5);
  }




  // Align pyramid images in coarse-to-fine direction
  bool eccOk = false;
  for ( int i = transform_pyramid_.size() - 1; i >= 0; --i ) {

    cv::Mat1f & T = transform_pyramid_[i];
    CF_DEBUG("H: i=%d T: %dx%d", i, T.rows, T.cols);

    if ( !method->align(image_pyramids_[0][i], image_pyramids_[1][i], T, mask_pyramids_[0][i], mask_pyramids_[1][i]) ) {
      CF_DEBUG("L[%2d] : eccAlign() fails: motion: %d size=%dx%d numiterations=%d rho=%g eps=%g", i,
          method->motion_type(),
          image_pyramids_[0][i].cols, image_pyramids_[0][i].rows,
          method->num_iterations(), method->rho(), method->current_eps());
      continue;
    }
    else {
      CF_DEBUG("L[%2d] : eccAlign() OK: motion: %d size=%dx%d numiterations=%d rho=%g eps=%g", i, method->motion_type(),
          image_pyramids_[0][i].cols, image_pyramids_[0][i].rows,
          method->num_iterations(), method->rho(), method->current_eps());
    }

    if ( i == 0 ) {
      eccOk = true;
    }
    else { // i > 0
      scaleTransform(method->motion_type(), T, transform_pyramid_[i - 1], 2);
    }
  }

  if ( !eccOk ) {
    return false;
  }

  transform_pyramid_[0].copyTo(inputOutputWarpMatrix);

  return true;
}


///////////////////////////////////////////////////////////////////////////////

static double estimate_image_noise(cv::InputArray inputImage, cv::InputArray inputMask)
{
  const cv::Scalar noise =
      estimate_noise(inputImage,
          cv::noArray(),
          inputMask);

  double max_noise =
      noise[0];

  for ( int i = 1, cn = inputImage.channels(); i < cn; ++i ) {
    max_noise = std::max(max_noise, noise[i]);
  }

  return max_noise;
}


void c_ecch_flow::set_support_scale(int v)
{
  support_scale_ = v;
}

int c_ecch_flow::support_scale() const
{
  return support_scale_;
}

void c_ecch_flow::set_max_iterations(int v)
{
  max_iterations_ = v;
}

int c_ecch_flow::max_iterations() const
{
  return max_iterations_;
}

void c_ecch_flow::set_update_multiplier(double v)
{
  update_multiplier_ = v;
}

double c_ecch_flow::update_multiplier() const
{
  return update_multiplier_;
}

void c_ecch_flow::set_normalization_scale(int v)
{
  normalization_scale_ = v;
}

int c_ecch_flow::normalization_scale() const
{
  return normalization_scale_;
}

void c_ecch_flow::set_input_smooth_sigma(double v)
{
  input_smooth_sigma_ = v;
}

double c_ecch_flow::input_smooth_sigma() const
{
  return input_smooth_sigma_;
}

void c_ecch_flow::set_reference_smooth_sigma(double v)
{
  reference_smooth_sigma_ = v;
}

double c_ecch_flow::reference_smooth_sigma() const
{
  return reference_smooth_sigma_;
}

void c_ecch_flow::set_max_pyramid_level(int v)
{
  max_pyramid_level_ = v;
}

int c_ecch_flow::max_pyramid_level() const
{
  return max_pyramid_level_;
}


void c_ecch_flow::flow2remap(const cv::Mat2f & uv, cv::Mat2f & rmap)
{
  rmap.create(uv.size());

  typedef tbb::blocked_range<int> range;
  tbb::parallel_for(range(0, rmap.rows, 256),
      [&rmap, &uv] (const range & r ) {
        const int nx = rmap.cols;
        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
          for ( int x = 0; x < nx; ++x ) {
            rmap[y][x][0] = x + uv[y][x][0];
            rmap[y][x][1] = y + uv[y][x][1];
          }
        }
      });
}

void c_ecch_flow::remap2flow(const cv::Mat2f & rmap,
    cv::Mat2f & uv)
{
  uv.create(rmap.size());

  typedef tbb::blocked_range<int> range;
  tbb::parallel_for(range(0, rmap.rows, 256),
      [&rmap, &uv] (const range & r ) {
        const int nx = rmap.cols;
        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
          for ( int x = 0; x < nx; ++x ) {
            uv[y][x][0] = rmap[y][x][0] - x;
            uv[y][x][1] = rmap[y][x][1] - y;
          }
        }
      });

}

const cv::Mat1f & c_ecch_flow::reference_image() const
{
  static const cv::Mat1f empty_stub;
  return pyramid_.empty() ?  empty_stub : pyramid_.front().reference_image;
}

const cv::Mat1b & c_ecch_flow::reference_mask() const
{
  static const cv::Mat1b empty_stub;
  return pyramid_.empty() ? empty_stub : pyramid_.front().reference_mask;
//  return reference_mask_;
}

//
//const cv::Mat1f & c_ecch_flow::input_image() const
//{
//  return current_image_;
//}
//
//const cv::Mat1b & c_ecch_flow::input_mask() const
//{
//  return current_mask_;
//}
//
const cv::Mat2f & c_ecch_flow::current_uv() const
{
  return cuv;
}
//
//const cv::Mat1f & c_ecch_flow::current_It() const
//{
//  return current_It_;
//}
//
//const cv::Mat1f & c_ecch_flow::current_worker_image() const
//{
//  return worker_image_;
//}
//
//const std::vector<cv::Mat1f> & c_ecch_flow::mu1() const
//{
//  return mu1_;
//}
//
//const std::vector<cv::Mat1f> & c_ecch_flow::mu2() const
//{
//  return mu2_;
//}


bool c_ecch_flow::convert_input_images(cv::InputArray src, cv::InputArray src_mask,
    cv::Mat1f & dst, cv::Mat1b & dst_mask) const
{
  if ( src.depth() == dst.depth() ) {
    dst = src.getMat();
  }
  else {
    src.getMat().convertTo(dst, dst.depth());
  }

  if ( src_mask.empty() || cv::countNonZero(src_mask) == src_mask.size().area() ) {
    dst_mask.release();
  }
  else {
    dst_mask = src_mask.getMat();
  }

  return true;
}


void c_ecch_flow::pnormalize(cv::InputArray src, cv::InputArray mask, cv::OutputArray dst) const
{
  int normalization_scale = this->normalization_scale_;
  if ( normalization_scale == 0 ) {
    src.copyTo(dst);
    return;
  }

  if ( normalization_scale_ < 0 ) {
    normalization_scale = support_scale_;
  }

  const int nscale =
      std::max(normalization_scale_,
          support_scale_);

  cv::Mat mean;
  cv::Scalar mv, sv;

  ecc_downscale(src, mean, nscale, cv::BORDER_REPLICATE);
  ecc_upscale(mean, src.size());
  cv::subtract(src, mean, mean);

  cv::meanStdDev(mean, mv, sv, mask);
  cv::multiply(mean, 1./sv[0], dst);
  //cv::GaussianBlur(mean, dst, cv::Size(3,3), 0, 0, cv::BORDER_REFLECT101);
}


bool c_ecch_flow::pscale(cv::InputArray src, cv::Mat & dst, bool ismask) const
{
  const int level = support_scale_;

  cv::Size size = src.size();
  for ( int i = 0; i < level; ++i ) {
    size.width = (size.width + 1) / 2;
    size.height = (size.height + 1) / 2;
  }
  cv::resize(src, dst, size, 0, 0, cv::INTER_AREA);

  if ( ismask ) {
    cv::compare(dst, 255, dst, cv::CMP_EQ);
  }
  else {
    static thread_local const cv::Mat G = cv::getGaussianKernel(3, 0, CV_32F);
    cv::sepFilter2D(dst, dst, -1, G, G, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
  }

  return true;
}


bool c_ecch_flow::compute_uv(pyramid_entry & e,
    cv::Mat2f & outuv) const
{
  cv::Mat1f worker_image;
  cv::Mat1f It, Itx, Ity;
  cv::Mat1b M;

  tbb::parallel_invoke(
    [&e, &worker_image]() {
      e.reference_image.copyTo(worker_image);
      cv::remap(e.current_image, worker_image,
          e.rmap, cv::noArray(),
          cv::INTER_AREA,
          cv::BORDER_TRANSPARENT);
    },

    [&e, &M]() {
      if ( !e.current_mask.empty() ) {
        cv::remap(e.current_mask, M,
            e.rmap, cv::noArray(),
            cv::INTER_NEAREST,
            cv::BORDER_CONSTANT);
      }
    });


  if ( !e.reference_mask.empty() ) {
    if ( M.empty() ) {
      e.reference_mask.copyTo(M);
    }
    else {
      cv::bitwise_and(e.reference_mask, M, M);
    }
  }


  const cv::Mat1f & I1 = worker_image;
  const cv::Mat1f & I2 = e.reference_image;

  cv::subtract(I2, I1, It, M);


#if 0
  cv::multiply(e.Ix, e.It, Itx);
  pscale(Itx, Itx);

  cv::multiply(e.Iy, e.It, Ity);
  pscale(Ity, Ity);

#else

  tbb::parallel_invoke(

    [this, &e, &It, &Itx]() {
      cv::multiply(e.Ix, It, Itx);
      pscale(Itx, Itx);
    },

    [this, &e, &It, &Ity]() {
      cv::multiply(e.Iy, It, Ity);
      pscale(Ity, Ity);
    }
  );
#endif


  //  a00 = Ixx;
  //  a01 = Ixy;
  //  a10 = Ixy;
  //  a11 = Iyy;
  //  b0  = 2 * Itx;
  //  b1  = 2 * Ity;
  //  D = a00 * a11 - a10 * a01
  //  u = 1/D * (a11 * b0 - a01 * b1);
  //  v = 1/D * (a00 * b1 - a10 * b0);

  outuv.create(e.D.size());

  CF_DEBUG("e.D.size=%dx%d Itx: %dx%d e.current_image: %dx%d e.current_mask: %dx%d",
      e.D.cols, e.D.rows,
      Itx.cols, Itx.rows,
      e.current_image.cols, e.current_image.rows,
      e.current_mask.cols, e.current_mask.rows);


  typedef tbb::blocked_range<int> range;

  tbb::parallel_for(range(0, outuv.rows, 256),
      [&Itx, &e, &Ity, &outuv](const range & r) {

        const cv::Mat4f & D = e.D;

        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
          for ( int x = 0, nx = outuv.cols; x < nx; ++x ) {
            const float & a00 = D[y][x][0];
            const float & a01 = D[y][x][1];
            const float & a10 = D[y][x][1];
            const float & a11 = D[y][x][2];
            const float & det = D[y][x][3];

            const float & b0 = Itx[y][x];
            const float & b1 = Ity[y][x];
            outuv[y][x][0] = det * (a11 * b0 - a01 * b1);
            outuv[y][x][1] = det * (a00 * b1 - a10 * b0);
          }
        }
      });


  if ( update_multiplier_ != 1 ) {
    cv::multiply(outuv, update_multiplier_, outuv);
  }

  ecc_upscale(outuv, I1.size());
  if ( outuv.size() != I1.size() ) {
    CF_ERROR("Invalid outuv size: %dx%d must be %dx%d", outuv.cols, outuv.rows, I1.cols, I1.rows);
    return false;
  }

  return true;
}


bool c_ecch_flow::compute(cv::InputArray inputImage, cv::InputArray referenceImage, cv::Mat2f & rmap,
    cv::InputArray inputMask, cv::InputArray referenceMask)
{
  set_reference_image(referenceImage, referenceMask);
  return compute(inputImage, rmap, inputMask);
}

bool c_ecch_flow::set_reference_image(cv::InputArray referenceImage,
    cv::InputArray referenceMask)
{
  cv::Mat1f I, Ixx, Iyy, Ixy, DD;
  cv::Mat1b M;

  const double noise_level =
      estimate_image_noise(referenceImage,
          referenceMask);

  pyramid_.clear();
  pyramid_.reserve(20);

  if ( !convert_input_images(referenceImage, referenceMask, I, M) ) {
    CF_ERROR("convert_input_images() fails");
    return false;
  }

  const int min_image_size =
      std::min(std::max(referenceImage.cols(), referenceImage.rows()),
          3 * (1 << (support_scale_)));

  pyramid_.emplace_back();
  pyramid_.back().reference_mask = M;


  for ( int current_level = 0; ; ++current_level ) {

    pyramid_entry & current_scale = pyramid_.back();
    const cv::Size currentSize = I.size();

    pnormalize(I, current_scale.current_mask, current_scale.reference_image);

    ecc_differentiate(current_scale.reference_image,
        current_scale.Ix, current_scale.Iy);

    tbb::parallel_invoke(
        [this, &current_scale, &Ixx]() {
          cv::multiply(current_scale.Ix, current_scale.Ix, Ixx);
          pscale(Ixx, Ixx);
        },
        [this, &current_scale, &Ixy]() {
          cv::multiply(current_scale.Ix, current_scale.Iy, Ixy);
          pscale(Ixy, Ixy);
        },
        [this, &current_scale, &Iyy]() {
          cv::multiply(current_scale.Iy, current_scale.Iy, Iyy);
          pscale(Iyy, Iyy);
        }
    );

    const double RegularizationTerm = // fxme: this regularization therm estimation seems crazy
        pow(1e-3 * noise_level / (1 << current_level), 4);

    cv::absdiff(Ixx.mul(Iyy), Ixy.mul(Ixy), DD);
    cv::add(DD, RegularizationTerm, DD);
    cv::divide(1, DD, DD);

    cv::Mat D_channels[4] = {
        Ixx,
        Ixy,
        Iyy,
        DD
      };

    cv::merge(D_channels, 4, current_scale.D);


    const cv::Size nextSize((currentSize.width + 1) / 2, (currentSize.height + 1) / 2);

    if ( nextSize.width < min_image_size || nextSize.height < min_image_size ) {
      CF_DEBUG("currentSize: %dx%d nextSize: %dx%d",
          currentSize.width, currentSize.height,
          nextSize.width, nextSize.height);
      break;
    }

    if ( max_pyramid_level_ >= 0 && current_level >= max_pyramid_level_ ) {
      break;
    }

    pyramid_.emplace_back();

    if ( pyramid_[pyramid_.size()-2].reference_mask.empty() ) {
      cv::pyrDown(I, I, nextSize, cv::BORDER_REPLICATE);
    }
    else {

      tbb::parallel_invoke(

          [&I, nextSize]() {
            cv::pyrDown(I, I, nextSize, cv::BORDER_REPLICATE);
          },

          [this, &M, nextSize]() {
            cv::resize(M, pyramid_.back().reference_mask, nextSize, 0, 0, cv::INTER_NEAREST);
            if ( cv::countNonZero(pyramid_.back().reference_mask) == nextSize.area() ) {
              pyramid_.back().reference_mask.release();
            }
          });
    }
  }

  CF_DEBUG("reference_pyramid_.size=%zu min:%dx%d", pyramid_.size(),
      pyramid_.back().reference_image.cols,
      pyramid_.back().reference_image.rows);

//  if ( true ) {
//    for ( int i = 0, n = pyramid_.size(); i < n; ++i ) {
//      save_image(pyramid_[i].reference_image, ssprintf("pyramid/ref.%03d.tiff", i));
//    }
//  }

  return true;
}


// FIXME: Make sure at caller side that both reference and current image are pre-inpained for missing pixels!!!!
bool c_ecch_flow::compute(cv::InputArray inputImage, cv::Mat2f & rmap, cv::InputArray inputMask)
{
  cv::Mat1f I;
  cv::Mat1b M;
  cv::Mat2f uv;

  if ( pyramid_.empty() ) {
    CF_ERROR("Invalid call to c_ecch_flow::compute(): reference image was not set");
    return false;
  }

  if ( rmap.empty() ) {
    createIdentityRemap(rmap, pyramid_.front().reference_image.size(), rmap.depth() );
  }
  else if ( rmap.size() != pyramid_.front().reference_image.size() ) {
    CF_ERROR("Invalid args to c_ecch_flow::compute(): reference image and rmap sizes not match");
    return false;
  }

  if ( !convert_input_images(inputImage, inputMask, I, M) ) {
    CF_ERROR("convert_input_images() fails");
    return false;
  }

  pyramid_.front().rmap = rmap; // attention to this!
  pyramid_.front().current_mask = M;
  pnormalize(I, M, pyramid_.front().current_image);

  // FIXME: limit downscale steps by input image size too

  for ( int i = 1, n = pyramid_.size(); i < n; ++i ) {

    const pyramid_entry & prev_scale = pyramid_[i - 1];
    pyramid_entry & next_scale = pyramid_[i];

    const cv::Size prev_image_size = prev_scale.current_image.size();
    const cv::Size next_image_size((prev_image_size.width + 1) / 2, (prev_image_size.height + 1) / 2);

    const cv::Size prev_rmap_size = prev_scale.rmap.size();
    const cv::Size next_rmap_size((prev_rmap_size.width + 1) / 2, (prev_rmap_size.height + 1) / 2);

    if ( !prev_scale.current_mask.empty()) {
      cv::resize(M, next_scale.current_mask, next_image_size, 0, 0, cv::INTER_NEAREST);
      if ( cv::countNonZero(next_scale.current_mask) == next_image_size.area() ) {
        next_scale.current_mask.release();
      }
    }

    tbb::parallel_invoke(

        [this, &I, &prev_scale, &next_scale, next_image_size]() {
          cv::pyrDown(I, I, next_image_size, cv::BORDER_REPLICATE);
          pnormalize(I, next_scale.current_mask, next_scale.current_image);
        },


        [&prev_scale, &next_scale, prev_image_size, next_image_size, next_rmap_size]() {

          const cv::Scalar size_ratio((double) next_image_size.width / (double) prev_image_size.width,
              (double) next_image_size.height / (double) prev_image_size.height);

          cv::pyrDown(prev_scale.rmap, next_scale.rmap, next_rmap_size, cv::BORDER_REPLICATE);
          cv::multiply(next_scale.rmap, size_ratio, next_scale.rmap);
        }
    );
  }


//  if ( true ) {
//    for ( int i = 0, n = pyramid_.size(); i < n; ++i ) {
//      save_image(pyramid_[i].current_image, ssprintf("pyramid/cur.%03d.tiff", i));
//    }
//  }


  compute_uv(pyramid_.back(), cuv);

  if ( pyramid_.size() == 1 ) {
    cv::add(rmap, cuv, rmap);
  }
  else {

    for ( int i = pyramid_.size() - 2; i >= 0; --i ) {

      pyramid_entry & current_scale = pyramid_[i];
      const pyramid_entry & prev_scale = pyramid_[i + 1];

      const cv::Size current_image_size = current_scale.current_image.size();
      const cv::Size prev_image_size = prev_scale.current_image.size();

      const cv::Size current_rmap_size = current_scale.rmap.size();
      const cv::Size prev_rmap_size = prev_scale.rmap.size();

      const cv::Scalar size_ratio((double) current_image_size.width / (double) prev_image_size.width,
          (double) current_image_size.height / (double) prev_image_size.height);

      cv::multiply(cuv, size_ratio, cuv);
      cv::pyrUp(cuv, cuv, current_rmap_size);
      cv::add(current_scale.rmap, cuv, current_scale.rmap);

      compute_uv(current_scale, uv);
      if ( i == 0 ) { // at zero level the current_scale.rmap is referenced directly to output
        cv::add(rmap, uv, rmap);
      }
      else {
        cv::add(uv, cuv, cuv);
      }
    }
  }

  return true;
}





///////////////////////////////////////////////////////////////////////////////
