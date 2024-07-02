/*
 * ecc2.cc
 *
 *  Created on: Feb 9, 2023
 *      Author: amyznikov
 */
#include "ecc2.h"
#include <core/proc/downstrike.h>
#include <core/io/save_image.h>
#include <core/ssprintf.h>
#include <core/debug.h>

#if HAVE_TBB
# include <tbb/tbb.h>

# define ECC_USE_TBB 1

typedef tbb::blocked_range<int>
  tbb_range;

constexpr int tbb_block_size =
    256;

#endif



template<>
const c_enum_member * members_of<ECC_INTERPOLATION_METHOD>()
{
  static const c_enum_member members[] = {
      { ECC_INTER_LINEAR, "LINEAR", "" },
      { ECC_INTER_LINEAR_EXACT, "LINEAR_EXACT", "" },
      { ECC_INTER_AREA, "AREA", "" },
      { ECC_INTER_CUBIC, "CUBIC", "" },
      { ECC_INTER_LANCZOS4, "LANCZOS4", "" },
      { ECC_INTER_NEAREST, "NEAREST", "" },
#if CV_VERSION_CURRRENT >= CV_VERSION_INT(4,5,0)
      { ECC_INTER_NEAREST_EXACT, "NEAREST_EXACT", "" },
#endif
      { ECC_INTER_NEAREST }  // must be last
  };
  return members;
}

template<>
const c_enum_member * members_of<ECC_BORDER_MODE>()
{
  static const c_enum_member members[] = {
      { ECC_BORDER_REFLECT101, "BORDER_REFLECT101", },
      { ECC_BORDER_REFLECT, "BORDER_REFLECT", },
      { ECC_BORDER_REPLICATE, "BORDER_REPLICATE", },
      { ECC_BORDER_WRAP, "BORDER_WRAP", },
      { ECC_BORDER_CONSTANT, "BORDER_CONSTANT", },
      { ECC_BORDER_TRANSPARENT, "BORDER_TRANSPARENT", },
      { ECC_BORDER_ISOLATED, "BORDER_ISOLATED", },
      { ECC_BORDER_DEFAULT, }  // must be last
  };
  return members;
}

template<>
const c_enum_member * members_of<ECC_ALIGN_METHOD>()
{
  static const c_enum_member members[] = {
    {ECC_ALIGN_FORWARD_ADDITIVE, "FORWARD_ADDITIVE", },
    {ECC_ALIGN_LM, "LM", },
    {ECC_ALIGN_INVERSE_COMPOSITIONAL, "INVERSE_COMPOSITIONAL", "Requires invertible image transform"},
    {ECC_ALIGN_INVERSE_COMPOSITIONAL_LM, "INVERSE_COMPOSITIONAL_LM", },
    {ECC_ALIGN_LM},
  };
  return members;
}




double compute_correlation(cv::InputArray src1, cv::InputArray src2, cv::InputArray mask)
{
  const cv::Size size =
      src1.size();

  const int cn =
      src1.channels();

  cv::Mat img1, img2;
//  cv::Mat img1(size, CV_MAKETYPE(CV_32F, cn), cv::Scalar::all(0));
//  cv::Mat img2(size, CV_MAKETYPE(CV_32F, cn), cv::Scalar::all(0));

  cv::Scalar m1, m2, s1, s2;

  const int npix =
      mask.empty() ? size.area() :
          cv::countNonZero(mask);

  cv::meanStdDev(src1, m1, s1, mask);
  cv::meanStdDev(src2, m2, s2, mask);

  cv::subtract(src1, m1, img1, mask, CV_32F);
  cv::subtract(src2, m2, img2, mask, CV_32F);

  if ( !mask.empty() ) {
    img1.setTo(0, ~mask.getMat());
    img2.setTo(0, ~mask.getMat());
  }

  const double covar =
      img1.dot(img2) / (npix);

  double std1 = 0, std2 = 0;

  for( int i = 0; i < cn; ++i ) {
    std1 += s1[i];
    std2 += s2[i];
  }

  // CF_DEBUG("m1=%g m2=%g s1=%g s2=%g npix=%d covar=%g", m1[0], m2[0], s1[0], s2[0], npix, covar);

  return covar * cn / (std1 * std2);
}


double compute_correlation(cv::InputArray current_image, cv::InputArray current_mask, cv::InputArray reference_image, cv::InputArray reference_mask,  const cv::Mat2f & rmap)
{
  cv::Mat remapped_image, remapped_mask;

  cv::remap(current_image, remapped_image,
      rmap, cv::noArray(),
      cv::INTER_LINEAR,
      cv::BORDER_CONSTANT);

  if ( current_mask.empty() ) {

    cv::remap(cv::Mat1b(current_image.size(), (uint8_t) 255), remapped_mask,
        rmap, cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT);

  }
  else {

    cv::remap(current_mask, remapped_mask,
        rmap, cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT);
  }

  cv::compare(remapped_mask, 254, remapped_mask,
      cv::CMP_GE);

  if ( !reference_mask.empty() ) {
    cv::bitwise_and(reference_mask, remapped_mask,
        remapped_mask);
  }

  return compute_correlation(reference_image, remapped_image, remapped_mask);
}


namespace {

template<class T>
inline T square(T x)
{
  return x * x;
}

static void ecc_differentiate(cv::InputArray src, cv::Mat & gx, cv::Mat & gy, cv::InputArray mask = cv::noArray() )
{
  INSTRUMENT_REGION("");

  static thread_local cv::Mat Kx, Ky;
  if( Kx.empty() ) {
    cv::getDerivKernels(Kx, Ky, 1, 0, 3, true, CV_32F);
    Kx *= M_SQRT2;
    Ky *= M_SQRT2;
  }

  cv::sepFilter2D(src, gx, CV_32F, Kx, Ky, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::sepFilter2D(src, gy, CV_32F, Ky, Kx, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

  if( !mask.empty() ) {
    cv::Mat1b m;
    cv::bitwise_not(mask, m);
    gx.setTo(0, m);
    gy.setTo(0, m);
  }
}


static void ecc_remap(cv::InputArray _src, cv::OutputArray _dst, const cv::Mat2f & rmap, cv::BorderTypes borderType = cv::BORDER_REPLICATE)
{
#if 1 // !ECC_USE_TBB
  INSTRUMENT_REGION("cv::remap");
  cv::remap(_src.getMat(), _dst,
      rmap, cv::noArray(),
      cv::INTER_LINEAR,
      borderType);
#else

  if( _src.type() != CV_32FC1 ) {

    INSTRUMENT_REGION("cv::remap");

    cv::remap(_src.getMat(), _dst,
        rmap, cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT);
  }
  else {

    INSTRUMENT_REGION("ecc_remap");

    const cv::Mat1f src =
        _src.getMat();

    const cv::Size src_size =
        src.size();

    const cv::Size dst_size =
        rmap.size();

    _dst.create(dst_size, CV_32FC1);

    cv::Mat1f dst =
        _dst.getMatRef();

    tbb::parallel_for(tbb_range(0, dst_size.height),
        [&](const tbb_range & r) {

          const int w = src_size.width;
          const int h = src_size.height;

          for ( int dst_y = r.begin(); dst_y < r.end(); ++dst_y ) {

            float * dstp = dst[dst_y];

            const cv::Vec2f * m = rmap[dst_y];

            for ( int dst_x = 0; dst_x < dst_size.width; ++dst_x ) {

              const float x = m[dst_x][0];
              const float y = m[dst_x][1];

              const int x1 = std::min(w - 1, std::max(0, (int) x));
              const int y1 = std::min(h - 1, std::max(0, (int) y));

              const int x2 = x1 + 1;
              const int y2 = y1 + 1;

              const float dx1 = x - x1;
              const float dx2 = x2 - x;

              const float dy1 = y - y1;
              const float dy2 = y2 - y;

              const float & Q11 = src[y1][x1];
              const float & Q12 = src[std::min(y1 + 1, h - 1)][x1];
              const float & Q21 = src[y1][std::min(x1 + 1, w - 1)];
              const float & Q22 = src[std::min(y1 + 1, h - 1)][std::min(x1 + 1, w - 1)];

              dstp[dst_x] = dx1 * (dy2 * Q21 + dy1 * Q22) + dx2 * (dy2 * Q11 + dy1 * Q12);

            }
          }
        });
  }
#endif
}

static bool ecc_remap(const c_image_transform * image_transform,
    const cv::Mat1f & params,
    const cv::Size & size,
    cv::InputArray src, cv::InputArray src_mask,
    cv::OutputArray dst, cv::OutputArray dst_mask,
    cv::BorderTypes borderType = cv::BORDER_REPLICATE)
{
  cv::Mat2f rmap;

  if( !image_transform->create_remap(params, size, rmap) ) {
    CF_ERROR("image_transform->create_remap() fails");
    return false;
  }

  ecc_remap(src, dst, rmap, borderType);

  if( !src_mask.empty() ) {

    INSTRUMENT_REGION("src_mask");

    cv::remap(src_mask.getMat(), dst_mask,
        rmap, cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT,
        cv::Scalar(0));

  }
  else {

    INSTRUMENT_REGION("dummy_mask");

    cv::remap(cv::Mat1b(src.size(), (uint8_t) (255)), dst_mask,
        rmap, cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT,
        cv::Scalar(0));

  }

  if( true ) {

    INSTRUMENT_REGION("compare");

    cv::compare(dst_mask.getMat(), 250, dst_mask,
        cv::CMP_GE);
  }

  return true;
}


/**
 * Five-point approximation to first order image derivative along epipolar lines.
 *  <https://en.wikipedia.org/wiki/Numerical_differentiation>
 * */
static void ecc_epipolar_gradient(cv::InputArray src, cv::Mat1f & g, const cv::Point2f & e)
{
  static thread_local const cv::Matx<float, 1, 5> K(
      (+1.f / 12),
      (-8.f / 12),
      0.f,
      (+8.f / 12),
      (-1.f / 12));

  cv::Mat1f gx, gy;

  cv::filter2D(src, gx, CV_32F, K, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::filter2D(src, gy, CV_32F, K.t(), cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

  g.create(src.size());

#if ECC_USE_TBB
  tbb::parallel_for(tbb_range(0, g.rows, tbb_block_size),
      [&gx, &gy, &g, e](const tbb_range & r) {
        for ( int y = r.begin(); y < r.end(); ++y ) {
#else
        for ( int y = 0; y < g.rows; ++y ) {
#endif
          const float * gxp = gx[y];
          const float * gyp = gy[y];
          float * gp = g[y];

          for ( int x = 0; x < g.cols; ++x ) {

            const float dx = x - e.x;
            const float dy = y - e.y;
            const float dr = std::max(2 * FLT_EPSILON, std::sqrt(dx * dx + dy * dy));
            const float ca = dx / dr;
            const float sa = dy / dr;

            gp[x] = (gxp[x] * ca + gyp[x] * sa);
          }
        }
#if ECC_USE_TBB
      });
#endif

}

/*
 * Pyramid down to specific level
 */
bool ecc_downscale(cv::InputArray src, cv::Mat & dst, int level, int border_mode)
{
  cv::pyrDown(src, dst, cv::Size(), border_mode);

  for( int l = 1; l < level; ++l ) {
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

  if( inputSize != dstSize ) {

    std::vector<cv::Size> sizes;

    sizes.emplace_back(dstSize);

    while (42) {

      const cv::Size nextSize((sizes.back().width + 1) / 2,
          (sizes.back().height + 1) / 2);

      if( nextSize == inputSize ) {
        break;
      }

      if( nextSize.width < inputSize.width || nextSize.height < inputSize.height ) {
        CF_ERROR("FATAL: invalid next size : nextSize=%dx%d inputSize=%dx%d",
            nextSize.width, nextSize.height,
            inputSize.width, inputSize.height);
        return false;
      }

      sizes.emplace_back(nextSize);
    }

    for( int i = sizes.size() - 1; i >= 0; --i ) {
      cv::pyrUp(image, image, sizes[i]);
    }
  }

  return true;
}

//void gaussian_average(const cv::Mat & src, cv::Mat & dst, double delta)
//{
//  static constexpr double sigma = 2;
//  static constexpr int ksize = 2 * ((int) (sigma * 4)) + 1;
//  static const thread_local cv::Mat1f G = cv::getGaussianKernel(ksize, sigma, CV_32F);
//  cv::sepFilter2D(src, dst, -1, G, G.t(), cv::Point(-1, -1), delta, cv::BORDER_REPLICATE);
//}


/*
 * Estimate the standard deviation of the noise in a gray-scale image.
 *  J. Immerkr, Fast Noise Variance Estimation,
 *    Computer Vision and Image Understanding,
 *    Vol. 64, No. 2, pp. 300-302, Sep. 1996
 *
 * Matlab code:
 *  https://www.mathworks.com/matlabcentral/fileexchange/36941-fast-noise-estimation-in-images
 */
double ecc_estimate_image_noise(cv::InputArray src, cv::InputArray mask = cv::noArray())
{
  cv::Mat H, m;

  /* Compute sum of absolute values of special laplacian */

  // sqrt(M_PI_2) / 6.0
  constexpr double S = 0.20888568955258338;

  static thread_local float C[3 * 3] = {
      +1 * S, -2 * S, +1 * S,
      -2 * S, +4 * S, -2 * S,
      +1 * S, -2 * S, +1 * S
  };

  static thread_local cv::Mat1f K(3, 3, C);

  cv::filter2D(src, H,
  CV_32F,
      K,
      cv::Point(-1, -1),
      0,
      cv::BORDER_REPLICATE);

  cv::absdiff(H, 0, H);

  if( !mask.empty() ) {
    cv::dilate(~mask.getMat(), m, cv::Mat1b(3, 3, 255));
  }

  const cv::Scalar sigma =
      cv::mean(H, m);

  // Select max value over channels
  double max_noise = sigma[0];
  for( int i = 1, cn = src.channels(); i < cn; ++i ) {
    max_noise = std::max(max_noise, sigma[i]);
  }

  return max_noise;
}



/*
 * Create identity remap
 */
void ecc_create_identity_remap(cv::Mat2f & map, const cv::Size & size)
{
  map.create(size);

#if  ECC_USE_TBB
  tbb::parallel_for(tbb_range(0, map.rows, tbb_block_size),
      [&map](const tbb_range & r) {
        for ( int y = r.begin(); y < r.end(); ++y ) {
#else
        for ( int y = 0; y < map.rows; ++y ) {
#endif
          cv::Vec2f * m = map[y];
          for ( int x = 0; x < map.cols; ++x ) {
            m[x][0] = x;
            m[x][1] = y;
          }
        }
#if  ECC_USE_TBB
     });
#endif
}


/**
 * Compute Hessian matrix for ECC image alignment
 */
void ecc_compute_hessian_matrix(const std::vector<cv::Mat1f> & J, cv::Mat1f & H/*, int nparams*/)
{
  const int M =
      J.size();

  H.create(M, M);

#if  ECC_USE_TBB
  tbb::parallel_for(tbb_range(0, M),
      [&](const tbb_range & r) {
        for( int i = r.begin(); i < r.end(); ++i ) {
#else
        for( int i = 0; i < M; ++i ) {
#endif
          for( int j = 0; j <= i; ++j ) {
            H[i][j] = J[i].dot(J[j]);
          }
        }
#if  ECC_USE_TBB
        });
#endif

  for( int i = 0; i < M; ++i ) {
    for( int j = i + 1; j < M; ++j ) {
      H[i][j] = H[j][i];
    }
  }

}

/*
 * Project error image to jacobian for ECC image alignment
 * */
void ecc_project_error_image(const std::vector<cv::Mat1f> & J, const cv::Mat & rhs, cv::Mat1f & v)
{
  const int M =
      J.size();

  v.create(M, 1);

#if ECC_USE_TBB
  tbb::parallel_for(tbb_range(0, M),
      [&](const tbb_range & r) {
        for( int i = r.begin(); i < r.end(); ++i ) {
#else
        for( int i = 0; i < M; ++i ) {
#endif
          v[i][0] = J[i].dot(rhs);
        }
#if ECC_USE_TBB
      });
#endif
}

} // namespace



bool ecc_convert_input_image(cv::InputArray src, cv::InputArray src_mask,
    cv::Mat1f & dst, cv::Mat1b & dst_mask)
{
  if( !src_mask.empty() && (src_mask.size() != src.size() || src_mask.type() != CV_8UC1) ) {

    CF_ERROR("Invalid input mask: %dx%d %d channels depth=%d. Must be %dx%d CV_8UC1",
        src_mask.cols(), src_mask.rows(), src_mask.channels(), src_mask.depth(),
        src.cols(), src.rows());

    return false;
  }


  if( src.channels() == 1 ) {

    src.getMat().convertTo(dst,
        dst.depth());

  }
  else {
    cv::Mat tmp;

    cv::cvtColor(src, tmp,
        cv::COLOR_BGR2GRAY);

    if( tmp.depth() == dst.depth() ) {

      dst = tmp;
    }
    else {

      tmp.convertTo(dst,
          dst.depth());
    }
  }

  if( src_mask.empty() ) {
    dst_mask.release();
  }
  else {
    src_mask.copyTo(dst_mask);
  }

  return true;
}


void ecc_normalize_meanstdev(cv::InputArray _src, cv::InputArray _src_mask, cv::OutputArray dst, int lvl, double eps)
{
  const cv::Mat src =
      _src.getMat();

  const cv::Size src_size =
      src.size();

  cv::Size dst_size =
      src_size;

  for( int l = 0; l < lvl; ++l ) {

    const cv::Size next_size((dst_size.width + 1) / 2, (dst_size.height + 1) / 2);
    if( next_size.width < 4 || next_size.height < 4 ) {
      break;
    }

    dst_size =
        next_size;
  }

  cv::Mat m, s;

  cv::resize(src, m, dst_size, 0, 0, cv::INTER_AREA);
  cv::resize(src.mul(src), s, dst_size, 0, 0, cv::INTER_AREA);
  cv::add(s, eps, s);

  ecc_upscale(m, src_size);
  ecc_upscale(s, src_size);

  cv::subtract(src, m, dst, cv::noArray(), CV_32F);
  cv::divide(dst, s, dst);

  if ( !_src_mask.empty() ) {
    dst.setTo(0, ~_src_mask.getMat());
  }

}


/* Remap to Flow
 * */
void ecc_remap_to_optflow(const cv::Mat2f & rmap, cv::Mat2f & flow)
{
  typedef tbb::blocked_range<int> range;

  if( &flow == &rmap ) {

#if ECC_USE_TBB
    tbb::parallel_for(range(0, rmap.rows, 256),
        [&](const range & r) {
          for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
#else
          for ( int y = 0, ny = rmap.rows; y < ny; ++y ) {
#endif
            for( int x = 0; x < rmap.cols; ++x ) {
              flow[y][x][0] -= x;
              flow[y][x][1] -= y;
            }
          }
#if ECC_USE_TBB
        });
#endif

  }
  else if( flow.data != rmap.data ) {

    flow.create(rmap.size());

#if ECC_USE_TBB
    tbb::parallel_for(range(0, rmap.rows, 256),
        [&](const range & r) {
          for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
#else
          for ( int y = 0, ny = rmap.rows; y < ny; ++y ) {
#endif
            for( int x = 0; x < rmap.cols; ++x ) {
              flow[y][x][0] = rmap[y][x][0] - x;
              flow[y][x][1] = rmap[y][x][1] - y;
            }
          }
#if ECC_USE_TBB
        });
#endif
  }
  else {

    cv::Mat2f tmp(rmap.size());

#if ECC_USE_TBB
    tbb::parallel_for(range(0, rmap.rows, 256),
        [&](const range & r) {
          for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
#else
          for ( int y = 0, ny = rmap.rows; y < ny; ++y ) {
#endif
            for( int x = 0; x < rmap.cols; ++x ) {
              tmp[y][x][0] = rmap[y][x][0] - x;
              tmp[y][x][1] = rmap[y][x][1] - y;
            }
          }
#if ECC_USE_TBB
        });
#endif

    flow = std::move(tmp);
  }

}

/* Flow to Remap
 * */
void ecc_flow_to_remap(const cv::Mat2f & flow, cv::Mat2f & rmap)
{
  typedef tbb::blocked_range<int> range;

  if( &flow == &rmap ) {

    tbb::parallel_for(range(0, rmap.rows, 256),
        [&](const range & r) {
          for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
            for( int x = 0; x < rmap.cols; ++x ) {
              rmap[y][x][0] += x;
              rmap[y][x][1] += y;
            }
          }
        });

  }
  else if( flow.data != rmap.data ) {

    rmap.create(flow.size());

    tbb::parallel_for(range(0, rmap.rows, 256),
        [&](const range & r) {
          for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
            for( int x = 0; x < rmap.cols; ++x ) {
              rmap[y][x][0] = flow[y][x][0] + x;
              rmap[y][x][1] = flow[y][x][1] + y;
            }
          }
        });

  }
  else {

    cv::Mat2f tmp(flow.size());

    tbb::parallel_for(range(0, tmp.rows, 256),
        [&](const range & r) {
          for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
            for( int x = 0; x < tmp.cols; ++x ) {
              tmp[y][x][0] = flow[y][x][0] + x;
              tmp[y][x][1] = flow[y][x][1] + y;
            }
          }
        });

    rmap = std::move(tmp);
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////

c_ecc_align::c_ecc_align(c_image_transform * transform) :
    image_transform_(transform)
{
}

void c_ecc_align::set_image_transform(c_image_transform * image_transform)
{
  image_transform_ = image_transform;
}

c_image_transform * c_ecc_align::image_transform() const
{
  return image_transform_;
}

void c_ecc_align::set_max_iterations(int v)
{
  this->max_iterations_ = v;
}

int c_ecc_align::max_iterations() const
{
  return this->max_iterations_;
}

void c_ecc_align::set_max_eps(double v)
{
  this->max_eps_ = v;
}

double c_ecc_align::max_eps() const
{
  return this->max_eps_;
}

void c_ecc_align::set_interpolation(enum ECC_INTERPOLATION_METHOD  v)
{
  interpolation_ = v;
}

enum ECC_INTERPOLATION_METHOD c_ecc_align::interpolation() const
{
  return interpolation_;
}

void c_ecc_align::set_update_step_scale(double v)
{
  update_step_scale_ = v;
}

double c_ecc_align::update_step_scale() const
{
  return update_step_scale_;
}

void c_ecc_align::copy_parameters(const this_class & rhs)
{
  interpolation_ = rhs.interpolation_;
  num_iterations_  = rhs.num_iterations_;
  max_iterations_ = rhs.max_iterations_;
  update_step_scale_ = rhs.update_step_scale_;
  max_eps_ = rhs.max_eps_;
}


bool c_ecc_align::failed() const
{
  return this->failed_;
}

int c_ecc_align::num_iterations() const
{
  return this->num_iterations_;
}

double c_ecc_align::eps() const
{
  return eps_;
}

bool c_ecc_align::set_reference_image(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  if( !reference_mask.empty()
      && (reference_mask.size() != reference_image.size() || reference_mask.type() != CV_8UC1) ) {

    CF_ERROR("Invalid input mask: %dx%d %d channels depth=%d. Must be %dx%d CV_8UC1",
        reference_mask.cols(), reference_mask.rows(),  reference_mask.channels(), reference_mask.depth(),
        reference_image.cols(), reference_image.rows());

    return false;
  }

  if( reference_image.type() != CV_32FC1 ) {
    CF_ERROR("Invalid image type : %dx%d depth=%d channels=%d. Must be CV_32FC1 type",
        reference_image.cols(), reference_image.rows(),
        reference_image.depth(),
        reference_image.channels());
    return false;
  }

  reference_image.copyTo(reference_image_);
  reference_mask.copyTo(reference_mask_);

  return true;
}

bool c_ecc_align::set_current_image(cv::InputArray current_image, cv::InputArray current_mask)
{
  if( !current_mask.empty() && (current_mask.size() != current_image.size() || current_mask.type() != CV_8UC1) ) {

     CF_ERROR("Invalid input mask: %dx%d %d channels depth=%d. Must be %dx%d CV_8UC1",
         current_mask.cols(), current_mask.rows(), current_mask.channels(), current_mask.depth(),
         current_image.cols(), current_image.rows());

     return false;
   }

   if( current_image.type() != CV_32FC1 ) {
     CF_ERROR("Invalid image type : %dx%d depth=%d channels=%d. Must be CV_32FC1 type",
         current_image.cols(), current_image.rows(), current_image.depth(), current_image.channels());
     return false;
   }

   current_image.copyTo(current_image_);
   current_mask.copyTo(current_mask_);

   return true;
}

void c_ecc_align::release_current_image()
{
  current_image_.release();
  current_mask_.release();
}

const cv::Mat1f & c_ecc_align::reference_image() const
{
  return reference_image_;
}

const cv::Mat1b & c_ecc_align::reference_mask() const
{
  return reference_mask_;
}

const cv::Mat1f & c_ecc_align::current_image() const
{
  return current_image_;
}

const cv::Mat1b & c_ecc_align::current_mask() const
{
  return current_mask_;
}

bool c_ecc_align::align(cv::InputArray current_image, cv::InputArray reference_image,
    cv::InputArray current_mask, cv::InputArray reference_mask)
{
  if ( !set_reference_image(reference_image, reference_mask) ) {
    CF_ERROR("c_ecc_align: set_reference_image() fails");
    return false;
  }

  if ( !set_current_image(current_image, current_mask) ) {
    CF_ERROR("c_ecc_align: set_current_image() fails");
    return false;
  }

  return align();
}

bool c_ecc_align::align_to_reference(cv::InputArray current_image, cv::InputArray current_mask)
{
  if ( reference_image_.empty() ) {
    CF_ERROR("c_ecc_align: reference image was not set");
    return false;
  }

  if ( !set_current_image(current_image, current_mask) ) {
    CF_ERROR("c_ecc_align: set_current_image() fails");
    return false;
  }

  return align();
}

///////////////////////////////////////////////////////////////////////////////////////////////////


c_ecch::c_ecch( c_image_transform * image_transform) :
    image_transform_(image_transform)
{
}

c_ecch::c_ecch(ECC_ALIGN_METHOD method) :
    image_transform_(nullptr),
    method_(method)
{
}

c_ecch::c_ecch(c_image_transform * image_transform, ECC_ALIGN_METHOD method) :
    image_transform_(image_transform),
    method_(method)
{
}


void c_ecch::set_image_transform(c_image_transform * image_transform)
{
  if ( image_transform_ != image_transform ) {

    image_transform_ = image_transform;
    for ( const auto & m : pyramid_ ) {
      m->set_image_transform(image_transform);
    }
  }
}

c_image_transform * c_ecch::image_transform() const
{
  return image_transform_;
}

void c_ecch::set_method(ECC_ALIGN_METHOD v)
{
  if ( this->method_ != v ) {
    this->method_ = v;
    pyramid_.clear();
  }
}

ECC_ALIGN_METHOD c_ecch::method() const
{
  return method_;
}

void c_ecch::set_maxlevel(int v)
{
  maxlevel_ = v;
  pyramid_.clear();
}

int c_ecch::maxlevel() const
{
  return maxlevel_;
}

void c_ecch::set_minimum_image_size(int v)
{
  minimum_image_size_ = v;
  pyramid_.clear();
}

int c_ecch::minimum_image_size() const
{
  return minimum_image_size_;
}

void c_ecch::set_epsx(double v)
{
  epsx_ = v;
  for( const auto & m : pyramid_ ) {
    m->set_max_eps(v);
    v *= 2;
  }
}

double c_ecch::epsx() const
{
  return epsx_;
}

void c_ecch::set_max_iterations(int v)
{
  max_iterations_ = v;
  for( const auto & m : pyramid_ ) {
    m->set_max_iterations(v);
  }
}

int c_ecch::max_iterations() const
{
  return max_iterations_;
}

void c_ecch::set_max_eps(double v)
{
  set_epsx(v);
}

double c_ecch::max_eps() const
{
  return epsx_;
}

void c_ecch::set_min_rho(double v)
{
  min_rho_ = v;
}

double c_ecch::min_rho() const
{
  return min_rho_;
}

double c_ecch::eps() const
{
  return pyramid_.empty() ? -1 : pyramid_.front()->eps();
}

int c_ecch::num_iterations() const
{
  return num_iterations_;//  pyramid_.empty() ? -1 : pyramid_.front()->num_iterations();
}

void c_ecch::set_interpolation(enum ECC_INTERPOLATION_METHOD v)
{
  interpolation_ = v;
  for( const auto & m : pyramid_ ) {
    m->set_interpolation(v);
  }
}

enum ECC_INTERPOLATION_METHOD c_ecch::interpolation() const
{
  return interpolation_;
}

void c_ecch::set_input_smooth_sigma(double v)
{
  input_smooth_sigma_ = v;
}

double c_ecch::input_smooth_sigma() const
{
  return input_smooth_sigma_;
}

void c_ecch::set_reference_smooth_sigma(double v)
{
  reference_smooth_sigma_ = v;
}

double c_ecch::reference_smooth_sigma() const
{
  return reference_smooth_sigma_;
}

void c_ecch::set_update_step_scale(double v)
{
  update_step_scale_ = v;
}

double c_ecch::update_step_scale() const
{
  return update_step_scale_;
}

void c_ecch::copy_parameters(const this_class & rhs)
{
  epsx_ = rhs.epsx_;
  min_rho_ = rhs.min_rho_;
  reference_smooth_sigma_ = rhs.reference_smooth_sigma_;
  input_smooth_sigma_ = rhs.input_smooth_sigma_;
  update_step_scale_ = rhs.update_step_scale_;
  interpolation_ = rhs.interpolation_;
  max_iterations_ = rhs.max_iterations_;
  minimum_image_size_ = rhs.minimum_image_size_;
  maxlevel_ = rhs.maxlevel_;
}


const cv::Mat1f & c_ecch::reference_image() const
{
  if( pyramid_.empty() ) {
    static const cv::Mat1f empty_image;
    return empty_image;
  }
  return pyramid_.front()->reference_image();
}

const cv::Mat1b & c_ecch::reference_mask() const
{
  if( pyramid_.empty() ) {
    static const cv::Mat1b empty_image;
    return empty_image;
  }
  return pyramid_.front()->reference_mask();
}

const cv::Mat1f & c_ecch::current_image() const
{
  if( pyramid_.empty() ) {
    static const cv::Mat1f empty_image;
    return empty_image;
  }
  return pyramid_.front()->current_image();
}

const cv::Mat1b & c_ecch::current_mask() const
{
  if( pyramid_.empty() ) {
    static const cv::Mat1b empty_image;
    return empty_image;
  }
  return pyramid_.front()->current_mask();
}

cv::Mat2f c_ecch::create_remap() const
{
  cv::Mat2f rmap;
  if ( image_transform_ ) {
    const cv::Mat & rimage = reference_image();
    if ( !rimage.size().empty() ) {
      image_transform_->create_remap(rimage.size(), rmap);
    }
  }
  return rmap;
}

c_ecc_align::uptr c_ecch::create_ecc_align(double epsx) const
{
  c_ecc_align::uptr ecc;

  switch (method_) {
    case ECC_ALIGN_FORWARD_ADDITIVE:
      ecc.reset(new c_ecc_forward_additive());
      break;
    case ECC_ALIGN_INVERSE_COMPOSITIONAL:
      ecc.reset(new c_ecc_inverse_compositional());
      break;
    case ECC_ALIGN_INVERSE_COMPOSITIONAL_LM:
      ecc.reset(new c_ecclm_inverse_compositional());
      break;
    default:
      ecc.reset(new c_ecclm());
      break;
  }

  ecc->set_image_transform(image_transform_);
  ecc->set_interpolation(interpolation_);
  ecc->set_max_iterations(max_iterations_);
  ecc->set_update_step_scale(update_step_scale_);
  ecc->set_max_eps(epsx);

  return ecc;
}


bool c_ecch::set_reference_image(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  cv::Mat1f image;
  cv::Mat1b mask;

  pyramid_.clear();
  num_iterations_ = -1;

  if( !ecc_convert_input_image(reference_image, reference_mask, image, mask) ) {
    CF_ERROR("ecclm_convert_input_image() fails");
    return false;
  }

  if( reference_smooth_sigma_ > 0 ) {


    const int ksize =
        std::max(3, 2 * ((int) (3 * reference_smooth_sigma_)) + 1);

    const cv::Mat1f G =
        cv::getGaussianKernel(ksize,
            reference_smooth_sigma_);

    cv::sepFilter2D(image, image, -1,
        G, G,
        cv::Point(-1, -1),
        0,
        cv::BORDER_REPLICATE);
  }


  double epsx =
      epsx_;

  pyramid_.emplace_back(create_ecc_align(epsx));

  if( !pyramid_.back()->set_reference_image(image, mask) ) {
    CF_ERROR("pyramid_.back()->set_reference_image() fails");
    pyramid_.clear();
    return false;
  }

  const int min_image_size =
      std::max(4, this->minimum_image_size_);

  for( int lvl = 1; ; ++lvl ) {

    if( maxlevel_ >= 0 && lvl >= std::max(0, maxlevel_) ) {
      break;
    }

    const cv::Size previous_size =
        image.size();

    const cv::Size next_size((previous_size.width + 1) / 2,
        (previous_size.height + 1) / 2);

    // CF_DEBUG("next_size: %dx%d", next_size.width, next_size.height);

    if( next_size.width < min_image_size || next_size.height < min_image_size ) {
      break;
    }

    cv::pyrDown(image, image, next_size);

    if( !mask.empty() ) {
      cv::pyrDown(mask, mask, next_size);
      cv::compare(mask, 250, mask, cv::CMP_GE);
    }

    pyramid_.emplace_back(create_ecc_align(epsx *= 2));

    if( !pyramid_.back()->set_reference_image(image, mask) ) {
      CF_ERROR("L[%d] pyramid_.back()->set_reference_image() fails", lvl);
      pyramid_.clear();
      return false;
    }
  }


  return true;
}


bool c_ecch::set_current_image(cv::InputArray current_image, cv::InputArray current_mask)
{
  if( pyramid_.empty() ) {
    CF_ERROR("Reference image must be set first");
    return false;
  }

  cv::Mat1f image;
  cv::Mat1b mask;

  num_iterations_ = -1;

  if( !ecc_convert_input_image(current_image, current_mask, image, mask) ) {
    CF_ERROR("ecclm_convert_input_image() fails");
    return false;
  }

  if( input_smooth_sigma_ > 0 ) {

    const int ksize =
        std::max(3, 2 * ((int) (3 * input_smooth_sigma_)) + 1);

    const cv::Mat1f G =
        cv::getGaussianKernel(ksize,
            input_smooth_sigma_);

    cv::sepFilter2D(image, image, -1,
        G, G,
        cv::Point(-1, -1),
        0,
        cv::BORDER_REPLICATE);
  }

  const int lvls =
      pyramid_.size();

  int lvl = 0;

  for( ; lvl < lvls; ++lvl ) {

    if ( !pyramid_[lvl]->set_current_image(image, mask) ) {
      CF_ERROR("L[%d] pyramid_[lvl]->set_current_image() fails");
      return false;
    }

    if( lvl < lvls - 1 ) {

      const cv::Size previous_size =
          image.size();

      const cv::Size next_size((previous_size.width + 1) / 2,
          (previous_size.height + 1) / 2);

      if( next_size.width < 4 || next_size.height < 4 ) {
        break;
      }

      cv::pyrDown(image, image, next_size);

      if( !mask.empty() ) {
        cv::pyrDown(mask, mask, next_size);
        cv::compare(mask, 250, mask, cv::CMP_GE);
      }
    }
  }

  for( ; lvl < lvls; ++lvl ) {
    pyramid_[lvl]->release_current_image();
  }

  return true;
}


bool c_ecch::align(cv::InputArray current_image, cv::InputArray current_mask)
{
  if ( !set_current_image(current_image, current_mask) ) {
    CF_ERROR("c_ecch: set_current_image() fails");
    return false;
  }

  return align();
}

bool c_ecch::align()
{
  // CF_DEBUG("image_transform_=%p maxlevel_=%d", image_transform_, maxlevel_);

  if( pyramid_.empty() ) {
    CF_ERROR("c_ecch: no reference image was set");
    return false;
  }

  if( pyramid_.front()->current_image().empty() ) {
    CF_ERROR("c_ecch: no current_image image was set");
    return false;
  }

  const int lvls =
      pyramid_.size();

  int lvl = lvls - 1;
  while (lvl > 0 && pyramid_[lvl]->current_image().empty()) {
    --lvl;
  }

  if ( lvl > 0 ) {

    const cv::Size size0 =
        pyramid_[0]->reference_image().size();

    const cv::Size size1 =
        pyramid_[lvl]->reference_image().size();

    image_transform_->scale_transfrom((double) size1.width / (double) size0.width);
  }

  num_iterations_ = 0;

  for( ; lvl >= 0; --lvl ) {

    if( !pyramid_[lvl]->align() ) {
      CF_ERROR("pyramid_[lvl=%d]->align() fails", lvl);
    }
    else if( lvl > 0 ) {

      const cv::Size size0 =
          pyramid_[lvl]->reference_image().size();

      const cv::Size size1 =
          pyramid_[lvl - 1]->reference_image().size();

      image_transform_->scale_transfrom((double) size1.width / (double) size0.width);
    }

    num_iterations_ +=
        pyramid_[lvl]->num_iterations();
  }

  return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////


c_ecc_forward_additive::c_ecc_forward_additive(c_image_transform * image_transform) :
    base(image_transform)
{
}

bool c_ecc_forward_additive::set_reference_image(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  //jac.clear();

  if( !base::set_reference_image(reference_image, reference_mask) ) {
    CF_ERROR("base::set_reference_image() fails");
    return false;
  }

  if ( !reference_mask_.empty() ) {

    if ( cv::countNonZero(reference_mask_) == reference_mask_.size().area() ) {
      reference_mask_.release();
    }
    else { // may need to protect some border near mask edges because of differentiation

      cv::erode(reference_mask_, reference_mask_, cv::Mat1b(5, 5, 255),
          cv::Point(-1, -1), 1,
          cv::BORDER_REPLICATE);

    }
  }

  return true;
}


bool c_ecc_forward_additive::set_current_image(cv::InputArray current_image, cv::InputArray current_mask)
{

//  if ( !precompute_jac ) {
//    jac.clear();
//  }


  if( !base::set_current_image(current_image, current_mask) ) {
    CF_ERROR("base::set_current_image() fails");
    return false;
  }

  if( current_mask_.empty() ) {
    current_mask_.create(current_image_.size());
    current_mask_.setTo(255);
  }
  else if( cv::countNonZero(current_mask_) != current_mask_.size().area() ) {
    // may need to protect some border near mask edges because of differentiation

    cv::erode(current_mask_, current_mask_, cv::Mat1b(5, 5, 255),
        cv::Point(-1, -1), 1,
        cv::BORDER_REPLICATE);
  }

  return true;
}

bool c_ecc_forward_additive::align(cv::InputArray current_image, cv::InputArray reference_image,
    cv::InputArray current_mask, cv::InputArray reference_mask)
{
  return base::align(current_image, reference_image, current_mask, reference_mask);
}

bool c_ecc_forward_additive::align_to_reference(cv::InputArray current_image, cv::InputArray current_mask)
{
  return base::align_to_reference(current_image, current_mask);
}

bool c_ecc_forward_additive::align()
{
  INSTRUMENT_REGION("");

  failed_ = true;

  if ( reference_image_.empty() ) {
    CF_ERROR("c_ecc_forward_additive: reference image was not set");
    return false;
  }

  if ( current_image_.empty() ) {
    CF_ERROR("c_ecc_forward_additive: current image was not set");
    return false;
  }

  if( !image_transform_ ) {
    CF_ERROR("c_ecc_forward_additive: image transform was not set");
    return false;
  }

  failed_ = false;
  num_iterations_ = -1;
  //rho_ = -1;

  if( max_eps_ <= 0 ) {
    max_eps_ = 1e-3;
  }

  if( (nparams_ = image_transform_->parameters().rows) < 1 ) {
    CF_FATAL("image_transform_->image_transform_->parameters().rows return %d", nparams_);
    failed_ = true;
    return false;
  }

  if ( jac.size() != nparams_ ) {
    jac.resize(nparams_);
  }

  //
  // Iterate
  //

  cv::Scalar gMean, gStd, fMean, fStd;
  double stdev_ratio;
  cv::Mat2f current_remap;

  num_iterations_ = 0;
  while (num_iterations_++ < max_iterations_) {

    // Warp g, gx and gy with W(x; p) to compute warped input image g(W(x; p)) and it's gradients

    if( !image_transform_->create_remap(reference_image_.size(), current_remap) ) {
      CF_ERROR("[i %d] create_current_remap() fails", num_iterations_);
      failed_ = true;
      break;
    }

    tbb::parallel_invoke(
        [this, &current_remap]() {
          cv::remap(current_image_, gw, current_remap, cv::noArray(), interpolation_, cv::BORDER_REPLICATE);
          ecc_differentiate(gw, gxw, gyw);
        },
        [this, &current_remap]() {
          cv::remap(current_mask_, wmask, current_remap, cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
          cv::compare(wmask, 255, wmask, cv::CMP_GE);
          if( !reference_mask_.empty() ) {
            bitwise_and(wmask, reference_mask_, wmask);
          }
          cv::bitwise_not(wmask, iwmask);
        });


    gxw.setTo(0, iwmask);
    gyw.setTo(0, iwmask);

    // compute stdev ratio stdev(g)/stdev(f) and mean values
    cv::meanStdDev(reference_image_, fMean, fStd, wmask);
    cv::meanStdDev(gw, gMean, gStd, wmask);
    stdev_ratio = gStd[0] / fStd[0];

    // create steepest descent images
    image_transform_->create_steepest_descent_images(gxw, gyw, jac.data());
    ecc_compute_hessian_matrix(jac, H);
    if( !cv::invert(H, H, cv::DECOMP_CHOLESKY) ) {
      CF_ERROR("[i %d] cv::invert(H) fails", num_iterations_);
      failed_ = true;
      break;
    }

    // calculate Hessian and its inverse


    // compute update parameters
    // e = -(gwzm - stdev_ratio * fzm);
    cv::scaleAdd(reference_image_, -stdev_ratio, gw, rhs);
    cv::subtract(rhs, gMean - stdev_ratio * fMean, rhs);
    rhs.setTo(0, iwmask);

    // compute projected error
    ecc_project_error_image(jac, rhs, ep);

    // compute update parameters
    dp = -update_step_scale_ * (H * ep);

    // update warping matrix
    image_transform_->set_parameters(image_transform_->parameters() + dp);

    //eps_ = cv::norm(dp, cv::NORM_INF);
    eps_ = image_transform_->eps(dp, reference_image_.size());
    if( eps_ < max_eps_ ) {
      break;
    }
  }

//  CF_DEBUG("RET: num_iterations=%d eps_=%g ", num_iterations_, eps_);

  return !failed_; //  && rho_ > 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////


c_ecclm::c_ecclm(c_image_transform * image_transform) :
    base(image_transform)
{
}

void c_ecclm::set_image_transform(c_image_transform * image_transform)
{
  if ( image_transform != this->image_transform_ ) {
    J.clear();
    base::set_image_transform(image_transform);
  }
}

bool c_ecclm::set_reference_image(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  J.clear();

  if( !base::set_reference_image(reference_image, reference_mask) ) {
    CF_ERROR("base::set_reference_image() fails");
    return false;
  }

  if ( !reference_mask_.empty() ) {

    cv::erode(reference_mask_, reference_mask_,
        cv::Mat1b(5, 5, 255));
  }

  if ( !image_transform_ ) {
    // CF_DEBUG("Still wait for image transform");
    return true;
  }


  return true;
}

bool c_ecclm::set_current_image(cv::InputArray current_image, cv::InputArray current_mask)
{
  if ( !base::set_current_image(current_image, current_mask) ) {
    CF_ERROR("base::set_current_image() fails");
    return false;
  }

   return true;
}

bool c_ecclm::align(cv::InputArray current_image, cv::InputArray reference_image,
    cv::InputArray current_mask, cv::InputArray reference_mask)
{
  if ( !set_reference_image(reference_image, reference_mask) ) {
    CF_ERROR("set_reference_image() fails");
    return false;
  }

  if ( !set_current_image(current_image, current_mask) ) {
    CF_ERROR("set_current_image() fails");
    return false;
  }

  return align();
}

bool c_ecclm::align_to_reference(cv::InputArray current_image, cv::InputArray current_mask)
{
  if ( !set_current_image(current_image, current_mask) ) {
    CF_ERROR("set_current_image() fails");
    return false;
  }

  return align();
}

double c_ecclm::compute_remap(const cv::Mat1f & params,
    cv::Mat1f & remapped_image, cv::Mat1b & remapped_mask, cv::Mat1f & rhs)
{
  const int M =
      params.rows;

  const cv::Size size(reference_image_.size());

  ecc_remap(image_transform_, params, size,
      current_image_, current_mask_,
      remapped_image, remapped_mask,
      cv::BORDER_REPLICATE);

  if( remapped_mask.empty() ) {
    remapped_mask = reference_mask_;
  }
  else if( !reference_mask_.empty() ) {
    cv::bitwise_and(reference_mask_, remapped_mask,
        remapped_mask);
  }

  cv::subtract(remapped_image, reference_image_, rhs);
  if ( !remapped_mask.empty() ) {
    rhs.setTo(0, ~remapped_mask);
  }

  const double nrms =
      remapped_mask.empty() ? size.area() :
          cv::countNonZero(remapped_mask);

  // cv::multiply(rhs, 1./nrms, rhs);

  // CF_DEBUG("nrms= %g / %d", nrms, size.area());

  return nrms;

}

double c_ecclm::compute_rhs(const cv::Mat1f & params)
{
  //nrms =
      compute_remap(params, remapped_image, remapped_mask,
          rhs);

  rms =
      rhs.dot(rhs);// / nrms;

  return rms;
}

double c_ecclm::compute_jac(const cv::Mat1f & params, bool recompute_remap,
    cv::Mat1f & H, cv::Mat1f & v)
{

  if( recompute_remap ) {

    // nrms =
        compute_remap(params, remapped_image, remapped_mask,
            rhs);

    rms =
        rhs.dot(rhs);// / nrms;
  }

  const int M =
      params.rows;

  if( J.size() != M ) {
    J.resize(M);
  }

  ecc_differentiate(remapped_image, gx_, gy_, remapped_mask);
  //  gx_ /= nrms;
  //  gy_ /= nrms;

  image_transform_->create_steepest_descent_images(params, gx_, gy_, J.data());

  v.create(M, 1);

  tbb::parallel_for(tbb_range(0, M),
      [&](const tbb_range & r) {

        for( int i = r.begin(); i < r.end(); ++i ) {
          v[i][0] = J[i].dot(rhs);// / nrms;
        }
      });

  H.create(M, M);

  tbb::parallel_for(tbb_range(0, M),
      [&](const tbb_range & r) {

        for( int i = r.begin(); i < r.end(); ++i ) {
          for( int j = 0; j <= i; ++j ) {
            H[i][j] = J[i].dot(J[j]);// / nrms;
          }
        }
      });

  for( int i = 0; i < M; ++i ) {
    for( int j = i + 1; j < M; ++j ) {
      H[i][j] = H[j][i];
    }
  }

  return rms;
}


bool c_ecclm::align()
{
  INSTRUMENT_REGION("");

  if ( !image_transform_ ) {
    CF_ERROR("c_ecclm: image_transform_ is null");
    return false;
  }

  if ( reference_image_.empty() ) {
    CF_ERROR("c_ecclm: reference_image_ is empty");
    return false;
  }

  if ( current_image_.empty() ) {
    CF_ERROR("c_ecclm: current_image_ is empty");
    return false;
  }

  cv::Mat1f H, Hp, v, deltap, temp_d;
  cv::Mat1f params, newparams;

  params =
      image_transform_->parameters();

  const int M =
      params.rows;

  constexpr double eps =
      std::numeric_limits<double>::epsilon();

  double lambda = 0.01;
  double err = 0, newerr = 0, dp = 0;

  int iteration = 0;

//  CF_DEBUG("\n---------------------------------------------");

//  CF_DEBUG("initial parameters: T={\n"
//      "%+g %+g\n"
//      "}\n",
//      params[0][0],
//      params[1][0]
//      );

//  CF_DEBUG("initial parameters: T={\n"
//      "%+g %+g %+g\n"
//      "%+g %+g %+g\n"
//      "}\n",
//      params[0][0],
//      params[1][0],
//      params[2][0],
//      params[3][0],
//      params[4][0],
//      params[5][0]
//  );

//  CF_DEBUG("initial parameters: T={\n"
//      "%+g %+g %+g\n"
//      "%+g %+g %+g\n"
//      "%+g %+g\n"
//      "}\n",
//      params[0][0],
//      params[1][0],
//      params[2][0],
//      params[3][0],
//      params[4][0],
//      params[5][0],
//      params[6][0],
//      params[7][0]
//  );

  bool recompute_remap = true;

  J.clear();

  while (iteration < max_iterations_) {

    //CF_DEBUG("> IT %d model_->compute_jac()", iteration);

    err =
        compute_jac(params, recompute_remap, H, v);

//    CF_DEBUG("* JJ > IT %d lambda=%g err=%g\n",
//        iteration,
//        lambda,
//        err);

    H.copyTo(Hp);

    /*
     * Solve normal equation for given Jacobian and lambda
     * */
    do {

      ++iteration;
      recompute_remap = true;

      /*
       * Increase diagonal elements by lambda
       * */
      for( int i = 0; i < M; ++i ) {
        H[i][i] = (1 + lambda) * Hp[i][i];
      }

      /* Solve system to define delta and define new value of params */
      cv::solve(H, v, deltap, cv::DECOMP_CHOLESKY);
      cv::scaleAdd(deltap, -update_step_scale_, params, newparams);


//      CF_DEBUG("IT %d Compute function for newparams: \n"
//          "deltap = { \n"
//          "  %+20g %+20g\n"
//          "}\n"
//          "newparams = {\n"
//          "  %+20g %+20g\n"
//          "}"
//          "\n",
//          iteration,
//          deltap[0][0], deltap[1][0],
//          newparams[0][0], newparams[1][0]);

      /* Compute function for newparams */
      newerr =
          compute_rhs(newparams);

      /* Check for increments in parameters  */
      if ((dp = image_transform_->eps(deltap, reference_image_.size())) < max_eps_ ) {
        // CF_DEBUG("BREAK by eps= %g / %g ", dp, max_eps_);
        break;
      }

      /*
       * Compute update to lambda
       * */

      cv::gemm(Hp, deltap, -1, v, 2,
          temp_d);

      const double dS =
          deltap.dot(temp_d);

      const double rho =
          (err - newerr) / (std::abs(dS) > eps ? dS : 1);


//      CF_DEBUG("IT %d err=%g newerr=%g dp=%g lambda=%g rho=%+g\n",
//          iteration,
//          err, newerr,
//          dp,
//          lambda,
//          rho);


      if( rho > 0.25 ) {
        /* Accept new params and decrease lambda ==> Gauss-Newton method */
        if( lambda > 1e-6 ) {
          lambda = std::max(1e-6, lambda / 5);
        }
        // CF_DEBUG("  lambda->%g", lambda);
      }
      else if( rho > 0.1 ) {
        // CF_DEBUG(" NO CHANGE lambda->%g", lambda);

      }
      else if( lambda < 1 ) {       /** Try increase lambda ==> gradient descend */
        lambda = 1;
        // CF_DEBUG("  lambda->%g", lambda);
      }
      else {
        lambda *= 10;
        // CF_DEBUG("  lambda->%g", lambda);
      }

      if ( newerr < err ) {
        // CF_DEBUG("  ACCEPT");
        break;
      }

    } while (iteration < max_iterations_);

    if( newerr < err ) {
      /*
       * Accept new params if were not yet accepted
       * */
      err = newerr;
      recompute_remap = false;
      image_transform_->set_parameters(newparams);
      params = image_transform_->parameters();
    }

    if( dp < max_eps_ ) {
      // CF_DEBUG("BREAK2 by dp");
      break;
    }
  }


//  CF_DEBUG("newparams: T={%+g %+g}\n",
//      params[0][0],
//      params[1][0]);

  eps_ = dp;
  num_iterations_ = iteration;
//  dp = cv::norm(deltap, cv::NORM_INF);
//  CF_DEBUG("RET: iteration=%d err=%g eps_=%g dp=%g", iteration, err, eps_, dp);

//  CF_DEBUG("\n---------------------------------------------");
  return true;
}



///////////////////////////////////////////////////////////////////////////////////////////////////

c_ecc_inverse_compositional::c_ecc_inverse_compositional(c_image_transform * image_transform) :
    base(image_transform)
{
}

void c_ecc_inverse_compositional::set_image_transform(c_image_transform * image_transform)
{
  jac.clear();
  gx.release();
  gy.release();
  return base::set_image_transform(image_transform);
}

bool c_ecc_inverse_compositional::set_reference_image(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  jac.clear();
  gx.release();
  gy.release();
  return base::set_reference_image(reference_image, reference_mask);
}

bool c_ecc_inverse_compositional::set_current_image(cv::InputArray current_image, cv::InputArray current_mask )
{
  return base::set_current_image(current_image, current_mask);
}

bool c_ecc_inverse_compositional::align(cv::InputArray current_image, cv::InputArray reference_image,
    cv::InputArray current_mask, cv::InputArray reference_mask )
{
  return base::align(current_image, reference_image, current_mask, reference_mask);
}

bool c_ecc_inverse_compositional::align_to_reference(cv::InputArray current_image, cv::InputArray current_mask)
{
  return base::align_to_reference(current_image, current_mask);
}

bool c_ecc_inverse_compositional::align()
{
  failed_ = true;

  if ( !image_transform_ ) {
    CF_ERROR("c_ecc_inverse_compositional: image_transform_ is null");
    return false;
  }

  if ( !image_transform_->invertible() ) {
    CF_ERROR("c_ecc_inverse_compositional: image_transform_ is not invertible");
    return false;
  }


  if ( reference_image_.empty() ) {
    CF_ERROR("c_ecc_inverse_compositional: reference_image_ is empty");
    return false;
  }

  if ( current_image_.empty() ) {
    CF_ERROR("c_ecc_inverse_compositional: current_image_ is empty");
    return false;
  }

  cv::Mat1f params, newparams, deltap;
  cv::Mat1f remapped_image;
  cv::Mat1b remapped_mask;
  cv::Mat1f rhs;
  cv::Mat1f v;

  double rmsold, rmsnew;

  params =
      image_transform_->parameters();

  const int M =
      params.rows;

  const double RMA =
      reference_mask_.empty() ?
          reference_image_.size().area() :
          cv::countNonZero(reference_mask_);

  /**
   * PreCompute
   * */
  if( jac.size() != M || gx.empty() || gy.empty() ) {
    jac.resize(M);
    ecc_differentiate(reference_image_, gx, gy, reference_mask_);
    image_transform_->create_steepest_descent_images(gx, gy, jac.data());
    ecc_compute_hessian_matrix(jac, H);
  }

  num_iterations_ = 0;
  eps_ = FLT_MAX;
  failed_ = false;


  rmsold = FLT_MAX;

  const double lambda =
      update_step_scale_;

  while ( num_iterations_++ < max_iterations_ ) {

    params = image_transform_->parameters();

    ecc_remap(image_transform_, params, reference_image().size(),
        current_image_, current_mask_, remapped_image, remapped_mask, cv::BORDER_CONSTANT);

    cv::subtract(remapped_image, reference_image_, rhs);
    if ( !remapped_mask.empty() ) {
      rhs.setTo(0, ~remapped_mask);
    }

    const double CMA =
        remapped_mask.empty() ? rhs.size().area() :
            cv::countNonZero(remapped_mask);

#if ECC_USE_TBB
    tbb::parallel_invoke(
        [&] () {
#endif
          rmsnew = rhs.dot(rhs) * (RMA * RMA) / (CMA * CMA);
#if ECC_USE_TBB
        },
        [&]() {
#endif
          ecc_project_error_image(jac, rhs, v);
          cv::solve(H, v * (RMA / CMA), deltap, cv::DECOMP_CHOLESKY);
#if ECC_USE_TBB
        });
#endif

    if ( rmsnew >= rmsold ) {
      // CF_DEBUG("BREAK by rmsnew = %g rmsold = %g ", rmsnew, rmsold);
      break;
    }

    newparams =
        image_transform_->invert_and_compose(params,
            lambda * deltap);

//    CF_DEBUG("[i %d] CMA=%g rms=%g rmsnew=%g drms=%g\n"
//        "params = {\n"
//        "      %+20g %+20g %+20g\n"
//        "      %+20g %+20g %+20g\n"
//        "      %+20g %+20g \n"
//        "}\n"
//        "deltap = {\n"
//        "      %+20g %+20g %+20g\n"
//        "      %+20g %+20g %+20g\n"
//        "      %+20g %+20g \n"
//        "}\n"
//        "newparams = {\n"
//        "      %+20g %+20g %+20g\n"
//        "      %+20g %+20g %+20g\n"
//        "      %+20g %+20g \n"
//        "}\n"
//        "\n",
//        num_iterations_,
//        CMA, rmsold, rmsnew, rmsold - rmsnew,
//
//        params(0, 0), params(1, 0), params(2, 0),
//        params(3, 0), params(4, 0), params(5, 0),
//        params(6, 0), params(7, 0),
//
//        deltap(0, 0), deltap(1, 0), deltap(2, 0),
//        deltap(3, 0), deltap(4, 0), deltap(5, 0),
//        deltap(6, 0), deltap(7, 0),
//
//        newparams(0, 0), newparams(1, 0), newparams(2, 0),
//        newparams(3, 0), newparams(4, 0), newparams(5, 0),
//        newparams(6, 0), newparams(7, 0)
//        );
//

    rmsold = rmsnew;
    image_transform_->set_parameters(newparams);

    if ( (eps_ = image_transform_->eps(deltap, reference_image_.size())) < max_eps_ ) {
      // CF_DEBUG("BREAK by eps= %g / %g ", eps_, max_eps_);
      break;
    }

    //CF_DEBUG("[i %d] eps_= %g / %g", num_iterations_, eps_, max_eps_);
  }

  //CF_DEBUG("RET num_iterations_=%d / %d eps=%g / %g", num_iterations_, max_iterations_, eps_, max_eps_);

  return true;
}



///////////////////////////////////////////////////////////////////////////////////////////////////

c_ecclm_inverse_compositional::c_ecclm_inverse_compositional(c_image_transform * image_transform) :
    base(image_transform)
{
}

void c_ecclm_inverse_compositional::set_image_transform(c_image_transform * image_transform)
{
  return base::set_image_transform(image_transform);
}

bool c_ecclm_inverse_compositional::set_reference_image(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  return base::set_reference_image(reference_image, reference_mask);
}

bool c_ecclm_inverse_compositional::set_current_image(cv::InputArray current_image, cv::InputArray current_mask)
{
  return base::set_current_image(current_image, current_mask);
}

bool c_ecclm_inverse_compositional::align(cv::InputArray current_image, cv::InputArray reference_image,
    cv::InputArray current_mask, cv::InputArray reference_mask)
{
  return base::align(current_image, reference_image, current_mask, reference_mask);
}

bool c_ecclm_inverse_compositional::align_to_reference(cv::InputArray current_image, cv::InputArray current_mask)
{
  return base::align_to_reference(current_image, current_mask);
}

void c_ecclm_inverse_compositional::compute_remap(const cv::Mat1f & params,
    cv::Mat1f & remapped_image, cv::Mat1b & remapped_mask, cv::Mat1f & rhs)
{
  const int M =
      params.rows;

  const cv::Size size(reference_image_.size());

  ecc_remap(image_transform_, params, size,
      current_image_, current_mask_,
      remapped_image, remapped_mask,
      cv::BORDER_REPLICATE);

  if( remapped_mask.empty() ) {
    remapped_mask = reference_mask_;
  }
  else if( !reference_mask_.empty() ) {
    cv::bitwise_and(reference_mask_, remapped_mask,
        remapped_mask);
  }

  cv::subtract(remapped_image, reference_image_, rhs);
  if ( remapped_mask.empty() ) {
    CMA = rhs.size().area();
  }
  else {
    CMA = cv::countNonZero(remapped_mask);
    rhs.setTo(0, ~remapped_mask);
  }

  rms =
      rhs.dot(rhs) * (RMA * RMA) / (CMA * CMA);
}

double c_ecclm_inverse_compositional::compute_rhs(const cv::Mat1f & params)
{
  compute_remap(params, remapped_image, remapped_mask, rhs);
  return rms;
}

double c_ecclm_inverse_compositional::compute_v(const cv::Mat1f & params, bool recompute_remap, cv::Mat1f & v)
{
  if( recompute_remap ) {
    compute_remap(params, remapped_image, remapped_mask, rhs);
  }

  ecc_project_error_image(jac, rhs, v);
  v *= (RMA / CMA);

  return rms;
}

bool c_ecclm_inverse_compositional::align()
{
  failed_ = true;

  if ( !image_transform_ ) {
    CF_ERROR("c_ecc_inverse_compositional: image_transform_ is null");
    return false;
  }

  if ( !image_transform_->invertible() ) {
    CF_ERROR("c_ecc_inverse_compositional: image_transform_ is not invertible");
    return false;
  }

  if ( reference_image_.empty() ) {
    CF_ERROR("c_ecc_inverse_compositional: reference_image_ is empty");
    return false;
  }

  if ( current_image_.empty() ) {
    CF_ERROR("c_ecc_inverse_compositional: current_image_ is empty");
    return false;
  }

  cv::Mat1f params, newparams, deltap;
  cv::Mat1f H, temp_d, v;

  double err, newerr;

  params =
      image_transform_->parameters();

  const int M =
      params.rows;

  constexpr double eps =
      std::numeric_limits<double>::epsilon();

  double lambda = 0.001;
  double dp = 0;
  bool recompute_remap = true;

  RMA =
      reference_mask_.empty() ?
          reference_image_.size().area() :
          cv::countNonZero(reference_mask_);

//  CF_DEBUG("\n---------------------------------------------");

  /**
   * PreCompute
   * */
  if( jac.size() != M || gx.empty() || gy.empty() ) {
    jac.resize(M);
    ecc_differentiate(reference_image_, gx, gy, reference_mask_);
    image_transform_->create_steepest_descent_images(gx, gy, jac.data());
    ecc_compute_hessian_matrix(jac, Hp);
  }

  num_iterations_ = 0;
  eps_ = FLT_MAX;
  failed_ = false;


  while (num_iterations_ < max_iterations_) {

    //CF_DEBUG("> IT %d model_->compute_jac()", num_iterations_);

    // CF_DEBUG("recompute_remap=%d", recompute_remap);

    err = compute_v(params, recompute_remap, v);

    Hp.copyTo(H);

    /*
     * Solve normal equation for given Jacobian and lambda
     * */
    do {

      ++num_iterations_;
      recompute_remap = true;

      /*
       * Increase diagonal elements by lambda
       * */
      for( int i = 0; i < M; ++i ) {
        H[i][i] = (1 + lambda) * Hp[i][i];
      }

      /* Solve system to define delta and define new value of params */
      cv::solve(H, v, deltap, cv::DECOMP_CHOLESKY);

      newparams =
          image_transform_->invert_and_compose(params,
              update_step_scale_ * deltap);

//      CF_DEBUG("IT %d Compute function for newparams: \n"
//          "deltap = { \n"
//          "  %+20g %+20g\n"
//          "}\n"
//          "newparams = {\n"
//          "  %+20g %+20g\n"
//          "}"
//          "\n",
//          num_iterations_,
//          deltap[0][0], deltap[1][0],
//          newparams[0][0], newparams[1][0]);

      /* Compute function for newparams */
      newerr =
          compute_rhs(newparams);

      /* Check for increments in parameters  */
      if( (dp = image_transform_->eps(deltap, reference_image_.size())) < max_eps_ ) {
        // CF_DEBUG("BREAK by eps= %g / %g ", dp, max_eps_);
        break;
      }

      /*
       * Compute update to lambda
       * */

      cv::gemm(Hp, deltap, -1, v, 2,
          temp_d);

      const double dS =
          deltap.dot(temp_d);

      const double rho =
          (err - newerr) / (std::abs(dS) > eps ? dS : 1);


//      CF_DEBUG("IT %d err=%g newerr=%g dp=%g lambda=%g rho=%+g\n",
//          num_iterations_,
//          err, newerr,
//          dp,
//          lambda,
//          rho);


      if( rho > 0.25 ) {
        /* Accept new params and decrease lambda ==> Gauss-Newton method */
        if( lambda > 1e-6 ) {
          lambda = std::max(1e-6, lambda / 5);
        }
        // CF_DEBUG("  lambda->%g", lambda);
      }
      else if( rho > 0.1 ) {
        // CF_DEBUG(" NO CHANGE lambda->%g", lambda);

      }
      else if( lambda < 1 ) {       /** Try increase lambda ==> gradient descend */
        lambda = 1;
        // CF_DEBUG("  lambda->%g", lambda);
      }
      else {
        lambda *= 10;
        // CF_DEBUG("  lambda->%g", lambda);
      }

      if ( newerr < err ) {
        // CF_DEBUG("  ACCEPT");
        break;
      }

    } while (num_iterations_ < max_iterations_);

    if( newerr < err ) {
      /*
       * Accept new params if were not yet accepted
       * */
      err = newerr;
      recompute_remap = false;
      image_transform_->set_parameters(newparams);
      params = image_transform_->parameters();
    }

    if( dp < max_eps_ ) {
      // CF_DEBUG("BREAK2 by dp");
      break;
    }
  }


//  CF_DEBUG("newparams: T={%+g %+g}\n",
//      params[0][0],
//      params[1][0]);

  eps_ = dp;
  dp = cv::norm(deltap, cv::NORM_INF);
//  CF_DEBUG("RET: iteration=%d err=%g eps_=%g dp=%g", num_iterations_, err, eps_, dp);
//  CF_DEBUG("\n---------------------------------------------");
  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

template<>
const c_enum_member* members_of<c_eccflow::DownscaleMethod>()
{
  static const c_enum_member members[] = {
      { c_eccflow::DownscaleRecursiveResize, "RecursiveResize", "Recursive cv::resize() with scale factor" },
      { c_eccflow::DownscaleFullResize, "FullResize", "Direct cv::resize() from full to target resolution" },
      { c_eccflow::DownscalePyramid, "Pyramid", "Recursive resize using cv::pyrDown()" },
      { c_eccflow::DownscaleRecursiveResize },
  };

  return members;
}

void c_eccflow::set_support_scale(int v)
{
  support_scale_ = v;
}

int c_eccflow::support_scale() const
{
  return support_scale_;
}


void c_eccflow::set_max_iterations(int v)
{
  max_iterations_ = v;
}

int c_eccflow::max_iterations() const
{
  return max_iterations_;
}

void c_eccflow::set_update_multiplier(double v)
{
  update_multiplier_ = v;
}

double c_eccflow::update_multiplier() const
{
  return update_multiplier_;
}

void c_eccflow::set_input_smooth_sigma(double v)
{
  input_smooth_sigma_ = v;
}

double c_eccflow::input_smooth_sigma() const
{
  return input_smooth_sigma_;
}


void c_eccflow::set_reference_smooth_sigma(double v)
{
  reference_smooth_sigma_ = v;
}

double c_eccflow::reference_smooth_sigma() const
{
  return reference_smooth_sigma_;
}

void c_eccflow::set_downscale_method(DownscaleMethod v)
{
  downscale_method_ = v;
}

c_eccflow::DownscaleMethod c_eccflow::downscale_method() const
{
  return downscale_method_;
}

void c_eccflow::set_scale_factor(double v)
{
  scale_factor_ = v;
}

double c_eccflow::scale_factor() const
{
  return scale_factor_;
}

void c_eccflow::set_min_image_size(int v)
{
  min_image_size_ = v;
}

int c_eccflow::min_image_size() const
{
  return min_image_size_;
}

void c_eccflow::set_max_pyramid_level(int v)
{
  max_pyramid_level_ = v;
}

int c_eccflow::max_pyramid_level() const
{
  return max_pyramid_level_;
}

void c_eccflow::set_noise_level(double v)
{
  noise_level_ = v;
}

double c_eccflow::noise_level() const
{
  return noise_level_;
}

void c_eccflow::copy_parameters(const this_class & rhs)
{
  input_smooth_sigma_ = rhs.input_smooth_sigma_;
  reference_smooth_sigma_ = rhs.reference_smooth_sigma_;
  update_multiplier_ = rhs.update_multiplier_;
  noise_level_ = rhs.noise_level_;
  max_iterations_ = rhs.max_iterations_;
  support_scale_ = rhs.support_scale_;
  min_image_size_ = rhs.min_image_size_;
  scale_factor_ = rhs.scale_factor_;
  downscale_method_ = rhs.downscale_method_;
}

const cv::Mat2f& c_eccflow::current_uv() const
{
  return uv;
}

const std::vector<c_eccflow::pyramid_entry>& c_eccflow::current_pyramid() const
{
  return pyramid_;
}

bool c_eccflow::convert_input_images(cv::InputArray src, cv::InputArray src_mask,
    cv::Mat1f & dst, cv::Mat1b & dst_mask) const
{
  src.getMat().convertTo(dst, dst.depth());

  if ( src_mask.empty() || cv::countNonZero(src_mask) == src_mask.size().area() ) {
    dst_mask.release();
  }
  else {
    src_mask.getMat().copyTo(dst_mask);
  }

  return true;
}


bool c_eccflow::compute_uv(pyramid_entry & e, const cv::Mat2f & rmap, cv::Mat2f & uv) const
{
  tbb::parallel_invoke(
    [this, &e, &rmap]() {
      //e.reference_image.copyTo(W);
      cv::remap(e.current_image, W,
          rmap, cv::noArray(),
          cv::INTER_CUBIC,
          cv::BORDER_REPLICATE/*cv::BORDER_TRANSPARENT*/);
    },

    [this, &e, &rmap]() {
      if ( e.current_mask.empty() ) {
        M.release();
      }
      else {
        cv::remap(e.current_mask, M,
            rmap, cv::noArray(),
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

  const cv::Mat1f & I1 =
      W;

  const cv::Mat1f & I2 =
      e.reference_image;

  cv::subtract(I2, I1, It, M);

  tbb::parallel_invoke(

    [this, &e]() {
      avgp(e.Ix, It, Itx);
    },

    [this, &e]() {
      avgp(e.Iy, It, Ity);
    }
  );

  //  a00 = Ixx;
  //  a01 = Ixy;
  //  a10 = Ixy;
  //  a11 = Iyy;
  //  b0  = 2 * Itx;
  //  b1  = 2 * Ity;
  //  D = a00 * a11 - a10 * a01
  //  u = 1/D * (a11 * b0 - a01 * b1);
  //  v = 1/D * (a00 * b1 - a10 * b0);

  uv.create(e.D.size());

  typedef tbb::blocked_range<int> range;

  tbb::parallel_for(range(0, uv.rows, 256),
      [this, &e, &uv](const range & r) {

        const cv::Mat4f & D =
            e.D;

        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
          for ( int x = 0, nx = uv.cols; x < nx; ++x ) {
            const float & a00 = D[y][x][0];
            const float & a01 = D[y][x][1];
            const float & a10 = D[y][x][1];
            const float & a11 = D[y][x][2];
            const float & det = D[y][x][3];

            const float & b0 = Itx[y][x];
            const float & b1 = Ity[y][x];
            uv[y][x][0] = det * (a11 * b0 - a01 * b1);
            uv[y][x][1] = det * (a00 * b1 - a10 * b0);
          }
        }
      });

  avgup(uv, I1.size());
  if ( uv.size() != I1.size() ) {
    CF_ERROR("Invalid uv size: %dx%d must be %dx%d", uv.cols, uv.rows, I1.cols, I1.rows);
    return false;
  }

  return true;
}

void c_eccflow::avgdown(cv::InputArray src, cv::Mat & dst) const
{
  cv::Size size =
      src.size();

  for ( int i = 0; i < support_scale_; ++i ) {
    size.width = (size.width + 1) / 2;
    size.height = (size.height + 1) / 2;
  }

  cv::resize(src, dst, size, 0, 0, cv::INTER_AREA);

  static thread_local const cv::Mat G =
      cv::getGaussianKernel(3, 0, CV_32F);

  cv::sepFilter2D(dst, dst, -1, G, G, cv::Point(-1,-1), 0,
      cv::BORDER_REPLICATE);
}

void c_eccflow::avgup(cv::Mat & image, const cv::Size & dstSize) const
{
  ecc_upscale(image, dstSize);
}

void c_eccflow::avgp(cv::InputArray src1, cv::InputArray src2, cv::Mat & dst) const
{
  cv::multiply(src1, src2, dst);
  avgdown(dst, dst);
}


// TODO: meanshift segmentation of an image
// TODO: consider also compute matches for different feature scales in separate pyramids, then join;

void c_eccflow::downscale(cv::InputArray src, cv::InputArray src_mask,
    cv::OutputArray dst, cv::OutputArray dst_mask,
    const cv::Size & dst_size) const
{

  switch (downscale_method_) {
    case DownscalePyramid:
      cv::pyrDown(src, dst, dst_size);
      break;
    default:
      cv::resize(src, dst, dst_size, 0, 0,
          cv::INTER_AREA);
      break;
  }

  if( dst_mask.needed() ) {

    if( src_mask.empty() ) {
      dst_mask.release();
    }
    else {

      cv::resize(src_mask, dst_mask, dst.size(), 0, 0,
          cv::INTER_AREA);

      cv::compare(dst_mask.getMat(), cv::Scalar::all(250), dst_mask,
          cv::CMP_GE);
    }
  }
}

void c_eccflow::upscale(cv::InputArray src, cv::InputArray src_mask,
    cv::OutputArray dst, cv::OutputArray dst_mask,
    const cv::Size & dst_size) const
{
  cv::resize(src, dst, dst_size, 0, 0,
      cv::INTER_CUBIC);

  if( dst_mask.needed() ) {

    if( src_mask.empty() ) {
      dst_mask.release();
    }
    else {

      cv::resize(src_mask, dst_mask, dst.size(), 0, 0,
          cv::INTER_AREA);

      cv::compare(dst_mask.getMat(), cv::Scalar::all(250), dst_mask,
          cv::CMP_GE);
    }
  }
}

const cv::Mat1f & c_eccflow::reference_image() const
{
  static const cv::Mat1f empty_stub;
  return pyramid_.empty() ?  empty_stub : pyramid_.front().reference_image;
}

const cv::Mat1b & c_eccflow::reference_mask() const
{
  static const cv::Mat1b empty_stub;
  return pyramid_.empty() ? empty_stub : pyramid_.front().reference_mask;
}

bool c_eccflow::set_reference_image(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  INSTRUMENT_REGION("");

  if ( reference_image.channels() != 1 ) {
    CF_ERROR("Single channel input image is expected on input. reference_image.channels=%d",
        reference_image.channels());
    return false;
  }

  if ( !reference_mask.empty() ) {

    if ( reference_mask.size() != reference_image.size() ) {
      CF_ERROR("Invalid reference mask size: %dx%d. Must be is %dx%d",
          reference_mask.cols(), reference_mask.rows(),
          reference_image.cols(), reference_image.rows());

      return false;
    }

    if ( reference_mask.type() != CV_8UC1 ) {
      CF_ERROR("Invalid reference mask type: %d. Must be CV_8UC1",
          reference_mask.type());
      return false;
    }
  }

  const double noise_level =
      noise_level_ >= 0 ? noise_level_ : 1e-3;

  cv::Mat1f & Ixx = DC[0];
  cv::Mat1f & Ixy = DC[1];
  cv::Mat1f & Iyy = DC[2];
  cv::Mat1f & DD = DC[3];


  pyramid_.clear();
  pyramid_.reserve(32);

  const int min_image_size =
      std::max(4, min_image_size_);

  const cv::Size image_size =
      reference_image.size();

  const bool big_aspect_ratio =
      std::max(image_size.width, image_size.height) / std::min(image_size.width, image_size.height) >= 2;

  for( int current_level = 0; ; ++current_level ) {

    if ( current_level == 0 ) {

      pyramid_.emplace_back();

      convert_input_images(reference_image, reference_mask,
          pyramid_.back().reference_image,
          pyramid_.back().reference_mask);
    }
    else {

      const cv::Size previous_size =
          pyramid_.back().reference_image.size();

      if( downscale_method_ == DownscaleRecursiveResize ) {

        const cv::Size next_size(std::max(min_image_size_, (int) ((previous_size.width + 1) * scale_factor_)),
            std::max(min_image_size_, (int) ((previous_size.height + 1) * scale_factor_)));

        if( previous_size == next_size || std::max(next_size.width, next_size.height) <= min_image_size ) {
          break;
        }

        pyramid_.emplace_back();

        if( big_aspect_ratio && std::min(next_size.width, next_size.height) <= min_image_size + 1 ) {
          downscale(pyramid_.front().reference_image, pyramid_.front().reference_mask,
              pyramid_.back().reference_image, pyramid_.back().reference_mask,
              next_size);
        }
        else {
          downscale(pyramid_[current_level - 1].reference_image, pyramid_[current_level - 1].reference_mask,
              pyramid_.back().reference_image, pyramid_.back().reference_mask,
              next_size);
        }

      }
      else if( downscale_method_ == DownscaleFullResize ) {

        const cv::Size next_size(std::max(min_image_size_, (int) ((previous_size.width + 1) * scale_factor_)),
            std::max(min_image_size_, (int) ((previous_size.height + 1) * scale_factor_)));

        if( previous_size == next_size || std::max(next_size.width, next_size.height) <= min_image_size ) {
          break;
        }

        pyramid_.emplace_back();

        downscale(pyramid_.front().reference_image, pyramid_.front().reference_mask,
            pyramid_.back().reference_image, pyramid_.back().reference_mask,
            next_size);

      }
      else {  // DownscalePyramid

        const cv::Size next_size(std::max(min_image_size_, (previous_size.width + 1) / 2),
            std::max(min_image_size_, (previous_size.height + 1) / 2));

        if( previous_size == next_size || std::min(next_size.width, next_size.height) <= min_image_size ) {
          break;
        }

        pyramid_.emplace_back();

        downscale(pyramid_[current_level - 1].reference_image, pyramid_[current_level - 1].reference_mask,
            pyramid_.back().reference_image, pyramid_.back().reference_mask,
            next_size);

      }

    }

    pyramid_entry & current_scale =
        pyramid_.back();

    ecc_differentiate(current_scale.reference_image,
        current_scale.Ix, current_scale.Iy);

    if( false ) {
      avgp(current_scale.Ix, current_scale.Ix, Ixx);
      avgp(current_scale.Ix, current_scale.Iy, Ixy);
      avgp(current_scale.Iy, current_scale.Iy, Iyy);
    }
    else {
      tbb::parallel_invoke(
          [this, &current_scale, &Ixx]() {
            avgp(current_scale.Ix, current_scale.Ix, Ixx);
          },
          [this, &current_scale, &Ixy]() {
            avgp(current_scale.Ix, current_scale.Iy, Ixy);
          },
          [this, &current_scale, &Iyy]() {
            avgp(current_scale.Iy, current_scale.Iy, Iyy);
          }
       );
    }

    // FIXME: this regularization term estimation looks crazy
    const double RegularizationTerm =
        noise_level > 0 ? pow(1e-5 * noise_level / (1 << current_level), 4) : 0;

    cv::absdiff(Ixx.mul(Iyy), Ixy.mul(Ixy), DD);
    cv::add(DD, RegularizationTerm, DD);
    cv::divide(update_multiplier_, DD, DD);

    cv::merge(DC, 4, current_scale.D);

    if ( max_pyramid_level_ >= 0 && current_level >= max_pyramid_level_ ) {
      break;
    }

  }

  return true;
}

bool c_eccflow::setup_input_image(cv::InputArray input_image, cv::InputArray input_mask)
{
  INSTRUMENT_REGION("");

  if( pyramid_.empty() ) {
    CF_ERROR("Reference pyramid is empty: set_reference_image() must be called first");
    return false;
  }

  if ( input_image.channels() != 1 ) {
    CF_ERROR("Single channel input image is expected. input_image.channels=%d",
        input_image.channels());
    return false;
  }

  if( !input_mask.empty() ) {

    if( input_mask.size() != input_image.size() ) {
      CF_ERROR("Invalid input mask size: %dx%d. Must be is %dx%d",
          input_mask.cols(), input_mask.rows(),
          input_image.cols(), input_image.rows());

      return false;
    }

    if( input_mask.type() != CV_8UC1 ) {
      CF_ERROR("Invalid input mask type: %d. Must be CV_8UC1",
          input_mask.type());
      return false;
    }
  }

  const cv::Size image_size =
      input_image.size();

  const bool big_aspect_ratio =
      std::max(image_size.width, image_size.height) / std::min(image_size.width, image_size.height) >= 2;

  const int num_levels =
      (int)(pyramid_.size());

  for( int current_level = 0; current_level < num_levels; ++current_level ) {

    pyramid_entry & current_scale =
        pyramid_[current_level];

    if ( current_level == 0 ) {
      convert_input_images(input_image, input_mask,
          current_scale.current_image,
          current_scale.current_mask);
    }
    else if( downscale_method_ == DownscaleFullResize ) {

      const pyramid_entry & base_scale =
          pyramid_.front();

      downscale(base_scale.current_image, base_scale.current_mask,
          current_scale.current_image, current_scale.current_mask,
          current_scale.reference_image.size());
    }
    else if( downscale_method_ == DownscaleRecursiveResize ) {

      const cv::Size next_size =
          current_scale.reference_image.size();

      const int min_image_size =
          std::max(4, min_image_size_);

        if( big_aspect_ratio && std::min(next_size.width, next_size.height) <= min_image_size + 1 ) {

          const pyramid_entry & base_scale =
              pyramid_.front();

          downscale(base_scale.current_image, base_scale.current_mask,
              current_scale.current_image, current_scale.current_mask,
              current_scale.reference_image.size());
        }
        else {

          const pyramid_entry & previous_scale =
              pyramid_[current_level - 1];

          downscale(previous_scale.current_image, previous_scale.current_mask,
              current_scale.current_image, current_scale.current_mask,
              current_scale.reference_image.size());

        }
    }
    else {

      const pyramid_entry & previous_scale =
          pyramid_[current_level - 1];

      downscale(previous_scale.current_image, previous_scale.current_mask,
          current_scale.current_image, current_scale.current_mask,
          current_scale.reference_image.size());
    }

  }

  return true;
}



bool c_eccflow::compute(cv::InputArray input_image, cv::InputArray reference_image, cv::Mat2f & rmap,
    cv::InputArray input_mask, cv::InputArray reference_mask)
{
  INSTRUMENT_REGION("");

  set_reference_image(reference_image, reference_mask);
  return compute(input_image, rmap, input_mask);
}

bool c_eccflow::compute(cv::InputArray input_image, cv::Mat2f & rmap, cv::InputArray input_mask)
{
  cv::Mat2f cuv, crmap;

  if ( !setup_input_image(input_image, input_mask) ) {
    CF_ERROR("setup_input_image() fails");
    return false;
  }

  INSTRUMENT_REGION("");

  M.release();

  const int num_levels =
      (int) (pyramid_.size());

  if( rmap.empty() ) {

    uv.create(pyramid_.back().reference_image.size());
    uv.setTo(cv::Scalar::all(0));

  }
  else if( rmap.size() == pyramid_.front().reference_image.size() ) {

    const pyramid_entry & first_scale =
        pyramid_.front();

    const cv::Size first_size =
        first_scale.reference_image.size();

    const pyramid_entry & last_scale =
        pyramid_.back();

    const cv::Size last_size =
        last_scale.reference_image.size();

    const cv::Scalar size_ratio((double) last_size.width / (double) first_size.width,
        (double) last_size.height / (double) first_size.height);

    ecc_remap_to_optflow(rmap, uv);
    cv::resize(uv, uv, last_size, 0, 0, cv::INTER_AREA/*cv::INTER_CUBIC*/);
    cv::multiply(uv, size_ratio, uv);

  }
  else {
    CF_ERROR("Invalid args to c_eccflow::compute(): reference image and rmap sizes not match");
    return false;
  }

  /////////////////////////////////////

  for( int i = num_levels - 1; i >= 0; --i ) {

    pyramid_entry & current_scale =
        pyramid_[i];

    if( i < num_levels - 1 ) {

      const pyramid_entry & prev_scale =
          pyramid_[i + 1];

      const cv::Size current_size =
          current_scale.current_image.size();

      const cv::Size prev_size =
          prev_scale.current_image.size();

      const cv::Scalar size_ratio((double) current_size.width / (double) prev_size.width,
          (double) current_size.height / (double) prev_size.height);

      cv::multiply(uv, size_ratio, uv);
      upscale(uv, cv::noArray(), uv, cv::noArray(), current_size);
    }

    for( int j = 0; j < max_iterations_; ++j ) {

      ecc_flow_to_remap(uv, crmap);

      compute_uv(current_scale, crmap, cuv);

      cv::add(uv, cuv, uv);
    }

  }

  ecc_flow_to_remap(uv, rmap);

  return true;
}


