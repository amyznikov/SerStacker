/*
 * ecc2.cc
 *
 *  Created on: Feb 9, 2023
 *      Author: amyznikov
 */
#include "ecc2.h"
#include <tbb/tbb.h>
#include <core/proc/downstrike.h>
#include <core/io/save_image.h>
#include <core/ssprintf.h>
#include <core/debug.h>




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


namespace {

typedef tbb::blocked_range<int> tbb_range;
constexpr int tbb_block_size = 512;


template<class T>
inline T square(T x)
{
  return x * x;
}

void ecc_differentiate(cv::InputArray src, cv::Mat & gx, cv::Mat & gy, int ddepth = CV_32F)
{
  static thread_local cv::Mat Kx, Ky;
  if( Kx.empty() ) {
    cv::getDerivKernels(Kx, Ky, 1, 0, 3, true, CV_32F);
    Kx *= M_SQRT2;
    Ky *= M_SQRT2;
  }

  if( ddepth < 0 ) {
    ddepth = std::max(src.depth(), CV_32F);
  }

  cv::sepFilter2D(src, gx, -1, Kx, Ky, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::sepFilter2D(src, gy, -1, Ky, Kx, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
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

  tbb::parallel_for(tbb_range(0, g.rows, tbb_block_size),
      [&gx, &gy, &g, e](const tbb_range & r) {

        for ( int y = r.begin(); y < r.end(); ++y ) {

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
      });

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

void gaussian_average(const cv::Mat & src, cv::Mat & dst, double delta)
{
  static constexpr double sigma = 2;
  static constexpr int ksize = 2 * ((int) (sigma * 4)) + 1;
  static const thread_local cv::Mat1f G = cv::getGaussianKernel(ksize, sigma, CV_32F);
  cv::sepFilter2D(src, dst, -1, G, G.t(), cv::Point(-1, -1), delta, cv::BORDER_REPLICATE);
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

  tbb::parallel_for(tbb_range(0, map.rows, tbb_block_size),
      [&map](const tbb_range & r) {
        for ( int y = r.begin(); y < r.end(); ++y ) {
          cv::Vec2f * m = map[y];
          for ( int x = 0; x < map.cols; ++x ) {
            m[x][0] = x;
            m[x][1] = y;
          }
        }
    });
}


/**
 * Compute Hessian matrix for ECC image alignment
 */
void ecc_compute_hessian_matrix(const cv::Mat1f & jac, cv::Mat1f & H, int nparams)
{
  if( jac.data == H.data ) {
    CF_FATAL("APP BUG: SRC and DST must be different objects, inplace not supported");
    exit(1);
  }

  const int h =
      jac.rows / nparams;

  H.create(nparams, nparams);

  for( int i = 0; i < nparams; i++ ) {

    const cv::Mat m(jac.rowRange(i * h, (i + 1) * h));

    const float nn = cv::norm(m);

    H[i][i] = nn * nn;  // diagonal elements

    for( int j = i + 1; j < nparams; j++ ) {
      H[i][j] = m.dot(jac.rowRange(j * h, (j + 1) * h));
      H[j][i] = H[i][j];
    }
  }
}

/*
 * Project error image to jacobian for ECC image alignment
 * */
void ecc_project_error_image(const cv::Mat1f & jac, const cv::Mat & error_image, cv::Mat1f & dst, int nparams)
{
  if( error_image.data == dst.data ) {
    CF_FATAL("APP BUG: SRC and DST must be different objects, inplace not supported");
    exit(1);
  }

  const int h =
      jac.rows / nparams;

  dst.create(nparams, 1);

  for( int i = 0; i < nparams; ++i ) {
    dst[i][0] = error_image.dot(jac.rowRange(i * h, (i + 1) * h));
  }
}

} // namespace


/* Remap to Flow
 * */
void ecc_remap_to_optflow(const cv::Mat2f & rmap, cv::Mat2f & flow)
{
  typedef tbb::blocked_range<int> range;

  if( &flow == &rmap ) {

    tbb::parallel_for(range(0, rmap.rows, 256),
        [&](const range & r) {
          for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
            for( int x = 0; x < rmap.cols; ++x ) {
              flow[y][x][0] -= x;
              flow[y][x][1] -= y;
            }
          }
        });

  }
  else if( flow.data != rmap.data ) {

    flow.create(rmap.size());

    tbb::parallel_for(range(0, rmap.rows, 256),
        [&](const range & r) {
          for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
            for( int x = 0; x < rmap.cols; ++x ) {
              flow[y][x][0] = rmap[y][x][0] - x;
              flow[y][x][1] = rmap[y][x][1] - y;
            }
          }
        });

  }
  else {

    cv::Mat2f tmp(rmap.size());

    tbb::parallel_for(range(0, rmap.rows, 256),
        [&](const range & r) {
          for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
            for( int x = 0; x < rmap.cols; ++x ) {
              tmp[y][x][0] = rmap[y][x][0] - x;
              tmp[y][x][1] = rmap[y][x][1] - y;
            }
          }
        });

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

/////////////////////////////////////////

c_ecc_align::c_ecc_align( c_ecc_motion_model * model) :
    model_(model)
{
}

void c_ecc_align::set_model(c_ecc_motion_model * model)
{
  model_ = model;
}

c_ecc_motion_model * c_ecc_align::model() const
{
  return model_;
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

void c_ecc_align::set_min_rho(double v)
{
  min_rho_ = v;
}

double c_ecc_align::min_rho() const
{
  return min_rho_;
}


void c_ecc_align::set_interpolation(enum ECC_INTERPOLATION_METHOD  v)
{
  interpolation_ = v;
}

enum ECC_INTERPOLATION_METHOD c_ecc_align::interpolation() const
{
  return interpolation_;
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

void c_ecc_align::copy_parameters(const this_class & rhs)
{
  interpolation_ = rhs.interpolation_;
  num_iterations_  = rhs.num_iterations_;
  max_iterations_ = rhs.max_iterations_;
  reference_smooth_sigma_ = rhs.reference_smooth_sigma_;
  input_smooth_sigma_ = rhs.input_smooth_sigma_;
  update_step_scale_ = rhs.update_step_scale_;
  max_eps_ = rhs.max_eps_;
  min_rho_ = rhs.min_rho_;
}


bool c_ecc_align::failed() const
{
  return this->failed_;
}

double c_ecc_align::rho() const
{
  return this->rho_;
}

int c_ecc_align::num_iterations() const
{
  return this->num_iterations_;
}

double c_ecc_align::eps() const
{
  return eps_;
}

const cv::Mat2f & c_ecc_align::current_remap() const
{
  return current_remap_;
}

bool c_ecc_align::create_current_remap(const cv::Size & size)
{
  if ( !model_ ) {
    CF_ERROR("ERROR: ECC motion model not set");
    return false;
  }

  if( !model_->create_remap(current_remap_, size) ) {
    CF_ERROR("model_->create_remap() fails");
    return false;
  }

  return true;
}


// convert image to 32-bit float with optional gaussian smoothing
void c_ecc_align::prepare_input_image(cv::InputArray src, cv::InputArray src_mask,
    double smooth_sigma, bool force_create_mask,
    cv::Mat1f & dst, cv::Mat1b & dst_mask) const
{
  INSTRUMENT_REGION("");

  if ( smooth_sigma <= 0 ) {
    src.getMat().convertTo(dst, dst.depth());
  }
  else {

    const cv::Mat1f G =
        cv::getGaussianKernel(2 * ((int) (4 * smooth_sigma)) + 1,
            smooth_sigma);

#if 1
    cv::sepFilter2D(src, dst, dst.depth(),
        G, G,
        cv::Point(-1, -1),
        0,
        cv::BORDER_REPLICATE);
#else
    if ( src_mask.empty() ) {

      cv::sepFilter2D(src, dst, dst.depth(),
          G, G,
          cv::Point(-1, -1),
          0,
          cv::BORDER_REPLICATE);

    }
    else {

      cv::Mat1f fmask;

      src_mask.getMat().convertTo(fmask,
          fmask.type(),
          1. / 255);

      cv::sepFilter2D(src, dst, dst.depth(),
          G, G,
          cv::Point(-1, -1),
          0,
          cv::BORDER_REPLICATE);

      cv::sepFilter2D(fmask, fmask, fmask.depth(),
          G, G,
          cv::Point(-1, -1),
          +1e-12,
          cv::BORDER_REPLICATE);

      cv::divide(dst, fmask, dst);
    }
#endif
  }


  if ( !src_mask.empty() ) {

    if ( cv::countNonZero(src_mask) == src_mask.size().area() ) {
      src_mask.copyTo(dst_mask);
    }
    else { // may need to protect some border near mask edges because of differentiation

      cv::erode(src_mask, dst_mask, cv::Mat1b(5, 5, 255),
          cv::Point(-1, -1), 1,
          cv::BORDER_REPLICATE);

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




/////////////////////////////////////////


c_ecc_forward_additive::c_ecc_forward_additive(c_ecc_motion_model * transfrom) :
    base(transfrom)
{
}

const cv::Mat1f & c_ecc_forward_additive::input_image() const
{
  return g;
}

const cv::Mat1f & c_ecc_forward_additive::reference_image() const
{
  return f;
}

const cv::Mat1b & c_ecc_forward_additive::input_mask() const
{
  return gmask;
}

const cv::Mat1b & c_ecc_forward_additive::reference_mask() const
{
  return fmask;
}

bool c_ecc_forward_additive::set_reference_image(cv::InputArray referenceImage, cv::InputArray referenceMask)
{
  INSTRUMENT_REGION("");

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
  prepare_input_image(referenceImage, referenceMask,
      reference_smooth_sigma_, false, f, fmask);

  return true;
}

bool c_ecc_forward_additive::align(cv::InputArray inputImage, cv::InputArray referenceImage,
    cv::InputArray inputMask, cv::InputArray referenceMask)
{
  INSTRUMENT_REGION("");

  num_iterations_ = -1, rho_ = -1;
  if ( !set_reference_image(referenceImage, referenceMask) ) {
    CF_FATAL("c_ecc_forward_additive: set_reference_image() fails");
    return false;
  }
  return align_to_reference(inputImage, inputMask);
}

bool c_ecc_forward_additive::align_to_reference(cv::InputArray inputImage, cv::InputArray inputMask)
{
  INSTRUMENT_REGION("");

  failed_ = false;

  if( !model_ ) {
    CF_ERROR("Pointer to image transformation interface was not set");
    failed_ = true;
    return false;
  }

  if( inputImage.channels() != 1 ) {
    CF_ERROR("input image must be single-channel (grayscale)");
    failed_ = true;
    return false;
  }

  if( !inputMask.empty() && (inputMask.type() != CV_8UC1 || inputMask.size() != inputImage.size()) ) {
    CF_ERROR("input mask must CV_8UC1 type of the same size as input image");
    failed_ = true;
    return false;
  }

  num_iterations_ = -1;
  rho_ = -1;
  if( max_eps_ <= 0 ) {
    max_eps_ = 1e-3;
  }

  if( (nparams_ = model_->num_adustable_parameters()) < 1 ) {
    CF_FATAL("model_->num_adustable_parameters() return %d", nparams_);
    failed_ = true;
    return false;
  }

  //
  // Precompute
  //

  // convert input image (to be aligned) to float and create the mask
  prepare_input_image(inputImage, inputMask, input_smooth_sigma_, true, g, gmask);

  // Evaluate the gradient ∇G of the input image G(x)
  ecc_differentiate(g, gx, gy);
  if( !inputMask.empty() ) {
    cv::bitwise_not(gmask, iwmask);
    gx.setTo(0, iwmask);
    gy.setTo(0, iwmask);
  }

  //
  // Iterate
  //

  cv::Scalar gMean, gStd, fMean, fStd;
  double stdev_ratio;

  for( num_iterations_ = 0; num_iterations_ <= max_iterations_; ++num_iterations_ ) {

    INSTRUMENT_REGION("iterate");

  // Warp g, gx and gy with W(x; p) to compute warped input image g(W(x; p)) and it's gradients
    if( !create_current_remap(f.size()) ) {
      CF_ERROR("[i %d] create_current_remap() fails", num_iterations_);
      failed_ = true;
      break;
    }

    cv::remap(g, gw, current_remap_, cv::noArray(), interpolation_, cv::BORDER_REPLICATE);
    cv::remap(gx, gxw, current_remap_, cv::noArray(), interpolation_, cv::BORDER_REPLICATE);
    cv::remap(gy, gyw, current_remap_, cv::noArray(), interpolation_, cv::BORDER_REPLICATE);
    cv::remap(gmask, wmask, current_remap_, cv::noArray(), cv::INTER_AREA, cv::BORDER_CONSTANT, 0);

    if( !fmask.empty() ) {
      bitwise_and(wmask, fmask, wmask);
    }

    cv::bitwise_not(wmask, iwmask);
    gxw.setTo(0, iwmask);
    gyw.setTo(0, iwmask);

    // compute stdev ratio stdev(g)/stdev(f) and mean values
    cv::meanStdDev(f, fMean, fStd, wmask);
    cv::meanStdDev(gw, gMean, gStd, wmask);
    stdev_ratio = gStd[0] / fStd[0];

    // create steepest descent images
    model_->create_steepest_descent_images(gxw, gyw, jac);

    // calculate Hessian and its inverse
    ecc_compute_hessian_matrix(jac, H, nparams_);

    if( !cv::invert(H, H, cv::DECOMP_CHOLESKY) ) {
      CF_ERROR("[i %d] cv::invert(H) fails", num_iterations_);
      failed_ = true;
      break;
    }

    // compute update parameters
    // e = -(gwzm - stdev_ratio * fzm);
    cv::scaleAdd(f, -stdev_ratio, gw, e);
    cv::subtract(e, gMean - stdev_ratio * fMean, e);
    e.setTo(0, iwmask);

    // compute projected error
    ecc_project_error_image(jac, e, ep, nparams_);

    // compute update parameters
    dp = -update_step_scale_ * (H * ep);

    // update warping matrix
    if( !model_->update_forward_additive(dp, &eps_, f.size()) ) {
      CF_ERROR("[i %d] model_->update_forward_additive() fails", num_iterations_);
      failed_ = true;
      break;
    }

    if( eps_ < max_eps_ || num_iterations_ == max_iterations_ ) {

      const int wmask_area =
          cv::countNonZero(wmask);

      if( wmask_area <= nparams_ ) {

        CF_ERROR("[i %d] Bad wmask area: nnz=%d / %d  < %d", num_iterations_,
            wmask_area, wmask.size().area(),
            nparams_ + 1);

        failed_ = 1;
      }
      else {
        cv::subtract(f, fMean, e), e.setTo(0, iwmask);
        cv::subtract(gw, gMean, gw), gw.setTo(0, iwmask);
        rho_ = e.dot(gw) / (wmask_area * fStd[0] * gStd[0]);

        if( isnan(rho_) ) {
          CF_ERROR("[i %d] e.dot() returns rho=%g. eps_=%g nnz(wmask)=%d / %d",
              num_iterations_,
              rho_,
              eps_,
              wmask_area,
              wmask.size().area());
        }
      }

      break;
    }
  }

  return !failed_ && rho_ > 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////

c_ecc_inverse_compositional::c_ecc_inverse_compositional(c_ecc_motion_model * transfrom) :
    base(transfrom)
{
}

const cv::Mat1f & c_ecc_inverse_compositional::input_image() const
{
  return g;
}

const cv::Mat1f & c_ecc_inverse_compositional::reference_image() const
{
  return f;
}

bool c_ecc_inverse_compositional::set_reference_image(cv::InputArray referenceImage, cv::InputArray referenceMask)
{
  // Convert reference image to floating point,
  // Pre-Evaluate the gradient ∇T of reference image T(x),
  // Pre-Compute Jacobian ∂W/∂p at (x,0) and steepest descent images ▽f*∂W/∂p

  prepare_input_image(referenceImage, referenceMask, reference_smooth_sigma_, false, f, fmask);

  ecc_differentiate(f, fx, fy);

  if( !model_->create_steepest_descent_images(fx, fy, jac_) ) {
    CF_ERROR("model_->compute_jacobian() fails");
    return false;
  }

  return true;
}

bool c_ecc_inverse_compositional::align(cv::InputArray inputImage, cv::InputArray referenceImage,
    cv::InputArray inputMask, cv::InputArray referenceMask)
{
  num_iterations_ = -1, rho_ = -1;
  if ( !set_reference_image(referenceImage, referenceMask) ) {
    CF_FATAL("c_ecc_inverse_compositional: set_reference_image() fails");
    return false;
  }
  return align_to_reference(inputImage, inputMask);
}

bool c_ecc_inverse_compositional::align_to_reference(cv::InputArray inputImage, cv::InputArray inputMask)
{
  cv::Mat1f jac;

  failed_ = false;
  num_iterations_ = -1;
  rho_ = -1;
  if ( max_eps_ <= 0 ) {
    max_eps_ = 1e-3;
  }


  if ( (nparams_ = model_->num_adustable_parameters()) < 1 ) {
    CF_ERROR("c_ecc2_inverse_compositional: model_->get_num_params() returns %d",
        nparams_);
    failed_ = true;
    return false;
  }

  // Convert input image (to be aligned) to floating point and create the mask
  prepare_input_image(inputImage, inputMask, input_smooth_sigma_, true, g, gmask);

  // Assume the initial warp matrix p is good enough to avoid the hessian matrix evaluation on each iteration
  if ( !create_current_remap(f.size()) ) {
    CF_ERROR("create_current_remap() fails");
    return false;
  }

  cv::remap(gmask, wmask, current_remap_, cv::noArray(),
      cv::INTER_AREA,
      cv::BORDER_CONSTANT,
      cv::Scalar(0));

  if ( !fmask.empty() ) {
    bitwise_and(wmask, fmask, wmask);
  }

  wmask.copyTo(fmask);
  cv::bitwise_not(wmask, iwmask);

  const double nnz0 =
      cv::countNonZero(wmask);

  jac_.copyTo(jac);

  for ( int i = 0; i < nparams_; ++i ) {
    jac.rowRange(i * f.rows, (i + 1) * f.rows).setTo(0, iwmask);
  }


  // Compute the Hessian matrix H = ∑x[▽f*∂W/∂p]^T*[▽f*∂W/∂p] and it's inverse
  ecc_compute_hessian_matrix(jac, H, nparams_);
  cv::invert(H, H, cv::DECOMP_CHOLESKY);
  H *= -update_step_scale_;

  //
  // Iterate
  //

  cv::Scalar gMean, gStd, fMean, fStd;
  double stdev_ratio, nnz1;

  for ( num_iterations_ = 0; num_iterations_ <= max_iterations_; ++num_iterations_ ) {

    // Warp g with W(x; p) to compute g(W(x; p))

    if ( num_iterations_ < 1 ) {
      nnz1 = nnz0;
    }
    else {

      if ( !create_current_remap(f.size()) ) {
        CF_ERROR("create_current_remap() fails");
        failed_ = true;
        break;
      }

      cv::remap(gmask, wmask, current_remap_, cv::noArray(),
          cv::INTER_AREA,
          cv::BORDER_CONSTANT,
          cv::Scalar(0));

      cv::bitwise_not(wmask, iwmask);

      nnz1 = cv::countNonZero(wmask);
    }

    // The cv::BORDER_REPLICATE is to avoid some annoying edge effects caused by interpolation
    cv::remap(g, gw, current_remap_, cv::noArray(),
        interpolation_,
        cv::BORDER_REPLICATE);

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
    ecc_project_error_image(jac, e, ep, nparams_);  // ep now contains projected error
    dp = (H * ep) * square(nnz0 / nnz1);

    //
    // update warping parameters
    //
    if ( !model_->update_inverse_composite(dp, &eps_, f.size()) ) {
      CF_ERROR("model_->update_inverse_composite() fails");
      failed_ = true;
      break;
    }

    // check eps
    if ( eps_ < max_eps_ || num_iterations_ == max_iterations_ || failed_ ) {
      cv::subtract(f, fMean, e), e.setTo(0, iwmask);
      cv::subtract(gw, gMean, gw), gw.setTo(0, iwmask);
      rho_ = e.dot(gw) / (nnz1 * fStd[0] * gStd[0]);
      break;
    }
  }

  return !failed_ && rho_ > min_rho_;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

c_ecch::c_ecch(c_ecc_align * method) :
    method_(method)
{
}

void c_ecch::set_method(c_ecc_align * method)
{
  method_ = method;
}

c_ecc_align * c_ecch::method() const
{
  return method_;
}

void c_ecch::set_minimum_image_size(int v)
{
  minimum_image_size_ = v;
}

int c_ecch::minimum_image_size() const
{
  return minimum_image_size_;
}

void c_ecch::set_minimum_pyramid_level(int v)
{
  minimum_pyramid_level_ = v;
}

int c_ecch::minimum_pyramid_level() const
{
  return minimum_pyramid_level_;
}

const std::vector<cv::Mat> & c_ecch::image_pyramid(int index) const
{
  return image_pyramids_[index];
}

const std::vector<cv::Mat> & c_ecch::mask_pyramid(int index) const
{
  return mask_pyramids_[index];
}

const std::vector<cv::Mat1f> & c_ecch::transform_pyramid() const
{
  return transform_pyramid_;
}

void c_ecch::copy_parameters(const this_class & rhs)
{
  minimum_image_size_ = rhs.minimum_image_size_;
  minimum_pyramid_level_ = rhs.minimum_pyramid_level_;
}

bool c_ecch::set_reference_image(cv::InputArray referenceImage, cv::InputArray referenceMask)
{
  INSTRUMENT_REGION("");

  if( referenceImage.channels() != 1 ) {
    CF_ERROR("c_ecch: Both input and reference images must be single-channel (grayscale)");
    return false;
  }

  // Build reference image pyramid
  constexpr int R = reference_image_index;

  transform_pyramid_.clear();
  transform_pyramid_.reserve(10);
  image_pyramids_[R].clear();
  image_pyramids_[R].reserve(10);
  mask_pyramids_[R].clear();
  mask_pyramids_[R].reserve(10);

  image_pyramids_[R].emplace_back(referenceImage.getMat());
  mask_pyramids_[R].emplace_back(referenceMask.getMat());

  // Build pyramid for reference image
  while (42) {

    const int currentMinSize =
        std::min(image_pyramids_[R].back().cols,
            image_pyramids_[R].back().rows);

    const int nextMinSize =
        currentMinSize / 2;

    if( nextMinSize < minimum_image_size_ ) {
      break;
    }

    image_pyramids_[R].emplace_back();

    cv::pyrDown(image_pyramids_[R][image_pyramids_[R].size() - 2],
        image_pyramids_[R].back());

    mask_pyramids_[R].emplace_back();

    cv::Mat & cmask =
        mask_pyramids_[R].back();

    cv::Mat & pmask =
        mask_pyramids_[R][mask_pyramids_[R].size() - 2];

    if( !pmask.empty() && cv::countNonZero(pmask) < pmask.size().area() ) {
      cv::pyrDown(pmask, cmask);
      cv::compare(cmask, 255, cmask, cv::CMP_GE);
    }
  }

  return true;
}

bool c_ecch::align(cv::InputArray inputImage, cv::InputArray inputMask)
{
  INSTRUMENT_REGION("");

  c_ecc_motion_model *model;

  constexpr int C = current_image_index;
  constexpr int R = reference_image_index;

  if( !method_ ) {
    CF_ERROR("c_ecch2: underlying align method not specified");
    return false;
  }

  if( !(model = method_->model()) ) {
    CF_ERROR("c_ecch2: image transform not specified");
    return false;
  }

  if( image_pyramids_[R].empty() ) {
    CF_ERROR("c_ecch: No reference image was set");
    return false;
  }

  if( inputImage.channels() != 1 ) {
    CF_ERROR("c_ecch2: Both input and reference images must be single-channel (grayscale)");
    return false;
  }

  cv::Mat1f T =
      model->parameters();

  if( T.empty() ) {
    CF_ERROR("c_ecch2: method_->transfrom()->parameters() returns empty matrix");
    return false;
  }

  // Build current image and initial transform pyramid

  image_pyramids_[C].clear();
  image_pyramids_[C].reserve(10);
  mask_pyramids_[C].clear();
  mask_pyramids_[C].reserve(10);
  transform_pyramid_.clear();
  transform_pyramid_.reserve(10);

  image_pyramids_[C].emplace_back(inputImage.getMat());
  mask_pyramids_[C].emplace_back(inputMask.getMat());
  transform_pyramid_.emplace_back(T);

  while ( 42 ) {

    int I = image_pyramids_[C].size() - 1;

    const int currentMinSize = std::min(std::min(std::min(
        image_pyramids_[C][I].cols, image_pyramids_[C][I].rows),
        image_pyramids_[R][I].cols), image_pyramids_[R][I].rows);

    const int nextMinSize = currentMinSize / 2;

    if( nextMinSize < minimum_image_size_ ) {
      break;
    }

    image_pyramids_[C].emplace_back();
    cv::pyrDown(image_pyramids_[C][image_pyramids_[C].size() - 2],
        image_pyramids_[C].back());

    mask_pyramids_[C].emplace_back();

    cv::Mat & cmask =
        mask_pyramids_[C].back();

    cv::Mat & pmask =
        mask_pyramids_[C][mask_pyramids_[C].size() - 2];

    if( !pmask.empty() && cv::countNonZero(pmask) < pmask.size().area() ) {
      cv::pyrDown(pmask, cmask);
      cv::compare(cmask, 255, cmask, cv::CMP_GE);
    }

    transform_pyramid_.emplace_back(model->scale_transfrom(transform_pyramid_.back(), 0.5));
    if( transform_pyramid_.back().empty() ) {
      CF_ERROR("transform->scale() fails");
      return false;
    }
  }

  // Align pyramid images in coarse-to-fine direction
  bool eccOk = false;

  const int min_level =
      std::max(0, minimum_pyramid_level_);

  for( int i = transform_pyramid_.size() - 1; i >= 0; --i ) {

    if( !model->set_parameters(transform_pyramid_[i]) ) {
      CF_ERROR("L[%d] transform->set_parameters() fails", i);
      return false;
    }

    if( i >= min_level ) {

      bool fOk =
          method_->align(image_pyramids_[C][i],
              image_pyramids_[R][i],
              mask_pyramids_[C][i],
              mask_pyramids_[R][i]);

      if( !fOk ) {

        CF_DEBUG("L[%2d/%zu] : align fails: size=%dx%d num_iterations=%d rho=%g eps=%g", i,
            transform_pyramid_.size(),
            image_pyramids_[C][i].cols, image_pyramids_[C][i].rows,
            method_->num_iterations(), method_->rho(), method_->eps());

        continue;
      }
    }

    if( i == 0 ) {

      if( min_level > 0 && !method_->create_current_remap(image_pyramids_[R][i].size()) ) {
        CF_ERROR("method_->create_current_remap() fails");
      }
      else {
        eccOk = true;
      }

      break;
    }

    // i > 0
    if( (transform_pyramid_[i - 1] = model->scale_transfrom(model->parameters(), 2)).empty() ) {
      CF_ERROR("L[%d] transform->scale() fails", i);
      return false;
    }
  }

  return eccOk;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////

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

//
// Use for images which violate brightness constancy assumption,
// for example on strong vignetting or planetary disk derotation
void c_eccflow::set_normalization_scale(int v)
{
  normalization_scale_ = v;
}

int c_eccflow::normalization_scale() const
{
  return normalization_scale_;
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
  normalization_scale_ = rhs.normalization_scale_;
  min_image_size_ = rhs.min_image_size_;
  scale_factor_ = rhs.scale_factor_;
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
          cv::INTER_AREA,
          cv::BORDER_REPLICATE/*cv::BORDER_TRANSPARENT*/);
    },

    [this, &e, &rmap]() {
      if ( !e.current_mask.empty() ) {
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
  cv::resize(src, dst, dst_size, 0, 0,
      cv::INTER_AREA);

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

#define TEST_DOWNSCALE  1

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

//  cv::Mat1f DC[4];
//  = {
//      Ixx,
//      Ixy,
//      Iyy,
//      DD
//  };
//
  cv::Mat1f & Ixx = DC[0];
  cv::Mat1f & Ixy = DC[1];
  cv::Mat1f & Iyy = DC[2];
  cv::Mat1f & DD = DC[3];


  pyramid_.clear();
  pyramid_.reserve(32);

  const int min_image_size =
      std::max(4, min_image_size_);

  for( int current_level = 0;; ++current_level ) {

    if ( current_level == 0 ) {
      pyramid_.emplace_back();
      convert_input_images(reference_image, reference_mask,
          pyramid_.back().reference_image,
          pyramid_.back().reference_mask);
    }
    else {

      const cv::Size previous_size =
          pyramid_.back().reference_image.size();

      const cv::Size next_size(std::max(min_image_size_, (int)((previous_size.width + 1) * scale_factor_)),
          std::max(min_image_size_, (int)((previous_size.height + 1) * scale_factor_)) );

      if( previous_size == next_size || std::max(next_size.width, next_size.height) <= min_image_size ) {
        break;
      }

      pyramid_.emplace_back();

#if TEST_DOWNSCALE
      downscale(pyramid_[current_level - 1].reference_image, pyramid_[current_level - 1].reference_mask,
          pyramid_.back().reference_image, pyramid_.back().reference_mask,
          next_size);
#else
      downscale(pyramid_.front().reference_image, pyramid_.front().reference_mask,
          pyramid_.back().reference_image, pyramid_.back().reference_mask,
          next_size);
#endif
    }

    pyramid_entry & current_scale =
        pyramid_.back();

    if ( true ) {
      INSTRUMENT_REGION("ecc_differentiate");
      ecc_differentiate(current_scale.reference_image,
          current_scale.Ix, current_scale.Iy);
    }

    if( true ) {
      INSTRUMENT_REGION("avgp");

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
    }

    if( true ) {
      INSTRUMENT_REGION("merge_D");

      // FIXME: this regularization term estimation seems crazy
      const double RegularizationTerm =
          noise_level > 0 ? pow(1e-5 * noise_level / (1 << current_level), 4) : 0;

      cv::absdiff(Ixx.mul(Iyy), Ixy.mul(Ixy), DD);
      cv::add(DD, RegularizationTerm, DD);
      cv::divide(update_multiplier_, DD, DD);

      cv::merge(DC, 4, current_scale.D);
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
    else {

      const pyramid_entry & base_scale =
          pyramid_.front();

       const pyramid_entry & previous_scale =
           pyramid_[current_level - 1];

#if TEST_DOWNSCALE
      downscale(previous_scale.current_image, previous_scale.current_mask,
          current_scale.current_image, current_scale.current_mask,
          current_scale.reference_image.size());
#else
      downscale(base_scale.current_image, base_scale.current_mask,
          current_scale.current_image, current_scale.current_mask,
          current_scale.reference_image.size());
#endif
    }
  }

  return true;
}



bool c_eccflow::compute(cv::InputArray inputImage, cv::InputArray referenceImage, cv::Mat2f & rmap,
    cv::InputArray inputMask, cv::InputArray referenceMask)
{
  INSTRUMENT_REGION("");

  set_reference_image(referenceImage, referenceMask);
  return compute(inputImage, rmap, inputMask);
}

bool c_eccflow::compute(cv::InputArray inputImage, cv::Mat2f & rmap, cv::InputArray inputMask)
{
  cv::Mat2f cuv, crmap;

  if ( !setup_input_image(inputImage, inputMask) ) {
    CF_ERROR("setup_input_image() fails");
    return false;
  }

  INSTRUMENT_REGION("");

  const int num_levels =
      (int) (pyramid_.size());

  if( rmap.empty() ) {

    uv.create(pyramid_.back().reference_image.size());
    uv.setTo(cv::Scalar::all(0));

  }
  else if( rmap.size() == pyramid_.front().reference_image.size() ) {

    const pyramid_entry & first_scale =
        pyramid_.front();

    const pyramid_entry & last_scale =
        pyramid_.back();

    const cv::Size first_size =
        first_scale.reference_image.size();

    const cv::Size last_size =
        last_scale.reference_image.size();

    const cv::Scalar size_ratio((double) last_size.width / (double) first_size.width,
        (double) last_size.height / (double) first_size.height);

    ecc_remap_to_optflow(rmap, uv);
    downscale(uv, cv::noArray(), uv, cv::noArray(), last_size);
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


