/*
 * c_ecclm.cc
 *
 *  Created on: Jun 20, 2024
 *      Author: amyznikov
 */

#include "c_ecclm.h"

#if HAVE_TBB
# include <tbb/tbb.h>


# define ECCLM_USE_TBB 1

typedef tbb::blocked_range<int>
  tbb_range;

constexpr int tbb_block_size =
    256;

#endif


#include <core/io/save_image.h>
#include <core/ssprintf.h>
#include <core/debug.h>



namespace {

static void ecclm_differentiate(cv::InputArray src, cv::Mat & gx, cv::Mat & gy, cv::InputArray mask = cv::noArray() )
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


static void ecclm_remap(cv::InputArray _src, cv::OutputArray _dst, const cv::Mat2f & rmap, cv::BorderTypes borderType = cv::BORDER_REPLICATE)
{
#if 1 // !ECCLM_USE_TBB
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

    INSTRUMENT_REGION("ecclm_remap");

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

static bool ecclm_remap(const c_image_transform * image_transform,
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

  ecclm_remap(src, dst, rmap, borderType);

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

} // namespace


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
    CF_DEBUG("Still wait for image transform");
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

  ecclm_remap(image_transform_, params, size,
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

  // CF_DEBUG("nrms= %g / %d", nrms, size.area());

  //  const double rms =
  //      rhs.dot(rhs);

  return nrms;

}

double c_ecclm::compute_rhs(const cv::Mat1f & params)
{
  nrms =
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

    nrms =
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

  ecclm_differentiate(remapped_image, gx_, gy_, remapped_mask);
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
  ++rc;

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

    // CF_DEBUG("> IT %d model_->compute_jac()", iteration);

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

      /*
       * Increase diagonal elements by lambda
       * */
      for( int i = 0; i < M; ++i ) {
        H[i][i] = (1 + lambda) * Hp[i][i];
      }

      /* Solve system to define delta and define new value of params */
      cv::solve(H, v, deltap, cv::DECOMP_EIG);
      cv::scaleAdd(deltap, -update_step_scale_, params, newparams);


//      CF_DEBUG("IT %d Compute function for newparams: \n"
//          "deltap = { \n"
//          "  %+20g %+20g %+20g \n"
//          "  %+20g %+20g %+20g \n"
//          "}\n"
//          "newparams = {\n"
//          "  %+20g %+20g %+20g\n"
//          "  %+20g %+20g %+20g\n"
//          "}"
//          "\n",
//          iteration,
//          deltap[0][0], deltap[1][0],deltap[2][0],deltap[3][0],deltap[4][0],deltap[4][0],
//          newparams[0][0], newparams[1][0], newparams[2][0], newparams[3][0], newparams[4][0], newparams[5][0]);

      /* Compute function for newparams */
      newerr =
          compute_rhs(newparams);

      /* Check for increments in parameters  */
      if( (dp = image_transform_->eps(deltap, reference_image_.size())) < max_eps_ ) {
        //CF_DEBUG("BREAK by dp=%g / %g", dp, max_eps_);
        break;
      }

//      if( (dp = cv::norm(deltap, cv::NORM_INF)) < max_eps_ ) {
//        break;
//      }


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
//          rho
//          );


      if( rho > 0.25 ) {
        /* Accept new params and decrease lambda ==> Gauss-Newton method */
        if( lambda > 1e-6 ) {
          lambda = std::max(1e-6, lambda / 5);
        }
        //CF_DEBUG("  lambda->%g", lambda);
      }
      else if( rho > 0.1 ) {
        //CF_DEBUG(" NO CHANGE lambda->%g", lambda);

      }
      else if( lambda < 1 ) {       /** Try increase lambda ==> gradient descend */
        lambda = 1;
        //CF_DEBUG("  lambda->%g", lambda);
      }
      else {
        lambda *= 10;
        //CF_DEBUG("  lambda->%g", lambda);
      }

      if ( newerr < err ) {
        //CF_DEBUG("  ACCEPT");
        break;
      }

    } while (iteration < max_iterations_);

    if( newerr < err ) {
      /*
       * Accept new params if were not yet accepted
       * */
      err = newerr;
      cv::swap(params, newparams);
      recompute_remap = false;
    }
    else {
      recompute_remap = true;
    }

    if( dp < max_eps_ ) {
      //CF_DEBUG("BREAK2 by dp");
      break;
    }
  }


//  CF_DEBUG("newparams: T={%+g %+g}\n",
//      newparams[0][0],
//      newparams[1][0]);

  if ( !newparams.empty() ) {
    image_transform_->set_parameters(newparams);
  }
  // rmse_ = sqrt(err / NM);


  eps_ = dp;
  dp = cv::norm(deltap, cv::NORM_INF);
  num_iterations_ = iteration + 1;
  //CF_DEBUG("RET: iteration=%d err=%g eps_=%g dp=%g", iteration, err, eps_, dp);

  return true;
}


/////////////////////////////////////////////////////////////////////////////////////////

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


 /////////////////////////////////////////////////////////////////////////////////////////
