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


#include <core/debug.h>



namespace {

static void ecclm_differentiate(cv::InputArray src, cv::Mat & gx, cv::Mat & gy, int ddepth = CV_32F)
{
  INSTRUMENT_REGION("");

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


static void ecclm_remap(cv::InputArray _src, cv::OutputArray _dst, const cv::Mat2f & rmap)
{
#if !ECCLM_USE_TBB
  INSTRUMENT_REGION("cv::remap");
  cv::remap(_src, _dst,
      rmap, cv::noArray(),
      cv::INTER_LINEAR,
      cv::BORDER_CONSTANT);
#else

  if( _src.type() != CV_32FC1 ) {

    INSTRUMENT_REGION("cv::remap");

    cv::remap(_src, _dst,
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

            for ( int dst_x = 0; dst_x < w; ++dst_x ) {

              const float & x = m[dst_x][0];
              const float & y = m[dst_x][1];

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

              dstp[dst_x] =
              dx1 * (dy2 * Q21 + dy1 * Q22) +
              dx2 * (dy2 * Q11 + dy1 * Q12);

            }
          }
        });
  }
#endif
}

static bool ecclm_remap(const c_image_transform * image_transform,
    const cv::Mat1f & params,
    cv::InputArray src, cv::InputArray src_mask,
    cv::OutputArray dst, cv::OutputArray dst_mask)
{
  const cv::Size size =
      src.size();

  cv::Mat2f rmap;

  if( !image_transform->create_remap(params, size, rmap) ) {
    CF_ERROR("image_transform->create_remap() fails");
    return false;
  }

  ecclm_remap(src, dst, rmap);

  if( !src_mask.empty() ) {

    INSTRUMENT_REGION("src_mask");

    cv::remap(src_mask, dst_mask,
        rmap, cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT,
        cv::Scalar(0));

  }
  else {

    INSTRUMENT_REGION("dummy_mask");

    cv::remap(cv::Mat1b(size, (uint8_t) (255)), dst_mask,
        rmap, cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT,
        cv::Scalar(0));

  }

  if( true ) {

    INSTRUMENT_REGION("compare");

    cv::compare(dst_mask, 254, dst_mask,
        cv::CMP_GE);
  }

  return true;
}

} // namespace

c_ecclm::c_ecclm()
{
}

c_ecclm::c_ecclm(c_image_transform * image_transform) :
    image_transform_(image_transform)
{
}


void c_ecclm::set_image_transform(c_image_transform * image_transform)
{
  image_transform_ = image_transform;
}

c_image_transform * c_ecclm::image_transform() const
{
  return image_transform_;
}

void c_ecclm::set_max_iterations(int v)
{
  max_iterations_ = v;
}

int c_ecclm::max_iterations() const
{
  return max_iterations_;
}

void c_ecclm::set_update_step_scale(double v)
{
  update_step_scale_ = v;
}

double c_ecclm::update_step_scale() const
{
  return update_step_scale_;
}

void c_ecclm::set_epsx(double v)
{
  epsx_ = v;
}

double c_ecclm::epsx() const
{
  return epsx_;
}

//void c_ecclm::set_epsfn(double v)
//{
//  epsfn_ = v;
//}
//
//double c_ecclm::epsfn() const
//{
//  return epsfn_;
//}
//
const cv::Mat1f & c_ecclm::reference_image() const
{
  return reference_image_;
}

const cv::Mat1b & c_ecclm::reference_mask() const
{
  return reference_mask_;
}

const cv::Mat1f & c_ecclm::current_image() const
{
  return current_image_;
}

const cv::Mat1b & c_ecclm::current_mask() const
{
  return current_mask_;
}

bool c_ecclm::set_reference_image(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  if( reference_image.channels() == 1 ) {
    reference_image.getMat().convertTo(reference_image_,
        reference_image_.depth());
  }
  else {
    cv::Mat tmp;

    cv::cvtColor(reference_image, tmp,
        cv::COLOR_BGR2GRAY);

    if( tmp.depth() == reference_image_.depth() ) {
      reference_image_ = tmp;
    }
    else {
      tmp.convertTo(reference_image_,
          reference_image_.depth());
    }
  }

  ecclm_differentiate(reference_image_,
      gx_, gy_);

  if( reference_mask.empty() ) {
    reference_mask_.release();
  }
  else {
    cv::erode(reference_mask, reference_mask_,
        cv::Mat1b(5, 5, 255));
  }

  J.resize(image_transform_->parameters().rows);

  image_transform_->create_steepest_descent_images(image_transform_->parameters(),
      gx_, gy_, J.data());

  return true;
}

bool c_ecclm::set_current_image(cv::InputArray current_image, cv::InputArray current_mask)
{
  current_image.getMat().convertTo(current_image_, current_image_.depth());
  current_mask.copyTo(current_mask_);
  return true;
}

bool c_ecclm::align(cv::InputArray current_image, cv::InputArray current_mask,
    cv::InputArray reference_image, cv::InputArray reference_mask)
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

bool c_ecclm::align(cv::InputArray current_image, cv::InputArray current_mask)
{
  if ( !set_current_image(current_image, current_mask) ) {
    CF_ERROR("set_current_image() fails");
    return false;
  }

  return align();
}

double c_ecclm::compute_rhs(const cv::Mat1f & params)
{
  INSTRUMENT_REGION("");

  const int M =
      params.rows;

  const cv::Size size(reference_image_.size());

  cv::Mat1f remapped_image;
  cv::Mat1b remapped_mask;

  ecclm_remap(image_transform_, params, current_image_, current_mask_,
      remapped_image, remapped_mask);

  if( remapped_mask.empty() ) {
    remapped_mask = reference_mask_;
  }
  else if( !reference_mask_.empty() ) {
    cv::bitwise_and(reference_mask_, remapped_mask,
        remapped_mask);
  }

  const double nrms =
      remapped_mask.empty() ? size.area() :
          cv::countNonZero(remapped_mask);

  cv::Mat1f rhs(size, 0.0f);

  cv::subtract(remapped_image, reference_image_, rhs,
      remapped_mask);

  return rhs.dot(rhs) / nrms;
}

double c_ecclm::compute_jac(const cv::Mat1f & params,
    cv::Mat1f & H, cv::Mat1f & v)
{
  INSTRUMENT_REGION("");

  const int M =
      params.rows;

  const cv::Size size(reference_image_.size());

  cv::Mat1f remapped_image;
  cv::Mat1b remapped_mask;

  ecclm_remap(image_transform_, params, current_image_, current_mask_,
      remapped_image, remapped_mask);

  if( remapped_mask.empty() ) {
    remapped_mask = reference_mask_;
  }
  else if( !reference_mask_.empty() ) {
    cv::bitwise_and(reference_mask_, remapped_mask,
        remapped_mask);
  }


  const double nrms =
      remapped_mask.empty() ? size.area() :
          cv::countNonZero(remapped_mask);

  cv::Mat1f rhs(size, 0.0f);

  cv::subtract(remapped_image, reference_image_, rhs,
      remapped_mask);

  const double rms =
      rhs.dot(rhs) / nrms;


  H.create(M, M);
  v.create(M, 1);

  if( true ) {

    INSTRUMENT_REGION("dots");

    tbb::parallel_for(tbb_range(0, M),
        [&](const tbb_range & r) {

          for( int i = r.begin(); i < r.end(); ++i ) {

            v[i][0] = J[i].dot(rhs) / nrms;

            for( int j = 0; j <= i; ++j ) {
              H[i][j] = J[i].dot(J[j]) / nrms;
            }

          }
        });

    for( int i = 0; i < M; ++i ) {
      for( int j = i + 1; j < M; ++j ) {
        H[i][j] = H[j][i];
      }
    }

  }

  return rms;
}


bool c_ecclm::align()
{
  INSTRUMENT_REGION("");

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

  while (iteration < max_iterations_) {

    // CF_DEBUG("> IT %d model_->compute_jac()", iteration);

    err =
        compute_jac(params, H, v);

//    CF_DEBUG("> IT %d err=%g\n"
//        "H = {\n"
//        "%+20g %+20g %+20g %+20g %+20g %+20g\n"
//        "%+20g %+20g %+20g %+20g %+20g %+20g\n"
//        "%+20g %+20g %+20g %+20g %+20g %+20g\n"
//        "%+20g %+20g %+20g %+20g %+20g %+20g\n"
//        "%+20g %+20g %+20g %+20g %+20g %+20g\n"
//        "%+20g %+20g %+20g %+20g %+20g %+20g\n"
//        "}\n"
//        "v={%+g %+g %+g %+g %+g %+g}\n",
//        iteration,
//        err,
//        H[0][0], H[0][1], H[0][2], H[0][3], H[0][4], H[0][5],
//        H[1][0], H[1][1], H[1][2], H[1][3], H[1][4], H[1][5],
//        H[2][0], H[2][1], H[2][2], H[2][3], H[2][4], H[2][5],
//        H[3][0], H[3][1], H[3][2], H[3][3], H[3][4], H[3][5],
//        H[4][0], H[4][1], H[4][2], H[4][3], H[4][4], H[4][5],
//        H[5][0], H[5][1], H[5][2], H[5][3], H[5][4], H[5][5],
//
//        v[0][0], v[1][0], v[2][0], v[3][0], v[4][0], v[5][0]);

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
//          "deltap = {\n"
//          "  %+20g %+20g %+20g\n"
//          "  %+20g %+20g %+20g\n"
//          "  %+20g %+20g\n"
//          " }\n"
//          "newparams = {\n"
//          "  %+20g %+20g %+20g %+20g\n"
//          "  %+20g %+20g %+20g %+20g\n"
//          "}\n"
//          "\n",
//          iteration,
//          deltap[0][0], deltap[1][0], deltap[2][0],
//          deltap[3][0], deltap[4][0], deltap[5][0],
//          deltap[6][0], deltap[7][0],
//          newparams[0][0], newparams[1][0], newparams[2][0],
//          newparams[3][0], newparams[4][0], newparams[5][0],
//          newparams[6][0], newparams[7][0]);

      /* Compute function for newparams */
      newerr =
          compute_rhs(newparams);

      /* Check for increments in parameters  */
      if( (dp = cv::norm(deltap, cv::NORM_INF)) < epsx_ ) {
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


//      CF_DEBUG("IT %d err=%g newerr=%g dp=%g lambda=%g rho=%+g\n"
//          "\n",
//          iteration,
//          err, newerr,
//          dp,
//          lambda,
//          rho
//          );
//

      if( rho > 0.25 ) {
        /* Accept new params and decrease lambda ==> Gauss-Newton method */
        if( lambda > 1e-4 ) {
          lambda = std::max(1e-4, lambda / 5);
        }
        //CF_DEBUG("  lambda->%g", lambda);
      }
      else if( rho > 0.1 ) {

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
      std::swap(params, newparams);
    }

    if( dp < epsx_ ) {
      break;
    }
  }


//  CF_DEBUG("final parameters: T={\n"
//      "%+g %+g %+g %+g\n"
//      "%+g %+g %+g %+g\n"
//      "}\n",
//      newparams[0][0],
//      newparams[1][0],
//      newparams[2][0],
//      newparams[3][0],
//      newparams[4][0],
//      newparams[5][0],
//      newparams[6][0],
//      newparams[7][0]
//  );

  if ( !newparams.empty() ) {
    image_transform_->set_parameters(newparams);
  }
  // rmse_ = sqrt(err / NM);


  CF_DEBUG("RET: iteration=%d err=%g dp=%g", iteration, err, dp);

  return true;
}


/////////////////////////////////////////////////////////////////////////////////////////

c_ecclmp::c_ecclmp()
{

}

c_ecclmp::c_ecclmp( c_image_transform * image_transform) :
    image_transform_(image_transform)
{
}



void c_ecclmp::set_image_transform(c_image_transform * image_transform)
{
  image_transform_ = image_transform;
}

c_image_transform * c_ecclmp::image_transform() const
{
  return image_transform_;
}

void c_ecclmp::set_maxlevel(int v)
{
  maxlevel_ = v;
}

int c_ecclmp::maxlevel() const
{
  return maxlevel_;
}

void c_ecclmp::set_epsx(double v)
{
  epsx_ = v;
}

double c_ecclmp::epsx() const
{
  return epsx_;
}

bool c_ecclmp::set_reference_image(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  cv::Mat1f image;
  cv::Mat1b mask;

  pyramid_.clear();

  double epsx =
      epsx_;

  for( int lvl = 0, lvlmax = std::max(0, maxlevel_); lvl <= lvlmax; ++lvl ) {

    pyramid_.emplace_back(new c_ecclm(image_transform_));

    const auto & next =
        pyramid_.back();

    next->set_epsx(epsx);
    epsx *= 2;

    if( lvl == 0 ) {
      next->set_reference_image(reference_image,
          reference_mask);
    }
    else {

      const auto & previous =
          pyramid_[lvl - 1];

      const cv::Size previous_size =
          previous->reference_image().size();

      const cv::Size next_size((previous_size.width + 1) / 2,
          (previous_size.height + 1) / 2);

      if( next_size.width < 4 || next_size.height < 4 ) {
        break;
      }

      cv::pyrDown(previous->reference_image(), image, next_size);

      if( !previous->reference_mask().empty() ) {
        cv::pyrDown(previous->reference_mask(), mask, next_size);
        cv::compare(mask, 250, mask, cv::CMP_GE);
      }

      next->set_reference_image(image, mask);
      // CF_DEBUG("L %d size=%dx%d", lvl, next->reference_image().cols, next->reference_image().rows);
    }

  }


  return true;
}


bool c_ecclmp::align(cv::InputArray current_image, cv::InputArray current_mask)
{
  std::vector<cv::Mat> images;
  std::vector<cv::Mat> masks;

  const int lvls =
      pyramid_.size();

  for( int lvl = 0; lvl < lvls; ++lvl ) {

    const auto & ecclm =
        pyramid_[lvl];

    if( images.empty() ) {
      images.emplace_back(current_image.getMat());
      masks.emplace_back(current_mask.getMat());
    }
    else {

      const cv::Size next_size =
          ecclm->reference_image().size();

      images.emplace_back();
      masks.emplace_back();

      cv::pyrDown(images[images.size() - 2], images.back(), next_size);

      if( !masks[masks.size() - 2].empty() ) {
        cv::pyrDown(masks[images.size() - 2], masks.back(), next_size);
        cv::compare(masks.back(), 250, masks.back(), cv::CMP_GE);
      }
    }

    //CF_DEBUG("L %d %dx%d", lvl, images.back().cols, images.back().rows);
  }

  image_transform_->scale_transfrom((double) images.back().cols / (double) images.front().cols);

  for( int lvl = lvls - 1; lvl >= 0; --lvl ) {

    const auto & ecclm =
        pyramid_[lvl];

//    CF_DEBUG("\n\nXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n"
//        "LVL %d %dx%d epsx=%g", lvl, ecclm->reference_image().cols, ecclm->reference_image().rows, ecclm->epsx());

    ecclm->align(images[lvl], masks[lvl]);

    if( lvl > 0 ) {

      const double scale =
          (double) images[lvl-1].cols / (double) images[lvl].cols;

      // CF_DEBUG("LVL %d scale=%g", lvl, scale);

      image_transform_->scale_transfrom(scale);
    }

    // CF_DEBUG("\nXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
  }

  return true;
}

 /////////////////////////////////////////////////////////////////////////////////////////
