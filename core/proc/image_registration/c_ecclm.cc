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


bool ecclm_convert_input_image(cv::InputArray src, cv::InputArray src_mask,
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
  if( !reference_mask.empty() && (reference_mask.size() != reference_image.size() || reference_mask.type() != CV_8UC1) ) {

    CF_ERROR("Invalid input mask: %dx%d %d channels depth=%d. Must be %dx%d CV_8UC1",
        reference_mask.cols(), reference_mask.rows(), reference_mask.channels(), reference_mask.depth(),
        reference_image.cols(), reference_image.rows());

    return false;
  }

  if( reference_image.type() != CV_32FC1 ) {
    CF_ERROR("Invalid image type : %dx%d depth=%d channels=%d. Must be CV_32FC1 type",
        reference_image.cols(), reference_image.rows(), reference_image.depth(), reference_image.channels());
    return false;
  }

  reference_image.copyTo(reference_image_);
  ecclm_differentiate(reference_image_, gx_, gy_);

  if( reference_mask.empty() ) {
    reference_mask_.release();
  }
  else {
    cv::erode(reference_mask, reference_mask_,
        cv::Mat1b(5, 5, 255));
  }

  const int M =
      image_transform_->parameters().rows;

  J.resize(M);

  image_transform_->create_steepest_descent_images(image_transform_->parameters(),
      gx_, gy_, J.data());

  if( true ) {

    INSTRUMENT_REGION("rdots");

    JJ.create(M, M);

    tbb::parallel_for(tbb_range(0, M),
        [&](const tbb_range & r) {

          for( int i = r.begin(); i < r.end(); ++i ) {
            for( int j = 0; j <= i; ++j ) {
              JJ[i][j] = J[i].dot(J[j]);
            }
          }
        });

    for( int i = 0; i < M; ++i ) {
      for( int j = i + 1; j < M; ++j ) {
        JJ[i][j] = JJ[j][i];
      }
    }

  }

  return true;
}

bool c_ecclm::set_current_image(cv::InputArray current_image, cv::InputArray current_mask)
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

   if( current_mask.empty() ) {
     current_mask_.release();
   }
   else {
     current_mask.copyTo(current_mask_);
   }

   return true;
}

void c_ecclm::release_current_image()
{
  current_image_.release();
  current_mask_.release();
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

    if ( true ) {

      tbb::parallel_for(tbb_range(0, M),
          [&](const tbb_range & r) {

            for( int i = r.begin(); i < r.end(); ++i ) {
              v[i][0] = J[i].dot(rhs) / nrms;
            }
          });

      for( int i = 0; i < M; ++i ) {
        for( int j = 0; j < M; ++j ) {
          H[i][j] = JJ[i][j] / nrms;
        }
      }
    }
    else {

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

void c_ecclmp::set_max_iterations(int v)
{
  max_iterations_ = v;
}

int c_ecclmp::max_iterations() const
{
  return max_iterations_;
}

bool c_ecclmp::set_reference_image(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  cv::Mat1f image;
  cv::Mat1b mask;

  pyramid_.clear();

  if( !ecclm_convert_input_image(reference_image, reference_mask, image, mask) ) {
    CF_ERROR("ecclm_convert_input_image() fails");
    return false;
  }

  double epsx =
      epsx_;

  pyramid_.emplace_back(new c_ecclm(image_transform_));
  pyramid_.back()->set_max_iterations(max_iterations_);
  pyramid_.back()->set_epsx(epsx);


  if( !pyramid_.back()->set_reference_image(image, mask) ) {
    CF_ERROR("pyramid_.back()->set_reference_image() fails");
    pyramid_.clear();
    return false;
  }


  for( int lvl = 1, lvlmax = std::max(0, maxlevel_); lvl <= lvlmax; ++lvl ) {

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

    pyramid_.emplace_back(new c_ecclm(image_transform_));
    pyramid_.back()->set_max_iterations(max_iterations_);
    pyramid_.back()->set_epsx(epsx *= 2);

    if( !pyramid_.back()->set_reference_image(image, mask) ) {
      CF_ERROR("L[%d] pyramid_.back()->set_reference_image() fails", lvl);
      pyramid_.clear();
      return false;
    }
  }


  return true;
}


bool c_ecclmp::set_current_image(cv::InputArray current_image, cv::InputArray current_mask)
{
  if( pyramid_.empty() ) {
    CF_ERROR("Reference image must be set first");
    return false;
  }

  cv::Mat1f image;
  cv::Mat1b mask;

  if( !ecclm_convert_input_image(current_image, current_mask, image, mask) ) {
    CF_ERROR("ecclm_convert_input_image() fails");
    return false;
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


bool c_ecclmp::align(cv::InputArray current_image, cv::InputArray current_mask)
{
  if ( !set_current_image(current_image, current_mask) ) {
    CF_ERROR("c_ecclmp: set_current_image() fails");
    return false;
  }

  return align();
}

bool c_ecclmp::align()
{
  if( pyramid_.empty() ) {
    CF_ERROR("c_ecclmp: no reference image was set");
    return false;
  }

  if( pyramid_.front()->current_image().empty() ) {
    CF_ERROR("c_ecclmp: no current_image image was set");
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
  }

  return true;
}


 /////////////////////////////////////////////////////////////////////////////////////////
