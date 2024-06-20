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

c_ecclm::c_ecclm()
{
}

c_ecclm::c_ecclm(c_ecclm_motion_model * model) :
    model_(model)
{
}

void c_ecclm::set_model(c_ecclm_motion_model * model)
{
  model_ = model;
}

c_ecclm_motion_model* c_ecclm::model() const
{
  return model_;
}

void c_ecclm::set_max_iterations(int v)
{
  max_iterations_ = v;
}

int c_ecclm::max_iterations() const
{
  return max_iterations_;
}

void c_ecclm::set_epsx(double v)
{
  epsx_ = v;
}

double c_ecclm::epsx() const
{
  return epsx_;
}

void c_ecclm::set_epsfn(double v)
{
  epsfn_ = v;
}

double c_ecclm::epsfn() const
{
  return epsfn_;
}


bool c_ecclm::set_reference_image(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  reference_image.getMat().convertTo(reference_image_, reference_image_.depth());
  reference_mask.copyTo(reference_mask_);
  return true;
}

bool c_ecclm::set_current_image(cv::InputArray current_image, cv::InputArray current_mask)
{
  current_image.getMat().convertTo(current_image_, current_image_.depth());
  current_mask.copyTo(current_mask_);
  return true;
}

bool c_ecclm::align(cv::InputArray current_image, cv::InputArray reference_image, cv::InputArray current_mask, cv::InputArray reference_mask)
{
  set_reference_image(reference_image, reference_mask);
  return align_to_reference(current_image, current_mask);
}

bool c_ecclm::align_to_reference(cv::InputArray current_image, cv::InputArray current_mask)
{
  if ( !set_current_image(current_image, current_mask) ) {
    CF_ERROR("set_current_image() fails");
    return false;
  }

  return align();
}


bool c_ecclm::align()
{
  INSTRUMENT_REGION("");

  cv::Mat1d H, Hp, v, deltap, temp_d;
  cv::Mat1d params, newparams;

  params = model_->parameters();

  const int M =
      params.rows;


  constexpr double eps =
      std::numeric_limits<double>::epsilon();

  double lambda = 0.01;
  double err = 0, newerr = 0, dp = 0;

  int iteration = 0;

  CF_DEBUG("initial parameters: T={\n"
      "%+g %+g %+g\n"
      "%+g %+g %+g\n"
      "}\n",
      params[0][0],
      params[1][0],
      params[2][0],
      params[3][0],
      params[4][0],
      params[5][0]
  );


  while (iteration < max_iterations_) {

    // CF_DEBUG("> IT %d model_->compute_jac()", iteration);

    err =
        model_->compute_jac(params,
            reference_image_, reference_mask_,
            current_image_, current_mask_,
            H, v);

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


    /* Solve normal equation for given Jacobian and lambda */
    do {

      ++iteration;

      /* Increase diagonal elements by lambda */
      for( int i = 0; i < M; ++i ) {
        H[i][i] = (1 + lambda) * Hp[i][i];
      }

      /* Solve system to define delta and define new value of params */
      cv::solve(H, v, deltap, cv::DECOMP_EIG);
      cv::subtract(cv::Mat(params), deltap, newparams);

      /* Compute function for newparams */
      newerr =
          model_->compute_rhs(newparams,
              reference_image_, reference_mask_,
              current_image_, current_mask_);

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
//          "deltap = {\n"
//          "  %+20g %+20g %+20g\n"
//          "  %+20g %+20g %+20g\n"
//          " }\n"
//          "newparams = {\n"
//          "  %+20g %+20g %+20g\n"
//          "  %+20g %+20g %+20g\n"
//          "}\n"
//          "\n",
//          iteration,
//          err, newerr,
//          dp,
//          lambda,
//          rho,
//          deltap[0][0], deltap[1][0], deltap[2][0], deltap[3][0], deltap[4][0], deltap[5][0],
//          newparams[0][0], newparams[1][0], newparams[2][0], newparams[3][0], newparams[4][0], newparams[5][0]
//          );


      if( rho > 0.25 ) {

        /*
         * Accept new params and decrease lambda ==> Gauss-Newton method
         * */

        err = newerr;
        std::swap(params, newparams);
        lambda = std::max(1e-4, lambda / 5);

      //  CF_DEBUG("IT %d  ACCEPT: lambda --> %g ", iteration, lambda);


        break;
      }

      /**
       * Try increase lambda ==> gradient descend
       * */
      if( lambda < 1 ) {
        lambda = 1;
      }
      else {
        lambda *= 10;
      }

      //CF_DEBUG("IT %d  REJECT: lambda --> %g ", iteration, lambda);

    } while (iteration < max_iterations_);

    if( newerr < err ) {
      /*
       * Accept new params if were not yet accepted
       * */
      err = newerr;
      std::swap(params, newparams);
    }

    if( err <= epsfn_ || dp < epsx_ ) {
      break;
    }
  }


  model_->set_parameters(newparams);
  // rmse_ = sqrt(err / NM);

  CF_DEBUG("RET: iteration=%d err=%g dp=%g", iteration, err, dp);


  return true;
}


/////////////////////////////////////////////////////////////////////////////////////////

void ecclm_differentiate(cv::InputArray src, cv::Mat & gx, cv::Mat & gy, int ddepth = CV_32F)
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

/////////////////////////////////////////////////////////////////////////////////////////

double c_ecclm_motion_model::compute_rhs(const cv::Mat1f & reference_image,
    const cv::Mat1f & remapped_current_image,
    const cv::Mat1b & mask,
    cv::Mat1f & rhs)
{
  INSTRUMENT_REGION("base");
  cv::subtract(remapped_current_image, reference_image, rhs, mask);
  return rhs.dot(rhs);

}

/////////////////////////////////////////////////////////////////////////////////////////
void c_ecclm_translation::set_translation(const cv::Vec2d & v)
{
  translation_ = v;
}

const cv::Vec2d & c_ecclm_translation::translation() const
{
  return translation_;
}

cv::Mat1d c_ecclm_translation::parameters() const
{
  return cv::Mat1d(translation_);
}

bool c_ecclm_translation::set_parameters(const cv::Mat1d & p)
{
  translation_[0] = p[0][0];
  translation_[1] = p[1][0];
  return true;
}


double c_ecclm_translation::compute_rhs(const cv::Mat1d & params,
    const cv::Mat1f & reference_image, const cv::Mat1b & reference_mask,
    const cv::Mat1f & current_image, const cv::Mat1b & current_mask)
{

  const cv::Size size(reference_image.size());
  const cv::Vec2d T(params[0][0], params[1][0]);

  cv::Mat1f remapped_image;
  cv::Mat1b remapped_mask;
  cv::Mat1f rhs (size, 0.f);

  remap_image(size, T, current_image, current_mask,
      remapped_image, remapped_mask);

  if( !reference_mask.empty() ) {
    cv::bitwise_and(reference_mask, remapped_mask,
        remapped_mask);
  }

  const double nrms =
      cv::countNonZero(remapped_mask);

  return base::compute_rhs(reference_image,
      remapped_image,
      remapped_mask,
      rhs) / nrms;
}



double c_ecclm_translation::compute_jac(const cv::Mat1d & params,
    const cv::Mat1f & reference_image, const cv::Mat1b & reference_mask,
    const cv::Mat1f & current_image, const cv::Mat1b & current_mask,
    cv::Mat1d & A, cv::Mat1d & v)
{
  const int M = 2;

  const cv::Size size(reference_image.size());
  const cv::Vec2d T0(params[0][0], params[1][0]);

  cv::Mat1f remapped_image;
  cv::Mat1b remapped_mask;

  remap_image(size, T0, current_image, current_mask,
      remapped_image, remapped_mask);

  if ( !reference_mask.empty() ) {
    cv::bitwise_and(reference_mask, remapped_mask,
        remapped_mask);
  }


  cv::Mat1f J[2];

  cv::Vec2d T = T0;

  ecclm_differentiate(reference_image, J[0], J[1]);
  cv::erode(remapped_mask, remapped_mask, cv::Mat1b(7, 7, 255));

  A.create(M, M);
  A.setTo(0);

  v.create(M, 1);
  v.setTo(0);

  double rms = 0;
  int nrms = 0;

  for( int y = 0; y < size.height; ++y ) {
    for( int x = 0; x < size.width; ++x ) {
      if( remapped_mask[y][x] ) {

        const double rhs =
            (remapped_image[y][x] - reference_image[y][x]);

        rms += rhs * rhs;

        for( int i = 0; i < M; ++i ) {

          v[i][0] += J[i][y][x] * rhs;

          for( int j = 0; j < M; ++j ) {
            A[i][j] += J[i][y][x] * J[j][y][x];
          }

        }

        ++nrms;
      }
    }
  }

  for( int i = 0; i < M; ++i ) {
    v[i][0] /= nrms;
    for( int j = 0; j < M; ++j ) {
      A[i][j] /= nrms;
    }
  }

  return rms / nrms;
}


bool c_ecclm_translation::remap_image(const cv::Size & size, const cv::Vec2d & T,
    const cv::Mat1f & src, const cv::Mat1b & src_mask,
    cv::Mat1f & dst, cv::Mat1b & dst_mask)
{
  cv::Mat2f rmap;

  if( !create_remap(size, T, rmap) ) {
    CF_ERROR("create_remap() fails");
    return false;
  }

  cv::remap(src, dst,
      rmap, cv::noArray(),
      cv::INTER_LINEAR,
      cv::BORDER_REFLECT101);

  if( !src_mask.empty() ) {

    cv::remap(src_mask, dst_mask,
        rmap, cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT,
        cv::Scalar(0));

  }
  else {

    cv::remap(cv::Mat1b(size, (uint8_t) (255)), dst_mask,
        rmap, cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT,
        cv::Scalar(0));

  }

  cv::compare(dst_mask, 250, dst_mask,
      cv::CMP_GE);

  return true;
}

bool c_ecclm_translation::create_remap(const cv::Size & size, const cv::Vec2d & T, cv::Mat2f & map)
{
  INSTRUMENT_REGION("");

  //  Wx =  x + tx
  //  Wy =  y + ty


  const float tx =
      T[0];

  const float ty =
      T[1];

  map.create(size);

#if ECCLM_USE_TBB

  tbb::parallel_for(tbb_range(0, map.rows, tbb_block_size),
      [&map, tx, ty](const tbb_range & r) {

        for ( int y = r.begin(); y < r.end(); ++y ) {

          cv::Vec2f * m = map[y];

          for ( int x = 0; x < map.cols; ++x ) {

            m[x][0] = x + tx;
            m[x][1] = y + ty;

          }
        }
      });

#else

  for ( int y = 0; y < map.rows; ++y ) {

    cv::Vec2f * m = map[y];

    for ( int x = 0; x < map.cols; ++x ) {

      m[x][0] = x + tx;
      m[x][1] = y + ty;
    }

  }

#endif // ECCLM_USE_TBB

  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////

void c_ecclm_affine::set_transform(const cv::Matx23d & v)
{
  a_ = v;
}

const cv::Matx23d & c_ecclm_affine::transform() const
{
  return a_;
}

cv::Mat1d c_ecclm_affine::parameters() const
{
  cv::Mat1d m;
  cv::Mat1d(6, 1, (double*) a_.val).copyTo(m);
  return m;
}

bool c_ecclm_affine::set_parameters(const cv::Mat1d & p)
{
  memcpy(a_.val, p.data, sizeof(a_.val) );
  return true;
}

double c_ecclm_affine::compute_rhs(const cv::Mat1d & params,
    const cv::Mat1f & reference_image, const cv::Mat1b & reference_mask,
    const cv::Mat1f & current_image, const cv::Mat1b & current_mask)
{
  INSTRUMENT_REGION("");

  const cv::Size size(reference_image.size());


  cv::Matx23d m((double*) params.data);

  cv::Mat1f remapped_image;
  cv::Mat1b remapped_mask;
  cv::Mat1f rhs;

  remap_image(size, m, current_image, current_mask,
      remapped_image, remapped_mask);

  if( !reference_mask.empty() ) {
    cv::bitwise_and(reference_mask, remapped_mask,
        remapped_mask);
  }

  const double nrms =
      cv::countNonZero(remapped_mask);

  return base::compute_rhs(reference_image,
      remapped_image,
      remapped_mask,
      rhs) / nrms;

}

double c_ecclm_affine::compute_jac(const cv::Mat1d & params,
    const cv::Mat1f & reference_image, const cv::Mat1b & reference_mask,
    const cv::Mat1f & current_image, const cv::Mat1b & current_mask,
    cv::Mat1d & A, cv::Mat1d & v)
{
  INSTRUMENT_REGION("");

  const int M = 6;

  const cv::Size size(reference_image.size());

  cv::Matx23d a0((double*) params.data);

  cv::Mat1f remapped_image;
  cv::Mat1b remapped_mask;

  remap_image(size, a0, current_image, current_mask,
      remapped_image, remapped_mask);

  if( !reference_mask.empty() ) {
    cv::bitwise_and(reference_mask, remapped_mask,
        remapped_mask);
  }

  cv::Mat1f rhs(size, 0.0f);

  const double nrms =
      cv::countNonZero(remapped_mask);

  const double rms =
      base::compute_rhs(reference_image,
          remapped_image,
          remapped_mask,
          rhs) / nrms;



  cv::Mat1f J[M];

  cv::Mat1f & gx = J[2];
  cv::Mat1f & gy = J[5];

  ecclm_differentiate(reference_image, gx, gy);
  cv::erode(remapped_mask, remapped_mask, cv::Mat1b(5, 5, 255));


  if ( true ) {

    INSTRUMENT_REGION("J");

    for ( int i = 0;i < M; ++i ) {
      J[i].create(size);
    }

  #if ECCLM_USE_TBB
    tbb::parallel_for(tbb_range(0, size.height, tbb_block_size),
        [size, &gx, &gy, &J](const tbb_range & r) {
          for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
  #else
          for ( int y = 0; y < size.height; ++y ) {
  #endif
            for ( int x = 0; x < size.width; ++x ) {

              J[0][y][x] = gx[y][x] * x;   // a00
              J[1][y][x] = gx[y][x] * y;   // a01
              //J[2][y][x] = gx[y][x];       // a02

              J[3][y][x] = gy[y][x] * x;   // a10
              J[4][y][x] = gy[y][x] * y;   // a11
              //J[5][y][x] = gy[y][x];       // a12
            }
          }
        }
  #if ECCLM_USE_TBB
    );
  #endif
    }

  A.create(M, M);
  A.setTo(0);

  v.create(M, 1);
  v.setTo(0);

  if( true ) {

    INSTRUMENT_REGION("dots");

    tbb::parallel_for(tbb_range(0, M),
        [&](const tbb_range & r) {

          for( int i = r.begin(); i < r.end(); ++i ) {

            v[i][0] = J[i].dot(rhs) / nrms;

            for( int j = 0; j <= i; ++j ) {
              A[i][j] = J[i].dot(J[j]) / nrms;
            }

          }
        });

    for( int i = 0; i < A.rows; ++i ) {
      for( int j = i + 1; j < A.cols; ++j ) {
        A[i][j] = A[j][i];
      }
    }

  }

  return rms;
}

bool c_ecclm_affine::create_remap(const cv::Size & size, const cv::Matx23d & _a, cv::Mat2f & map)
{
  INSTRUMENT_REGION("");

  //  Wx =  a00 * x  + a01 * y + a02
  //  Wy =  a10 * x  + a11 * y + a12


  const cv::Matx23f a =
      _a;

  map.create(size);

#if 1 //  HAVE_TBB && !defined(Q_MOC_RUN)

  tbb::parallel_for(tbb_range(0, map.rows, tbb_block_size),
      [&map, a](const tbb_range & r) {

        for ( int y = r.begin(); y < r.end(); ++y ) {

          cv::Vec2f * m =
              map[y];

          for ( int x = 0; x < map.cols; ++x ) {

            m[x][0] = a(0,0) * x + a(0,1) * y + a(0,2);
            m[x][1] = a(1,0) * x + a(1,1) * y + a(1,2);
          }

        }
      });

#else

  for ( int y = 0; y < map.rows; ++y ) {

    cv::Vec2f * m =
        map[y];

    for ( int x = 0; x < map.cols; ++x ) {

      m[x][0] = a(0,0) * x + a(0,1) * y + a(0,2);
      m[x][1] = a(1,0) * x + a(1,1) * y + a(1,2);

    }

  }

#endif // TBB

  return true;
}

bool c_ecclm_affine::remap_image(const cv::Size & size, const cv::Matx23d & m,
    const cv::Mat1f & src, const cv::Mat1b & src_mask,
    cv::Mat1f & dst, cv::Mat1b & dst_mask)
{
  INSTRUMENT_REGION("");

  cv::Mat2f rmap;

  if( !create_remap(size, m, rmap) ) {
    CF_ERROR("create_remap() fails");
    return false;
  }

  cv::remap(src, dst,
      rmap, cv::noArray(),
      cv::INTER_LINEAR,
      cv::BORDER_REFLECT101);

  if( !src_mask.empty() ) {

    cv::remap(src_mask, dst_mask,
        rmap, cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT,
        cv::Scalar(0));

  }
  else {

    cv::remap(cv::Mat1b(size, (uint8_t) (255)), dst_mask,
        rmap, cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT,
        cv::Scalar(0));

  }

  cv::compare(dst_mask, 254, dst_mask,
      cv::CMP_GE);

  return true;
}

  /////////////////////////////////////////////////////////////////////////////////////////
