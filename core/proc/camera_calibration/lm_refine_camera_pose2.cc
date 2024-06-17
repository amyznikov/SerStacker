/*
 * lm_refine_camera_pose2.cc
 *
 *  Created on: Jun 17, 2024
 *      Author: amyznikov
 */



#include "camera_pose.h"
#include <core/proc/levmar2.h>
#include <core/proc/bfgs.h>
#include <core/ssprintf.h>
#include <core/debug.h>


#if HAVE_TBB
# include <tbb/tbb.h>
#endif



template<class _Tp>
static inline cv::Vec<_Tp, 3> from_spherical(_Tp phi, _Tp theta)
{
  return cv::Vec<_Tp, 3>(std::sin(theta) * std::cos(phi),
      std::sin(theta) * std::sin(phi),
      std::cos(theta));
}

template<class _Tp>
static inline void to_spherical(const cv::Vec<_Tp, 3> & T, _Tp * phi, _Tp * theha)
{
  *phi = std::atan2(T(1), T(0));
  *theha = std::atan2(std::sqrt(T(0) * T(0) + T(1) * T(1)), T(2));
}



/**
 * compute_projection_errors()
 *    helper utility for lm_refine_camera_pose()
 *
 * Project vector of difference between warped current point and reference point onto
 * vector originating from epipole towards to reference point.
 *
 * Compute rhs error based on components of projected vector.
 */
template<class _Tp>
static inline _Tp compute_projection_error(const cv::Point2f & cp, const cv::Point2f & rp,
    const cv::Matx<_Tp, 3, 3> & H, const cv::Point_<_Tp> & E,
    EPIPOLAR_MOTION_DIRECTION direction)
{
  using Vec2 =
      cv::Vec<_Tp, 2>;

  using Vec3 =
      cv::Vec<_Tp, 3>;

  const Vec3 cv =
      H * Vec3(cp.x, cp.y, 1);

  // current point relative to epipole
  const Vec2 ecv(cv[0] / cv[2] - E.x,
      cv[1] / cv[2] - E.y);

  // reference point relative to epipole
  const Vec2 erv(rp.x - E.x,
      rp.y - E.y);

  // displacement vector
  const Vec2 flow =
      (ecv - erv) / std::sqrt(erv[0] * erv[0] + erv[1] * erv[1]);

  // displacement perpendicular to epipolar line
  _Tp rhs =
      std::abs(flow.dot(Vec2(-erv[1], erv[0])));

  // displacement along epipolar line
  switch (direction) {
    case EPIPOLAR_DIRECTION_FORWARD: {
      const _Tp rhs2 =
          flow.dot(erv);
      if( rhs2 < 0 ) {
        rhs -= rhs2;
      }
      break;
    }
    case EPIPOLAR_DIRECTION_BACKWARD: {
      const _Tp rhs2 =
          flow.dot(erv);
      if( rhs2 > 0 ) {
        rhs += rhs2;
      }
      break;
    }
  }

  return rhs;
}

/**
 * Use of c_levmar_solver to refine camera pose estimated from essential matrix
 */
bool lm_refine_camera_pose2(cv::Vec3d & A, cv::Vec3d & T,
    const cv::Matx33d & _camera_matrix,
    const std::vector<cv::Point2f> & current_keypoints,
    const std::vector<cv::Point2f> & reference_keypoints,
    cv::Mat1b & inliers,
    const c_lm_camera_pose_options * opts)
{
  typedef float
      _Tp;

  typedef c_levmar2_solver<_Tp>
    c_levmar2_solver;

  typedef cv::Matx<_Tp, 3, 3>
    Matx33;

  typedef cv::Vec<_Tp,2>
    Vec2;

  typedef cv::Vec<_Tp,3>
    Vec3;

  typedef cv::Point_<_Tp>
    Point;

  /* Unpack Euler angles from storage array of levmar parameters */
  static const auto unpack_A =
      [](const std::vector<_Tp> & p) -> Vec3 {
        return Vec3(p[0], p[1], p[2]);
      };

  /* Unpack camera translation from storage array of levmar parameters */
  static const auto unpack_T =
      [](const std::vector<_Tp> & p) -> Vec3 {
          return from_spherical(p[3], p[4]);
      };

  class c_levmar_solver_callback:
      public c_levmar2_solver::callback
  {

    struct PARAMS {
      Matx33 Hf;
      Matx33 Hi;
      Point Ef;
      Point Ei;
    };

    PARAMS P0;
    PARAMS PJF[5];
    PARAMS PJB[5];
    _Tp Jden[5] = {0};


    std::vector<cv::Point2f> current_keypoints;
    std::vector<cv::Point2f> reference_keypoints;
    Matx33 camera_matrix;
    Matx33 camera_matrix_inv;
    Matx33 camera_matrix_t_inv;
    EPIPOLAR_MOTION_DIRECTION direction;
    _Tp robust_threshold = 10;

  public:
    c_levmar_solver_callback(const std::vector<cv::Point2f> & _current_keypoints,
        const std::vector<cv::Point2f> & _reference_keypoints,
        const cv::Mat1b & _inliers,
        const Matx33 & _camera_matrix,
        const Matx33 & _camera_matrix_inv,
        const Matx33 & _camera_matrix_t_inv,
        EPIPOLAR_MOTION_DIRECTION _direction) :

        camera_matrix(_camera_matrix),
        camera_matrix_inv(_camera_matrix_inv),
        camera_matrix_t_inv(_camera_matrix_t_inv),
        direction(_direction)
    {
      if ( _inliers.empty() ) {
        current_keypoints = _current_keypoints;
        reference_keypoints = _reference_keypoints;
      }
      else {

        current_keypoints.reserve(_current_keypoints.size());
        reference_keypoints.reserve(_current_keypoints.size());

        for ( size_t i = 0, n = _current_keypoints.size(); i < n; ++i ) {
          if ( _inliers[i][0] ) {
            current_keypoints.emplace_back(_current_keypoints[i]);
            reference_keypoints.emplace_back(_reference_keypoints[i]);
          }
        }
      }
    }

    void set_robust_threshold(_Tp v )
    {
      robust_threshold = v;
    }

    bool thread_safe_compute() const override
    {
      return false;
    }


    inline _Tp robust_function(_Tp v) const
    {
      return std::min(v, robust_threshold);
    }


    int num_equations() override
    {
      return current_keypoints.size();
    }

    void set_params(const std::vector<_Tp> & p, PARAMS * P)
    {
      const Vec3 A =
          unpack_A(p);

      const Vec3 T =
          unpack_T(p);

      P->Hf = camera_matrix * build_rotation(A) * camera_matrix_inv;
      P->Hi = camera_matrix * build_rotation(-A) * camera_matrix_inv;
      P->Ef = compute_epipole(camera_matrix, T);
      P->Ei = compute_epipole(camera_matrix, -T);
    }

    _Tp compute_rhs(int i, const PARAMS & P)
    {
      // compute projection error and apply robust function
      const EPIPOLAR_MOTION_DIRECTION direction1 =
          direction;

      const EPIPOLAR_MOTION_DIRECTION direction2 =
          direction == EPIPOLAR_DIRECTION_FORWARD ? EPIPOLAR_DIRECTION_BACKWARD :
          direction == EPIPOLAR_DIRECTION_BACKWARD ? EPIPOLAR_DIRECTION_FORWARD :
              EPIPOLAR_DIRECTION_IGNORE;

      const Matx33 & Hf = P.Hf;
      const Matx33 & Hi = P.Hi;
      const Point & Ef = P.Ef;
      const Point & Ei = P.Ei;

      const _Tp rhs1 =
          compute_projection_error(current_keypoints[i],
              reference_keypoints[i],
              Hf, Ef,
              direction1);

      const _Tp rhs2 =
          compute_projection_error(reference_keypoints[i],
              current_keypoints[i],
              Hi, Ei,
              direction2);

      return robust_function(rhs1 + rhs2);
    }


    bool set_params(const std::vector<_Tp> & p, bool fjac) override
    {
      set_params(p, &P0);

      if( fjac ) {

        std::vector<_Tp> P = p;

        for( int j = 0, n = P.size(); j < n; ++j ) {

          const _Tp v =
              P[j];

          const _Tp delta =
              (std::max)((_Tp) (1e-6), (_Tp) (1e-4) * std::abs(v));

          P[j] = v + delta;
          set_params(P, &PJF[j]);

          P[j] = v - delta;
          set_params(P, &PJB[j]);

          Jden[j] = (_Tp) (0.5) / delta;

          P[j] = v;
        }
      }

      return true;
    }

    bool compute(int i, _Tp * rhs, _Tp J[/* params.size*/] )
    {
      * rhs = compute_rhs(i, P0);

      if ( J ) {

        for ( int j = 0; j < 5; ++j ) {

          const _Tp rhs1 =
              compute_rhs(i, PJF[j]);

          const _Tp rhs2 =
              compute_rhs(i, PJB[j]);

          J[j] =
              Jden[j] * (rhs1 - rhs2);
        }
      }

      return true;
    }

//
//
//    /**
//     * compute rhs errors for specified vector of parameters
//     */
//    bool compute(const std::vector<_Tp> & p, std::vector<_Tp> & rhs, cv::Mat_<_Tp> * , bool * ) const override
//    {
//      INSTRUMENT_REGION("");
//
//#if HAVE_TBB
//      typedef tbb::blocked_range<int> tbb_range;
//      constexpr int tbb_grain_size = 256;
//#endif
//
//      if ( current_keypoints.size() < 5 ) {
//        CF_ERROR("current_keypoints.size()=%zu < 5", current_keypoints.size());
//        return false;
//      }
//
//      const Vec3 A = unpack_A(p);
//      const Vec3 T = unpack_T(p);
//      const size_t N = current_keypoints.size();
//
//      // compute projection error and apply robust function
//      const EPIPOLAR_MOTION_DIRECTION direction1 =
//          direction;
//
//      const EPIPOLAR_MOTION_DIRECTION direction2 =
//          direction == EPIPOLAR_DIRECTION_FORWARD ? EPIPOLAR_DIRECTION_BACKWARD :
//          direction == EPIPOLAR_DIRECTION_BACKWARD ? EPIPOLAR_DIRECTION_FORWARD :
//              EPIPOLAR_DIRECTION_IGNORE;
//
//      const Matx33 Hf =
//          camera_matrix * build_rotation(A) * camera_matrix_inv;
//
//      const Point Ef =
//          compute_epipole(camera_matrix, T);
//
//      const Matx33 Hi =
//          camera_matrix * build_rotation(-A) * camera_matrix_inv;
//
//      const Point Ei =
//          compute_epipole(camera_matrix, -T);
//
//      rhs.resize(N);
//
//#if HAVE_TBB
//      tbb::parallel_for(tbb_range(0, N, tbb_grain_size),
//          [this, &rhs, Hf, Ef, Hi, Ei, N, direction1, direction2](const tbb_range & range) {
//            for( int i = range.begin(), n = range.end(); i < n; ++i ) {
//#else
//            for( int i = 0; i < N; ++i ) {
//#endif
//              const _Tp rhs1 =
//                compute_projection_error(current_keypoints[i],
//                    reference_keypoints[i],
//                    Hf, Ef,
//                    direction1);
//
//              const _Tp rhs2 =
//                compute_projection_error(reference_keypoints[i],
//                    current_keypoints[i],
//                    Hi, Ei,
//                    direction2);
//
//              rhs[i] =
//                  robust_function(rhs1 + rhs2);
//            }
//#if HAVE_TBB
//      });
//#endif
//
//      return true;
//    }
  };

  ////////////////////////////////////////

  if ( current_keypoints.size() != reference_keypoints.size() ) {

    CF_ERROR("Invalid args: reference_positions.size()=%zu not equal to current_positions.size()=%zu",
        reference_keypoints.size(), current_keypoints.size());

    return false;
  }

  if ( !inliers.empty() ) {

    if ( inliers.type() != CV_8UC1 ) {

      CF_ERROR("Invalid args: inliers_mask.type()=%d is invalid. Must be CV_8UC1",
          inliers.type());

      return false;
    }

    if ( inliers.rows != (int)reference_keypoints.size() ) {

      CF_ERROR("Invalid args: inliers_mask.rows()=%d not equal to matched_reference_keypoints.size()=%zu",
          inliers.rows, reference_keypoints.size());

      return false;
    }
  }

  /*
   * Setup c_levmar_solver instance
   * */

  c_levmar2_solver lm(100, 1e-7);

  if ( opts ) {
    if ( opts->max_levmar_iterations > 0 ) {
      lm.set_max_iterations(opts->max_levmar_iterations);
    }
    if ( opts->epsf >= 0 ) {
      lm.set_epsf(opts->epsf);
    }
    if ( opts->epsx >= 0 ) {
      lm.set_epsx(opts->epsx);
    }
  }

  const EPIPOLAR_MOTION_DIRECTION direction =
      opts ? opts->direction :
          EPIPOLAR_DIRECTION_IGNORE;

  /*
   * Pack parameters into cv::Mat1d for levmar
   * */
  std::vector<_Tp> p(5);

  int num_inliers =
      inliers.empty() ? current_keypoints.size() :
          cv::countNonZero(inliers);

  const int max_iterations =
      opts && opts->max_iterations > 0 ? opts->max_iterations :
          5;

  const Matx33 camera_matrix =
      _camera_matrix;

  const Matx33 camera_matrix_inv =
      camera_matrix.inv();

  const Matx33 camera_matrix_t_inv =
      camera_matrix.t().inv();

  /*
   * Pack euler angles and translation vector into levmar parameters array
   * */
  p[0] = A(0);
  p[1] = A(1);
  p[2] = A(2);
  to_spherical(Vec3(T), &p[3], &p[4]);

  for ( int ii = 0; ii < max_iterations; ++ii ) {

    /*
     * Setup c_levmar_solver callback instance
     * */
    c_levmar_solver_callback callback(current_keypoints, reference_keypoints, inliers,
        camera_matrix, camera_matrix_inv, camera_matrix_t_inv,
        direction);

    if ( opts && opts->robust_threshold > 0 ) {
      callback.set_robust_threshold(opts->robust_threshold);
    }

    /*
     * Run levmar solver
     * */
    int iterations;

    if ( true ) {
      INSTRUMENT_REGION("run");

      iterations =
          lm.run(callback, p);
    }

      if ( iterations < 0 ) {
        CF_ERROR("lm.run() fails");
        return false;
      }

      CF_DEBUG("ii=%d rmse=%g iterations=%d num_inliers = %d / %zu",
          ii, lm.rmse(),
          iterations,
          num_inliers,
          current_keypoints.size());

      if( num_inliers < 8 ) {
        break;
      }

    /*
     * Check for outliers
     * */

    int num_outliers = 0;

    if( ii + 1 < max_iterations ) {

      if( inliers.empty() ) {
        inliers.create(current_keypoints.size(), 1);
        inliers.setTo(255);
      }

      const double rmse =
          lm.rmse();

      // compute projection error and apply robust function
      const EPIPOLAR_MOTION_DIRECTION direction1 =
          direction;

      const EPIPOLAR_MOTION_DIRECTION direction2 =
          direction == EPIPOLAR_DIRECTION_FORWARD ? EPIPOLAR_DIRECTION_BACKWARD :
          direction == EPIPOLAR_DIRECTION_BACKWARD ? EPIPOLAR_DIRECTION_FORWARD :
              EPIPOLAR_DIRECTION_IGNORE;

      const Vec3 A =
          unpack_A(p);

      const Vec3 T =
          unpack_T(p);

      const Matx33 Hf =
          camera_matrix * build_rotation(A) * camera_matrix_inv;

      const Point Ef =
          compute_epipole(camera_matrix, T);

      const Matx33 Hi =
          camera_matrix * build_rotation(-A) * camera_matrix_inv;

      const Point Ei =
          compute_epipole(camera_matrix, -T);

      for( int i = 0, j = 0, n = inliers.rows; i < n; ++i ) {
        if( inliers[i][0] ) {

          const cv::Point2f & cp =
              current_keypoints[i];

          const cv::Point2f & rp =
              reference_keypoints[i];

          const _Tp rhs1 =
            compute_projection_error(cp,
                rp,
                Hf,
                Ef,
                direction1);

          const _Tp rhs2 =
            compute_projection_error(rp,
                cp,
                Hi,
                Ei,
                direction2);

          if( rhs1 + rhs2 > 5 * rmse ) {
            inliers[i][0] = 0;
            ++num_outliers;
            continue;
          }

        }
      }
    }

    //    CF_DEBUG("lm.run(pass %d): %d iterations rmse=%g num_outliers=%d / %zu", ii,
    //        iterations, lm.rmse(), num_outliers, current_keypoints.size());

    if ( num_outliers < 1 ) {
      break;
    }

    num_inliers -= num_outliers;
  }

  /*
   * Unpack euler angles and translation vector from levmar parameters array
   * */

  A = unpack_A(p);
  T = unpack_T(p);

  return true;
}
