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



/* Unpack Euler angles from storage array of levmar parameters */
template<class _Tp>
static inline cv::Vec<_Tp, 3> unpack_A(const std::vector<double> & p)
{
  return cv::Vec<_Tp, 3>((_Tp)p[0], (_Tp)p[1], (_Tp)p[2]);
}

/* Unpack camera translation from storage array of levmar parameters */
template<class _Tp>
static inline cv::Vec<_Tp, 3> unpack_T(const std::vector<double> & p)
{
  return from_spherical<_Tp>(p[3], p[4]);
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
  INSTRUMENT_REGION("");

  typedef float
      _Tp;

  typedef cv::Matx<_Tp, 3, 3>
    Matx33;

  typedef cv::Vec<_Tp,2>
    Vec2;

  typedef cv::Vec<_Tp,3>
    Vec3;

  typedef cv::Point_<_Tp>
    Point;

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
    double Jden[5] = {0};


    std::vector<cv::Point2f> current_keypoints;
    std::vector<cv::Point2f> reference_keypoints;
    Matx33 camera_matrix;
    Matx33 camera_matrix_inv;
    Matx33 camera_matrix_t_inv;
    EPIPOLAR_MOTION_DIRECTION direction;
    double robust_threshold = 10;

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

    void set_robust_threshold(double v )
    {
      robust_threshold = v;
    }

    inline double robust_function(double v) const
    {
      return std::min(v, robust_threshold);
    }

    int num_equations() final
    {
      return current_keypoints.size();
    }

    void set_params(const std::vector<double> & p, PARAMS * P)
    {
      const Vec3 A =
          unpack_A<_Tp>(p);

      const Vec3 T =
          unpack_T<_Tp>(p);

      P->Hf = camera_matrix * build_rotation(A) * camera_matrix_inv;
      P->Hi = camera_matrix * build_rotation(-A) * camera_matrix_inv;
      P->Ef = compute_epipole(camera_matrix, T);
      P->Ei = compute_epipole(camera_matrix, -T);
    }

    double compute_rhs(int i, const PARAMS & P)
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

      const double rhs1 =
          compute_projection_error(current_keypoints[i],
              reference_keypoints[i],
              Hf, Ef,
              direction1);

      const double rhs2 =
          compute_projection_error(reference_keypoints[i],
              current_keypoints[i],
              Hi, Ei,
              direction2);

      return robust_function(rhs1 + rhs2);
    }


    bool set_params(const std::vector<double> & p, bool fjac) final
    {
      set_params(p, &P0);

      if( fjac ) {

        std::vector<double> P =
            p;

        for( int j = 0, n = P.size(); j < n; ++j ) {

          const double v =
              P[j];

          const double delta =
              (std::max)(1e-6, 1e-4 * std::abs(v));

          P[j] = v + delta;
          set_params(P, &PJF[j]);

          P[j] = v - delta;
          set_params(P, &PJB[j]);

          Jden[j] = 0.5 / delta;

          P[j] = v;
        }
      }

      return true;
    }

    double compute(int i, double J[/* params.size*/] ) final
    {
      if ( J ) {

        for ( int j = 0; j < 5; ++j ) {

          const double rhs1 =
              compute_rhs(i, PJF[j]);

          const double rhs2 =
              compute_rhs(i, PJB[j]);

          J[j] =
              Jden[j] * (rhs1 - rhs2);
        }
      }

      return compute_rhs(i, P0);
    }

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

  c_levmar2_solver lm(100, 1e-5);

  double epsx = 1e-5;

  if ( opts ) {
    if ( opts->max_levmar_iterations > 0 ) {
      lm.set_max_iterations(opts->max_levmar_iterations);
    }
    if ( opts->epsf >= 0 ) {
      lm.set_epsf(opts->epsf);
    }
    if ( opts->epsx >= 0 ) {
      epsx = opts->epsx;
      //lm.set_epsx(opts->epsx);
    }
  }

  const EPIPOLAR_MOTION_DIRECTION direction =
      opts ? opts->direction :
          EPIPOLAR_DIRECTION_IGNORE;

  /*
   * Pack parameters into cv::Mat1d for levmar
   * */
  std::vector<double> p(5);

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
  to_spherical(T, &p[3], &p[4]);

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

      lm.set_epsx(epsx * (1 << (max_iterations - ii)));

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
          unpack_A<_Tp>(p);

      const Vec3 T =
          unpack_T<_Tp>(p);

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

  A =
      unpack_A<double>(p);

  T =
      unpack_T<double>(p);

  return true;
}
