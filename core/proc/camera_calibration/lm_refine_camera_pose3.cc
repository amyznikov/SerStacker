/*
 * lm_refine_camera_pose3.cc
 *
 *  Created on: Mar 13, 2025
 *      Author: amyznikov
 */

#include "camera_pose.h"
#include <core/proc/c_lse2_estimate.h>
#include <core/proc/levmar.h>
//#include <core/proc/bfgs.h>
//#include <core/ssprintf.h>
#include <core/debug.h>


#if HAVE_TBB
# include <tbb/tbb.h>
#endif

#if 1

namespace {

static inline void pack_params(std::vector<float> & params, const cv::Vec3f & A)
{
  if( params.size() != 5 ) {
    params.resize(5);
  }
  params[0] = A[0];
  params[1] = A[1];
  params[2] = A[2];
}

static inline void unpack_params(const std::vector<float> & params, cv::Vec3f & A)
{
  A[0] = params[0];
  A[1] = params[1];
  A[2] = params[2];
}

class c_levmar_solver_callback:
    public c_levmarf_solver::callback
{
  const cv::Matx33f & _camera_matrix;
  const cv::Matx33f & _camera_matrix_inv;
  const std::vector<cv::KeyPoint> & _current_keypoints;
  const std::vector<cv::KeyPoint> & _reference_keypoints;
  const std::vector<int32_t> & _matches;
  cv::Mat1b & _inliers;

  //cv::Matx33d _camera_matrix, _camera_matrix_inv;
  double _robust_threshold = 0; // (double) std::numeric_limits<float>::max();
  double _erfactor = 1.0;

  std::vector<float> rptsx;
  std::vector<float> rptsy;
  std::vector<float> cptsx;
  std::vector<float> cptsy;
  std::vector<uint32_t> ks;

  cv::Point2f E;

public:

  c_levmar_solver_callback(const cv::Matx33f & camera_matrix, const cv::Matx33f & camera_matrix_inv,
      const std::vector<cv::KeyPoint> & current_keypoints,
      const std::vector<cv::KeyPoint> & reference_keypoints,
      const std::vector<int32_t> & matches /* reference -> current */,
      cv::Mat1b & inliers /* reference */ ) :
        _camera_matrix(camera_matrix),
        _camera_matrix_inv(camera_matrix_inv),
        _current_keypoints(current_keypoints),
        _reference_keypoints(reference_keypoints),
        _matches(matches),
        _inliers(inliers)
  {

    for( size_t k = 0, nk = matches.size(); k < nk; ++k ) {
      if( matches[k] >= 0 ) {

        const cv::Point2f & cp =
            current_keypoints[matches[k]].pt;

        const cv::Point2f & rp =
            reference_keypoints[k].pt;

        rptsx.emplace_back(rp.x);
        rptsy.emplace_back(rp.y);

        cptsx.emplace_back(cp.x);
        cptsy.emplace_back(cp.y);

        ks.emplace_back(k);
      }
    }
  }

  void set_robust_threshold(double v)
  {
    _robust_threshold = v;
  }

  double robust_threshold() const
  {
    return _robust_threshold;
  }

  void set_erfactor(double v)
  {
    _erfactor = v;
  }

  double erfactor() const
  {
    return _erfactor;
  }

  bool allow_tbb() const final
  {
    return true;
  }

  // test
  void set_epipole(const cv::Point2f & e )
  {
    E = e;
  }
  const cv::Point2f & epipole() const
  {
    return E;
  }

  // test
  bool estimate_epipole_location(const cv::Matx33f & H, cv::Point2f & E) const
  {
    c_lse2_estimate<float> lse;

    const int nj =
        ks.size();

    const cv::Mat1b & inliers =
        _inliers;

    const float & h00 = H(0, 0);
    const float & h01 = H(0, 1);
    const float & h02 = H(0, 2);

    const float & h10 = H(1, 0);
    const float & h11 = H(1, 1);
    const float & h12 = H(1, 2);

    const float & h20 = H(2, 0);
    const float & h21 = H(2, 1);
    const float & h22 = H(2, 2);

    for( int j = 0; j < nj; ++j ) {
      if( inliers[0][ks[j]] ) {

        const float & rx = rptsx[j];
        const float & ry = rptsy[j];

        const float cz = (h20 * cptsx[j] + h21 * cptsy[j] + h22);
        const float cx = (h00 * cptsx[j] + h01 * cptsy[j] + h02) / cz;
        const float cy = (h10 * cptsx[j] + h11 * cptsy[j] + h12) / cz;

        lse.update(cy - ry, rx - cx, rx * cy - ry * cx);
      }
    }


    if( !lse.compute(E.x, E.y) ) {
      CF_ERROR("lse.compute() fails");
      return false;
    }


    return true;
  }

protected:

  bool compute_reprojection_errors(const std::vector<float> & p, std::vector<float> & e)
  {
    //INSTRUMENT_REGION("");

    cv::Vec3f A;

    unpack_params(p, A);

    const cv::Matx33f H =
        _camera_matrix * build_rotation(A) * _camera_matrix_inv;

    const size_t nj =
        ks.size();

    std::vector<float> ervx(nj);
    std::vector<float> ervy(nj);
    std::vector<float> flowx(nj);
    std::vector<float> flowy(nj);
    std::vector<float> flowz(nj);

    const float & h20 = H(2, 0);
    const float & h21 = H(2, 1);
    const float & h22 = H(2, 2);

    const float & h00 = H(0, 0);
    const float & h01 = H(0, 1);
    const float & h02 = H(0, 2);

    const float & h10 = H(1, 0);
    const float & h11 = H(1, 1);
    const float & h12 = H(1, 2);

    for( size_t j = 0; j < nj; ++j ) {
      flowz[j] = h20 * cptsx[j] + h21 * cptsy[j] + h22;
    }

    for( size_t j = 0; j < nj; ++j ) {
      flowx[j] = (h00 * cptsx[j] + h01 * cptsy[j] + h02) / flowz[j] - rptsx[j];
      ervx[j] = rptsx[j] - (float) E.x;
    }

    for( size_t j = 0; j < nj; ++j ) {
      flowy[j] = (h10 * cptsx[j] + h11 * cptsy[j] + h12) / flowz[j] - rptsy[j];
      ervy[j] = rptsy[j] - (float) E.y;
    }


    e.resize(nj);

    for( size_t j = 0; j < nj; ++j ) {

      // epipolar distance
      const float er2 =
          ervx[j] * ervx[j] + ervy[j] * ervy[j];

      // displacement perpendicular to epipolar line
      //  elateral = flow.dot(Vec2(-erv[1], erv[0]))
      const float elateral =
          (ervx[j] * flowy[j] - ervy[j] * flowx[j]);

      // displacement along epipolar line
      //  eradial = flow.dot(erv)
      float eradial =
          (ervx[j] * flowx[j] + ervy[j] * flowy[j]);

      if( eradial > 0 ) {
        eradial /= (1. + er2 / (_erfactor * _erfactor));
      }

      e[j] =
          std::sqrt((elateral * elateral +
              eradial * eradial) / er2);

    }

    return true;
  }

public:

  bool compute(const std::vector<float> & p, std::vector<float> & rhs,
      cv::Mat1f * J, bool * have_analytical_jac) final
  {
    const size_t nj =
        ks.size();

    const cv::Mat1b & inliers =
        _inliers;

//    CF_DEBUG("_inliers.size=%dx%d", _inliers.rows, _inliers.cols);
    compute_reprojection_errors(p, rhs);

    if( _robust_threshold <= 0 ) {
      for( size_t j = 0; j < nj; ++j ) {
        if( !inliers[0][ks[j]] ) {
          rhs[j] = 0;
        }
      }
    }
    else {

      const double scale =
          _robust_threshold / M_LN2;

      for( size_t j = 0; j < nj; ++j ) {
        rhs[j] = inliers[0][ks[j]] ? scale * std::log(1. + rhs[j] / _robust_threshold) : 0;
      }
    }

    return true;
  }

  int mark_outliers(const std::vector<float> & p, double thresh)
  {
    std::vector<float> e;

    const size_t nj =
        ks.size();

    cv::Mat1b & inliers =
        _inliers;

    int num_outliers = 0;

    compute_reprojection_errors(p, e);

    for( size_t j = 0; j < nj; ++j ) {

      uint8_t & inlier =
          inliers[0][ks[j]];

      if ( e[j] < thresh ) {
        inlier = 255U;
      }
      else if( inlier ) {
        inlier = 0;
        ++num_outliers;
      }
    }

    return num_outliers;
  }

};



}


bool lm_refine_camera_pose3(cv::Vec3f & AA, cv::Point2f & EE,
    const cv::Matx33f & camera_matrix,
    const cv::Matx33f & camera_matrix_inv /* pre-compured by caller to avoid extra computations if called in a loop*/,
    const std::vector<cv::KeyPoint> & current_keypoints,
    const std::vector<cv::KeyPoint> & reference_keypoints,
    const std::vector<int32_t> & matches/* reference -> current */,
    cv::Mat1b & inliers /* reference */,
    const c_lm_camera_pose3_options * opts)
{
  INSTRUMENT_REGION("");

  c_levmarf_solver lm(100, 1e-12);
  std::vector<float> p;
  cv::Vec3f A;
  cv::Point2f E;
  cv::Matx33f H;
  bool EOk = false;

  if( opts->max_levmar_iterations > 0 ) {
    lm.set_max_iterations(opts->max_levmar_iterations);
  }

  CF_DEBUG("*");

  const int max_iterations =
      std::max(1, opts->max_iterations);

  const cv::Vec3f Ainitial =
      AA;

  const cv::Point2f Einitial =
      EE;

  CF_DEBUG("INITIAL A=(%+g %+g %+g) E=(%+g %+g)",
      Ainitial(0) * 180 / CV_PI, Ainitial(1) * 180 / CV_PI, Ainitial(2) * 180 / CV_PI,
      Einitial.x, Einitial.y);

  for( int iteration = 0; iteration < max_iterations; ++iteration ) {

//    if( canceled() ) {
//      return false;
//    }

    pack_params(p, Ainitial);

    c_levmar_solver_callback callback(camera_matrix, camera_matrix_inv,
              current_keypoints,
              reference_keypoints,
              matches,
              inliers);

    callback.set_epipole(Einitial);

    if( opts->epsf >= 0 ) {
      lm.set_epsfn(opts->epsf);
    }

    if( opts->epsx >= 0 ) {
      lm.set_epsx(opts->epsx);
    }

    if( opts->erfactor > 0 ) {
      callback.set_erfactor(opts->erfactor);
    }

    callback.set_robust_threshold(opts->robust_threshold);

    CF_DEBUG("C lm.run[%d]", iteration);

    const int num_levmar_iterations =
        lm.run(callback, p);

    CF_DEBUG("R lm.run[%d] num_levmar_iterations=%d", iteration, num_levmar_iterations);

//    if( canceled() ) {
//      return false;
//    }

    const double rmse =
        lm.rmse();

    const int num_outliers =
        callback.mark_outliers(p, 3 * rmse);

    unpack_params(p, A);
    H = camera_matrix * build_rotation(A) * camera_matrix_inv;
    EOk = callback.estimate_epipole_location(H, E);

    if( true ) {

      CF_DEBUG("lm.run[%d]: iterations=%d rmse=%g num_outliers=%d A=(%g %g %g) E=(%g %g) EOk=%d",
          iteration,
          num_levmar_iterations,
          rmse,
          num_outliers,
          A[0] * 180 / CV_PI, A[1] * 180 / CV_PI, A[2] * 180 / CV_PI,
          EE.x, EE.y,
          EOk);
    }

    if( num_outliers < 1 ) {
      break;
    }

  }

  if( true ) {

    //unpack_params(p, A);
    //E = callback.estimate_epipole_location(camera_matrix * build_rotation(A) * camera_matrix_inv);

    //lock_guard lock(mutex());

    AA = A;
    //current_frame->H = H;

    if ( EOk ) {
      EE = (Einitial * opts->ew + E) / (opts->ew + 1);
    }


    CF_DEBUG("<UPDATE>");
    CF_DEBUG("[%zu] A=(%g %g %g) E=(%g %g) Eok=%d", 1, AA[0] * 180 / CV_PI, AA[1] * 180 / CV_PI, AA[2] * 180 / CV_PI, EE.x, EE.y, EOk);
    CF_DEBUG("</UPDATE>");
  }


  CF_DEBUG("*");


//  if( canceled() ) {
//    return false;
//  }

  if( true ) {

    // compose_rt_matrix();

  }

  return true;
}



#endif

