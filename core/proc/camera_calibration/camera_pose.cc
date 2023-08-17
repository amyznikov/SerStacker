/*
 * camera_pose.cc
 *
 *  Created on: Mar 4, 2023
 *      Author: amyznikov
 *
 *  @sa https://towardsdatascience.com/a-comprehensive-tutorial-on-stereo-geometry-and-stereo-rectification-with-python-7f368b09924a
 *
 */

#include "camera_pose.h"
#include <core/proc/levmar.h>
#include <core/proc/bfgs.h>
#include <core/ssprintf.h>
#include <core/debug.h>


template<>
const c_enum_member * members_of<ESSENTIAL_MATRIX_ESTIMATION_METHOD>()
{
  static const c_enum_member members[] = {
      { EMM_LMEDS, "LMEDS", "least-median of squares algorithm" },
      { EMM_RANSAC, "RANSAC", "RANSAC algorithm" },
      { EMM_RHO, "RHO", "RHO algorithm" },
#if CV_VERSION_CURRRENT >= CV_VERSION_INT(4, 5, 0)
      { EMM_USAC_DEFAULT, "USAC_DEFAULT", "USAC algorithm, default settings" },
      { EMM_USAC_PARALLEL, "USAC_PARALLEL", "USAC, parallel version" },
      { EMM_USAC_FM_8PTS, "USAC_FM_8PTS", "USAC, fundamental matrix 8 points" },
      { EMM_USAC_FAST, "USAC_FAST", "USAC, fast settings" },
      { EMM_USAC_ACCURATE, "USAC_ACCURATE", "USAC, accurate settings" },
      { EMM_USAC_PROSAC, "USAC_PROSAC", "USAC, sorted points, runs PROSAC" },
      { EMM_USAC_MAGSAC, "USAC_MAGSAC", "USAC, runs MAGSAC++" },
#endif
      { EMM_LMEDS, nullptr, "" },
  };

  return members;
}


template<class T>
static inline T sqr(T x)
{
  return x * x;
}



/**
 * Compute two (left and right) epipoles from given fundamental matrix F.
 * Return false if epipoles can not be computed.
 *
 * CSE486, Penn State Robert Collins
 * Lecture 19: Essential and Fundamental Matrices
 * http://www.cse.psu.edu/~rtc12/CSE486/lecture19.pdf
 */
bool compute_epipoles(const cv::Matx33d & F, cv::Point2d * e1, cv::Point2d * e2)
{

#if 1

  /*
   * SVD-based solution is a little faster
   */

  try {

    cv::SVD svd(F);

    if ( e1 ) {
      const cv::Matx13d e = svd.vt.row(2);
      e1->x = e(0, 0) / e(0, 2);
      e1->y = e(0, 1) / e(0, 2);
    }

    if ( e2 ) {
      const cv::Matx31d e = svd.u.col(2);
      e2->x = e(0, 0) / e(2, 0);
      e2->y = e(1, 0) / e(2, 0);
    }

    return true;
  }
  catch (const std::exception & e) {
    CF_ERROR("cv::SVD() fails in %s(): %s", __func__, e.what());
  }
  catch (...) {
    CF_ERROR("cv::SVD() fails in %s()", __func__);
  }

#else

  /*
   * EigenVectors-based solution
   */

  try {

    cv::Matx33d eigenvectors;
    std::vector<double> eigenvalues;

    if ( e1 ) {
      cv::eigen(F.t() * F, eigenvalues, eigenvectors);
      e1->x = eigenvectors(2, 0) / eigenvectors(2, 2);
      e1->y = eigenvectors(2, 1) / eigenvectors(2, 2);
    }

    if ( e2 ) {
      cv::eigen(F * F.t(), eigenvalues, eigenvectors);
      e2->x = eigenvectors(2, 0) / eigenvectors(2, 2);
      e2->y = eigenvectors(2, 1) / eigenvectors(2, 2);
    }

    return true;
  }
  catch (const std::exception & e) {
    CF_ERROR("cv::eigen() fails in %s(): %s", __func__, e.what());
  }
  catch (...) {
    CF_ERROR("cv::eigen() fails in %s()", __func__);
  }
#endif

  return false;
}



/*
 * estimate_essential_matrix()
 *
 * Use of cv::findEssentialMat(C, R, camera_matrix, cv::USAC_PROSAC)
 * for essential matrix estimation from two corresponding points sets.
 *
 * The 'inliers_mask' is optional input/output mask for inliers.
 * If it is not empty then only inlier points will used.
 * On the output this mask will updated.
 *
 */
bool estimate_essential_matrix(cv::Matx33d * outputEsentialMatrix,
    const cv::Matx33d & camera_matrix,
    const std::vector<cv::Point2f> & matched_current_keypoints,
    const std::vector<cv::Point2f> & matched_reference_keypoints,
    cv::InputOutputArray inliers_mask,
    ESSENTIAL_MATRIX_ESTIMATION_METHOD method)
{
  INSTRUMENT_REGION("");

  // Check input arguments

  if( matched_reference_keypoints.size() < 5 ) {

    CF_ERROR("Invalid args: matched_reference_keypoints.size()=%zu must be >= 5",
        matched_reference_keypoints.size());

    return false;
  }

  if( matched_current_keypoints.size() != matched_reference_keypoints.size() ) {

    CF_ERROR("Invalid args: reference_keypoints.size()=%zu not equal to current_keypoints.size()=%zu",
        matched_reference_keypoints.size(), matched_current_keypoints.size());

    return false;
  }

  if( inliers_mask.needed() && inliers_mask.fixedType() && inliers_mask.type() != CV_8UC1 ) {

    CF_ERROR("Invalid args: inliers_mask.type()=%d is invalid. Must be CV_8UC1",
        inliers_mask.type());

    return false;
  }

  if( !inliers_mask.empty() ) {

    if( inliers_mask.type() != CV_8UC1 ) {

      CF_ERROR("Invalid args: inliers_mask.type()=%d is invalid. Must be CV_8UC1",
          inliers_mask.type());

      return false;
    }

    if( inliers_mask.rows() != (int) matched_reference_keypoints.size() ) {

      CF_ERROR("Invalid args: inliers_mask.rows()=%d not equal to matched_reference_keypoints.size()=%zu",
          inliers_mask.rows(), matched_reference_keypoints.size());

      return false;
    }
  }

  // Go forward

  std::vector<cv::Point2f> Rtmp, Ctmp;
  std::vector<int> Itmp;

  // Copy inliers points into temporary arrays if inliers_mask is not empty,
  // because cv::findFundamentalMat() does not accept the inliers mask as input argument
  if( !inliers_mask.empty() ) {

    const cv::Mat1b Mtmp =
        inliers_mask.getMatRef();

    Rtmp.reserve(Mtmp.rows);
    Ctmp.reserve(Mtmp.rows);
    Itmp.reserve(Mtmp.rows);

    for( int i = 0; i < Mtmp.rows; ++i ) {
      if( Mtmp[i][0] ) {
        Rtmp.emplace_back(matched_reference_keypoints[i]);
        Ctmp.emplace_back(matched_current_keypoints[i]);
        Itmp.emplace_back(i);
      }
    }
  }

  // Call cv::findEssentialMat()

  const std::vector<cv::Point2f> &R =
      inliers_mask.empty() ?
          matched_reference_keypoints :
          Rtmp;

  const std::vector<cv::Point2f> &C =
      inliers_mask.empty() ?
          matched_current_keypoints :
          Ctmp;

  constexpr double threshold =
      3;

  cv::Mat1b M;

  // cv::theRNG().state = 123456789;

#if  CV_VERSION_CURRRENT >= CV_VERSION_INT(4, 6, 0)

  const cv::Mat EE =
      cv::findEssentialMat(C, R,
          camera_matrix,
          (int)method,
          0.999,
          threshold,
          1000,
          M);
#else

  const cv::Mat EE =
      cv::findEssentialMat(C, R,
          camera_matrix,
          (int) method,
          0.999,
          threshold,
          M);


#endif

  if( EE.rows != 3 || EE.cols != 3 ) {
    CF_ERROR("cv::findEssentialMat() fails");
    return false;
  }

  // Convert result to fixed-size matrix
  *outputEsentialMatrix = EE;

  // Update inliers_mask if required
  if( inliers_mask.needed() ) {

    if( inliers_mask.empty() ) {
      M.copyTo(inliers_mask);
    }
    else {

      cv::Mat1b outputMask =
          inliers_mask.getMatRef();

      for( int i = 0; i < M.rows; ++i ) {
        if( !M[i][0] ) {
          outputMask[Itmp[i]][0] = 0;
        }
      }
    }
  }

  return true;
}


/**
 * decompose_essential_matrix_svd()
 *
 * This routine uses cv::SVD to decompose essential matrix E into two rorations R1, R2 and translation vector T.
 * As the translation T is determined up to sign, the 4 potential solutions are found.
 * The Translation T is defined upt to scale only and it's eucledian norm is set to 1.
 *
 * Use @ref recover_camera_pose() to select only single correct solution from 4 possible.
 *
 */
bool decompose_essential_matrix_svd(const cv::Matx33d & E,
    cv::Matx33d &R1, cv::Matx33d &R2, cv::Vec3d &T)
{
  INSTRUMENT_REGION("");

  cv::Matx33d u;
  cv::Matx31d w;
  cv::Matx33d vt;
  double scale;

  cv::SVD::compute(E, w, u, vt);

  const cv::Matx33d W(
      0, -1, 0,
      1,  0, 0,
      0,  0, 1);

  R1 = u * W * vt;
  if ( cv::determinant(R1) < 0 ) {
    R1 = -R1;
  }

  R2 = u * W.t() * vt;
  if ( cv::determinant(R2) < 0 ) {
    R2 = -R2;
  }

  T(0) = u(0, 2);
  T(1) = u(1, 2);
  T(2) = u(2, 2);

  if ( (scale = cv::norm(T)) != 0 && scale != 1 ) {
    T /= scale;
  }

  return true;
}




/**
 * @brief recover_camera_pose()
 *  Recovers single solution for the relative camera rotation and the translation from 4 possible solutions
 *  using cheirality check for corresponding points in two images. Returns the number of
 *  inliers that pass the check.
 *
 * This routine select one physically reasonable rotation and translation.
 *
 * For the rotation the proposition from @aniukhin is adopted:
 *  select the minimal camera rotation :
 *            argmin(i, argmax(j, abs(A[i](j))).
 *
 *  For the translation the condition of positive Z after triangulation is used,
 *    see /opencv/samples/cpp/essential_mat_reconstr.cpp for camera pose checking example.
 *
 */

bool recover_camera_pose(const cv::Matx33d & camera_matrix,
    const cv::Matx33d & R1,
    const cv::Matx33d & R2,
    const cv::Vec3d & T,
    const std::vector<cv::Point2f> & current_keypoints,
    const std::vector<cv::Point2f> & reference_keypoints,
    cv::Matx33d * outputR,
    cv::Vec3d * outputA,
    cv::Vec3d * outputT,
    cv::InputOutputArray inliers)
{
  INSTRUMENT_REGION("");

  cv::Vec3d A1, A2, absA1, absA2;
  cv::Matx33d R;

  int ngood = 1;

  //
  // Select best rotation matrix
  //

  cv::absdiff(A1 = euler_angles(R1), 0, absA1);
  cv::absdiff(A2 = euler_angles(R2), 0, absA2);

//  CF_DEBUG("\n"
//      "POSSIBLE ROTATIONS:");
//  pmat(A1 * 180 / CV_PI, "A1");
//  pmat(A2 * 180 / CV_PI, "A2");

  if ( *std::max_element(absA1.val, absA1.val + 3) < *std::max_element(absA2.val, absA2.val + 3) ) {

    R = R1;
    if ( outputA ) {
      *outputA = A1;
    }

    //CF_DEBUG("Selected A1");

  }
  else {

    R = R2;
    if ( outputA ) {
      *outputA = A2;
    }

    //CF_DEBUG("Selected A2");
  }

  if ( outputR ) {
    *outputR = R;
  }

  //
  // Select best translation vector if requested
  //
  if ( outputT ) {

    // normalizes cv::Vec3d by its cv::norm()
    static const auto normalized =
        [](const cv::Vec3d & v) -> cv::Vec3d {
          return v / cv::norm(v);
        };

    // Assume that T is already normed:  cv::norm(T) == 1.
    // For each input pair of points this quick test checks the angles
    // of the triangle formed by the baseline and normalized camera coordinates
    // of these points.
    static const auto fast_cheirality_test =
        []( const std::vector<cv::Point2f> & Cpts, const std::vector<cv::Point2f> & Rpts,
            const cv::Matx33d & camera_matrix, const cv::Matx33d & camera_matrix_inv,
            const cv::Matx33d & R, const cv::Vec3d & T,
            cv::Mat1b & mask) {

        const cv::Matx33d & R_camera_matrix_inv =
            R * camera_matrix_inv;

        mask.create(Cpts.size(), 1);

        for ( uint i = 0, n = Cpts.size(); i < n; ++i ) {

          const cv::Vec3d vr =
              normalized(camera_matrix_inv *
                  cv::Vec3d(Rpts[i].x, Rpts[i].y, 1));

          const cv::Vec3d vc =
              normalized(R_camera_matrix_inv *
                  cv::Vec3d(Cpts[i].x, Cpts[i].y, 1));

          const double alpha = vr.dot(T);
          const double beta = vc.dot(T);
          mask[i][0] = (alpha > beta);  // acccept this measurement

          //const double alpha = acos( vr.dot(T) );
          //const double beta = acos( vc.dot(T) );
          //mask[i][0] = (alpha < beta);  // acccept this measurement
        }
      };

    cv::Mat & M =
        inliers.getMatRef();


    if ( !M.empty() ) {
      CF_DEBUG("POSE: initial inliers=%d/%d",
          cv::countNonZero(M),
          M.rows);
    }


    struct {
      cv::Matx33d R;
      cv::Vec3d T;
      int ngood;
    } pose_hypothesis[2] = {
        {R, T, 0},
        {R, -T, 0},
    };

    cv::Mat1b masks[2];

    const cv::Matx33d & camera_matrix_inv =
        camera_matrix.inv();

    for ( int i = 0; i < 2; ++i ) {

      fast_cheirality_test(current_keypoints, reference_keypoints,
          camera_matrix, camera_matrix_inv,
          pose_hypothesis[i].R,
          pose_hypothesis[i].T,
          masks[i]);

      if ( !M.empty() ) {
        cv::bitwise_and(M,
            masks[i],
            masks[i]);
      }

      pose_hypothesis[i].ngood =
          cv::countNonZero(masks[i]);

//      CF_DEBUG("pose[%d] ngood=%d T=(%g %g %g)", i,
//          pose_hypothesis[i].ngood,
//          pose_hypothesis[i].T(0),
//          pose_hypothesis[i].T(1),
//          pose_hypothesis[i].T(2));
    }

    if ( pose_hypothesis[0].ngood >= pose_hypothesis[1].ngood ) {
      ngood = pose_hypothesis[0].ngood;
      *outputT = T;
      if ( inliers.needed() ) {
        masks[0].copyTo(M);
      }
    }
    else {
      ngood = pose_hypothesis[1].ngood;
      *outputT = -T;
      if ( inliers.needed() ) {
        masks[1].copyTo(M);
      }
    }
  }

  return ngood > 0;
}


/**
 * @brief
 *  recover_camera_pose_from_essential_matrix()
 *
 * Decompose essential matrix and recover camera pose.
 */
bool recover_camera_pose_from_essential_matrix(
    const cv::Matx33d & E,
    const cv::Matx33d & camera_matrix,
    const std::vector<cv::Point2f> & current_keypoints,
    const std::vector<cv::Point2f> & reference_keypoints,
    cv::Matx33d * outputR,
    cv::Vec3d * outputA,
    cv::Vec3d * outputT,
    cv::InputOutputArray inliers)
{

  INSTRUMENT_REGION("");

  cv::Matx33d R1, R2;
  cv::Vec3d T;
  bool fOk;

  fOk =
      decompose_essential_matrix_svd(E, R1, R2, T);

  if ( !fOk ) {
    CF_ERROR("decompose_essential_matrix_svd() fails");
    return false;
  }


  fOk =
      recover_camera_pose(camera_matrix,
          R1, R2,
          T,
          current_keypoints,
          reference_keypoints,
          outputR,
          outputA,
          outputT,
          inliers);

  if ( !fOk ) {
    CF_ERROR("recover_camera_pose() fails");
    return false;
  }

  return true;
}

/**
 * Use cv::LMSolver to refine camera pose estimated from essential matrix
 */
static bool lm_refine_camera_pose(cv::Vec3d & A, cv::Vec3d & T,
    const cv::Matx33d & camera_matrix,
    const std::vector<cv::Point2f> & current_keypoints,
    const std::vector<cv::Point2f> & reference_keypoints,
    cv::Mat1b & inliers)
{

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

  static const auto compute_fundamental_matrix =
      [](const std::vector<double> & p, const cv::Matx33d & camera_matrix_inv,
          const cv::Matx33d & camera_matrix_t_inv,
          double Tfix, int iTfix) -> cv::Matx33d  {

            const cv::Vec3d A(p[0], p[1], p[2]);

            cv::Vec3d T;

            switch (iTfix) {
              case 0:
                T = cv::Vec3d(Tfix, p[3], p[4]);
                break;
              case 1:
                T = cv::Vec3d(p[3], Tfix, p[4]);
                break;
              case 2:
                T = cv::Vec3d(p[3], p[4], Tfix);
                break;
              default:
                CF_FATAL("APP BUG: invalid iTfix=%d received", iTfix);
                break;
            }

            return compose_fundamental_matrix(camera_matrix_t_inv,
                camera_matrix_inv,
                build_rotation(A),
                T / cv::norm(T));
          };


  class c_levmar_solver_callback:
      public c_levmar_solver::callback
  {
    cv::Matx33d camera_matrix;
    cv::Matx33d camera_matrix_t_inv;
    cv::Matx33d camera_matrix_inv;
    const std::vector<cv::Point2f> & current_keypoints;
    const std::vector<cv::Point2f> & reference_keypoints;
    const cv::Mat1b & inliers;
    int numInliers;
    const double Tfix;
    const int iTfix;

  public:
    c_levmar_solver_callback(const cv::Matx33d & _camera_matrix,
        const std::vector<cv::Point2f> & _current_keypoints,
        const std::vector<cv::Point2f> & _reference_keypoints,
        const cv::Mat1b & _inliers,
        double _Tfix,
        int _iTfix ) :

        camera_matrix(_camera_matrix),
        current_keypoints(_current_keypoints),
        reference_keypoints(_reference_keypoints),
        inliers(_inliers),
        Tfix(_Tfix),
        iTfix(_iTfix),
        numInliers(_inliers.empty() ? _current_keypoints.size() :
            cv::countNonZero(_inliers))
    {
      camera_matrix_t_inv =
          camera_matrix.t().inv();

      camera_matrix_inv =
          camera_matrix.inv();
    }

    /**
     * compute rhs errors for specified vector of parameters
     */
    bool compute(const std::vector<double> & p, std::vector<double> & rhs, cv::Mat1d * , bool * ) const override
    {
      const cv::Matx33d F =
          compute_fundamental_matrix(p,
              camera_matrix_inv,
              camera_matrix_t_inv,
              Tfix, iTfix);

      rhs.resize(numInliers);

      if( inliers.empty() ) { // case when no inliers mask

        for( int i = 0, n = current_keypoints.size(); i < n; ++i ) {

          rhs[i] = distance_from_point_to_corresponding_epipolar_line(F,
              current_keypoints[i], reference_keypoints[i]);


        }
      }
      else { // case when inliers mask is provided

        for( int i = 0, j = 0, n = current_keypoints.size(); i < n; ++i ) {
          if( inliers[i][0] ) {

            rhs[j++] = distance_from_point_to_corresponding_epipolar_line(F,
                current_keypoints[i], reference_keypoints[i]);

          }
        }
      }

      return true;
    }
  };


  c_levmar_solver lm(100, 1e-7);

  /*
   * Find the translation component of maximal magnitude and fix it
   * iTmax = argmax(i, abs( T[i] ) )
   */
  double Tfix = T(0);
  int iTfix = 0;
  for( int i = 1; i < 3; ++i ) {
    if( fabs(T[i]) > fabs(Tfix) ) {
      Tfix = T[iTfix = i];
    }
  }

  /*
   * Pack parameters into cv::Mat1d for levmar
   * */
  std::vector<double> p(5);

  for ( int ii = 0; ii < 5; ++ii ) {

    p[0] = A(0);
    p[1] = A(1);
    p[2] = A(2);
    for( int i = 0, j = 3; i < 3; ++i ) {
      if( i != iTfix ) {
        p[j++] = T[i];
      }
    }

    c_levmar_solver_callback lmcb(camera_matrix,
        current_keypoints,
        reference_keypoints,
        inliers,
        Tfix,
        iTfix);

    int iterations =
        lm.run(lmcb, p);

    if ( iterations < 0 ) {
      CF_ERROR("lm.run() fails");
      return false;
    }


    /*
     * Check for outliers
     * */

    const std::vector<double> & rhs =
        lm.rhs();

    int num_outliers = 0;

    if ( rhs.size() > 5 ) {

      if( inliers.empty() ) {
        inliers.create(current_keypoints.size(), 1);
        inliers.setTo(255);
      }

      const double rmse2 =
          9 * lm.rmse() * lm.rmse();

      const cv::Matx33d F =
          compute_fundamental_matrix(p,
              camera_matrix.inv(),
              camera_matrix.t().inv(),
              Tfix, iTfix);

      for( int i = 0, j = 0, n = inliers.rows; i < n; ++i ) {
        if( inliers[i][0] ) {
          const double d = sqr(rhs[j++]);
          if( d > rmse2 ) {
            inliers[i][0] = 0;
            ++num_outliers;
          }
        }
      }
    }

    CF_DEBUG("lm.run(pass %d): %d iterations rmse=%g num_outliers=%d", ii,
        iterations, lm.rmse(), num_outliers);

    if ( num_outliers < 1 ) {
      break;
    }
  }

  /*
   * Unpack parameters from cv::Mat1d
   * */

  A(0) = p[0];
  A(1) = p[1];
  A(2) = p[2];

  switch (iTfix) {
    case 0:
      T = cv::Vec3d(Tfix, p[3], p[4]);
      break;
    case 1:
      T = cv::Vec3d(p[3], Tfix, p[4]);
      break;
    case 2:
      T = cv::Vec3d(p[3], p[4], Tfix);
      break;
    default:
      CF_FATAL("APP BUG: invalid iTfix=%d received", iTfix);
      return false;
  }

  T /= cv::norm(T);



  return true;
}

/**
 * @brief estimate_camera_pose_and_derotation_homography()
 *
 * Estimate camera pose for forward / backward moving monocular camera.
 *
 * This method is not much appropriate for stereo camera calibration because
 * it uses distances from points to corresponding epipolar lines as metric.
 * For true stereo camera this metric is not sensitive to small epipole changes
 * when epipole is very far away from image center (~ at left or right infinity).
 *
 *
 * This routine calls @ref estimate_essential_matrix() in order to
 * find essential matrix from sparse feature matches and recover pose of current camera relative to reference camera.
 *
 * On the output it returns :
 *    - EulerAnges of rotation matrix R in radians,
 *        can be used to rotate CURRENT image to REFERENSE;
 *
 *    - Normalized translation vector T of CURRENT camera relative to REFERENCE camera;
 *
 *    - Rotation matrix R of CURRENT camera relative to REFERENCE camera,
 *        can be used to rotate CURRENT image to REFERENSE;
 *
 *    - Derotation homography matrix H of CURRENT camera relative to REFERENCE camera,
 *              can be used to rotate CURRENT image to REFERENSE;
 *
 *    - Essential matrix E of CURRENT camera relative to REFERENCE camera;
 *
 *    - Fundamental matrix F of CURRENT camera relative to REFERENCE camera
 *        AFTER derotation with homography H;
 *
 * The derotation homography matrix H can be used later to derotate and rectify CURRENT RGB frame
 * or sparse feature positions.
 *
 * In order to apply H to CURRENT RGB image use:
 * @code
 *    cv::Mat derotated_current_image;
 *    cv::warpPerspective(current_image, derotated_current_image,
 *      H,
 *      current_image.size(),
 *      cv::INTER_LINEAR,
 *      cv::BORDER_CONSTANT);
 * @endcode
 *
 * In order to apply H to CURRENT sparse points use:
 * @code
 *    std::vector<cv::Point2f> derotated_current_keypoints;
 *    cv::perspectiveTransform(current_keypoints,
 *      derotated_current_keypoints,
 *      H);
 * @endcode
 *
 *
 * The normalized translation vector T (defined up to scale only) can be used later
 * for estimatioon of the stereo base and for triangulation of matched pixels.
 *
 *
 * @see Section "3.2.4 Rotating Images via a Homography",
 *      Johannes SAUER, "Structure from Motion Based on Monocular Image Sequences with a Virtual Stereo Camera"
 *
 * @see <https://en.wikipedia.org/wiki/Homography_(computer_vision)>
 */
bool estimate_camera_pose_and_derotation_homography(
    /* in */ const cv::Matx33d & camera_matrix,
    /* in */ const std::vector<cv::Point2f> & current_keypoints,
    /* in */ const std::vector<cv::Point2f> & reference_keypoints,
    /* in */ ESSENTIAL_MATRIX_ESTIMATION_METHOD emm,
    /* out, opt */ cv::Vec3d * outputEulerAnges,
    /* out, opt */ cv::Vec3d * outputTranslationVector,
    /* out, opt */ cv::Matx33d * outputRotationMatrix,
    /* out, opt */ cv::Matx33d * outputEssentialMatrix,
    /* out, opt */ cv::Matx33d * outputFundamentalMatrix,
    /* out, opt */ cv::Matx33d * outputDerotationHomography,
    /* in, out, opt */ cv::InputOutputArray M)
{
  INSTRUMENT_REGION("");

  // Check input arguments
  const int min_matched_points_required = 5;

  if ( (int)reference_keypoints.size() < min_matched_points_required ) {

    CF_ERROR("Invalid args: reference_keypoints.size()=%zu must be >= %d",
        reference_keypoints.size(),
        min_matched_points_required);

    return false;
  }

  if ( current_keypoints.size() != reference_keypoints.size() ) {

    CF_ERROR("Invalid args: current_keypoints.size()=%zu not equal to reference_keypoints.size()=%zu ",
        current_keypoints.size(), reference_keypoints.size());

    return false;
  }

  if ( M.needed() ) {

    if ( M.fixedType() && M.type() != CV_8UC1 ) {
      CF_ERROR("Invalid args: inliers_mask.type()=%d is invalid. Must be CV_8UC1", M.type());
      return false;
    }

    if ( !M.empty() && M.rows() != (int) current_keypoints.size() ) {
      CF_ERROR("Invalid args: inliers_mask.rows()=%d is invalid. Must be %zu", M.rows(), current_keypoints.size());
      return false;
    }
  }

  //
  // Go on
  //

  cv::Matx33d R, E, H;
  cv::Vec3d A, T;
  cv::Mat1b mask;
  bool fOk;

  if ( !M.empty() ) {
    mask = M.getMatRef();

    CF_DEBUG("estimate_essential_matrix: initial inliers = %d / %d",
        cv::countNonZero(mask),
        mask.rows);

  }
  //
  // Estimate essential matrix
  //

  fOk =
      estimate_essential_matrix(&E,
          camera_matrix,
          current_keypoints,
          reference_keypoints,
          mask,
          emm);

  if ( !fOk ) {
    CF_ERROR("estimate_essential_matrix() fails");
    return false;
  }

  CF_DEBUG("estimate_essential_matrix: inliers=%d/%d",
      cv::countNonZero(mask),
      mask.rows);

  //
  // Decompose essential matrix and recover relative pose
  //
  fOk =
      recover_camera_pose_from_essential_matrix(E,
          camera_matrix,
          current_keypoints,
          reference_keypoints,
          nullptr,
          &A,
          &T,
          mask);

  if ( !fOk ) {
    CF_ERROR("recover_camera_pose_from_essential_matrix() fails");
    return false;
  }

  //
  // Update camera pose using levmar if enabled
  //
  if( true ) {
    fOk =
        lm_refine_camera_pose(A, T,
            camera_matrix,
            current_keypoints,
            reference_keypoints,
            mask);
  }

  //
  // Return to caller everything requested
  //
  if ( M.needed() && M.empty() ) {
    if ( M.kind() == cv::_InputArray::KindFlag::MAT ) {
      M.move(mask);
    }
    else {
      mask.copyTo(M);
    }
  }

  if ( outputEulerAnges ) {
    *outputEulerAnges = A;
  }

  if ( outputTranslationVector ) {
    * outputTranslationVector = T;
  }

  if ( outputRotationMatrix || outputEssentialMatrix || outputDerotationHomography || outputFundamentalMatrix ) {
    R = build_rotation(A);
    if ( outputRotationMatrix ) {
      * outputRotationMatrix = R;
    }
  }

  if ( outputDerotationHomography || outputFundamentalMatrix  ) {
    H = camera_matrix * R * camera_matrix.inv();
    if ( outputDerotationHomography ) {
      * outputDerotationHomography = H;
    }
  }

  if ( outputEssentialMatrix || outputFundamentalMatrix ) {
    E = compose_essential_matrix(R, T);
    if ( outputEssentialMatrix ) {
      * outputEssentialMatrix = E;
    }
  }

  if ( outputFundamentalMatrix ) {
    * outputFundamentalMatrix =
        compose_fundamental_matrix(E, camera_matrix) * H.inv() ;
  }

  return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////


/**
 * Use of c_levmar_solver to refine camera pose estimated from essential matrix
 */
static bool lm_refine_camera_pose2(cv::Vec3d & A, cv::Vec3d & T,
    const cv::Matx33d & camera_matrix,
    const std::vector<cv::Point2f> & current_keypoints,
    const std::vector<cv::Point2f> & reference_keypoints,
    cv::Mat1b & inliers)
{

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

  static const auto compute_epipole =
      [](const cv::Matx33d & F) -> cv::Point2d {
        cv::Point2d E[2];
        compute_epipoles(F, E);
        return cv::Point2d(0.5 * (E[0].x + E[1].x), 0.5 * (E[0].y + E[1].y) );
      };


  static const auto unpack_A =
      [](const std::vector<double> & p) -> cv::Vec3d {
        return cv::Vec3d(p[0], p[1], p[2]);
      };

  static const auto unpack_T =
      [](const std::vector<double> & p, double Tfix, int iTfix) -> cv::Vec3d {

        cv::Vec3d T;

        switch (iTfix) {
          case 0:
          T = cv::Vec3d(Tfix, p[3], p[4]);
          break;
          case 1:
          T = cv::Vec3d(p[3], Tfix, p[4]);
          break;
          case 2:
          T = cv::Vec3d(p[3], p[4], Tfix);
          break;
          default:
          CF_FATAL("APP BUG: invalid iTfix=%d received", iTfix);
          break;
        }
        return T / cv::norm(T);
      };

  static const auto compute_fundamental_matrix =
      [](const std::vector<double> & p, const cv::Matx33d & camera_matrix_inv,
          const cv::Matx33d & camera_matrix_t_inv,
          double Tfix, int iTfix) -> cv::Matx33d {

            return compose_fundamental_matrix(camera_matrix_t_inv,
                camera_matrix_inv,
                build_rotation(unpack_A(p)),
                unpack_T(p, Tfix, iTfix));
          };


  static const auto compute_error =
      [](const cv::Point2f & wcp, const cv::Point2f & rp, const cv::Point2f & E) -> cv::Point2f {

        const cv::Point2f erp(rp.x - E.x, rp.y - E.y);
        const cv::Point2f ecp(wcp.x - E.x, wcp.y - E.y);
        const double RP = sqrt(erp.x * erp.x + erp.y * erp.y);
        return cv::Point2f(
            (erp.x * (ecp.x - erp.x) + erp.y * (ecp.y - erp.y)) / RP,
            (-erp.y * (ecp.x - erp.x) + erp.x * (ecp.y - erp.y)) / RP);
      };

  class c_levmar_solver_callback:
      public c_levmar_solver::callback
  {
    cv::Matx33d camera_matrix;
    cv::Matx33d camera_matrix_t_inv;
    cv::Matx33d camera_matrix_inv;
    const std::vector<cv::Point2f> & current_keypoints;
    const std::vector<cv::Point2f> & reference_keypoints;
    const cv::Mat1b & inliers;
    int numInliers;
    const double Tfix;
    const int iTfix;

  public:
    c_levmar_solver_callback(const cv::Matx33d & _camera_matrix,
        const std::vector<cv::Point2f> & _current_keypoints,
        const std::vector<cv::Point2f> & _reference_keypoints,
        const cv::Mat1b & _inliers,
        double _Tfix,
        int _iTfix ) :

        camera_matrix(_camera_matrix),
        current_keypoints(_current_keypoints),
        reference_keypoints(_reference_keypoints),
        inliers(_inliers),
        Tfix(_Tfix),
        iTfix(_iTfix),
        numInliers(_inliers.empty() ? _current_keypoints.size() :
            cv::countNonZero(_inliers))
    {
      camera_matrix_t_inv =
          camera_matrix.t().inv();

      camera_matrix_inv =
          camera_matrix.inv();
    }

    /**
     * compute rhs errors for specified vector of parameters
     */
    bool compute(const std::vector<double> & p, std::vector<double> & rhs, cv::Mat1d * , bool * ) const override
    {
      // project vector of difference between warped cp and rp onto rp
      // compute rhs error based on components of projected vector and robust function
      static const auto compute_rhs =
          [](const cv::Point2f & wcp, const cv::Point2f & rp, const cv::Point2f & E) -> double {

              const cv::Point2f ecp(wcp.x - E.x, wcp.y - E.y);
              const cv::Point2f erp(rp.x - E.x, rp.y - E.y);
              const double L = sqrt(erp.x * erp.x + erp.y * erp.y);

              //  cos  sin
              // -sin  cos
              const cv::Point2f p(
                  erp.x * (ecp.x - erp.x) + erp.y * (ecp.y - erp.y),
                  -erp.y * (ecp.x - erp.x) + erp.x * (ecp.y - erp.y));

              double r = std::abs(p.y);
              if ( p.x < 0 ) {
                r -= p.x;
              }

              return std::min(r / L, 5.);
            };

      const cv::Vec3d A = unpack_A(p);
      const cv::Vec3d T = unpack_T(p, Tfix, iTfix);
      const cv::Matx33d R = build_rotation(A);
      const cv::Matx33d H = camera_matrix * R * camera_matrix_inv;
      const cv::Matx33d F = compose_fundamental_matrix(camera_matrix_t_inv, camera_matrix_inv, R, T);

      cv::Point2d EE[2];
      compute_epipoles(F, EE);

      // CF_DEBUG("E: {%+g %+g} {%+g %+g}", EE[0].x, EE[0].y, EE[1].x, EE[1].y);


      const cv::Point2f E(EE[1].x, EE[1].y);

      std::vector<cv::Point2f> warped_current_keypoints;
      cv::perspectiveTransform(current_keypoints, warped_current_keypoints, H);

      rhs.resize(numInliers);


      if( inliers.empty() ) {
        for( int i = 0, n = warped_current_keypoints.size(); i < n; ++i ) {
          rhs[i] = compute_rhs(warped_current_keypoints[i], reference_keypoints[i], E);
        }
      }
      else {
        for( int i = 0, j = 0, n = warped_current_keypoints.size(); i < n; ++i ) {
          if( inliers[i][0] ) {
            rhs[j++] = compute_rhs(warped_current_keypoints[i], reference_keypoints[i], E);
          }
        }
      }

      return true;
    }
  };


  c_levmar_solver lm(100, 1e-7);

  /*
   * Find the translation component of maximal magnitude and fix it
   * iTmax = argmax(i, abs( T[i] ) )
   */
  double Tfix = T(0);
  int iTfix = 0;
  for( int i = 1; i < 3; ++i ) {
    if( fabs(T[i]) > fabs(Tfix) ) {
      Tfix = T[iTfix = i];
    }
  }

  /*
   * Pack parameters into cv::Mat1d for levmar
   * */
  std::vector<double> p(5);

  for ( int ii = 0; ii < 5; ++ii ) {

    p[0] = A(0);
    p[1] = A(1);
    p[2] = A(2);
    for( int i = 0, j = 3; i < 3; ++i ) {
      if( i != iTfix ) {
        p[j++] = T[i];
      }
    }

    c_levmar_solver_callback lmcb(camera_matrix,
        current_keypoints,
        reference_keypoints,
        inliers,
        Tfix,
        iTfix);

    int iterations =
        lm.run(lmcb, p);

    if ( iterations < 0 ) {
      CF_ERROR("lm.run() fails");
      return false;
    }


    /*
     * Check for outliers
     * */

    const std::vector<double> & rhs =
        lm.rhs();

    int num_outliers = 0;

    if ( rhs.size() > 10 ) {

      if( inliers.empty() ) {
        inliers.create(current_keypoints.size(), 1);
        inliers.setTo(255);
      }

      const double rmse =
          lm.rmse();

      //      const double rmse2 =
      //          50 * rmse * rmse;

      const cv::Vec3d AA = unpack_A(p);
      const cv::Vec3d TT = unpack_T(p, Tfix, iTfix);
      const cv::Matx33d camera_matrix_inv = camera_matrix.inv();
      const cv::Matx33d camera_matrix_t_inv = camera_matrix.t().inv();
      const cv::Matx33d RR = build_rotation(AA);
      const cv::Matx33d HH = camera_matrix * RR * camera_matrix_inv;
      const cv::Matx33d FF = compose_fundamental_matrix(camera_matrix_t_inv, camera_matrix_inv, RR, TT);

      cv::Point2d EE[2];
      compute_epipoles(FF, EE);
      const cv::Point2d E = EE[1];

      //CF_DEBUG("EEEEE: {%+g %+g} {%+g %+g}", EE[0].x, EE[0].y, EE[1].x, EE[1].y);

      std::vector<cv::Point2f> warped_current_keypoints;
      cv::perspectiveTransform(current_keypoints, warped_current_keypoints, HH);

      for( int i = 0, j = 0, n = inliers.rows; i < n; ++i ) {
        if( inliers[i][0] ) {
          if( rhs[j++] > 5 * rmse ) {
            inliers[i][0] = 0;
            ++num_outliers;
          }

//          const double d = sqr(rhs[j++]);
//          if( d > rmse2 ) {
//            inliers[i][0] = 0;
//            ++num_outliers;
//          }
//          else {
//            const cv::Point2f e = compute_error(warped_current_keypoints[i], reference_keypoints[i], E);
//            if( e.x < -3 ) {
//              inliers[i][0] = 0;
//              ++num_outliers;
//            }
//          }
        }
      }
    }


    CF_DEBUG("lm.run(pass %d): %d iterations rmse=%g num_outliers=%d / %zu", ii,
        iterations, lm.rmse(), num_outliers, current_keypoints.size());

    if ( num_outliers < 1 ) {
      break;
    }
  }

  /*
   * Unpack parameters from cv::Mat1d
   * */

  A(0) = p[0];
  A(1) = p[1];
  A(2) = p[2];

  switch (iTfix) {
    case 0:
      T = cv::Vec3d(Tfix, p[3], p[4]);
      break;
    case 1:
      T = cv::Vec3d(p[3], Tfix, p[4]);
      break;
    case 2:
      T = cv::Vec3d(p[3], p[4], Tfix);
      break;
    default:
      CF_FATAL("APP BUG: invalid iTfix=%d received", iTfix);
      return false;
  }

  T /= cv::norm(T);



  return true;
}

/** Experimental pure levar
 *  */
bool lm_camera_pose_and_derotation_homography(
    /* in */ const cv::Matx33d & camera_matrix,
    /* in */ const std::vector<cv::Point2f> & current_keypoints,
    /* in */ const std::vector<cv::Point2f> & reference_keypoints,
    /* in, out */ cv::Vec3d & eulerAnges,
    /* in, out */ cv::Vec3d & translationVector,
    /* out, opt */ cv::Matx33d * outputRotationMatrix,
    /* out, opt */ cv::Matx33d * outputEssentialMatrix,
    /* out, opt */ cv::Matx33d * outputFundamentalMatrix,
    /* out, opt */ cv::Matx33d * outputDerotationHomography,
    /* in, out, opt */ cv::InputOutputArray M)
{
  INSTRUMENT_REGION("");

  // Check input arguments
  const int min_matched_points_required = 5;

  if ( (int)reference_keypoints.size() < min_matched_points_required ) {

    CF_ERROR("Invalid args: reference_keypoints.size()=%zu must be >= %d",
        reference_keypoints.size(),
        min_matched_points_required);

    return false;
  }

  if ( current_keypoints.size() != reference_keypoints.size() ) {

    CF_ERROR("Invalid args: current_keypoints.size()=%zu not equal to reference_keypoints.size()=%zu ",
        current_keypoints.size(), reference_keypoints.size());

    return false;
  }

  if ( M.needed() ) {

    if ( M.fixedType() && M.type() != CV_8UC1 ) {
      CF_ERROR("Invalid args: inliers_mask.type()=%d is invalid. Must be CV_8UC1", M.type());
      return false;
    }

    if ( !M.empty() && M.rows() != (int) current_keypoints.size() ) {
      CF_ERROR("Invalid args: inliers_mask.rows()=%d is invalid. Must be %zu", M.rows(), current_keypoints.size());
      return false;
    }
  }

  //
  // Go on
  //

  cv::Vec3d & A = eulerAnges;
  cv::Vec3d & T = translationVector;

  cv::Matx33d R, E, H;
  cv::Mat1b mask;
  bool fOk;

  if ( !M.empty() ) {
    mask = M.getMatRef();

    CF_DEBUG("estimate_essential_matrix: initial inliers = %d / %d",
        cv::countNonZero(mask),
        mask.rows);

  }


  //
  // Extimate camera pose using levmar
  //
  fOk =
      lm_refine_camera_pose2(A, T,
          camera_matrix,
          current_keypoints,
          reference_keypoints,
          mask);

  if ( !fOk ) {
    CF_ERROR("lm_refine_camera_pose() fails");
  }


  //
  // Return to caller everything requested
  //
  if ( M.needed() && M.empty() ) {
    if ( M.kind() == cv::_InputArray::KindFlag::MAT ) {
      M.move(mask);
    }
    else {
      mask.copyTo(M);
    }
  }

  if ( outputRotationMatrix || outputEssentialMatrix || outputDerotationHomography || outputFundamentalMatrix ) {
    R = build_rotation(A);
    if ( outputRotationMatrix ) {
      * outputRotationMatrix = R;
    }
  }

  if ( outputDerotationHomography || outputFundamentalMatrix  ) {
    H = camera_matrix * R * camera_matrix.inv();
    if ( outputDerotationHomography ) {
      * outputDerotationHomography = H;
    }
  }

  if ( outputEssentialMatrix || outputFundamentalMatrix ) {
    E = compose_essential_matrix(R, T);
    if ( outputEssentialMatrix ) {
      * outputEssentialMatrix = E;
    }
  }

  if ( outputFundamentalMatrix ) {
    * outputFundamentalMatrix =
        compose_fundamental_matrix(E, camera_matrix) * H.inv() ;
  }

  return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////

namespace cv {
  typedef Vec<float, 9> Vec9f;
  typedef Vec<double, 9> Vec9d;
}

/**
 * Use cv::LMSolver to refine camera pose estimated from essential matrix
 */
static bool bfgs_refine_camera_pose2(cv::Vec3d & A, cv::Vec3d & T,
    const cv::Matx33d & camera_matrix,
    const std::vector<cv::Point2f> & current_keypoints,
    const std::vector<cv::Point2f> & reference_keypoints,
    cv::Mat1b & inliers)
{

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


  static const auto kronecker_product =
      [](const cv::Vec3d & v1, const cv::Vec3d & v2) -> cv::Vec9d {

        cv::Vec9d v;

        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j ) {
            v[i*3 + j] = v1[i] * v2[j];
          }
        }
        return v;
      };


  static const auto compute_epipole =
      [](const cv::Matx33d & F) -> cv::Point2d {
        cv::Point2d E[2];
        compute_epipoles(F, E);
        return cv::Point2d(0.5 * (E[0].x + E[1].x), 0.5 * (E[0].y + E[1].y) );
      };


  static const auto unpack_A =
      [](const std::vector<double> & p) -> cv::Vec3d {
        return cv::Vec3d(p[0], p[1], p[2]);
      };

  static const auto unpack_T =
      [](const std::vector<double> & p, double Tfix, int iTfix) -> cv::Vec3d {

        cv::Vec3d T;

        switch (iTfix) {
          case 0:
          T = cv::Vec3d(Tfix, p[3], p[4]);
          break;
          case 1:
          T = cv::Vec3d(p[3], Tfix, p[4]);
          break;
          case 2:
          T = cv::Vec3d(p[3], p[4], Tfix);
          break;
          default:
          CF_FATAL("APP BUG: invalid iTfix=%d received", iTfix);
          break;
        }
        return T / cv::norm(T);
      };

  static const auto compute_fundamental_matrix =
      [](const std::vector<double> & p, const cv::Matx33d & camera_matrix_inv,
          const cv::Matx33d & camera_matrix_t_inv,
          double Tfix, int iTfix) -> cv::Matx33d {

            return compose_fundamental_matrix(camera_matrix_t_inv,
                camera_matrix_inv,
                build_rotation(unpack_A(p)),
                unpack_T(p, Tfix, iTfix));
          };


  static const auto compute_error =
      [](const cv::Point2f & wcp, const cv::Point2f & rp, const cv::Point2f & E) -> cv::Point2f {

        const cv::Point2f erp(rp.x - E.x, rp.y - E.y);
        const cv::Point2f ecp(wcp.x - E.x, wcp.y - E.y);
        const double RP = sqrt(erp.x * erp.x + erp.y * erp.y);
        return cv::Point2f(
            (erp.x * (ecp.x - erp.x) + erp.y * (ecp.y - erp.y)) / RP,
            (-erp.y * (ecp.x - erp.x) + erp.x * (ecp.y - erp.y)) / RP);
      };

  class c_bfgs_callback:
      public c_bfgs::callback
  {
    cv::Matx33d camera_matrix;
    cv::Matx33d camera_matrix_inv;
    cv::Matx33d camera_matrix_t_inv;
    //const std::vector<cv::Point2f> & current_keypoints;
    //const std::vector<cv::Point2f> & reference_keypoints;

    // const cv::Mat1f & weights;
    const double Tfix;
    const int iTfix;

    cv::Matx<double, 9, 9> C;

  public:
    c_bfgs_callback(const cv::Matx33d & _camera_matrix,
        const cv::Matx33d & _camera_matrix_inv, const cv::Matx33d & _camera_matrix_t_inv,
        const std::vector<cv::Point2f> & _current_keypoints,
        const std::vector<cv::Point2f> & _reference_keypoints,
        //const cv::Mat1f & _weights,
        double _Tfix,
        int _iTfix) :
          camera_matrix(_camera_matrix),
          camera_matrix_inv(_camera_matrix_inv),
          camera_matrix_t_inv(_camera_matrix_t_inv),
          //weights(_weights),
          Tfix(_Tfix),
          iTfix(_iTfix)
    {
      copute_C(_current_keypoints, _reference_keypoints);
    }


    void copute_C(const std::vector<cv::Point2f> & _current_keypoints,
        const std::vector<cv::Point2f> & _reference_keypoints)
    {

      static const auto homogenize =
          [](const cv::Vec3f & v) -> cv::Vec3f {
            return v[2] == 1 ? v : cv::Vec3f (v[0]/v[2], v[1]/v[2], 1);
          };

      C = cv::Matx<double, 9, 9>::zeros();

      for( int i = 0, n = _current_keypoints.size(); i < n; ++i ) {
        const cv::Point2f & cp = _current_keypoints[i];
        const cv::Point2f & rp = _reference_keypoints[i];

        const cv::Vec3d cf = homogenize(camera_matrix_inv * cv::Vec3d(cp.x, cp.y, 1));
        const cv::Vec3d rf = homogenize(camera_matrix_inv * cv::Vec3d(rp.x, rp.y, 1));
        const cv::Vec9d kp = kronecker_product(cf, rf);

        C += kp * kp.t();
      }

    }

    /**
     * compute rhs errors for specified vector of parameters
     */
    bool compute(const std::vector<double> & p, double * f,
        std::vector<double> * g, bool * have_g) const override
    {

      const cv::Vec3d A = unpack_A(p);
      const cv::Vec3d T = unpack_T(p, Tfix, iTfix);
      const cv::Matx33d R = build_rotation(A);
      const cv::Matx33d E = compose_essential_matrix(R, T);
      const cv::Vec9d e(E.val);

      *f = (e.t() * C * e)[0];
      // CF_DEBUG("f=%g", *f);

      return true;
    }
  };


  c_bfgs bfgs(100);

  /*
   * Find the translation component of maximal magnitude and fix it
   * iTmax = argmax(i, abs( T[i] ) )
   */
  double Tfix = T(0);
  int iTfix = 0;
  for( int i = 1; i < 3; ++i ) {
    if( fabs(T[i]) > fabs(Tfix) ) {
      Tfix = T[iTfix = i];
    }
  }

  const cv::Matx33d camera_matrix_inv =
      camera_matrix.inv();

  const cv::Matx33d camera_matrix_t_inv =
      camera_matrix.t().inv();

  std::vector<double> p(5);

  bfgs.set_linesearch(c_bfgs::LINESEARCH::LBFGS_LINESEARCH_BACKTRACKING);
  bfgs.set_max_linesearch(100);

  for ( int ii = 0; ii < 1; ++ii ) {

    /*
     * Pack parameters into cv::Mat1d for levmar
     * */
    p[0] = A(0);
    p[1] = A(1);
    p[2] = A(2);
    for( int i = 0, j = 3; i < 3; ++i ) {
      if( i != iTfix ) {
        p[j++] = T[i];
      }
    }

    c_bfgs_callback cb(camera_matrix,
        camera_matrix_inv,
        camera_matrix_t_inv,
        current_keypoints,
        reference_keypoints,
        Tfix,
        iTfix);

    c_bfgs::STATUS status =
        bfgs.run(cb, p);

    CF_DEBUG("bfgs: status='%s' %d iterations", toString(status) , bfgs.iterations() );

    //    if ( status < 0 ) {
    //      CF_ERROR("bfgs.run() fails");
    //      return false;
    //    }

  }

  /*
   * Unpack parameters from cv::Mat1d
   * */

  A(0) = p[0];
  A(1) = p[1];
  A(2) = p[2];

  switch (iTfix) {
    case 0:
      T = cv::Vec3d(Tfix, p[3], p[4]);
      break;
    case 1:
      T = cv::Vec3d(p[3], Tfix, p[4]);
      break;
    case 2:
      T = cv::Vec3d(p[3], p[4], Tfix);
      break;
    default:
      CF_FATAL("APP BUG: invalid iTfix=%d received", iTfix);
      return false;
  }

  T /= cv::norm(T);



  return true;
}

/** Experimental pure bfgs
 * Bad results - very sensitive to outliers
 *
 * Ji Zhao, "An Efficient Solution to Non-Minimal Case Essential Matrix Estimation", 2020
 *
 *  <An Efficient Solution to Non-Minimal Case Essential Matrix Estimation.pdf>
 *
 *    - Returned Fundamental Matrix F of CURRENT camera relative to REFERENCE camera
 *        AFTER derotation with homography H;
 *  */
bool bfgs_camera_pose_and_derotation_homography(
    /* in */ const cv::Matx33d & camera_matrix,
    /* in */ const std::vector<cv::Point2f> & current_keypoints,
    /* in */ const std::vector<cv::Point2f> & reference_keypoints,
    /* out, opt */ cv::Vec3d * outputEulerAnges,
    /* out, opt */ cv::Vec3d * outputTranslationVector,
    /* out, opt */ cv::Matx33d * outputRotationMatrix,
    /* out, opt */ cv::Matx33d * outputEssentialMatrix,
    /* out, opt */ cv::Matx33d * outputFundamentalMatrix,
    /* out, opt */ cv::Matx33d * outputDerotationHomography,
    /* in, out, opt */ cv::InputOutputArray M)
{
  INSTRUMENT_REGION("");

  // Check input arguments
  const int min_matched_points_required = 5;

  if ( (int)reference_keypoints.size() < min_matched_points_required ) {

    CF_ERROR("Invalid args: reference_keypoints.size()=%zu must be >= %d",
        reference_keypoints.size(),
        min_matched_points_required);

    return false;
  }

  if ( current_keypoints.size() != reference_keypoints.size() ) {

    CF_ERROR("Invalid args: current_keypoints.size()=%zu not equal to reference_keypoints.size()=%zu ",
        current_keypoints.size(), reference_keypoints.size());

    return false;
  }

  if ( M.needed() ) {

    if ( M.fixedType() && M.type() != CV_8UC1 ) {
      CF_ERROR("Invalid args: inliers_mask.type()=%d is invalid. Must be CV_8UC1", M.type());
      return false;
    }

    if ( !M.empty() && M.rows() != (int) current_keypoints.size() ) {
      CF_ERROR("Invalid args: inliers_mask.rows()=%d is invalid. Must be %zu", M.rows(), current_keypoints.size());
      return false;
    }
  }

  //
  // Go on
  //

  cv::Matx33d R, E, H;
  cv::Vec3d A(0, 0, 0), T(0, 0, 1);
  cv::Mat1b mask;
  bool fOk;

  if ( !M.empty() ) {
    mask = M.getMatRef();

    CF_DEBUG("estimate_essential_matrix: initial inliers = %d / %d",
        cv::countNonZero(mask),
        mask.rows);

  }


  //
  // Extimate camera pose using levmar
  //
  fOk =
      bfgs_refine_camera_pose2(A, T,
          camera_matrix,
          current_keypoints,
          reference_keypoints,
          mask);

  if ( !fOk ) {
    CF_ERROR("bfgs_refine_camera_pose2() fails");
  }


  //
  // Return to caller everything requested
  //
  if ( M.needed() && M.empty() ) {
    if ( M.kind() == cv::_InputArray::KindFlag::MAT ) {
      M.move(mask);
    }
    else {
      mask.copyTo(M);
    }
  }

  if ( outputEulerAnges ) {
    *outputEulerAnges = A;
  }

  if ( outputTranslationVector ) {
    * outputTranslationVector = T;
  }

  if ( outputRotationMatrix || outputEssentialMatrix || outputDerotationHomography || outputFundamentalMatrix ) {
    R = build_rotation(A);
    if ( outputRotationMatrix ) {
      * outputRotationMatrix = R;
    }
  }

  if ( outputDerotationHomography || outputFundamentalMatrix  ) {
    H = camera_matrix * R * camera_matrix.inv();
    if ( outputDerotationHomography ) {
      * outputDerotationHomography = H;
    }
  }

  if ( outputEssentialMatrix || outputFundamentalMatrix ) {
    E = compose_essential_matrix(R, T);
    if ( outputEssentialMatrix ) {
      * outputEssentialMatrix = E;
    }
  }

  if ( outputFundamentalMatrix ) {
    * outputFundamentalMatrix =
        compose_fundamental_matrix(E, camera_matrix) * H.inv() ;
  }

  return true;

}


