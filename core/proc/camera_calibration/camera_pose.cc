/*
 * camera_pose.cc
 *
 *  Created on: Mar 4, 2023
 *      Author: amyznikov
 */

#include "camera_pose.h"
#include <core/ssprintf.h>
#include <core/debug.h>


template<>
const c_enum_member * members_of<ESSENTIAL_MATRIX_ESTIMATION_METHOD>()
{
  static const c_enum_member members[] = {
      { EMM_LMEDS, "LMEDS", "least-median of squares algorithm" },
      { EMM_RANSAC, "RANSAC", "RANSAC algorithm" },
      { EMM_RHO, "RHO", "RHO algorithm" },
      { EMM_USAC_DEFAULT, "USAC_DEFAULT", "USAC algorithm, default settings" },
      { EMM_USAC_PARALLEL, "USAC_PARALLEL", "USAC, parallel version" },
      { EMM_USAC_FM_8PTS, "USAC_FM_8PTS", "USAC, fundamental matrix 8 points" },
      { EMM_USAC_FAST, "USAC_FAST", "USAC, fast settings" },
      { EMM_USAC_ACCURATE, "USAC_ACCURATE", "USAC, accurate settings" },
      { EMM_USAC_PROSAC, "USAC_PROSAC", "USAC, sorted points, runs PROSAC" },
      { EMM_USAC_MAGSAC, "USAC_MAGSAC", "USAC, runs MAGSAC++" },
      { EMM_LMEDS, nullptr, "" },
  };

  return members;
}

/**
 * Use cv::LMSolver to refine camera pose estimated from essential matrix
 */
static bool lm_refine_camera_pose(cv::Vec3d & A, cv::Vec3d & T,
    const cv::Matx33d & camera_matrix,
    const std::vector<cv::Point2f> & current_keypoints,
    const std::vector<cv::Point2f> & reference_keypoints,
    const cv::Mat1b & inliers)
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

      CF_ERROR("Invalid args: inliers_mask.rows()=%d not equal to matched_reference_keypoints.size()=%d",
          inliers.rows, reference_keypoints.size());

      return false;
    }
  }

  class LMSolverCallback:
      public cv::LMSolver::Callback
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
    LMSolverCallback(const cv::Matx33d & _camera_matrix,
        const std::vector<cv::Point2f> & _current_keypoints,
        const std::vector<cv::Point2f> & _reference_keypoints,
        const cv::Mat1b & _inliers,
        const double _Tfix,
        const int _iTfix ) :

        camera_matrix(_camera_matrix),
        current_keypoints(_current_keypoints),
        reference_keypoints(_reference_keypoints),
        inliers(_inliers),
        Tfix(_Tfix),
        iTfix(_iTfix),
        numInliers(_inliers.empty() ? _current_keypoints.size() : cv::countNonZero(_inliers))
    {
      camera_matrix_t_inv =
          camera_matrix.t().inv();

      camera_matrix_inv =
          camera_matrix.inv();
    }

    /**
     * computes rhs error for the specified vector of parameters
     */
    bool compute(const cv::Mat1d & p, cv::Mat1d & rhs) const
    {
      const cv::Vec3d A(p[0][0], p[1][0], p[2][0]);

      cv::Vec3d T;

      switch (iTfix) {
        case 0:
          T = cv::Vec3d(Tfix, p[3][0], p[4][0]);
          break;
        case 1:
          T = cv::Vec3d(p[3][0], Tfix, p[4][0]);
          break;
        case 2:
          T = cv::Vec3d(p[3][0], p[4][0], Tfix);
          break;
        default:
          CF_FATAL("APP BUG: invalid iTfix=%d received", iTfix);
          return false;
      }

      T *= 1. / cv::norm(T);

      const cv::Matx33d F =
          compose_fundamental_matrix(camera_matrix_t_inv,
              camera_matrix_inv,
              build_rotation(A),
              T);

      if( inliers.empty() ) {

        // case when no inliers mask
        for( int i = 0, n = current_keypoints.size(); i < n; ++i ) {

          rhs[i][0] = distance_from_point_to_corresponding_epipolar_line(F,
              current_keypoints[i], reference_keypoints[i]);
        }
      }
      else {

        // case when inliers mask is provided
        for( int i = 0, j = 0, n = current_keypoints.size(); i < n; ++i ) {
          if( inliers[i][0] ) {
            rhs[j++][0] = distance_from_point_to_corresponding_epipolar_line(F,
                current_keypoints[i], reference_keypoints[i]);
          }
        }
      }

      return true;
    }

    /**
     computes error and Jacobian for the specified vector of parameters

     @param param the current vector of parameters
     @param err output vector of errors: err_i = actual_f_i - ideal_f_i
     @param J output Jacobian: J_ij = d(ideal_f_i)/d(param_j)

     when J=noArray(), it means that it does not need to be computed.
     Dimensionality of error vector and param vector can be different.
     The callback should explicitly allocate (with "create" method) each output array
     (unless it's noArray()).
     */
    bool compute(cv::InputArray param, cv::OutputArray err, cv::OutputArray J) const override
    {
      err.create(numInliers, 1, CV_64F);

      const cv::Mat1d P =
          param.getMat();

      cv::Mat1d rhs =
          err.getMat();

      if ( !compute(P, rhs) ) {
        return false;
      }

      if( J.needed() ) {

        cv::Mat1d rhs1(numInliers, 1);
        cv::Mat1d rhs2(numInliers, 1);

        J.create(numInliers, P.rows, CV_64F);
        cv::Mat1d jac = J.getMatRef();

        cv::Mat1d PP = P.clone();

        const double eps = 1e-3;

        for( int j = 0; j < PP.rows; ++j ) {

          const double v =
              PP[j][0];

          const double delta =
              std::max(1e-6, eps * std::abs(v));

          PP[j][0] = v + delta;
          compute(PP, rhs1);

          PP[j][0] = v - delta;
          compute(PP, rhs2);

          cv::subtract(rhs1, rhs2, jac.col(j));
          cv::multiply(jac.col(j), 0.5 / delta, jac.col(j));

          PP[j][0] = v;
        }
      }

      return true;
    }
  };


  /*
   * Find the translation component of maximal magnitude to fix it
   * iTmax = argmax(i, abs( T[i] ) )
  */
  double Tfix = T(0);
  int iTfix = 0;
  for ( int i = 1; i < 3; ++i ) {
    if ( fabs(T[i]) > fabs(Tfix) ) {
      Tfix = T[iTfix = i];
    }
  }

  /*
   * Pack parameters into cv::Mat1d
   * */
  cv::Mat1d p(5, 1);

  p[0][0] = A(0);
  p[1][0] = A(1);
  p[2][0] = A(2);
  for ( int i = 0, j = 3; i < 3; ++i ) {
    if ( i != iTfix ) {
      p[j++][0] = T[i];
    }
  }

  const cv::Ptr<cv::LMSolver::Callback> cb(
      new LMSolverCallback(
          camera_matrix,
          current_keypoints,
          reference_keypoints,
          inliers,
          Tfix,
          iTfix));

  cv::Ptr<cv::LMSolver> lm =
      cv::LMSolver::create(cb, 200);

  int iterations =
      lm->run(p);

  CF_DEBUG("lm->run(): iterations= %d", iterations);

  /*
   * Unpack parameters from cv::Mat1d
   * */

  A(0) = p[0][0];
  A(1) = p[1][0];
  A(2) = p[2][0];

  switch (iTfix) {
    case 0:
      T = cv::Vec3d(Tfix, p[3][0], p[4][0]);
      break;
    case 1:
      T = cv::Vec3d(p[3][0], Tfix, p[4][0]);
      break;
    case 2:
      T = cv::Vec3d(p[3][0], p[4][0], Tfix);
      break;
    default:
      CF_FATAL("APP BUG: invalid iTfix=%d received", iTfix);
      return false;
  }

  return true;
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

      CF_ERROR("Invalid args: inliers_mask.rows()=%d not equal to matched_reference_keypoints.size()=%d",
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

  const cv::Mat EE =
      cv::findEssentialMat(C, R,
          camera_matrix,
          method,
          0.999,
          threshold,
          1000,
          M);

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

    // Assume that T is alreasy normed:  cv::norm(T) == 1.
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

      CF_DEBUG("pose[%d] ngood=%d T=(%g %g %g)", i,
          pose_hypothesis[i].ngood,
          pose_hypothesis[i].T(0),
          pose_hypothesis[i].T(1),
          pose_hypothesis[i].T(2));
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
 * @brief estimate_camera_pose_and_derotation_homography()
 *
 * This routine calls @ref estimate_essential_matrix() in order to
 * find essential matrix from sparse feature matches and recover pose of current camera relative to reference camera.
 *
 * On the output it returns :
 *    EulerAnges of rotation matrix R in radians;
 *    Normalized translation vector T of current camera relative to reference camera;
 *    Rotation matrix R of current camera relative to reference camera;
 *    Derotation homography matrix H current camera relative to reference camera;
 *    Essential matrix E of current camera relative to reference camera;
 *    Fundamental matrix F of current camera relative to reference camera after derotation with homography H;
 *
 * The derotation homography matrix H can be used later to derotate and rectify RGB frames or sparse feature positions.
 *
 * In order to apply H to current RGB image use:
 * @code
 *    cv::Mat derotated_current_image;
 *    cv::warpPerspective(current_image, derotated_current_image,
 *      H,
 *      current_image.size(),
 *      cv::INTER_LINEAR,
 *      cv::BORDER_CONSTANT);
 * @endcode
 *
 * In order to apply H to current sparse points use:
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

    CF_DEBUG("estimate_essential_matrix: initial inliers=%d/%d",
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
  if( true /*is_levmar_enabled() */) {

    CF_DEBUG("\nbefore lm_refine_camera_pose:\n"
        "A = (%+g %+g %+g)\n"
        "T = (%+g %+g %+g)\n"
        "\n",
        A(0) * 180 / CV_PI, A(1) * 180 / CV_PI, A(2) * 180 / CV_PI,
        T(0), T(1), T(2));

    fOk =
        lm_refine_camera_pose(A, T,
            camera_matrix,
            current_keypoints,
            reference_keypoints,
            mask);

    CF_DEBUG("\nafter lm_refine_camera_pose: %d\n"
        "A = (%+g %+g %+g)\n"
        "T = (%+g %+g %+g)\n"
        "\n",
        fOk,
        A(0) * 180 / CV_PI, A(1) * 180 / CV_PI, A(2) * 180 / CV_PI,
        T(0), T(1), T(2));
  }

  //
  // Return to caller everything requsted
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

