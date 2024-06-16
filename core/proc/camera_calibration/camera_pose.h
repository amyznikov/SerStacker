/*
 * camera_pose.h
 *
 *  Created on: Mar 4, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __camera_pose_h__
#define __camera_pose_h__

#include <opencv2/opencv.hpp>
#include <core/proc/pose.h>


// OpenCV version macro
#ifndef CV_VERSION_INT
# define CV_VERSION_INT(a,b,c) (((a)<<16)|((b)<<8)|(c))
#endif
#ifndef CV_VERSION_CURRRENT
# define CV_VERSION_CURRRENT CV_VERSION_INT(CV_VERSION_MAJOR, CV_VERSION_MINOR, CV_VERSION_REVISION)
#endif



/** @brief
 * enum ESSENTIAL_MATRIX_ESTIMATION_METHOD
 *
 * OpenCV has no typedef for such enum in calib3d.hpp,
 * but it is very desirable to have for usability reasons
 *
 * */
enum ESSENTIAL_MATRIX_ESTIMATION_METHOD {
  EMM_LMEDS = cv::LMEDS,
  EMM_RANSAC = cv::RANSAC,
  EMM_RHO = cv::RHO,

#if CV_VERSION_CURRRENT >= CV_VERSION_INT(4, 5, 0)
  EMM_USAC_DEFAULT = cv::USAC_DEFAULT,
  EMM_USAC_PARALLEL = cv::USAC_PARALLEL,
  EMM_USAC_FM_8PTS = cv::USAC_FM_8PTS,
  EMM_USAC_FAST = cv::USAC_FAST,
  EMM_USAC_ACCURATE = cv::USAC_ACCURATE,
  EMM_USAC_PROSAC = cv::USAC_PROSAC,
  EMM_USAC_MAGSAC = cv::USAC_MAGSAC,
#endif
};



/** @brief
 Combine given rotation matrix R and translation vector T into essentian matrix E

 <https://en.wikipedia.org/wiki/Essential_matrix>

 Berthold K.P. Horn, "Recovering Baseline and Orientation from Essential Matrix"
 <http://www-labs.iro.umontreal.ca/~sherknie/articles/HornBKP/essential.pdf>
 * */
template<class C>
inline cv::Matx<C, 3, 3> compose_essential_matrix(const cv::Matx<C, 3, 3> & R, const cv::Vec<C, 3> & T)
{
  return cross_product_matrix(T) * R; // this order gives regular not transposed E
}

/** @brief
 Combine given rotation matrix R, translation vector T and camera matrix into fundamental matrix F

 <https://en.wikipedia.org/wiki/Essential_matrix>

 Berthold K.P. Horn, "Recovering Baseline and Orientation from Essential Matrix"
 <http://www-labs.iro.umontreal.ca/~sherknie/articles/HornBKP/essential.pdf>
 * */
template<class C>
inline cv::Matx<C, 3, 3> compose_fundamental_matrix(const cv::Matx<C, 3, 3> & camera_matrix_t_inv,
    const cv::Matx<C, 3, 3> & camera_matrix_inv,
    const cv::Matx<C, 3, 3> & R,
    const cv::Vec<C, 3> & T)
{
  return camera_matrix_t_inv * compose_essential_matrix(R, T) * camera_matrix_inv;
}



/** @brief
 Combine given rotation matrix R, translation vector T and camera matrix into fundamental matrix F

 <https://en.wikipedia.org/wiki/Essential_matrix>

 Berthold K.P. Horn, "Recovering Baseline and Orientation from Essential Matrix"
 <http://www-labs.iro.umontreal.ca/~sherknie/articles/HornBKP/essential.pdf>
 * */
template<class C>
inline cv::Matx<C, 3, 3> compose_fundamental_matrix(const cv::Matx<C, 3, 3> & camera_matrix,
    const cv::Matx<C, 3, 3> & R, const cv::Vec<C, 3> & T)
{
  return compose_fundamental_matrix(camera_matrix.t().inv(), camera_matrix.inv(), R, T);
}


/** @brief
 Combine given essential matrix E, and camera matrix into fundamental matrix F
 * */
template<class C>
inline cv::Matx<C, 3, 3> compose_fundamental_matrix(const cv::Matx<C, 3, 3> & E,
    const cv::Matx<C, 3, 3> & camera_matrix)
{
  return camera_matrix.t().inv() * E * camera_matrix.inv();
}

/** @brief
 Combine given essential matrix E, and two camera matrces into fundamental matrix F
 * */
template<class C>
inline cv::Matx<C, 3, 3> compose_fundamental_matrix(const cv::Matx<C, 3, 3> & E,
    const cv::Matx<C, 3, 3> & camera_matrix0,
    const cv::Matx<C, 3, 3> & camera_matrix1)
{
  return camera_matrix1.t().inv() * E * camera_matrix0.inv();
}

/** @brief
 *  apply_homography()
 */
template<class MT, class PT>
inline cv::Vec<MT, 3> apply_homography(const cv::Matx<MT, 3, 3> & H, const cv::Vec<PT, 3> & cp)
{
  cv::Vec<MT, 3> v = H * cp;
  return v(2) ? v / v(2) : v;
}

/** @brief
 *  apply_homography()
 */
template<class MT, class PT>
inline cv::Point_<MT> apply_homography(const cv::Matx<MT, 3, 3> & H, const cv::Point_<PT> & cp)
{
  cv::Vec<MT, 3> v = H * cv::Vec<MT, 3>(cp.x, cp.y, 1);
  return v(2) ? cv::Point_<MT>(v(0) / v(2), v(1) / v(2)) : cv::Point_<MT>(v(0), v(1));
}


/** @brief
 *  compute_epipolar_line()
 *
 *  The returned line is given as
 *     a*x + b*y + c = 0
 */
template<class MT, class PT>
inline cv::Vec<MT, 3> compute_epipolar_line(const cv::Matx<MT, 3, 3> & F, const cv::Point_<PT> & cp)
{
  return F * cv::Vec<MT, 3>(cp.x, cp.y, 1);
}


/** @brief
 *  distance_from_point_to_corresponding_epipolar_line()
 *  Compute distance between reference pixel rp and epipolar line corresponding to pixel cp of current image
 */
template<class MT, class PT >
inline double distance_from_point_to_corresponding_epipolar_line(const cv::Matx<MT, 3, 3> & F,
    const cv::Point_<PT> & cp, const cv::Point_<PT> & rp)
{
  const cv::Vec<MT, 3> cvec(cp.x, cp.y, 1);
  const cv::Vec<MT, 3> rvec(rp.x, rp.y, 1);
  const cv::Vec<MT, 3> line = F * cvec;
  return rvec.dot(line) / sqrt(line(0) * line(0) + line(1) * line(1));
}

/** @brief
 *  compute_distances_from_points_to_corresponding_epipolar_lines()
 *
 *  Compute array of residual distances from points to corresponding epipolar lines.
 *  Used to estimate rms error in levmar solver.
 */
template<class MT, class PT>
inline void compute_distances_from_points_to_corresponding_epipolar_lines(cv::Mat1d & rhs,
    const cv::Matx<MT, 3, 3> & F,
    const std::vector<cv::Point_<PT> > & current_keypoints,
    const std::vector<cv::Point_<PT> > & reference_keypoints)
{
  const int n =
      current_keypoints.size();

  rhs.create(n, 1);

  for( int i = 0; i < n; ++i ) {
    rhs[i][0] = distance_from_point_to_corresponding_epipolar_line(F,
        current_keypoints[i], reference_keypoints[i]);
  }

}



/** @brief camera_direction()
 *    Return the camera's view forward direction for the camera rotated by R.
 * */
template<class C>
inline cv::Vec<C, 3> camera_direction(const cv::Matx<C, 3, 3> & R)
{
  return R.t() * cv::Vec3d(0,0,1);
}

/** @brief camera_up_direction()
 *    Return the camera's view up direction for the camera rotated by R.
 * */
template<class C>
inline cv::Vec<C, 3> camera_up_direction(const cv::Matx<C, 3, 3> & R)
{
  return R.t() * cv::Vec3d(0,-1,0);
}

/** @brief compute_relative_rotation()
 *    compute relative rotation Rij between two cameras Ri and Rj
 * */
template<class C>
inline const cv::Matx<C, 3, 3> compute_relative_rotation(const cv::Matx<C, 3, 3> & Ri, const cv::Matx<C, 3, 3> & Rj)
{
  return Rj * Ri.t();
}

/** @brief compute_relative_pose()
 *    compute relative pose between the two given cameras
 * */
template<class C>
inline void compute_relative_pose(const cv::Matx<C, 3, 3> & Ri, const cv::Vec<C, 3> & Ti,
    const cv::Matx<C, 3, 3> & Rj, const cv::Vec<C, 3> & Tj,
    cv::Matx<C, 3, 3> & Rij, cv::Vec<C, 3> & Tij)
{
  Rij = Rj * Ri.t();
  Tij = Ri * (Tj - Ti);
}

/**
 * Calculate Rotation Matrix to align vector v1 to vector v2 in 3D.
 * <https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d>
*/
template<class C>
inline cv::Matx<C, 3, 3> rotation_between_vectors(const cv::Vec<C, 3> & v1, const cv::Vec<C, 3> & v2)
{
  typedef cv::Vec<C, 3> cvVec;
  typedef cv::Matx<C, 3, 3> cvMat33;

  const cvVec a = v1 / cv::norm(v1);
  const cvVec b = v2 / cv::norm(v2);
  const cvVec v = a.cross(b);
  const cvMat33 vx = cross_product_matrix(v);
  const double c = a.dot(b);

  return cvMat33::eye() + vx + vx * vx / (1 + c);
}


/**
 * Compute two (left and right) epipoles from given fundamental matrix F.
 * Return false if epipoles can not be computed.
 *
 * CSE486, Penn State Robert Collins
 * Lecture 19: Essential and Fundamental Matrices
 * http://www.cse.psu.edu/~rtc12/CSE486/lecture19.pdf
 */
bool compute_epipoles(const cv::Matx33d & F, cv::Point2d * e1, cv::Point2d * e2);

/**
 * compute_epipoles()
 * @overload
 */
inline bool compute_epipoles(const cv::Matx33d & F, cv::Point2d e[2])
{
  return compute_epipoles(F, &e[0], &e[1]);
}

/**
 * compute_epipole()
 *
 *  Compute epipole position in pixels for given translation vector T using given camera matrix
 *
 *  The epipole is projection of second camera center onto first frame image
 */

template<class _Tp>
inline void compute_epipole(const cv::Matx<_Tp, 3, 3> & camera_matrix, const cv::Vec<_Tp, 3> & T,
    _Tp * Ex, _Tp * Ey)
{
  const cv::Vec<_Tp, 3> E =
      camera_matrix * T;

  *Ex = E[0] / E[2];
  *Ey = E[1] / E[2];
}

/**
 * compute_epipole()
 *
 *  Compute epipole position in pixels for given translation vector T using given camera matrix
 *
 *  The epipole is projection of second camera center onto first frame image
 *
 * @overload
 */
template<class _Tp>
inline cv::Point_<_Tp> compute_epipole(const cv::Matx<_Tp, 3, 3> & camera_matrix, const cv::Vec<_Tp, 3> & T)
{
  const cv::Vec<_Tp, 3> E =
      camera_matrix * T;

  return cv::Point_<_Tp> (E[0] / E[2], E[1] / E[2]);
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
    ESSENTIAL_MATRIX_ESTIMATION_METHOD method);

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
    cv::Matx33d &R1, cv::Matx33d &R2, cv::Vec3d &T);


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
    cv::InputOutputArray inliers);


/**
 * @brief
 *  recover_camera_pose_from_essential_matrix()
 *
 * Decompose essential matrix and recover camera pose using recover_camera_pose() above.
 */
bool recover_camera_pose_from_essential_matrix(
    const cv::Matx33d & E,
    const cv::Matx33d & camera_matrix,
    const std::vector<cv::Point2f> & current_keypoints,
    const std::vector<cv::Point2f> & reference_keypoints,
    cv::Matx33d * outputR,
    cv::Vec3d * outputA,
    cv::Vec3d * outputT,
    cv::InputOutputArray inliers);



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
    /* in, out, opt */ cv::InputOutputArray M);




/**
 *  Use of c_levmar_solver to refine camera pose guess
 *    - Returned Fundamental Matrix F of CURRENT camera relative to REFERENCE camera
 *        AFTER derotation with homography H;
 *  */

enum EPIPOLAR_MOTION_DIRECTION {
  EPIPOLAR_MOTION_FORWARD,
  EPIPOLAR_MOTION_BACKWARD,
  EPIPOLAR_MOTION_BOTH
};

struct c_lm_camera_pose_options
{
  double robust_threshold = 5.0;
  double epsf = 1e-5;
  double epsx = 1e-5;
  int max_iterations = 3;
  int max_levmar_iterations = 100;
  EPIPOLAR_MOTION_DIRECTION direction = EPIPOLAR_MOTION_FORWARD;
};

/**
 * Use of c_levmar_solver to refine camera pose estimated from essential matrix
 */
bool lm_refine_camera_pose(cv::Vec3d & A, cv::Vec3d & T,
    const cv::Matx33d & camera_matrix,
    const std::vector<cv::Point2f> & current_keypoints,
    const std::vector<cv::Point2f> & reference_keypoints,
    cv::Mat1b & inliers,
    const c_lm_camera_pose_options * opts = nullptr);

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
    /* in, out, opt */ cv::InputOutputArray M,
    /* in, opt */ const c_lm_camera_pose_options * opts = nullptr);


/** Experimental pure bfgs
 * Bad results - very sensitive to outliers,
 * ineffective solution for outliers detection,
 * robust function in lm_camera_pose_and_derotation_homography() works much better.
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
    /* in, out, opt */ cv::InputOutputArray M);

#endif /* __camera_pose_h__ */
