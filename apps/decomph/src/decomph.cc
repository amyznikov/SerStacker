/*
 * decomph.cc
 *
 *  Created on: Aug 16, 2024
 *      Author: amyznikov
 *
 * Experimental test to check for pure rotation homography decomposition into
 * pure rotation and camera intrinsics matrix.
 */

#include <core/proc/levmar.h>
#include <core/proc/levmar2.h>
#include <core/proc/pose.h>
#include <core/ssprintf.h>
#include <core/debug.h>
#include "../../../core/proc/levmar.h"

static void pmat(const std::string & name, const cv::Matx33d & m)
{
  fprintf(stderr, "%s: {\n", name.c_str());

  for ( int i = 0; i < 3; ++i ) {

    for ( int j = 0; j < 3; ++j ) {
      fprintf(stderr, "%+12.4f ", m(i, j));
    }

    fprintf(stderr, "\n");
  }

  fprintf(stderr, "}\n", name);
}

static void pmat(const std::string & name, const cv::Vec3d & m)
{
  fprintf(stderr, "%s: {\n", name.c_str());

  for ( int j = 0; j < 3; ++j ) {
    fprintf(stderr, "%+12.4f ", m(j));
  }

  fprintf(stderr, "\n}\n", name);
}

static void pmatcmp(const std::string & name, const cv::Matx33d & m1, const cv::Matx33d & m2)
{
  fprintf(stderr, "%s: {\n", name.c_str());

  for ( int i = 0; i < 3; ++i ) {

    for ( int j = 0; j < 3; ++j ) {
      fprintf(stderr, "%+12.4f ", m1(i, j));
    }

    fprintf(stderr, "| ");

    for ( int j = 0; j < 3; ++j ) {
      fprintf(stderr, "%+12.4f ", m2(i, j));
    }

    fprintf(stderr, "\n");
  }

  fprintf(stderr, "}\n", name);
}

static bool decompuse_pure_rotation_homograpy(const cv::Matx33d & H,
    cv::Vec3d & outputRotationVector, cv::Matx33d & outputCameraMatrix)
{

  typedef c_levmar_solver c_lm_solver;
  //typedef c_levmar_solver<double> c_lm_solver;

  class c_solver_callback:
      public c_lm_solver::callback
  {
    cv::Matx33d _H;
  public:

    c_solver_callback(cv::Matx33d H) :
      _H(H)
    {
    }

    static void pack_params(const cv::Vec3d & A, const cv::Matx33d & K,
        std::vector<double> & p)
    {
      p.resize(8);
      p[0] = A(0);
      p[1] = A(1);
      p[2] = A(2);
      p[3] = K(0, 0);
      p[4] = K(0, 2);
      p[5] = K(1, 1);
      p[6] = K(1, 2);
      p[7] = K(2, 2);
    }

    static void unpack_params(cv::Vec3d & A, cv::Matx33d & K,
        const std::vector<double> & p)
    {
      A(0) = p[0];
      A(1) = p[1];
      A(2) = p[2];

      K(0, 0) = p[3];
      K(0, 1) = 0;
      K(0, 2) = p[4];

      K(1, 0) = 0;
      K(1, 1) = p[5];
      K(1, 2) = p[6];

      K(2, 0) = 0;
      K(2, 1) = 0;
      //K(2, 2) = 1;
      K(2, 2) = p[7];
    }

    static cv::Matx33d unpack_homography(const std::vector<double> & p)
    {
      cv::Vec3d A;
      cv::Matx33d K;

      unpack_params(A, K, p);

      return K * build_rotation(A) * K.inv();
    }

    bool compute(const std::vector<double> & p, std::vector<double> & rhs,
        cv::Mat_<double> * jac, bool * have_analytical_jac) final
    {
      rhs.resize(13);

      const cv::Matx33d H =
          unpack_homography(p);

      for( int y = 0; y < 3; ++y ) {
        for( int x = 0; x < 3; ++x ) {
          rhs[y * 3 + x] = H(y, x) - _H(y, x);
        }
      }

      rhs[9] = std::max(0., -p[3]);
      rhs[10] = std::max(0., -p[5]);
      rhs[11] = std::max(0., -p[4]);
      rhs[12] = std::max(0., -p[6]);

      return true;
    }

  };

  /* Pack initial guess
   * */

  cv::Vec3d A =
      cv::Vec3d(-10, 10, -10) * CV_PI / 180;

  cv::Matx33d K(
      1e3, 0, 1e2,
      0, 1e3, 1e2,
      0, 0, 1);

  std::vector<double> params;
  c_solver_callback::pack_params(A, K, params);

  pmat("INITIAL K", K);
  pmat("INITIAL A", A * 180 / CV_PI);
  pmatcmp("INITIAL Hgt | Hguess", H, c_solver_callback::unpack_homography(params));


  /*  Create and run the lm solver
   * */

  c_lm_solver lm(10000, 1e-15);
  c_solver_callback cb(H);

  int iterations =
      lm.run(cb, params);

  CF_DEBUG("lm.run: %d iterations",
      iterations);

  pmatcmp("FINAL Hgt| Hcomp", H, c_solver_callback::unpack_homography(params));

  c_solver_callback::unpack_params(A, K, params);

  outputRotationVector = A;
  outputCameraMatrix = K / K(2, 2);

  return true;
}


int main(int argc, char *argv[])
{
  cf_set_loglevel(CF_LOG_DEBUG);
  cf_set_logfile(stderr);


  /* Ground truth data
   * */

  const cv::Vec3d givenRotationAngles =
      cv::Vec3d(10, 40, -20) * CV_PI / 180;

  const cv::Matx33d givenKameraMatrix =
      cv::Matx33d(
          5000, 0, 500,
          0, 5000, 600,
          0, 0, 1);

  const cv::Matx33d RotationMatrix =
      build_rotation(givenRotationAngles);


  /* Input homography matrix computed from GT input
   * */

  const cv::Matx33d HomographyMatrix =
      givenKameraMatrix * RotationMatrix * givenKameraMatrix.inv();


  /* Compute decomposition and compare with GT
   * */

  cv::Vec3d computedRotationVector;
  cv::Matx33d computedCameraMatrix;

  decompuse_pure_rotation_homograpy(HomographyMatrix,
      computedRotationVector, computedCameraMatrix);

  if ( true ) {

    const cv::Vec3d AComp =
        computedRotationVector * 180 / CV_PI;

    const cv::Vec3d AGT =
        givenRotationAngles * 180 / CV_PI;

    const cv::Matx33d & KComp =
        computedCameraMatrix;

    const cv::Matx33d & KGT =
        givenKameraMatrix;

    CF_DEBUG("\nA Comp: {%+g %+g %+g}",
        AComp(0), AComp(1), AComp(2));

    CF_DEBUG("\nAGT: {%+g %+g %+g}",
        AGT(0), AGT(1), AGT(2));


    CF_DEBUG("\nKComp: {\n"
        "  %+g %+g %+g\n"
        "  %+g %+g %+g\n"
        "  %+g %+g %+g\n"
        "}\n"
        "",
        KComp(0, 0), KComp(0,1), KComp(0, 2),
        KComp(1, 0), KComp(1,1), KComp(1, 2),
        KComp(2, 0), KComp(2,1), KComp(2, 2));


    CF_DEBUG("\nKGT: {\n"
        "  %+g %+g %+g\n"
        "  %+g %+g %+g\n"
        "  %+g %+g %+g\n"
        "}\n"
        "",
        KGT(0, 0), KGT(0,1), KGT(0, 2),
        KGT(1, 0), KGT(1,1), KGT(1, 2),
        KGT(2, 0), KGT(2,1), KGT(2, 2));

  }

  return 0;
}
