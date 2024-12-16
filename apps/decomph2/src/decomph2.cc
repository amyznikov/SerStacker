/*
 * decomph2.cc
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
#include <core/proc/image_registration/ecc2.h>
#include <core/proc/image_registration/c_frame_registration.h>
#include <core/io/load_image.h>
#include <core/io/save_image.h>
#include <core/ssprintf.h>
#include <core/debug.h>
#include "../../../core/proc/levmar.h"


static void pmat(const std::string & name, const cv::Matx23d & m)
{
  fprintf(stderr, "%s: {\n", name.c_str());

  for ( int i = 0; i < 2; ++i ) {

    for ( int j = 0; j < 3; ++j ) {
      fprintf(stderr, "%+12.4f ", m(i, j));
    }

    fprintf(stderr, "\n");
  }

  fprintf(stderr, "}\n", name);
}

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

  // cv::Vec3d A(0, 0, 0);
  cv::Vec3d A =
      cv::Vec3d(0, 0, 0) * CV_PI / 180;

  cv::Matx33d K(
      1e2, 0, 1e2,
      0, 1e2, 1e2,
      0, 0, 1);


  std::vector<double> params;
  c_solver_callback::pack_params(A, K, params);

//  pmat("INITIAL K", K);
//  pmat("INITIAL A", A * 180 / CV_PI);
//  pmatcmp("INITIAL Hgt | Hguess", H, c_solver_callback::unpack_homography(params));


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



  /**
   * Input images
   */
  std::string input_file_names[2] = {
      "/mnt/data/homography1/test4/F139.png",
      "/mnt/data/homography1/test4/F45.png",
      };

  cv::Mat input_images[3];
  cv::Mat remapped_image;

  for ( int i = 0; i < 2; ++i ) {

    if ( !load_image(input_file_names[i], input_images[i]) ) {
      CF_ERROR("load_image('%s') fails", input_file_names[i].c_str());
      return 1;
    }
  }


  c_image_registration_options opts;
  opts.motion_type = IMAGE_MOTION_HOMOGRAPHY;
  opts.ecc_registration_channel = color_channel_gray;
  opts.interpolation = ECC_INTER_LINEAR;
  opts.border_mode = ECC_BORDER_REFLECT101;
  opts.border_value = cv::Scalar(0, 0, 0);

  opts.feature_registration.enabled = true;
  opts.feature_registration.scale = 0.5;
  opts.feature_registration.registration_channel = color_channel_gray;

  // TBF
  opts.feature_registration.sparse_feature_extractor_and_matcher.detector.type = SPARSE_FEATURE_DETECTOR_AKAZE;
  opts.feature_registration.sparse_feature_extractor_and_matcher.detector.akaze.threshold = 0.00001;
  opts.feature_registration.sparse_feature_extractor_and_matcher.detector.akaze.nOctaves = 3;
  opts.feature_registration.sparse_feature_extractor_and_matcher.detector.akaze.nOctaveLayers = 4;

  //opts.feature_registration.sparse_feature_extractor_and_matcher.descriptor.type;
  opts.feature_registration.sparse_feature_extractor_and_matcher.matcher.type = FEATURE2D_MATCHER_HAMMING;
  opts.feature_registration.sparse_feature_extractor_and_matcher.matcher.hamming.max_acceptable_distance = 32;

  // TBF
  opts.ecc.enabled = true;
  opts.ecc.ecch_estimate_translation_first = false;
  opts.ecc.scale = 1;
  opts.ecc.min_rho = 0.5;
  opts.ecc.ecch_max_level = 5;
  opts.ecc.ecch_minimum_image_size = 4;

  opts.eccflow.enabled = false;

  c_frame_registration fg(opts);


    if( !fg.setup_reference_frame(input_images[1]) ) {
      CF_ERROR("fg.setup_reference_frame('%s') fails", input_file_names[1].c_str());
      return 1;
    }


  for ( int i = 0; i < 1; ++i ) {

    if ( !fg.register_frame(input_images[i], cv::noArray()) ) {
      CF_ERROR("fg.register_frame('%s') fails", input_file_names[i].c_str());
      continue;
    }

    const c_image_transform::sptr & transform =
        fg.image_transform();

    const c_homography_image_transform::sptr homography =
        std::dynamic_pointer_cast<c_homography_image_transform>(transform);

    if ( !homography ) {
      CF_ERROR("APP BUG : dynamic_pointer_cast fails");
      return 1;
    }

    const cv::Matx33d H =
        homography->matrix();//.inv();

//    H(2,0) = 0;
//    H(2,1) = 0;
//    H(2,2) = 1;
//    homography->set_matrix(H);

//    const c_affine_image_transform::sptr affine =
//        std::dynamic_pointer_cast<c_affine_image_transform>(transform);
//
//    if ( !affine ) {
//      CF_ERROR("APP BUG : dynamic_pointer_cast fails");
//      return 1;
//    }
//
//    const cv::Matx23d H =
//        affine->matrix();


    pmat(ssprintf("H[%d]", i), H);

    transform->remap(input_images[i], cv::noArray(),
        input_images[i].size(),
        remapped_image, cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT);


    const std::string output_file_name =
        ssprintf("%s.remapped.png", input_file_names[i].c_str());

    if ( !save_image(remapped_image, output_file_name)) {
      CF_ERROR("save_image('%s') fails", output_file_name.c_str());
      return 1;
    }

      /* Compute decomposition and compare with GT
     * */

    cv::Vec3d computedRotationVector;
    cv::Matx33d computedCameraMatrix;

    decompuse_pure_rotation_homograpy(H,
        computedRotationVector,
        computedCameraMatrix);

    const cv::Vec3d AComp =
        computedRotationVector * 180 / CV_PI;

    const cv::Matx33d & KComp =
        computedCameraMatrix;

    CF_DEBUG("\nA Comp[%d]: {%+g %+g %+g}", i,
        AComp(0), AComp(1), AComp(2));

    CF_DEBUG("\nKComp[%d]: {\n"
        "  %+g %+g %+g\n"
        "  %+g %+g %+g\n"
        "  %+g %+g %+g\n"
        "}\n"
        "",
        i,
        KComp(0, 0), KComp(0, 1), KComp(0, 2),
        KComp(1, 0), KComp(1, 1), KComp(1, 2),
        KComp(2, 0), KComp(2, 1), KComp(2, 2));


  }
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//  /* Ground truth data
//   * */
//
//  const cv::Vec3d givenRotationAngles =
//      cv::Vec3d(10, 40, -20) * CV_PI / 180;
//
//  const cv::Matx33d givenKameraMatrix =
//      cv::Matx33d(
//          1000, 0, 300,
//          0, 1000, 400,
//          0, 0, 1);
//
//  const cv::Matx33d RotationMatrix =
//      build_rotation(givenRotationAngles);
//
//
//  /* Input homography matrix computed from GT input
//   * */
//
//  const cv::Matx33d HomographyMatrix =
//      givenKameraMatrix * RotationMatrix * givenKameraMatrix.inv();
//
//
//  /* Compute decomposition and compare with GT
//   * */
//
//  cv::Vec3d computedRotationVector;
//  cv::Matx33d computedCameraMatrix;
//
//  decompuse_pure_rotation_homograpy(HomographyMatrix,
//      computedRotationVector, computedCameraMatrix);
//
//  if ( true ) {
//
//    const cv::Vec3d AComp =
//        computedRotationVector * 180 / CV_PI;
//
//    const cv::Vec3d AGT =
//        givenRotationAngles * 180 / CV_PI;
//
//    const cv::Matx33d & KComp =
//        computedCameraMatrix;
//
//    const cv::Matx33d & KGT =
//        givenKameraMatrix;
//
//    CF_DEBUG("\nA Comp: {%+g %+g %+g}",
//        AComp(0), AComp(1), AComp(2));
//
//    CF_DEBUG("\nAGT: {%+g %+g %+g}",
//        AGT(0), AGT(1), AGT(2));
//
//
//    CF_DEBUG("\nKComp: {\n"
//        "  %+g %+g %+g\n"
//        "  %+g %+g %+g\n"
//        "  %+g %+g %+g\n"
//        "}\n"
//        "",
//        KComp(0, 0), KComp(0,1), KComp(0, 2),
//        KComp(1, 0), KComp(1,1), KComp(1, 2),
//        KComp(2, 0), KComp(2,1), KComp(2, 2));
//
//
//    CF_DEBUG("\nKGT: {\n"
//        "  %+g %+g %+g\n"
//        "  %+g %+g %+g\n"
//        "  %+g %+g %+g\n"
//        "}\n"
//        "",
//        KGT(0, 0), KGT(0,1), KGT(0, 2),
//        KGT(1, 0), KGT(1,1), KGT(1, 2),
//        KGT(2, 0), KGT(2,1), KGT(2, 2));
//
//  }

  return 0;
}
