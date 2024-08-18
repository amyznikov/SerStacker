/*
 * decomph3.cc
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

static cv::Matx33d createHomography(const cv::Vec3d & RotationVector, const cv::Matx33d & CameraMatrix)
{
  return CameraMatrix * build_rotation(RotationVector) * CameraMatrix.inv();
}

static bool estimate_pure_rotation_homograpy(const std::vector<cv::Point2f> & current_positions,
    const std::vector<cv::Point2f> & reference_positions,
    cv::Vec3d & RotationVector, cv::Matx33d & CameraMatrix)
{

  typedef c_levmar_solver c_lm_solver;
  //typedef c_levmar_solver<double> c_lm_solver;

  class c_solver_callback:
      public c_lm_solver::callback
  {
    const std::vector<cv::Point2f> & _current_positions;
    const std::vector<cv::Point2f> & _reference_positions;
    const std::vector<uint8_t> & _inliers;
    size_t _num_inliers = 0;
    double _robust_threshold = 500;

  public:

    c_solver_callback(const std::vector<cv::Point2f> & current_positions,
        const std::vector<cv::Point2f> & reference_positions,
        const std::vector<uint8_t> & inliers) :
        _current_positions(current_positions),
        _reference_positions(reference_positions),
        _inliers(inliers)
    {

      for ( const uint8_t & v : _inliers ) {
        if ( v ) {
          ++_num_inliers;
        }
      }
    }

    void set_robust_threshold(double v)
    {
      _robust_threshold = v;
    }

    static void pack_params(const cv::Vec3d & A, const cv::Matx33d & K,
        std::vector<double> & p)
    {
      const double & F =
          K(0, 0);

      const double & cx =
          K(0, 2);

      const double & cy =
          K(1, 2);

      //  const double & w =
      //    K(2, 2);

      p.resize(6);

      p[0] = A(0);
      p[1] = A(1);
      p[2] = A(2);
      p[3] = F;
      p[4] = cx;
      p[5] = cy;
      // p[6] = w;
    }

    static void unpack_params(cv::Vec3d & A, cv::Matx33d & K,
        const std::vector<double> & p)
    {
      A(0) = p[0];
      A(1) = p[1];
      A(2) = p[2];

      const double & F =
          p[3];

      const double & cx =
         p[4];

      const double & cy =
          p[5];

      K = cv::Matx33d(
          F, 0, cx,
          0, F, cy,
          0, 0, 1);
    }

    static cv::Matx33d unpack_homography(const std::vector<double> & p)
    {
      cv::Vec3d A;
      cv::Matx33d K;

      unpack_params(A, K, p);

      return createHomography(A, K);
    }

    double robust_function(double x) const
    {
//      return x;
      //constexpr double robust_threshold = 200;
      return x > 0 ? std::min(x, _robust_threshold) :
          std::max(x, -_robust_threshold);
    }

    bool compute(const std::vector<double> & p, std::vector<double> & rhs,
        cv::Mat1d * jac, bool * have_analytical_jac) const final
    {
      const cv::Matx33d H =
          unpack_homography(p);

      std::vector<cv::Point2f> _warped_current_positions;

      cv::perspectiveTransform(_current_positions,
          _warped_current_positions,
          H);

      const size_t N =
          _reference_positions.size();

      rhs.resize(2 * _num_inliers);

      for( size_t i = 0, j = 0; i < N; ++i ) {
        if ( _inliers[i] ) {
          rhs[2 * j + 0] = robust_function(_reference_positions[i].x - _warped_current_positions[i].x);
          rhs[2 * j + 1] = robust_function(_reference_positions[i].y - _warped_current_positions[i].y);
          ++j;
        }
      }

      return true;
    }

  };

  /* Pack initial guess
   * */

  std::vector<double> params;
  c_solver_callback::pack_params(RotationVector, CameraMatrix, params);


  std::vector<uint8_t> inliers(current_positions.size(), 255);
  std::vector<cv::Point2f> warped_current_positions;

  /*  Create and run the lm solver
   * */

  c_lm_solver lm(10000, 1e-15);

  double robust_threshold = 500;

  for ( int i = 0; i < 10; ++i ) {

    c_solver_callback cb(current_positions, reference_positions, inliers);
    cb.set_robust_threshold(robust_threshold);

    const int iterations =
        lm.run(cb, params);

    CF_DEBUG("[%d] lm.run: %d iterations rmse=%g", i, iterations, lm.rmse());
    pmat("H", c_solver_callback::unpack_homography(params));

    if ( iterations < 0 ) {
      CF_ERROR("lm.run() fails");
      break;
    }


    const double rmse =
        lm.rmse();

    const double rmse_thresold =
        16 * rmse * rmse;

    int outliers = 0;

    cv::perspectiveTransform(current_positions,
        warped_current_positions,
        c_solver_callback::unpack_homography(params));

    for ( size_t k = 0, N = warped_current_positions.size(); k < N; ++k ) {
      if ( inliers[k] ) {

        const cv::Point2f & cp =
            warped_current_positions[k];

        const cv::Point2f & rp =
            reference_positions[k];

        const double d2 =
            (cp.x - rp.x) * (cp.x - rp.x) + (cp.y - rp.y) * (cp.y - rp.y);

        if ( d2 > rmse_thresold ) {
          inliers[k] = 0;
          ++outliers;
        }

      }
    }

    CF_DEBUG("%d outliers", outliers);

    if ( outliers < 1 ) {
      break;
    }

    robust_threshold =
        5 * rmse;

  }



  c_solver_callback::unpack_params(RotationVector, CameraMatrix, params);

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
      //"/mnt/data/homography1/test2/F501.png",
      //"/mnt/data/homography1/test2/F455.png",
      //"/mnt/data/homography1/test2/F508.png",

      //"/mnt/data/homography1/test5/F0.png",
      "/mnt/data/homography1/test5/F39.png",
       "/mnt/data/homography1/test5/F427.png",

//      "/mnt/data/homography1/test4/F139.png",
//      "/mnt/data/homography1/test4/F45.png",

      };

  cv::Mat input_images[2];
  cv::Mat remapped_image;

  for ( int i = 0; i < 2; ++i ) {

    if ( !load_image(input_file_names[i], input_images[i]) ) {
      CF_ERROR("load_image('%s') fails", input_file_names[i].c_str());
      return 1;
    }
  }

  c_sparse_feature_extractor_and_matcher_options opts;
  opts.detector.type = SPARSE_FEATURE_DETECTOR_AKAZE;
  opts.detector.akaze.threshold = 0.0001;
  opts.detector.akaze.nOctaves = 4;
  opts.detector.akaze.nOctaveLayers = 4;
  opts.matcher.type = FEATURE2D_MATCHER_HAMMING;
  opts.matcher.hamming.max_acceptable_distance = 24;

  c_sparse_feature_extractor_and_matcher::sptr matcher =
      c_sparse_feature_extractor_and_matcher::create(opts);

  if ( !matcher ) {
    CF_ERROR("sparse_feature_extractor_and_matcher::create() fails");
    return 1;
  }

  if( !matcher->setup_reference_frame(input_images[0], cv::noArray()) ) {
    CF_ERROR("matcher->setup_reference_frame() fails");
    return 1;
  }

  CF_DEBUG("reference_keypoints: %zu", matcher->referece_keypoints().size());
  if ( matcher->referece_keypoints().size() < 4 ) {
    CF_ERROR("No enough reference keypoints extracted");
    return 1;
  }

  if ( !matcher->match_current_frame(input_images[1], cv::noArray()) ) {
    CF_ERROR("matcher->match_current_frame() fails");
    return 1;
  }


  CF_DEBUG("matched keypoints: %zu", matcher->matched_reference_positions().size());
  if ( matcher->matched_reference_positions().size() < 4 ) {
    CF_ERROR("No enough matches extracted");
    return 1;
  }

  const std::vector<cv::Point2f> & reference_positions =
      matcher->matched_reference_positions();

  const std::vector<cv::Point2f> & current_positions =
      matcher->matched_current_positions();

//  cv::Vec3d RotationVector =
//      cv::Vec3d(2,15,1) * CV_PI / 180;

  cv::Vec3d RotationVector =
      cv::Vec3d(0, 0, 0) * CV_PI / 180;

  const double F = 1000;
      //10 * std::max(input_images[0].rows, input_images[0].cols);

  cv::Matx33d CameraMatrix(
      F, 0, input_images[0].cols / 2,
      0, F, input_images[0].rows / 2,
      0, 0, 1
      );

  estimate_pure_rotation_homograpy(current_positions, reference_positions,
      RotationVector, CameraMatrix);

  const cv::Matx33d H =
      createHomography(RotationVector,
          CameraMatrix);

  cv::warpPerspective(input_images[1], remapped_image,
      H,
      input_images[0].size(),
      cv::INTER_LINEAR,
      cv::BORDER_CONSTANT);

  const std::string output_file_name =
      ssprintf("%s.remapped.png", input_file_names[1].c_str());

  if ( !save_image(remapped_image, output_file_name)) {
    CF_ERROR("save_image('%s') fails", output_file_name.c_str());
  }



  if ( true ) {

    const cv::Vec3d AComp =
        RotationVector * 180 / CV_PI;

    const cv::Matx33d & KComp =
        CameraMatrix;

    CF_DEBUG("\nA Comp: {%+g %+g %+g}",
        AComp(0), AComp(1), AComp(2));

    CF_DEBUG("\nKComp: {\n"
        "  %+g %+g %+g\n"
        "  %+g %+g %+g\n"
        "  %+g %+g %+g\n"
        "}\n"
        "",
        KComp(0, 0), KComp(0,1), KComp(0, 2),
        KComp(1, 0), KComp(1,1), KComp(1, 2),
        KComp(2, 0), KComp(2,1), KComp(2, 2));

  }

  return 0;
}
