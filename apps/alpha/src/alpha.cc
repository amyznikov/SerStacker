/*
 * alpha.cc
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */

#include <core/proc/eccalign.h>
#include <core/improc/c_image_processor.h>
#include <core/improc/c_unsharp_mask_routine.h>
#include <core/improc/c_align_color_channels_routine.h>
#include <core/improc/c_rangeclip_routine.h>
#include <core/io/save_image.h>
#include <core/io/load_image.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/downstrike.h>
#include <core/proc/unsharp_mask.h>
#include <core/proc/morphology.h>
#include <core/proc/geo-reconstruction.h>
#include <core/registration/c_feature_based_registration.h>
#include <core/proc/planetary-disk-detection.h>
#include <core/proc/jupiter.h>
#include <core/proc/fft.h>
#include <core/settings.h>
#include <core/ssprintf.h>
#include <core/readdir.h>
#include <core/get_time.h>
#include <core/proc/inpaint.h>
#include <core/registration/c_planetary_disk_registration.h>
#include <tbb/tbb.h>
#include <core/debug.h>

#include <levmar/levmar.h>

//
// Model to be fitted:
//
//  dist( L([ex,ey], [xr,yr]), [x0, y0] )^2 = 0
//
//  x0 = | ca  -sa | | xc - ex |
//  y0 = | sa   ca | | yc - ey |
//
//  <https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line>
//   x1 = ex
//   y1 = ey
//   x2 = xr
//   y2 = yr
//
//  dist( L([ex,ey], [xr,yr]), [x0, y0] ) =
//
//     (  (xr-ex)*(ey-y0) - (ex-x0)*(yr-ey) )  / sqrt( (xr-ex)**2 + (yr-ey)**2  )
//
//


static const double square(double x)
{
  return x * x;
}

// Model to be fitted:

static double compute_model(const double p[], double lhs[], int m, int n, const double pts[])
{
  const double ex = p[0];
  const double ey = p[1];
  const double a  = m > 2 ? p[2] / 1000 : 0;

  double sum = 0;

  for ( int i = 0; i < n; ++i, pts += 4 ) {

    const double xc = pts[0];
    const double yc = pts[1];
    const double xr = pts[2];
    const double yr = pts[3];

    //  <https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line>

    const double x1 = ex; // P1.x
    const double y1 = ey; // P1.y

    const double x2 = xr; // P2.x
    const double y2 = yr; // P2.y

    const double x0 = xc;
    const double y0 = yc;

    const double v = abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1))
        / sqrt(square(x2 - x1) + square(y2 - y1));

    sum += v;

    if ( lhs ) {
      lhs[i] = v;
    }
  }

  return sum;
}

static void modelfunc(double p[], double lhs[], int m, int n, void *data)
{
  const double ex = p[0];
  const double ey = p[1];
  const double a  = m > 2 ? p[2] / 1000 : 0;

  double sum =
      compute_model(p, lhs, m, n,
          (const double*) data);

//  fprintf(stderr, "ex=%+12.6f ey=%+12.6f a=%+12.6f sum=%g\n",
//      ex, ey, a,
//      sum);
}


//// Jacobian of the model
//void jacfunc(double p[], double jac[], int m, int n, void * data)
//{
//  const double ex = p[0];
//  const double ey = p[1];
//  const double a  = p[2];
//  const double * pts = (const double * )data;
//
//  /* fill Jacobian row by row */
//  for ( int i = 0; i < n; ++i, jac += 3, pts += 4 ) {
//
//    const double xc = pts[0];
//    const double yc = pts[1];
//    const double xr = pts[2];
//    const double yr = pts[3];
//
//    const double xre = xr - ex;
//    const double yre = yr - ey;
//    const double xce = xc - ex;
//    const double yce = yc - ey;
//    const double den = (square(yre) + square(xre));
//
//    jac[0] = ;
//
//
//    jac[1] = ;
//
//
//    jac[2] = ;
//
//  }
//}

// Jacobian of the model
void jacfunc_sqr(double p[], double jac[], int m, int n, void * data)
{
  const double ex = p[0];
  const double ey = p[1];
  const double a  = p[2];
  const double * pts = (const double * )data;

  /* fill Jacobian row by row */
  for ( int i = 0; i < n; ++i, jac += 3, pts += 4 ) {

    const double xc = pts[0];
    const double yc = pts[1];
    const double xr = pts[2];
    const double yr = pts[3];

    const double xre = xr - ex;
    const double yre = yr - ey;
    const double xce = xc - ex;
    const double yce = yc - ey;
    const double den = (square(yre) + square(xre));

    jac[0] = 2 *
        ((-(cos(a) + 1) * yre + cos(a) * yce + sin(a) * xre + sin(a) * xce - ey) * (xre * (-cos(a) * yce - sin(a) * xce + ey) - (sin(a) * yce - cos(a) * xce + ex) * yre)) / (den)
        +
        xre *   square(xre * (-cos(a) * yce - sin(a) * xce + ey) - (sin(a) * yce - cos(a) * xce + ex) * yre) / (den * den);


    jac[1] = 2 *
        ((sin(a) * yre + sin(a) * yce + (cos(a) + 1) * xre - cos(a) * xce + ex) * (xre * (-cos(a) * yce - sin(a) * xce + ey) - (sin(a) * yce - cos(a) * xce + ex) * yre)) / (den)
        +
        yre * square(xre * (-cos(a) * yce - sin(a) * xce + ey) - (sin(a) * yce - cos(a) * xce + ex) * yre) / (den * den);


    jac[2] = 2 *
        (xre * (sin(a) * yce - cos(a) * xce) - yre * (cos(a) * yce + sin(a) * xce))
          *
        (xre * (-cos(a) * yce - sin(a) * xce + ey) - yre * (sin(a) * yce - cos(a) * xce + ex) ) / (den);

  }
}


static bool compute_epipole(const cv::Matx33d & F, double * ex, double * ey )
{
  /*
    * SVD-based solution is a little faster
    */

  double e1x, e1y, e2x, e2y ;

   try {

     cv::SVD svd(F);

    const cv::Matx13d e1 = svd.vt.row(2);

    e1x = e1(0, 0) / e1(0, 2);
    e1y = e1(0, 1) / e1(0, 2);

    const cv::Matx31d e2 = svd.u.col(2);
    e2x = e2(0, 0) / e2(2, 0);
    e2y = e2(1, 0) / e2(2, 0);

    *ex = 0.5 * (e1x + e2x);
    *ey = 0.5 * (e1y + e2y);

    return true;
   }
   catch (const std::exception & e) {
     CF_ERROR("cv::SVD() fails in %s(): %s",__func__, e.what());
   }
   catch (...) {
     CF_ERROR("cv::SVD() fails in %s()",__func__);
   }

   return false;
}

int main(int argc, char *argv[])
{
  std::string filenames[2];
  cv::Mat images[2], masks[2];
  cv::Mat feature_image, feature_mask;

  for ( int i = 1; i < argc; ++i ) {

    if ( strcmp(argv[i], "--help") == 0 ) {
      printf("Usage: alpha <input-file-name1.tiff> <input-file-name2.tiff> \n");
      return 0;
    }

    else if ( filenames[0].empty() ) {
      filenames[0] = argv[i];
    }
    else if ( filenames[1].empty() ) {
      filenames[1] = argv[i];
    }
    else {
      fprintf(stderr, "Invalid argument : %s\n", argv[i]);
      return 1;
    }
  }

  if ( filenames[0].empty() || filenames[1].empty() ) {
    fprintf(stderr, "Two input file names expected\n");
    return 1;
  }

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);


  for ( int i = 0; i < 2; ++i ) {
    if ( !load_image(filenames[i], images[i], masks[i]) ) {
      CF_ERROR("load_image(%s) fails", filenames[i].c_str());
      return 1;
    }
  }

  c_feature_based_registration::ptr reg =
      c_feature_based_registration::create();

  if ( !reg->setup_referece_frame(images[0], masks[0]) ) {
    CF_ERROR("reg->setup_referece_frame() fails");
    return 1;
  }

  if ( !reg->create_feature_image(images[1], masks[1], feature_image, feature_mask) ) {
    CF_ERROR("reg->create_feature_image() fails");
    return 1;
  }

  std::vector<cv::Point2f> matched_current_positions;
  std::vector<cv::Point2f> matched_reference_positions;

  if ( !reg->detect_and_match_keypoints(feature_image, feature_mask, matched_current_positions, matched_reference_positions ) ) {
    CF_ERROR("reg->detect_and_match_keypoints() fails");
    return 1;
  }

  CF_DEBUG("matched_current_positions.size=%zu",
      matched_current_positions.size());

  CF_DEBUG("matched_reference_positions.size=%zu",
      matched_reference_positions.size());

  cv::Mat1b M(matched_reference_positions.size(), 1, 255);

  cv::Matx33d F =
      cv::findFundamentalMat(matched_reference_positions, matched_current_positions,
      cv::FM_RANSAC,
      5,
      0.9,
      M);


  const int n = cv::countNonZero(M);
  CF_DEBUG("Num inliers: %d", n);


  const int m = 2;
  double p[std::max(m, 3)], x[n], opts[LM_OPTS_SZ], info[LM_INFO_SZ];
  double pts[4 * n];
  int status;


  for ( int i = 0, j = 0; i < matched_reference_positions.size(); ++i ) {
    if ( M[i][0] ) {
      x[j] = 0;
      pts[4 * j + 0] = matched_current_positions[i].x;
      pts[4 * j + 1] = matched_current_positions[i].y;
      pts[4 * j + 2] = matched_reference_positions[i].x;
      pts[4 * j + 3] = matched_reference_positions[i].y;
      ++j;
    }
  }



  FILE * fp = fopen("dump.txt", "w");

  fprintf(fp, "ex\tey\ts\n");

  for ( double ex = -100; ex <= images[0].cols + 100; ex += 10 ) {
    for ( double ey = -100; ey <= images[0].rows + 100; ey += 10 ) {

      p[0] = ex;
      p[1] = ey;

      double s =
          compute_model(p, nullptr, m, n, pts);

      fprintf(fp, "%9.3f\t%9.3f\t%12.6f\n",
          ex, ey, s);

    }
  }

  fclose(fp);


  /* optimization control parameters; passing to levmar NULL instead of opts reverts to defaults */

  p[0] = images[0].cols / 2; // Ex
  p[1] = images[0].rows / 2; // Ey
  p[2] = 0; // rotation

  compute_epipole(F, &p[0], &p[1]);

  printf("Initial parameters: %.7g %.7g %.7g\n", p[0], p[1], p[2]);

  opts[0] = 1e-3;//LM_INIT_MU;
  opts[1] = 1E-15;
  opts[2] = 1E-15;
  opts[3] = 1E-20;
  opts[4] = -LM_DIFF_DELTA;  // relevant only if the finite difference Jacobian version is used


//  status = dlevmar_der(modelfunc,
//      jacfunc, p, x, m, n, 1000,
//      opts, info,
//      NULL, NULL,
//      pts);

  status = dlevmar_dif(modelfunc,
          p, x, m, n, 1000,
          opts, info,
          NULL, NULL,
          pts); // without Jacobian

  printf("Levenberg-Marquardt returned status=%d in %g iter, reason %g, sumsq %g [%g]\n", status, info[5], info[6], info[1], info[0]);
  printf("Best fit parameters: %.7g %.7g %.7g\n", p[0], p[1], p[2]/1000);


  return 0;
}

