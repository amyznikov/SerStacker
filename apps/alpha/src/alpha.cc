/*
 * alpha.cc
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */

#include <core/io/save_image.h>
#include <core/io/load_image.h>
#include <core/io/c_stdio_file.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/downstrike.h>
#include <core/proc/unsharp_mask.h>
#include <core/proc/morphology.h>
#include <core/proc/geo-reconstruction.h>
#include <core/proc/planetary-disk-detection.h>
#include <core/proc/fft.h>
#include <core/settings.h>
#include <core/readdir.h>
#include <core/get_time.h>
#include <core/proc/inpaint.h>
#include <tbb/tbb.h>
#include <core/proc/lpg.h>
#include <core/proc/stereo/c_sweepscan_stereo_matcher.h>
#include <core/proc/camera_calibration/camera_pose.h>
#include <core/proc/laplacian_pyramid.h>
#include <core/proc/image_registration/ecc2.h>
//#include <core/proc/image_registration/ecc_motion_model.h>
#include <core/proc/reduce_channels.h>
#include <core/proc/pyrscale.h>
#include <core/proc/bfgs.h>
#include <core/io/hdl/c_hdl_frame_reader.h>
#include <core/io/image/c_regular_image_input_source.h>
#include <core/io/image/c_ffmpeg_input_source.h>

#include <core/proc/c_linear_regression3.h>
#include <core/proc/c_line_estimate.h>
#include <core/proc/c_quad_estimate.h>
#include <core/proc/fit_exponential.h>
#include <core/proc/fit_decreasing_exponent.h>
#include <core/proc/extract_channel.h>

#include <core/pipeline/c_cte_pipeline/c_cte_pipeline.h>

#include <limits>
#include <core/debug.h>

namespace temp {
}

int load_data_file(const std::string & fname, int maxpts,
    int NI[],
    double TS[], double Tx[], double Ty[], double Tz[],
    double lat[], double  lon[], double alt[] )
{
  c_stdio_file fp(fname, "r");
  char line [1024] = "";

  if ( !fp.open() ) {
    CF_ERROR("fp.open('%s') fails: %s", fname.c_str(), strerror(errno));
    return -1;
  }

  // NI      TS      Tx      Ty      Tz      lat     lon     alt


  if ( !fgets(line, sizeof(line), fp) ) {
    CF_ERROR("fgets('%s') fails: %s", fname.c_str(), strerror(errno));
    return -1;
  }

  int cn = 0;
  while ( !feof(fp) && cn < maxpts ) {

    int n =
        fscanf(fp, "%d %lf %lf %lf %lf %lf %lf %lf",
            &NI[cn], &TS[cn], &Tx[cn], &Ty[cn], &Tz[cn],
            &lat[cn], &lon[cn], &alt[cn]);

    if ( n == 8 ) {
      ++cn;
    }
  }

  return cn;
}


bool save_data_file(const std::string & fname, int npts,
    int NI[],
    double TS[], double Tx[], double Ty[], double Tz[],
    double lat[], double  lon[], double alt[],
    const cv::Mat1f & S3)
{

  c_stdio_file fp (fname, "w");
  if ( !fp.open() ) {
    CF_ERROR("fp.open('%s') fails: %s", fname.c_str(), strerror(errno));
    return false;
  }


  fprintf(fp, "NI\tTS\tTx\tTy\tTz\tlat\tlon\talt\tplat\tplon\tpalt\n");

  for ( int i = 0; i < npts; ++i ) {

    if ( S3.cols == 3 ) {

      fprintf(fp, "%6d"
          "\t%9.0f"
          "\t%+12.6f\t%+12.6f\t%+12.6f"
          "\t%+12.9f\t%+12.9f\t%+12.9f"
          "\t%+12.9f\t%+12.9f\t%+12.9f"
          "\n",
          NI[i],
          TS[i],
          Tx[i], Ty[i], Tz[i],
          lat[i], lon[i], alt[i],
          S3(i, 0), S3(i, 1), S3(i, 2)
         );
    }
    else {

      fprintf(fp, "%6d"
          "\t%9.0f"
          "\t%+12.6f\t%+12.6f\t%+12.6f"
          "\t%+12.9f\t%+12.9f\t%+12.9f"
          "\t%+12.9f\t%+12.9f\t%+12.9f"
          "\n",
          NI[i],
          TS[i],
          Tx[i], Ty[i], Tz[i],
          lat[i], lon[i], alt[i],
          S3(i, 0), S3(i, 1), 0.0f
         );

    }
  }


  return true;
}


int main(int argc, char *argv[])
{
  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

  constexpr int maxpts = 500;

  int NI[maxpts];
  double TS[maxpts];
  double Tx[maxpts];
  double Ty[maxpts];
  double Tz[maxpts];
  double lat[maxpts];
  double lon[maxpts];
  double alt[maxpts];

  std::string input_file_name;

  for ( int i = 1; i < argc; ++i  ) {

    if( strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-help") == 0 ) {
      fprintf(stdout, "Usage:\n"
          "   alpha <trajectory-selected-points.txt>\n");
      return 0;
    }

    if ( input_file_name.empty() ) {
      input_file_name = argv[i];
      continue;
    }

    fprintf(stderr, "Invalid argument: %s\n", argv[i]);
    return 1;
  }

  if ( input_file_name.empty() ) {
    fprintf(stderr, "No input file name specified\n");
    return 1;
  }

  const int npts =
      load_data_file(input_file_name, maxpts,
          NI, TS, Tx, Ty, Tz, lat, lon, alt);

  if( npts < 4 ) {
    CF_ERROR("load_data_file(): not enough points loaded : %d", npts);
    return 1;
  }

  if ( 1 ) {

    /*
     *
     lat = Tx * a00 + Ty * a01 + Tz * a02 + 1 * a03
     lon = Tx * a10 + Ty * a11 + Tz * a12 + 1 * a13
     alt = Tx * a20 + Ty * a21 + Tz * a22 + 1 * a23

     S2[0]  = [Tx  Ty  Tz  1]  [a00] [a10] [a20]
     S2[1]  = [Tx  Ty  Tz  1]  [a01] [a11] [a21]
     S2[2]  = [Tx  Ty  Tz  1]  [a02] [a12] [a22]
                               [a03] [a13] [a23]

     */

    cv::Mat1f S1 (npts, 4);
    cv::Mat1f S2 (npts, 3);
    cv::Mat1f S3;

    cv::Mat1f X;

    for( int i = 0; i < npts; ++i ) {

      S1[i][0] = Tx[i];
      S1[i][1] = Ty[i];
      S1[i][2] = Tz[i];
      S1[i][3] = 1;

      S2[i][0] = lat[i];
      S2[i][1] = lon[i];
      S2[i][2] = alt[i];
    }


    try {
      cv::solve(S1, S2, X, cv::DECOMP_NORMAL);

      CF_DEBUG("X: rows=%d cols=%d", X.rows, X.cols);

      CF_DEBUG("X{\n"
          "%+16.12f\t%+16.12f\t%+16.12f\n"
          "%+16.12f\t%+16.12f\t%+16.12f\n"
          "%+16.12f\t%+16.12f\t%+16.12f\n"
          "%+16.12f\t%+16.12f\t%+16.12f\n"
          "}\n",
          X(0, 0), X(0, 1), X(0, 2),
          X(1, 0), X(1, 1), X(1, 2),
          X(2, 0), X(2, 1), X(2, 2),
          X(3, 0), X(3, 1), X(3, 2));

      S3 = S1 * X;

      CF_DEBUG("S3: rows=%d cols=%d", S3.rows, S3.cols);

      save_data_file("TransformedOutput.txt", npts,
          NI,
          TS, Tx, Ty, Tz,
          lat, lon, alt,
          S3);

    }
    catch (const std::exception & e) {
      CF_ERROR("cv::solve() fails : %s", e.what());
      return 1;
    }

  }
  else {

    /*
     *
     lat = Tx * a00 + Tz * a01 + 1 * a02
     lon = Tx * a10 + Tz * a11 + 1 * a12

     S2[0]  = [Tx  Tz  1]  [a00] [a10]
     S2[1]  = [Tx  Tz  1]  [a01] [a11]
                           [a02] [a12]

     */

    cv::Mat1f S1 (npts, 3);
    cv::Mat1f S2 (npts, 2);
    cv::Mat1f S3;

    cv::Mat1f X;

    for( int i = 0; i < npts; ++i ) {

      S1[i][0] = Tx[i];
      S1[i][1] = Tz[i];
      S1[i][2] = 1;

      S2[i][0] = lat[i];
      S2[i][1] = lon[i];
    }


    try {
      cv::solve(S1, S2, X, cv::DECOMP_NORMAL);

      CF_DEBUG("X: rows=%d cols=%d", X.rows, X.cols);

      CF_DEBUG("X{\n"
          "%+16.12f\t%+16.12f\n"
          "%+16.12f\t%+16.12f\n"
          "%+16.12f\t%+16.12f\n"
          "}\n",
          X(0, 0), X(0, 1),
          X(1, 0), X(1, 1),
          X(2, 0), X(2, 1));

      S3 = S1 * X;

      CF_DEBUG("S3: rows=%d cols=%d", S3.rows, S3.cols);

      save_data_file("TransformedOutput.txt", npts,
          NI,
          TS, Tx, Ty, Tz,
          lat, lon, alt,
          S3);

    }
    catch (const std::exception & e) {
      CF_ERROR("cv::solve() fails : %s", e.what());
      return 1;
    }

  }
  return 0;
}

