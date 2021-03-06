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
#include <core/proc/planetary-disk-detection.h>
#include <core/proc/jupiter.h>
#include <core/proc/fft.h>
#include <core/settings.h>
#include <core/readdir.h>
#include <core/get_time.h>
#include <core/proc/inpaint.h>
#include <tbb/tbb.h>
#include <core/debug.h>
#include <variant>

enum TEST_ENUM {
  TEST_ENUM_M1,
  TEST_ENUM_M2,
};

class c_test_class
{
  double alpha_ = 0;
  TEST_ENUM e_ = TEST_ENUM_M1;

public:

  double alpha() const
  {
    return alpha_;
  }

  void set_alpha(double v)
  {
    alpha_ = v;
  }

  TEST_ENUM e() const
  {
    return e_;
  }

  void set_e(TEST_ENUM v)
  {
    e_ = v;
  }

  void do_test()
  {
    if ( std::is_enum<decltype(alpha())>::value ) {
      CF_DEBUG("alpha IS ENUM");
    }
    else {
      CF_DEBUG("alpha is NOT ENUM");
    }

    if ( std::is_enum<decltype(e())>::value ) {
      CF_DEBUG("e IS ENUM");
    }
    else {
      CF_DEBUG("e is NOT ENUM");
    }
  }
};

int main(int argc, char *argv[])
{
    cf_set_logfile(stderr);
    cf_set_loglevel(CF_LOG_DEBUG);

    c_test_class test_class;
    test_class.do_test();

  return 0;



//  std::string filenames[2];
//  cv::Mat images[2], masks[2];
//  cv::Mat feature_image, feature_mask;
//
//  for ( int i = 1; i < argc; ++i ) {
//
//    if ( strcmp(argv[i], "--help") == 0 ) {
//      printf("Usage: alpha <input-file-name1.tiff> <input-file-name2.tiff> \n");
//      return 0;
//    }
//
//    else if ( filenames[0].empty() ) {
//      filenames[0] = argv[i];
//    }
//    else if ( filenames[1].empty() ) {
//      filenames[1] = argv[i];
//    }
//    else {
//      fprintf(stderr, "Invalid argument : %s\n", argv[i]);
//      return 1;
//    }
//  }
//
//  if ( filenames[0].empty() /*|| filenames[1].empty()*/ ) {
//    fprintf(stderr, "Two input file names expected\n");
//    return 1;
//  }
//
//  cf_set_logfile(stderr);
//  cf_set_loglevel(CF_LOG_DEBUG);
//
//
//  for ( int i = 0; i < 1; ++i ) {
//    if ( !load_image(filenames[i], images[i], masks[i]) ) {
//      CF_ERROR("load_image(%s) fails", filenames[i].c_str());
//      return 1;
//    }
//  }

//  cv::Mat1f image(128, 512, 0.f);
//  cv::Mat_<std::complex<float>> spec;
//  cv::Mat1f spow;
//
//  cv::circle(image, cv::Point(image.cols/2, image.rows/2), 31, 1, -1, cv::LINE_8 );
//
//  save_image(image, "image.tiff");
//
//  cv::dft(image, spec, cv::DFT_COMPLEX_OUTPUT);
//  fftSwapQuadrants(spec);
//
//  cv::Mat channels[2];
//  cv::split(spec, channels);
//  cv::magnitude(channels[0], channels[1], spow);
//  cv::log(1 + spow, spow);
//  save_image(spow, "spow.tiff");


  return 0;
}

