/*
 * alpha.cc
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 */

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
#include <core/proc/fft.h>
#include <core/settings.h>
#include <core/readdir.h>
#include <core/get_time.h>
#include <core/proc/inpaint.h>
#include <tbb/tbb.h>
#include <core/proc/lpg.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/stereo/c_sweepscan_stereo_matcher.h>

#include <core/proc/laplacian_pyramid.h>
#include <core/proc/image_registration/ecc2.h>
#include <core/proc/image_registration/ecc_motion_model.h>
#include <core/proc/reduce_channels.h>
#include <core/proc/pyrscale.h>
#include <core/debug.h>


static void pnormalize(const cv::Mat & src, cv::Mat & dst, int pscale)
{
  cv::Mat m;
  cv::Scalar mean, stdev;
  double f = 0;

  if( m.channels() == 1 ) {
    pyramid_downscale(src, m, pscale, cv::BORDER_REPLICATE);
    pyramid_upscale(m, src.size());
  }
  else {
    cv::cvtColor(src, m, cv::COLOR_BGR2GRAY);
    pyramid_downscale(m, m, pscale, cv::BORDER_REPLICATE);
    pyramid_upscale(m, src.size());
    cv::cvtColor(m, m, cv::COLOR_GRAY2BGR);
  }

  cv::subtract(src, m, dst, cv::noArray());

//
//  cv::meanStdDev(m, mean, stdev);
//  for( int i = 0, cn = src.channels(); i < cn; ++i ) {
//    f += stdev[i];
//  }
//
//  m.convertTo(dst, CV_8U, 24. * src.channels() / f, 128);
}


static bool cmp(cv::InputArray src1, cv::InputArray src2, cv::OutputArray dst)
{
  if ( src1.size() != src2.size() ) {
    CF_ERROR("src sizes not match");
    return false;
  }

  if ( src1.type() != CV_32FC1 ) {
    CF_ERROR("src1.type() = %d, must be CV_32FC1=%d", src1.type(), CV_32FC1);
    return false;
  }

  if ( src2.type() != CV_32FC2 ) {
    CF_ERROR("src2.type() = %d, must be CV_32FC1=%d", src2.type(), CV_32FC1);
    return false;
  }

  dst.create(src1.size(), CV_8SC1);

  cv::Mat_<int8_t> d =
      dst.getMatRef();

  const cv::Mat1f s1 =
      src1.getMat();

  const cv::Mat1f s2 =
      src2.getMat();

  for ( int y = 0; y < s1.rows; ++y ) {
    for ( int x = 0; x < s1.cols; ++x ) {

      if ( s1[y][x] < s2[y][x] ) {
        d[y][x] = -1;
      }
      else if ( s1[y][x] > s2[y][x] ) {
        d[y][x] = +1;
      }
      else {
        d[y][x] = 0;
      }
    }
  }

  return true;
}


int main(int argc, char *argv[])
{
  std::string input_filenames[2];

  cv::Mat images[2];
  cv::Mat masks[2];
  cv::Mat texture_masks[2];

  cv::Mat & left_image =
      images[0];

  cv::Mat & right_image =
      images[1];

  for ( int i = 1; i < argc; ++i ) {

    if ( strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-help") == 0 ) {

      fprintf(stdout, "Test for c_ecc_stereo_flow.\n"
          "Usage:\n"
          "   alpha left_image.png right_image.png\n"
          "\n");

      return 0;
    }

    if ( input_filenames[0].empty() ) {
      input_filenames[0] = argv[i];
      continue;
    }

    if ( input_filenames[1].empty() ) {
      input_filenames[1] = argv[i];
      continue;
    }

    fprintf(stderr, "Invalid argument: %s\n", argv[i]);
    return 1;
  }

  if( input_filenames[0].empty() || input_filenames[1].empty() ) {
    fprintf(stderr, "two input images expected\n");
    return 1;
  }

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);


  for ( int i = 0; i < 2; ++i ) {

    if ( !load_image(input_filenames[i], images[i], masks[i]) ) {
      CF_ERROR("load_image(%s) fails", input_filenames[i].c_str());
      return 1;
    }

    cv::Scalar noise =
        estimate_noise(images[i]);

    double avgnoise = 0;
    for ( int i = 0; i < images[i].channels(); ++i ) {
      avgnoise += noise(i);
    }
    avgnoise /= images[i].channels();


    CF_DEBUG("image[%d] channels=%d depth=%d noise=(%g %g %g) avgnoise=%g", i,
        images[i].channels(),
        images[i].depth(),
        noise(0), noise(1), noise(2),
        avgnoise);

    lpg(images[i], texture_masks[i], 3, 1, 1, false, true, nullptr);

    avgnoise = estimate_noise(texture_masks[i])(0);
    CF_DEBUG("texture_noise=%g", avgnoise);

    cv::compare(texture_masks[i], 10 * avgnoise, texture_masks[i], cv::CMP_GT);

    save_image(texture_masks[i], ssprintf("debug/T.%d.tiff", i));
  }

  if ( images[0].size() != images[1].size() ) {
    CF_ERROR("image sizes not match");
    return 1;
  }

  c_sweepscan_stereo_matcher m;
  cv::Mat1w outputMatches;
  cv::Mat1b outputMask;

  m.set_max_disparity(128);
  m.set_max_scale(0);
  m.set_kernel_sigma(1.5);
  m.set_kernel_radius(7);
  m.set_normalization_scale(0) ;
  m.set_debug_directory("./debug/sss");

  bool fOk =
      m.match(images[0], masks[0],
          images[1], masks[1],
          outputMatches,
          &outputMask);

  if ( !fOk ) {
    CF_ERROR("m.match() fails");
    return 1;
  }

  return 0;

//
//
//  const int max_disparity = 128;
//
//  const cv::Size size =
//      images[0].size();
//
//  const cv::Mat1f G =
//      cv::getGaussianKernel(5, 1, CV_32F);
//
//  cv::Mat summ, diff;
//  std::deque<cv::Mat1f> Eq;
//  cv::Mat1f Emin;
//  cv::Mat1f D;
//
////  Emin.create(size);
////  Emin.setTo(HUGE_VAL);
//
//
//  D.create(size);
//  D.setTo(-1);
//
//  for ( int disparity = 0; disparity < max_disparity; ++disparity ) {
//
//    const cv::Mat & Q =
//        left_image(cv::Rect(disparity, 0,
//            size.width - disparity, size.height));
//
//    const cv::Mat & T =
//        right_image(cv::Rect(0, 0,
//            size.width - disparity, size.height));
//
//
//    cv::add(T, Q, summ);
//    cv::absdiff(T, Q, diff);
//    cv::sepFilter2D(summ, summ, -1, G, G, cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
//    cv::divide(diff, summ, diff);
//    if ( diff.channels() > 1 ) {
//      reduce_color_channels(diff, diff, cv::REDUCE_AVG);
//    }
//
//    // fixme: limit queue
//    Eq.emplace_back();
//    cv::GaussianBlur(diff, Eq.back(), cv::Size(15, 15), 3, 3);
//
//    if ( Eq.size() == 1 ) {
//      Eq.back().copyTo(Emin);
//    }
//    else if ( Eq.size() == 2 ) {
//
//      cv::Mat1f & Ec = Eq[Eq.size() - 1];
//      cv::Mat1f &Ep = Eq[Eq.size() - 2];
//
//      for( int y = 0; y < size.height; ++y ) {
//        for( int x = 0, nx = size.width - disparity; x < nx; ++x ) {
//          if( Ec[y][x] < Emin[y][x] ) {
//            Emin[y][x] = Ec[y][x];
//          }
//          else if( Ec[y][x] > Emin[y][x] ) {
//            D[y][x] = disparity - 1;
//          }
//        }
//      }
//    }
//    else {
//
//      cv::Mat1f & Ec = Eq[Eq.size() - 1];
//      cv::Mat1f & Ep = Eq[Eq.size() - 2];
//      cv::Mat1f & Epp = Eq[Eq.size() - 3];
//
//      for( int y = 0; y < size.height; ++y ) {
//        for( int x = 0, nx = size.width - disparity; x < nx; ++x ) {
//          if( Ec[y][x] < Emin[y][x] ) {
//            Emin[y][x] = Ec[y][x];
//            D[y][x] = -1;
//          }
//          else if( Ec[y][x] > Ep[y][x] && Ep[y][x] == Emin[y][x] && Ep[y][x] < Epp[y][x] ) {
//            D[y][x] = disparity - 1;
//          }
//        }
//      }
//
//      Eq.pop_front();
//    }
//  }
//
//
//  cv::Mat1b M = (D >= 0) & texture_masks[1];
//
//  save_image(D, M, "debug/D.tiff");
//  save_image(Emin, "debug/E.tiff");

  return 0;
}


