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
#include <core/settings.h>
#include <core/ssprintf.h>
#include <core/readdir.h>
#include <core/debug.h>

extern void ecc_differentiate(const cv::Mat & src, cv::Mat & gx, cv::Mat & gy);;

static void differentiate(const cv::Mat & src, cv::Mat & gx, cv::Mat & gy)
{
//  static thread_local const cv::Matx<float, 1, 3> K(
//      (+1.f / 2),
//        0.f,
//      (-1.f / 2));

//  static thread_local const cv::Matx<float, 1, 2> K(
//      (+1.f),
//      (-1.f));

  static thread_local const cv::Matx<float, 1, 5> K(
      (+1.f / 12),
      (-8.f / 12),
        0.f,
      (+8.f / 12),
      (-1.f / 12));

  cv::filter2D(src, gx, CV_32F, K, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::filter2D(src, gy, CV_32F, K.t(), cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
}



static void unsharp(cv::InputArray src, cv::OutputArray dst,
    double sigma, double alpha)
{
  if ( sigma <= 0 || alpha <= 0 ) {
    src.copyTo(dst);
    return;
  }

  cv::Mat lpass, hpass;
  cv::Mat1f G = cv::getGaussianKernel(2 * std::max(1, (int) (sigma * 5)) + 1, sigma, CV_32F);
  cv::sepFilter2D(src, lpass, -1, G, G, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::subtract(src, lpass, hpass);
  cv::scaleAdd(hpass, 1./alpha, lpass, dst);
}

int main(int argc, char *argv[])
{
  cv::Mat image, mask;
  std::string filename;

  for ( int i = 1; i < argc; ++i ) {

    if ( strcmp(argv[i], "--help") == 0 ) {
      printf("Usage: alpha <input-file-name.tiff>\n");
      return 0;
    }

    if ( filename.empty() ) {
      filename = argv[i];
    }
    else {
      fprintf(stderr, "Invalid argument : %s\n", argv[i]);
      return 1;
    }
  }

  if ( filename.empty() ) {
    fprintf(stderr, "No input file name specified\n");
    return 1;
  }

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);



//  if ( 1 )  {
//    std::string parent_directory;
//    std::string file_name;
//    std::string file_suffix;
//
//    /* split the fullpathname to parent directory file name, and suffix */
//    split_pathfilename(filename,
//        &parent_directory,
//        &file_name,
//        &file_suffix);
//
//    CF_DEBUG("parent_directory='%s'", parent_directory.c_str());
//    CF_DEBUG("file_name='%s'", file_name.c_str());
//    CF_DEBUG("file_suffix='%s'", file_suffix.c_str());
//    return 0;
//  }

  if ( !load_image(image, filename) ) {
    CF_ERROR("load_tiff_image() fails");
  }


//  cv::Mat sharp, sharpx, sharpy, gx, gy, g, smask;
//
//  if ( image.channels() == 4 || image.channels() == 2 ) {
//    CF_DEBUG("HAVE MASK");
//    splitbgra(image, image, &mask);
//    cv::erode(mask, smask, cv::Mat1b(55, 55, 255), cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
//  }

//  cv::Mat1f srcimage(30, 30);
//  for ( int y = 0; y < srcimage.rows; ++y ) {
//    for ( int x = 0; x < srcimage.cols; ++x ) {
//      srcimage[y][x] = 10 + ((double)(x)) / srcimage.cols + ((double)(y)) / srcimage.rows;
//    }
//  }


  cv::Mat srcimage;
  cv::Mat resized_image;

  cv::cvtColor(image, srcimage, cv::COLOR_BGR2GRAY);

  cv::resize(srcimage, resized_image, cv::Size(srcimage.cols*3/2, srcimage.rows*3/2), 0, 0, cv::INTER_LINEAR);
  save_image(srcimage, "src.tiff");
  save_image(resized_image, "upsampled.tiff");

//  cv::resize(image, resized_image, cv::Size(image.cols * 2, image.rows * 2), 0, 0, cv::INTER_LINEAR);
//  save_image(resized_image, "resized.tiff");

//  cv::pyrUp(image, resized_image);
//  save_image(resized_image, "resized-pup.tiff");

  return 0;
}



//
//
//int main(int argc, char *argv[])
//{
//  cv::Mat image, mask;
//  std::string filename;
//
//  for ( int i = 1; i < argc; ++i ) {
//
//    if ( strcmp(argv[i], "--help") == 0 ) {
//      printf("Usage: alpha <input-file-name.tiff>\n");
//      return 0;
//    }
//
//    if ( filename.empty() ) {
//      filename = argv[i];
//    }
//    else {
//      fprintf(stderr, "Invalid argument : %s\n", argv[i]);
//      return 1;
//    }
//  }
//
//  if ( filename.empty() ) {
//    fprintf(stderr, "No input file name specified\n");
//    return 1;
//  }
//
//  cf_set_logfile(stderr);
//  cf_set_loglevel(CF_LOG_DEBUG);
//
//
//  if ( !load_image(image, filename) ) {
//    CF_ERROR("load_tiff_image() fails");
//  }
//
//
//  cv::Mat sharp, sharpx, sharpy, gx, gy, g, smask;
//
//  if ( image.channels() == 4 || image.channels() == 2 ) {
//    CF_DEBUG("HAVE MASK");
//    splitbgra(image, image, &mask);
//    cv::erode(mask, smask, cv::Mat1b(55, 55, 255), cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
//
//  }
//  else {
//    cv::erode(cv::Mat1b(image.size(), 255), smask, cv::Mat1b(55, 55, 255), cv::Point(-1, -1), 1, cv::BORDER_CONSTANT, 0);
//  }
//
//
//
////  {
////  double l1, l2, l2s;
////    cv::Mat1f m(100,100, 2.f);
////    l1 = cv::norm(m, cv::NORM_L1);
////    l2 = cv::norm(m, cv::NORM_L2);
////    l2s = cv::norm(m, cv::NORM_L2SQR);
////
////
////    printf("\n"
////        "M: L1=%g L2=%g L2SQR=%g"
////        "\n",
////        l1, l2, l2s);
////
////    return 0;
////  }
//
//
//  if ( !smask.empty() ) {
//    cv::compare(image, 0.01, smask, cv::CMP_GT);
//    cv::cvtColor(smask, smask, cv::COLOR_BGR2GRAY);
//    save_image(smask, ssprintf("sharp/smask.png"));
//  }
//
//  double sigma = 0.5;
//  double l1, l2;
//
//  static const double alpha[] = {
//      0,
//      0.5, 0.75, 0.9,
//      0.95,
//      0.96,
//      0.970, 0.971, 0.972, 0.973, 0.974, 0.975, 0.976, 0.977, 0.978, 0.979,
//      0.980, 0.981, 0.982, 0.983, 0.984, 0.985, 0.986, 0.987, 0.988, 0.989,
//      0.990, 0.991, 0.992, 0.993, 0.994, 0.995, 0.996, 0.997, 0.998, 0.999,
//      0.9991, 0.9991, 0.9993, 0.9994, 0.9995, 0.9996, 0.9997, 0.9998, 0.9999,
////      0.99995, 0.99999,
////      0.999995, 0.999999,
////      0.9999995, 0.9999999,
//  };
//
//
//  rmfiles("sharp/", "*");
//  rmfiles("g/", "*");
//
////  cv::absdiff(image, 0, image);
////  cv::sqrt(image, image);
//
////  differentiate(image, gx, gy);
////  save_image(gx, ssprintf("sharp/gx.tiff"));
////  save_image(gy, ssprintf("sharp/gy.tiff"));
//
////  cv::GaussianBlur(image, image, cv::Size(0,0), 0.9);
////  differentiate(image, gx, gy);
//
//  //smask.release();
//  fprintf(stdout, "I\talpha\tm\ts\te\tim\tis\tie\n");
//
//  cv::Scalar ims, iss, ies;
//  double imv, isv, iev;
//  cv::meanStdDev(image, ims, iss, smask);
//  ies = estimate_noise(image, cv::noArray(), smask);
//  imv = (ims[0] + ims[1] + ims[2]);
//  isv = (iss[0] + iss[1] + iss[2]);
//  iev = (ies[0] + ies[1] + ies[2]);
//
//  for ( int i = 0, n = sizeof(alpha)/sizeof(alpha[0]); i < n; ++i ) {
//
//    cv::Scalar ms, ss, es;
//    double mv, sv, ev;
//
//    unsharp_mask(image, sharp, sigma, alpha[i]);
//
//    cv::meanStdDev(sharp, ms, ss, smask);
//    es = estimate_noise(sharp, cv::noArray(), smask);
//
//    mv = (ms[0] + ms[1] + ms[2]);
//    sv = (ss[0] + ss[1] + ss[2]);
//    ev = (es[0] + es[1] + es[2]);
//
//    fprintf(stdout, "%6d\t%12.9f\t%15.9f\t%15.9f\t%15.9f\t%15.9f\t%15.9f\t%15.9f\n",
//        i, alpha[i], mv, sv, ev, imv, isv, iev);
//
//    save_image(sharp, ssprintf("sharp/sharp.%05d.tiff", i));
//   }
//
//  return 0;
//}
//


//static bool convertTofp32(const cv::Mat & src, cv::Mat & dst)
//{
//  constexpr int ddepth = CV_32F;
//  switch (src.depth()) {
//    case CV_8S:
//      src.convertTo(dst, ddepth, 1./INT8_MAX, 0.5);
//      break;
//    case CV_8U:
//      src.convertTo(dst, ddepth, 1./UINT8_MAX, 0);
//      break;
//    case CV_16S:
//      src.convertTo(dst, ddepth, 1./INT16_MAX, 0.5);
//      break;
//    case CV_16U:
//      src.convertTo(dst, ddepth, 1./UINT16_MAX, 0);
//      break;
//    case CV_32S:
//      src.convertTo(dst, ddepth, 1./INT32_MAX, 0.5);
//      break;
//    case CV_32F:
//      if ( src.data != dst.data ) {
//        src.copyTo(dst);
//      }
//      break;
//    case CV_64F:
//      src.convertTo(dst, ddepth);
//      break;
//    default:
//      return  false;;
//  }
//
//  return true;
//}
//
//
//static void build_laplacian_pyramid(int max_levels, const cv::Mat & src, std::vector<cv::Mat> & laps, cv::Mat & res )
//{
//  laps.clear();
//  laps.reserve(max_levels);
//
//
//  cv::Mat s, gb;
//
//  cv::GaussianBlur(src, gb, cv::Size(5,5), 0, 0, cv::BORDER_DEFAULT);
//  laps.emplace_back(), cv::subtract(src, gb, laps.back());
//
////  for ( int i = 1; i < max_levels; ++i ) {
////    laps.emplace_back();
////    downstrike_uneven(gb, laps.back());
////    cv::GaussianBlur(laps.back(), gb, cv::Size(5, 5), 0, 0, cv::BORDER_DEFAULT);
////    cv::subtract(laps.back(), gb, laps.back());
////  }
////
////  downstrike_uneven(gb, res);
//
//  for ( int i = 1; i < max_levels; ++i ) {
//    gb.copyTo(s);
//    cv::GaussianBlur(s, gb, cv::Size(3, 3), 0.5, 0.5, cv::BORDER_DEFAULT);
//    laps.emplace_back(), cv::subtract(s, gb, laps.back());
//  }
//
//  gb.copyTo(res);
//}
//
//static double squale(double x)
//{
//  return x * x * x / (100);
//}
//
//static void merge_laplacian_pyramid(const std::vector<cv::Mat> & laps, const cv::Mat & res, cv::Mat & dst )
//{
//  cv::Mat s;
//
//  cv::scaleAdd(laps[laps.size() - 1], squale(laps.size() - 1), res, s);
//
//  for ( int i = laps.size() - 2; i >= 0; --i ) {
//    cv::scaleAdd(laps[i], squale(laps.size() - i), s, s);
//  }
//
//  dst = std::move(s);
//}
////
////int main(int argc, char *argv[])
////{
////  std::string input_file_name;
////  std::string output_path;
////  cv::Mat image;
////  cv::Mat mask;
////  std::vector<cv::Mat> laps;
////  cv::Mat res;
////  double min, max;
////
////  for ( int i = 1; i < argc; ++i ) {
////
////    if ( strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "--help") == 0 ) {
////      fprintf(stdout,
////          "Usage:\n"
////          "   alpha [OPTIONS] input_image -o output_path\n"
////          "\n"
////      );
////
////      return 0;
////    }
////
////    if ( strcmp(argv[i], "-o") == 0 ) {
////      if ( (++i >= argc) ) {
////        fprintf(stderr, "Output file name expected: %s\n", argv[i - 1]);
////        return 1;
////      }
////      output_path = argv[i];
////    }
////
////
////
////    else if ( input_file_name.empty() && is_regular_file(argv[i]) ) {
////      input_file_name = argv[i];
////    }
////
////
////    else {
////      fprintf(stderr, "Invalid argument or not existimng input file specified: %s\n", argv[i]);
////      return 1;
////    }
////  }
////
////  if ( input_file_name.empty() ) {
////    fprintf(stderr, "No input SER file specified\n");
////    return 1;
////  }
////
////  cf_set_logfile(stderr);
////  cf_set_loglevel(CF_LOG_DEBUG);
////
////
////  if ( !load_image(image, input_file_name) ) {
////    CF_ERROR("load_image(%s) fails", input_file_name.c_str());
////    return 1;
////  }
////
////
////  switch ( image.channels() ) {
////  case 1 :
////    break;
////  case 2 : {
////    cv::Mat channels[2];
////    cv::split(image, channels);
////    image = channels[0];
////    cv::compare(channels[1], 0, mask, cv::CMP_GT);
////    CF_DEBUG("HAVE MASK");
////    break;
////  }
////  case 3 :
////    break;
////  case 4 : // Assuming BGRA
////    cv::extractChannel(image, mask, 3);
////    cv::compare(mask, 0, mask, cv::CMP_GT);
////    cv::cvtColor(image, image, cv::COLOR_BGRA2BGR);
////    CF_DEBUG("HAVE MASK");
////    break;
////  default :
////    CF_FATAL("Invalid input: RGB or grayscale image expected, image.channels()=%d rgb_image.depth()=%d",
////        image.channels(), image.depth());
////    return 1;
////    break;
////  }
////
////  convertTofp32(image,
////      image);
////
////  cv::minMaxLoc(image, &min, &max);
////
////  CF_DEBUG("input: %dx%d channels=%d depth=%d min=%g max=%g",
////        image.cols, image.rows,
////        image.channels(), image.depth(),
////        min, max);
////
////
////  build_laplacian_pyramid(15, image, laps, res );
////
////  for ( uint i = 0, n = laps.size(); i < n; ++i ) {
////    save_image(laps[i], ssprintf("pyramid/lap.%03u.tiff", i));
////  }
////
////  save_image(res, ssprintf("pyramid/res.tiff"));
////
////  merge_laplacian_pyramid(laps, res, image );
////
////  save_image(image, ssprintf("pyramid/merged-back.tiff"));
////
////  return 0;
////}
//
//
//int main(int argc, char *argv[])
//{
//  cf_set_logfile(stderr);
//  cf_set_loglevel(CF_LOG_DEBUG);
//
//  c_image_processor::ptr processor =
//      c_image_processor::create("processor1");
//
//  processor->emplace_back(c_unsharp_mask_routine::create());
//  processor->emplace_back(c_align_color_channels_routine::create());
//  processor->emplace_back(c_rangeclip_routine::create());
//  processor->save("processor1.cfg");
//
//  processor = c_image_processor::load("processor1.cfg");
//  if ( !processor ) {
//    CF_ERROR("c_image_processor::load(\"processor1.cfg\") fails");
//    return 1;
//  }
//
//  processor->save("processor2.cfg");
//
//  return 0;
//}
