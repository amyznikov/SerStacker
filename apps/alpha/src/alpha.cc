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
#include <core/proc/downstrike.h>
#include <core/settings.h>
#include <core/ssprintf.h>
#include <core/readdir.h>
#include <core/debug.h>


int main(int argc, char *argv[])
{
  cv::Mat image;
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


  if ( !load_image(image, filename) ) {
    CF_ERROR("load_tiff_image() fails");
  }
  else {
    CF_DEBUG("image: %dx%d channels=%d depth=%d", image.cols, image.rows, image.channels(), image.depth());
    save_image(image, "load_tiff_image_result.tiff");
  }

  return 0;
}



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
