/*
 * median_pyramid.cc
 *
 *  Created on: Sep 10, 2023
 *      Author: amyznikov
 */

#include "median_pyramid.h"
#include <core/debug.h>

#if HAVE_TBB
# include <tbb/tbb.h>
#endif // HAVE_TBB


template<class T>
static void pixdup_(cv::Mat & image, DOWNSTRIKE_MODE downstrike_mode)
{
//  const int rows =
//      image.rows;
//
//  const int cols =
//      image.cols;
//
//  const int channels =
//      image.channels();
//
//  cv::Mat_<T> img = image;
//
//  switch (downstrike_mode) {
//    case DOWNSTRIKE_EVEN:
//      for( int y = 1; y < rows; y += 2 ) {
//        for( int x = 1; x < cols; x += 2 ) {
//          for( int c = 0; c < channels; ++c ) {
//            const T v = img[y][x * channels + c];
//            img[y - 1][(x - 1) * channels + c] = v;
//            img[y - 1][(x) * channels + c] = v;
//            img[y][(x - 1) * channels + c] = v;
//          }
//        }
//      }
//      break;
//    case DOWNSTRIKE_UNEVEN:
//      for( int y = 0; y < rows - 1; y += 2 ) {
//        for( int x = 0; x < cols - 1; x += 2 ) {
//          for( int c = 0; c < channels; ++c ) {
//            const T v = img[y][x * channels + c];
//            img[y][(x + 1) * channels + c] = v;
//            img[y + 1][(x) * channels + c] = v;
//            img[y + 1][(x + 1) * channels + c] = v;
//          }
//        }
//      }
//      break;
//  }
}

static void pixdup(cv::Mat & image, DOWNSTRIKE_MODE downstrike_mode)
{
  static float k [2*2] = {
      0.25, 0.25,
      0.25, 0.25,
  };

  static const thread_local cv::Mat1f SE(2,2, k);

  switch (downstrike_mode) {
    case DOWNSTRIKE_UNEVEN: {
      cv::filter2D(image, image, -1, SE, cv::Point(0, 0));
      break;
    }

    case DOWNSTRIKE_EVEN: {
      cv::filter2D(image, image, -1, SE, cv::Point(1, 1));
      break;
    }
  }


//  switch (image.depth()) {
//    case CV_8U:
//      return pixdup_<uint8_t>(image, downstrike_mode);
//    case CV_8S:
//      return pixdup_<int8_t>(image, downstrike_mode);
//    case CV_16U:
//      return pixdup_<uint16_t>(image, downstrike_mode);
//    case CV_16S:
//      return pixdup_<int16_t>(image, downstrike_mode);
//    case CV_32S:
//      return pixdup_<int32_t>(image, downstrike_mode);
//    case CV_32F:
//      return pixdup_<float>(image, downstrike_mode);
//    case CV_64F:
//      return pixdup_<double>(image, downstrike_mode);
//  }
}


bool build_median_pyramid(cv::InputArray _src,
    int median_ksize,
    int median_iterations,
    std::vector<cv::Mat> & scaled_images,
    std::vector<cv::Mat> & median_blurs,
    std::vector<cv::Mat> & median_hats)
{

  scaled_images.clear();
  scaled_images.reserve(16);

  median_blurs.clear();
  median_blurs.reserve(16);

  median_hats.clear();
  median_hats.reserve(16);

  const double scale = 0.5;

  cv::Size size =
      _src.size();

  for( double f = 1; std::min(size.width, size.height) > 4; f *= scale ) {

    scaled_images.emplace_back();
    median_blurs.emplace_back();
    median_hats.emplace_back();

    cv::Mat & s =
        scaled_images.back();

    cv::resize(_src, s, cv::Size(), f, f,
        cv::INTER_AREA);

    size =
        s.size();

    cv::medianBlur(s, median_blurs.back(),
        median_ksize);

    for( int ii = 1; ii < median_iterations; ++ii ) {
      cv::medianBlur(median_blurs.back(), median_blurs.back(),
          median_ksize);
    }

    cv::absdiff(s, median_blurs.back(),
        median_hats.back());
  }

  return true;
}

//bool build_median_pyramid(cv::InputArray _src,
//    int median_ksize,
//    int median_iterations,
//    DOWNSTRIKE_MODE downstrike_mode,
//    std::vector<cv::Mat> & median_blurs,
//    std::vector<cv::Mat> & median_hats)
//{
//
//  median_blurs.clear();
//  median_blurs.reserve(12);
//
//  median_hats.clear();
//  median_hats.reserve(12);
//
//  cv::Mat s = _src.getMat();
//
//  while (std::min(s.cols, s.rows) > 3) {
//
//    median_blurs.emplace_back();
//    median_hats.emplace_back();
//
//    cv::medianBlur(s, median_blurs.back(), median_ksize);
//    for ( int ii = 1; ii < median_iterations; ++ii ) {
//      cv::medianBlur(median_blurs.back(), median_blurs.back(), median_ksize);
//    }
//
//    cv::Mat & img = median_blurs.back();
//    cv::absdiff(s, img, median_hats.back());
//    cv::resize(img, s, cv::Size((img.cols + 1) / 2, (img.rows + 1) / 2), 0, 0, cv::INTER_AREA);
//    //cv::resize(img, s, cv::Size(3 * (img.cols + 3) / 4, 3 * (img.rows + 3) / 4), 0, 0, cv::INTER_AREA);
//  }
//
//  return true;
//}
