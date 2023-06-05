/*
 * c_histogram_normalization_routine.cc
 *
 *  Created on: Jun 19, 2022
 *      Author: amyznikov
 */

#include "c_histogram_normalization_routine.h"
#include <core/proc/histogram.h>
#include <core/ssprintf.h>
//#include <core/proc/median.h>


//static cv::Scalar compute_median(const cv::Mat & image, cv::InputArray _mask = cv::noArray())
//{
//  if ( image.channels() == 1 ) {
//    return median(image, _mask);
//  }
//
//  std::vector<cv::Mat> channels;
//  cv::split(image, channels);
//
//  cv::Scalar m = cv::Scalar::all(0);
//
//  for ( uint i = 0; i < channels.size(); ++i ) {
//    m[i] = median(channels[i], _mask);
//  }
//
//  return m;
//}
//
//static cv::Scalar compute_median(const cv::Mat & image, cv::InputArray _mask = cv::noArray())
//{
//  cv::Mat1f H;
//  double minval = -1, maxval = -1;
//  int nbins = -1;
//
//  bool fOK =
//      create_histogram(image,
//          _mask,
//          H,
//          &minval,
//          &maxval,
//          nbins,
//          true,
//          true);
//
//  if( !fOK ) {
//    CF_ERROR("create_histogram() fails");
//    return cv::Scalar();
//  }
//
//  cv::Scalar s;
//
//  for( int i = 0, n = (std::min)(4, H.cols); i < n; ++i ) {
//
//    if ( H.rows < 2 ) {
//      s[i] = minval;
//    }
//    else {
//      for( int j = 0, m = H.rows; j < m; ++j ) {
//        if( H[j][i] >= 0.5 ) {
//          s[i] = minval + j * (maxval - minval) / (m - 1);
//          break;
//        }
//      }
//    }
//  }
//
//  return s;
//}
//
//
//static cv::Scalar compute_mode(const cv::Mat & image, cv::InputArray _mask = cv::noArray())
//{
//  cv::Mat1f H;
//  double minval = -1, maxval = -1;
//  int nbins = -1;
//
//  bool fOK =
//      create_histogram(image,
//          _mask,
//          H,
//          &minval,
//          &maxval,
//          nbins,
//          false,
//          false);
//
//  if( !fOK ) {
//    CF_ERROR("create_histogram() fails");
//    return cv::Scalar();
//  }
//
//  cv::Scalar s;
//
//  for( int i = 0, n = (std::min)(4, H.cols); i < n; ++i ) {
//
//    if ( H.rows < 2 ) {
//      s[i] = minval;
//    }
//    else {
//
//      int jmax = 0;
//
//      float hmax = H[jmax][i];
//
//      for( int j = 1, m = H.rows; j < m; ++j ) {
//        if( H[j][i] > hmax ) {
//          jmax = j;
//          hmax = H[j][i];
//        }
//      }
//
//      s[i] = minval + jmax * (maxval - minval) / (H.rows - 1);
//    }
//  }
//
//  return s;
//}
//

// v' = (v - mv ) * stretch + offset
// v' = v* stretch + offset - mv * stretch
bool c_histogram_normalization_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( !image.empty() ) {
    nomalize_image_histogramm(image, mask, image, normalization_type_, stretch_, offset_, colorid_);
  }
  return true;
}
