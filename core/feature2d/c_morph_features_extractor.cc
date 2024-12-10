/*
 * c_morph_features_extractor.cc
 *
 *  Created on: Sep 24, 2023
 *      Author: amyznikov
 */

#include "c_morph_features_extractor.h"
#include <core/proc/morphology.h>
#include <core/proc/reduce_channels.h>
#include <core/ssprintf.h>


template<>
const c_enum_member* members_of<c_morph_features_extractor::MORPH_TYPE>()
{
  static const c_enum_member members[] = {
      { c_morph_features_extractor::MORPH_GRADIENT, "GRADIENT", "" },
      { c_morph_features_extractor::MORPH_LAPLACIAN, "LAPLACIAN", "" },
      { c_morph_features_extractor::MORPH_LAPLACIAN },
  };

  return members;
}

template<class T>
static void extract_keypoints_(const cv::Mat & img, const cv::Mat & mask, T threshold, int size, std::vector<cv::KeyPoint> & keypoints)
{
  const cv::Mat_<T> src =
      img;

  if ( mask.empty() ) {

    for( int y = 0; y < src.rows; ++y ) {
      for( int x = 0; x < src.cols; ++x ) {
        if( src[y][x] > threshold ) {
          keypoints.emplace_back(cv::KeyPoint(x, y, size, -1, src[y][x], 0, -1));
        }
      }
    }

  }
  else {

    const cv::Mat1b msk =
        mask;

    for( int y = 0; y < src.rows; ++y ) {
      for( int x = 0; x < src.cols; ++x ) {
        if( msk[y][x] && src[y][x] > threshold ) {
          keypoints.emplace_back(cv::KeyPoint(x, y, size, -1, src[y][x], 0, -1));
        }
      }
    }

  }
}

static void extract_keypoints(const cv::Mat & img, const cv::Mat & mask, double threshold, int size, std::vector<cv::KeyPoint> & keypoints)
{
  switch (img.depth()) {
    case CV_8U:
      return extract_keypoints_<uint8_t>(img, mask, (uint8_t) threshold, size, keypoints);
    case CV_8S:
      return extract_keypoints_<int8_t>(img, mask, (int8_t) threshold, size, keypoints);
    case CV_16U:
      return extract_keypoints_<uint16_t>(img, mask, (uint16_t) threshold, size, keypoints);
    case CV_16S:
      return extract_keypoints_<int16_t>(img, mask, (int16_t) threshold, size, keypoints);
    case CV_32S:
      return extract_keypoints_<int32_t>(img, mask, (int32_t) threshold, size, keypoints);
    case CV_32F:
      return extract_keypoints_<float>(img, mask, (float) threshold, size, keypoints);
    case CV_64F:
      return extract_keypoints_<double>(img, mask, (double) threshold, size, keypoints);
  }
}

c_morph_features_extractor::c_morph_features_extractor()
{
}

c_morph_features_extractor::c_morph_features_extractor(const Options & opts) :
    options_(opts)
{
}

cv::Ptr<c_morph_features_extractor> c_morph_features_extractor::create()
{
  return cv::Ptr<this_class>(new this_class());
}

cv::Ptr<c_morph_features_extractor> c_morph_features_extractor::create(const Options & opts)
{
  return cv::Ptr<this_class>(new this_class(opts));
}

void c_morph_features_extractor::detect(cv::InputArray _src, std::vector<cv::KeyPoint> & keypoints, cv::InputArray mask)
{
  const int ksize =
      2 * std::max(1, options_.se_radius) + 1;

  static thread_local cv::Mat1b SE;

  if( SE.rows != ksize ) {
    SE = cv::Mat1b(ksize, ksize, 255);
  }

  cv::Mat lap, lapm;

  switch (options_.morph_type) {
    case MORPH_GRADIENT:
      morphological_gradient(_src, lap, SE, cv::BORDER_REPLICATE);
      break;
    default:
      morphological_laplacian_abs(_src, lap, SE, cv::BORDER_REPLICATE);
      break;
  }

  if ( lap.channels() != 1 ) {
    reduce_color_channels(lap, lap, cv::REDUCE_MAX);
  }

  keypoints.clear();

  extract_keypoints(lap, mask.getMat(),
      options_.threshold,
      ksize,
      keypoints);
}
