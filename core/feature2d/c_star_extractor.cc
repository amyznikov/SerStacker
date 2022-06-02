/*
 * c_star_extractor.cc
 *
 *  Created on: Jun 2, 2022
 *      Author: amyznikov
 */

#include "c_star_extractor.h"
#include <core/io/save_image.h>
#include <core/ssprintf.h>
#include <core/debug.h>

static inline float hyp(float x, float y)
{
  return sqrt(x * x + y * y);
}

static bool detect_stars(cv::InputArray _src, cv::InputArray aperture_mask, std::vector<cv::KeyPoint> & keypoints,
    const std::string & dbgpath = "")
{
  cv::Mat1f src, filtered, mean, sharpen;
  cv::Mat mask;
  cv::Mat1i labels;
  int N;

  src = _src.getMat();

  cv::medianBlur(src, filtered, 3);
  cv::GaussianBlur(filtered, filtered, cv::Size(0, 0), 2);
  //cv::GaussianBlur(src, filtered, cv::Size(0, 0), 1.5);
  if ( !dbgpath.empty() ) {
    save_image(filtered, ssprintf("%s/star_detector_filtered.tiff", dbgpath.c_str()));
  }

  cv::GaussianBlur(filtered, mean, cv::Size(0,0), 25);
  cv::scaleAdd(mean, -2, filtered, sharpen);
  cv::max(sharpen, 0, sharpen);
  cv::compare(sharpen, 0, mask, cv::CMP_GT);

  if ( !dbgpath.empty() ) {
    save_image(sharpen, ssprintf("%s/star_detector_sharpened.tiff", dbgpath.c_str()));
    save_image(mask, ssprintf("%s/star_detector_mask.tiff", dbgpath.c_str()));
  }


  if ( aperture_mask.size() == mask.size() ) {
    cv::bitwise_and(mask, aperture_mask, mask);
  }


  keypoints.clear();

  if ( (N = cv::connectedComponents(mask, labels, 8, labels.type())) < 2 ) {
    CF_FATAL("connectedComponents() can not find connected components, N=%d", N);
    return false;
  }


  keypoints.resize(N - 1);

  for ( int y = 0; y < src.rows; ++y ) {
    for ( int x = 0; x < src.cols; ++x ) {
      const int lb = labels[y][x];
      if ( lb > 0 ) {
        const float I = src[y][x] - mean[y][x];
        if ( I > 0 ) {
          cv::KeyPoint & kp = keypoints[lb - 1];
          kp.pt.x += x * I;
          kp.pt.y += y * I;
          kp.response += I;
        }
      }
    }
  }

  for ( cv::KeyPoint & kp : keypoints) {
    kp.pt.x /= kp.response;
    kp.pt.y /= kp.response;
    kp.size = 25;
  }

  if ( true ) {
    for ( int i = 0, n = keypoints.size(); i < n - 1; ++i ) {
      cv::KeyPoint & cp = keypoints[i];
      if ( cp.response > 0 ) {
        for ( int j = i + 1; j < n; ++j ) {
          cv::KeyPoint & sp = keypoints[j];
          if ( sp.response > 0 ) {
            const float dist = hyp(sp.pt.x - cp.pt.x, sp.pt.y - cp.pt.y);
            if ( dist < 7 ) {
              const float w = 1.f / (cp.response + sp.response);
              cp.pt.x = (cp.pt.x * cp.response + sp.pt.x * sp.response) * w;
              cp.pt.y = (cp.pt.y * cp.response + sp.pt.y * sp.response) * w;
              //cp.size = (cp.size * cp.response + sp.size * sp.response) * w;
              cp.response += sp.response;
              sp.response  = 0;
            }
          }
        }
      }
    }
  }

  std::sort(keypoints.begin(), keypoints.end(),
      [](const cv::KeyPoint & prev, const cv::KeyPoint & next) -> bool {
        return prev.response > next.response;
      });

  for ( auto ii = keypoints.begin(); ii != keypoints.end(); ++ii ) {
    if ( ii->response <= 0 ) {
      keypoints.erase(ii, keypoints.end());
      break;
    }
  }

  CF_DEBUG("keypoints.size()=%zu", keypoints.size());
  if ( keypoints.size() > 150 ) {
    keypoints.erase(keypoints.begin()+150, keypoints.end());
  }

  if ( !dbgpath.empty() ) {
    cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);

    for ( auto ii = keypoints.begin(); ii != keypoints.end(); ++ii ) {
      cv::circle(mask, ii->pt, 5, CV_RGB(255, 0, 0), 1, cv::LINE_8);
    }

    save_image(mask, ssprintf("%s/star_detector_keypoints.tiff", dbgpath.c_str()));
  }


  return true;
}

c_star_extractor::c_star_extractor(int /*numOctaves*/)
{
}

cv::Ptr<c_star_extractor> c_star_extractor::create(int numOctaves)
{
  return cv::Ptr<this_class>(new this_class(numOctaves));
}

void c_star_extractor::detect(cv::InputArray _src, std::vector<cv::KeyPoint> & keypoints, cv::InputArray _mask)
{
  detect_stars(_src, _mask, keypoints);
}
