/*
 * c_star_extractor.cc
 *
 *  Created on: Jun 2, 2022
 *      Author: amyznikov
 */

#include "c_star_extractor.h"
#include <core/proc/estimate_noise.h>
#include <core/proc/fast_gaussian_blur.h>
#include <core/io/save_image.h>
#include <core/ssprintf.h>
#include <core/debug.h>
//
//static inline float hyp(float x, float y)
//{
//  return sqrt(x * x + y * y);
//}
//
//static bool detect_stars(cv::InputArray _src, cv::InputArray aperture_mask, std::vector<cv::KeyPoint> & keypoints,
//    const std::string & dbgpath = "")
//{
//  cv::Mat1f src, filtered, mean, sharpen;
//  cv::Mat s;
//  cv::Mat mask;
//  cv::Mat1i labels;
//  int N;
//
//  if( _src.channels() == 3 ) {
//    cv::cvtColor(_src, s, cv::COLOR_BGR2GRAY);
//  }
//  else {
//    s = _src.getMat();
//  }
//
//  if ( s.depth() == src.depth() ) {
//    src = s;
//  }
//  else {
//    s.convertTo(src, src.depth());
//  }
//
//  //cv::medianBlur(src, filtered, 3);
//  //cv::GaussianBlur(filtered, filtered, cv::Size(0, 0), 2);
//  cv::GaussianBlur(src, filtered, cv::Size(0, 0), 1);
//
//
//  if ( !dbgpath.empty() ) {
//    save_image(filtered, ssprintf("%s/star_detector_filtered.tiff", dbgpath.c_str()));
//  }
//
//  cv::GaussianBlur(filtered, mean, cv::Size(0,0), 15);
//  cv::scaleAdd(mean, -2, filtered, sharpen);
//  cv::max(sharpen, 0, sharpen);
//  cv::compare(sharpen, 0, mask, cv::CMP_GT);
//
//  if ( !dbgpath.empty() ) {
//    save_image(sharpen, ssprintf("%s/star_detector_sharpened.tiff", dbgpath.c_str()));
//    save_image(mask, ssprintf("%s/star_detector_mask.tiff", dbgpath.c_str()));
//  }
//
//  if ( aperture_mask.size() == mask.size() ) {
//    cv::bitwise_and(mask, aperture_mask, mask);
//  }
//
//  keypoints.clear();
//
//  if ( (N = cv::connectedComponents(mask, labels, 8, labels.type())) < 2 ) {
//    CF_FATAL("connectedComponents() can not find connected components, N=%d", N);
//    return false;
//  }
//
//  keypoints.resize(N - 1);
//
//  for ( int y = 0; y < src.rows; ++y ) {
//    for ( int x = 0; x < src.cols; ++x ) {
//      const int lb = labels[y][x];
//      if ( lb > 0 ) {
//        const float I = src[y][x] - mean[y][x];
//        if ( I > 0 ) {
//          cv::KeyPoint & kp = keypoints[lb - 1];
//          kp.pt.x += x * I;
//          kp.pt.y += y * I;
//          kp.response += I;
//        }
//      }
//    }
//  }
//
//
//  for ( cv::KeyPoint & kp : keypoints) {
//    kp.pt.x /= kp.response;
//    kp.pt.y /= kp.response;
//    kp.size = 25;
//  }
//
//  if ( true ) {
//    for ( int i = 0, n = keypoints.size(); i < n - 1; ++i ) {
//      cv::KeyPoint & cp = keypoints[i];
//      if ( cp.response > 0 ) {
//        for ( int j = i + 1; j < n; ++j ) {
//          cv::KeyPoint & sp = keypoints[j];
//          if ( sp.response > 0 ) {
//            const float dist = hyp(sp.pt.x - cp.pt.x, sp.pt.y - cp.pt.y);
//            if ( dist < 7 ) {
//              const float w = 1.f / (cp.response + sp.response);
//              cp.pt.x = (cp.pt.x * cp.response + sp.pt.x * sp.response) * w;
//              cp.pt.y = (cp.pt.y * cp.response + sp.pt.y * sp.response) * w;
//              //cp.size = (cp.size * cp.response + sp.size * sp.response) * w;
//              cp.response += sp.response;
//              sp.response  = 0;
//            }
//          }
//        }
//      }
//    }
//  }
//
//
//  std::sort(keypoints.begin(), keypoints.end(),
//      [](const cv::KeyPoint & prev, const cv::KeyPoint & next) -> bool {
//        return prev.response > next.response;
//      });
//
//  for ( auto ii = keypoints.begin(); ii != keypoints.end(); ++ii ) {
//    if ( ii->response <= 0 ) {
//      keypoints.erase(ii, keypoints.end());
//      break;
//    }
//  }
//
//  CF_DEBUG("keypoints.size()=%zu", keypoints.size());
//  if ( keypoints.size() > 150 ) {
//    keypoints.erase(keypoints.begin()+150, keypoints.end());
//  }
//
//  if ( !dbgpath.empty() ) {
//    cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);
//
//    for ( auto ii = keypoints.begin(); ii != keypoints.end(); ++ii ) {
//      cv::circle(mask, ii->pt, 5, CV_RGB(255, 0, 0), 1, cv::LINE_8);
//    }
//
//    save_image(mask, ssprintf("%s/star_detector_keypoints.tiff", dbgpath.c_str()));
//  }
//
//
//  return true;
//}

static void compute_dog(cv::InputArray src, cv::OutputArray dst, double sigma1, double sigma2, int ddepth)
{
  cv::Mat gb1, gb2;

  if( sigma1 <= 0 ) {
    gb1 = src.getMat();
  }
  else {
    fast_gaussian_blur(src.getMat(), cv::noArray(), gb1, sigma1);
  }

  if( sigma2 <= 0 ) {
    gb2 = src.getMat();
  }
  else {
    fast_gaussian_blur(src.getMat(), cv::noArray(), gb2, sigma2);
  }

  cv::subtract(gb1, gb2, dst, cv::noArray(), ddepth);
}


c_star_extractor::c_star_extractor()
{
}

cv::Ptr<c_star_extractor> c_star_extractor::create()
{
  return cv::Ptr<this_class>(new this_class());
}

void c_star_extractor::detect(cv::InputArray src, std::vector<cv::KeyPoint> & keypoints, cv::InputArray mask)
{
  //detect_stars(_src, _mask, keypoints);

  if ( src.channels() == 3 ) {
    cv::cvtColor(src, _image, cv::COLOR_BGR2GRAY);
  }
  else {
    _image = src.getMat();
  }

  create_noise_map(_image, _noisemap);
  fast_gaussian_blur(cv::abs(_noisemap), cv::noArray(),
      _noisemap,
      _noise_blur,
      cv::BORDER_DEFAULT,
      CV_32F);

  if( _median_filter_size > 1 ) {
    cv::medianBlur(_image, _image, _median_filter_size);
  }

  compute_dog(_image, _image, _sigma1, _sigma2, CV_32F);
  cv::scaleAdd(_noisemap, -_noise_threshold, _image, _image);
  cv::compare(_image, cv::Scalar::all(0), _cc, cv::CMP_GT);

  _blobs.clear();

  const int N = cv::connectedComponents(_cc, _labels, 8, _labels.type());
  if ( N > 1 ) {

    _blobs.resize(N);

    const cv::Mat1f dog = _image;

    for( int y = 0; y < _labels.rows; ++y ) {

      const int32_t * lbp = _labels[y];
      const float * dogp = dog[y];

      for( int x = 0; x < _labels.cols; ++x ) {
        const int32_t lb = lbp[x];
        if( lb ) {

          Blob & b = _blobs[lb];
          const float I = dogp[x];

          b.x += I * x;
          b.y += I * y;
          b.x2 += I * x * x;
          b.y2 += I * y * y;
          b.xy += I * x * y;
          b.I += I;
          b.n += 1;
        }
      }
    }

    double mamb = 0;
    double mabr = 0;
    double mtheta = 0;
    double stheta = 0;
    double w = 0;

    for( int lb = 1; lb < N; ++lb ) {

      Blob & b = _blobs[lb];
      if( b.n > _min_pts ) {
        b.x /= b.I;
        b.y /= b.I;
        b.x2 = b.x2 / b.I - b.x * b.x;
        b.y2 = b.y2 / b.I - b.y * b.y;
        b.xy = b.xy / b.I - b.x * b.y;
        b.theta = (b.x2 == b.y2) ? 0.0 : 0.5 * std::atan2(2 * b.xy, b.x2 - b.y2);

        const double D = b.theta == 0 ? 0.0 : b.xy / std::sin(2 * b.theta);
        b.a = std::sqrt(0.5 * (b.x2 + b.y2 + D));
        b.b = std::sqrt(0.5 * (b.x2 + b.y2 - D));

        if( b.b > _min_b && b.b / b.a > _min_ba_ratio ) {

          keypoints.emplace_back(cv::KeyPoint(b.x, b.y, 5 * b.a, b.theta * 180 / CV_PI, b.I));

          if  ( _min_score > 0 ) {
            const double ww = b.a;
            mamb += (b.a - b.b) * ww;
            mabr += b.b / b.a * ww;
            mtheta += b.theta * ww;
            stheta += b.theta * b.theta * ww;
            w += ww;
          }
        }
      }
    }

    if  ( _min_score > 0 ) {

      if (w > 0 ) {
        mamb = mamb / w;
        mabr = mabr / w;
        mtheta = mtheta / w;
        stheta = sqrt(stheta/w - mtheta * mtheta);
      }

      const double score =
          stheta * mabr;

      CF_DEBUG("mamb = %g  mabr = %g  mtheta = %g  stheta = %g score=%g",
          mamb, mabr, mtheta, stheta, score );

      if ( score < _min_score ) {
        CF_ERROR("Clear keypoints by average frame score = %g < %g", score, _min_score);
        keypoints.clear();
      }
    }
  }

}
