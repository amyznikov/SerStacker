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

  if( _median_filter_radius > 0 ) {
    cv::medianBlur(_image, _image, 2 * _median_filter_radius + 1);
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
