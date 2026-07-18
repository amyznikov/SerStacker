/*
 * c_simple_star_detector.cc
 *
 *  Created on: Jul 15, 2026
 *      Author: amyznikov
 */

#include "c_simple_star_detector.h"
#include <core/proc/downstrike.h>
// #include <core/proc/reduce_channels.h>

using Blob = c_simple_star_detector::Blob;

static void build_dog(cv::InputArray _src, cv::OutputArray _dst, const cv::Mat1f & G,
    int maxlevels, double weight_decay = 0.5)
{
  std::vector<cv::Mat> dogs;
  cv::Mat current_image, lpass;

//  const int ksize = std::max(5, 2 * int(sigma * 3) + 1);
//  const cv::Mat G = cv::getGaussianKernel(ksize, sigma, CV_32F);

  dogs.reserve(maxlevels);

  cv::sepFilter2D(_src, current_image, CV_32F, G, G, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

  for( int level = 0; level < maxlevels; ++level ) {

    dogs.emplace_back();
    cv::sepFilter2D(current_image, lpass, CV_32F, G, G, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
    cv::subtract(current_image, lpass, dogs.back());

    if( level < maxlevels - 1 ) {
      downstrike_uneven(lpass, current_image);
    }
  }

  current_image = dogs.back();
  for( int i = maxlevels - 2; i >= 0; --i ) {
    const cv::Mat & current_layer = dogs[i];
    cv::pyrUp(current_image, lpass, current_layer.size());
    if( weight_decay == 1 ) {
      cv::add(lpass, current_layer, current_image);
    }
    else {
      cv::scaleAdd(lpass, weight_decay, current_layer, current_image);
    }
  }

  _dst.move(current_image);
}

static inline void computeMAD(cv::InputArray _src, cv::Scalar &outputMean, cv::Scalar &outputMAD,
    cv::InputArray _mask = cv::noArray())
{
  cv::Mat tmp;
  cv::absdiff(_src, outputMean = cv::mean(_src, _mask), tmp);
  outputMAD = cv::mean(tmp, _mask);
}

const std::vector<Blob> & c_simple_star_detector::detect(cv::InputArray _image,
    cv::InputArray _mask)
{
  cv::Mat src;
  cv::Mat1i labels;
  cv::Scalar Mean, MAD;

  if ( _image.channels() == 1 ) {
    src = _image.getMat();
  }
  else {
    cv::cvtColor(_image, src, cv::COLOR_BGR2GRAY);
  }

  const int gksize = std::max(5, 2 * int(_opts.sigma * 3) + 1);
  if ( _opts.sigma != gsigma || gksize != G.rows ) {
    G = cv::getGaussianKernel(gksize, gsigma = _opts.sigma, CV_32F);
  }


  build_dog(src, dog, G, _opts.lvls, _opts.weight_decay);
  computeMAD(dog, Mean, MAD, _mask);
  cv::compare(dog, Mean + _opts.kmad * MAD, cc, cv::CMP_GT);

  if( _opts.se_radius > 0 ) {
    const int se_size = 2 * _opts.se_radius + 1;
    if( SE.rows != se_size ) {
      SE = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(se_size, se_size));
    }
    cv::dilate(cc, cc, SE);
  }

  if( !_mask.empty() ) {
    cv::bitwise_and(_mask, cc, cc);
  }

  const int N = cv::connectedComponents(cc, labels, 8, labels.type());
  _final_blobs.clear(), _final_blobs.reserve(N);

  if ( N > 1 ) {

    std::vector<Blob> all_blobs(N);
    for( int y = 0; y < labels.rows; ++y ) {

      const int32_t * lbp = labels[y];
      const float * dogp = dog[y];

      for( int x = 0; x < labels.cols; ++x ) {
        const int32_t lb = lbp[x];
        if( lb ) {

          Blob & b = all_blobs[lb];
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

    for( int lb = 1; lb < N; ++lb ) {
      Blob & b = all_blobs[lb];
      if( b.n > 3 ) {
        b.x /= b.I;
        b.y /= b.I;
        b.x2 = b.x2 / b.I - b.x * b.x;
        b.y2 = b.y2 / b.I - b.y * b.y;
        b.xy = b.xy / b.I - b.x * b.y;
        //b.theta = (b.x2 == b.y2) ? 0.0 : 0.5 * std::atan2(2 * b.xy, b.x2 - b.y2);
        if (b.x2 == b.y2 && b.xy == 0.0) {
          b.theta = 0.0;
        }
        else {
          b.theta = 0.5 * std::atan2(2.0 * b.xy, b.x2 - b.y2);
        }

        const double trace = b.x2 + b.y2;
        const double diff  = b.x2 - b.y2;
        const double D     = std::sqrt(diff * diff + 4.0 * b.xy * b.xy);
        b.a = std::sqrt(std::max(0.0, 0.5 * (trace + D)));
        b.b = std::sqrt(std::max(0.0, 0.5 * (trace - D)));
        if( !(b.b > 0 && b.a >= _opts.min_a && b.a <= _opts.max_a) ) {
          continue;
        }

        // Mathematical variance of the pixel grid (1/12 = ~0.08333)
        const double GRID_VARIANCE = 6.0 / 12.0;
        const double MAX_ELONGATION_BASE = _opts. max_elongation;
        const double max_elongation_sq = MAX_ELONGATION_BASE * MAX_ELONGATION_BASE;
        const double regularized_elongation_sq = (b.a * b.a + GRID_VARIANCE) / (b.b * b.b + GRID_VARIANCE);
        if (regularized_elongation_sq > max_elongation_sq) {
          continue; // The object is elongated (defect, cosmic ray track, etc.)
        }

        const double expected_area = 3.1415926535 * b.a * b.b;
        const double compactness = b.n / expected_area;
        if( !(compactness >= _opts.min_compactness) ) {
          continue;
        }

        _final_blobs.emplace_back(b);
      }
    }
  }

  return _final_blobs;
}

bool serialize_simple_star_detector_options(c_config_setting section, bool save,
    c_simple_star_detector_options & opts)
{
  SERIALIZE_OPTION(section, save, opts, sigma);
  SERIALIZE_OPTION(section, save, opts, weight_decay);
  SERIALIZE_OPTION(section, save, opts, kmad);
  SERIALIZE_OPTION(section, save, opts, se_radius);
  SERIALIZE_OPTION(section, save, opts, lvls);
  SERIALIZE_OPTION(section, save, opts, min_a);
  SERIALIZE_OPTION(section, save, opts, max_a);
  SERIALIZE_OPTION(section, save, opts, max_elongation);
  SERIALIZE_OPTION(section, save, opts, min_compactness);
  return true;
}

