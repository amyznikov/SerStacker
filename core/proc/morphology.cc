/*
 * morphology.cc
 *
 *  Created on: Mar 20, 2021
 *      Author: amyznikov
 */

#include "morphology.h"
#include <core/proc/downstrike.h>

void morphological_smooth_close(cv::InputArray src, cv::OutputArray dst, cv::InputArray SE,
    int borderType, const cv::Scalar & borderValue)
{
  cv::morphologyEx(src, dst, cv::MORPH_CLOSE, SE, cv::Point(-1, -1), 1, borderType, borderValue);
  cv::morphologyEx(dst, dst, cv::MORPH_OPEN, SE, cv::Point(-1, -1), 1, borderType, borderValue);
}

void morphological_smooth_open(cv::InputArray src, cv::OutputArray dst, cv::InputArray SE,
    int borderType, const cv::Scalar & borderValue)
{
  cv::morphologyEx(src, dst, cv::MORPH_OPEN, SE, cv::Point(-1, -1), 1, borderType, borderValue);
  cv::morphologyEx(dst, dst, cv::MORPH_CLOSE, SE, cv::Point(-1, -1), 1, borderType, borderValue);
}

void morphological_gradient(cv::InputArray src, cv::OutputArray dst, cv::InputArray SE,
    int borderType, const cv::Scalar& borderValue)
{
  cv::morphologyEx(src, dst, cv::MORPH_GRADIENT, SE, cv::Point(-1, -1), 1, borderType, borderValue);
}

void morphological_internal_gradient(cv::InputArray src, cv::OutputArray dst, cv::InputArray SE,
    int borderType, const cv::Scalar& borderValue)
{
  cv::Mat tmp;
  cv::erode(src, tmp, SE, cv::Point(-1, -1), 1, borderType, borderValue);
  cv::subtract(src, tmp, dst);
}

void morphological_external_gradient(cv::InputArray src, cv::OutputArray dst, cv::InputArray SE,
    int borderType, const cv::Scalar& borderValue)
{
  cv::Mat tmp;
  cv::morphologyEx(src, tmp, cv::MORPH_DILATE, SE, cv::Point(-1, -1), 1, borderType, borderValue);
  cv::subtract(tmp, src, dst);
}

void morphological_laplacian(cv::InputArray src, cv::OutputArray dst, cv::InputArray SE,
    int borderType, const cv::Scalar& borderValue)
{
  cv::Mat m1, m2;
  cv::morphologyEx(src, m1, cv::MORPH_DILATE, SE, cv::Point(-1, -1), 1, borderType, borderValue);
  subtract(m1, src, m1);

  cv::morphologyEx(src, m2, cv::MORPH_ERODE, SE, cv::Point(-1, -1), 1, borderType, borderValue);
  subtract(src, m2, m2);

  subtract(m1, m2, dst);
  // cv::multiply(dst, 0.5, dst);
}

void morphological_laplacian_abs(cv::InputArray src, cv::OutputArray dst, cv::InputArray SE,
    int borderType, const cv::Scalar& borderValue)
{
  cv::Mat m1, m2;
  cv::morphologyEx(src, m1, cv::MORPH_DILATE, SE, cv::Point(-1, -1), 1, borderType, borderValue);
  cv::subtract(m1, src, m1);

  cv::morphologyEx(src, m2, cv::MORPH_ERODE, SE, cv::Point(-1, -1), 1, borderType, borderValue);
  cv::subtract(src, m2, m2);

  cv::absdiff(m1, m2, dst);
  // cv::multiply(dst, 0.5, dst);
}

void rampLee(cv::InputArray src, cv::OutputArray dst, cv::InputArray SE,
    int borderType, const cv::Scalar& borderValue)
{
  // TOD: Optimize this code

  cv::Mat O, C, D, E;

  cv::morphologyEx(src, O, cv::MORPH_OPEN, SE, cv::Point(-1, -1), 1, borderType, borderValue);
  cv::morphologyEx(src, C, cv::MORPH_CLOSE, SE, cv::Point(-1, -1), 1, borderType, borderValue);
  cv::morphologyEx(src, D, cv::MORPH_DILATE, SE, cv::Point(-1, -1), 1, borderType, borderValue);
  cv::morphologyEx(src, E, cv::MORPH_ERODE, SE, cv::Point(-1, -1), 1, borderType, borderValue);

  cv::subtract(D, C, D);
  cv::subtract(O, E, O);

  dst.create(src.size(), src.type());
  cv::min(D, O, dst.getMatRef());
}

void texLee(cv::InputArray src, cv::OutputArray dst, cv::InputArray SE,
    int borderType, const cv::Scalar& borderValue)
{
  // TOD: Optimize this code
  cv::Mat O, C, CG, GO;

  cv::morphologyEx(src, O, cv::MORPH_OPEN, SE, cv::Point(-1, -1), 1, borderType, borderValue);
  cv::morphologyEx(src, C, cv::MORPH_CLOSE, SE, cv::Point(-1, -1), 1, borderType, borderValue);

  cv::subtract(C, src, CG);
  cv::subtract(src, O, GO);

  dst.create(src.size(), src.type());
  cv::min(GO, CG, dst.getMatRef());
}

void generate_morphological_line_filter_bank(std::vector<cv::Mat> & K, int ksize, int thickness)
{
  const int N = (ksize - 1) * 2;

  K.resize(N);
  for ( int i = 0; i < N; ++i ) {
    K[i] = cv::Mat1b::zeros(ksize, ksize);
  }

  int i = 0;
  for ( int t = ksize / 2; t < ksize; ++t ) {
    const cv::Point p0(ksize - 1 - t, ksize - 1);
    const cv::Point p1(t, 0);
    cv::line(K[i++], p0, p1, 255, thickness, cv::LINE_8);
  }
  for ( int t = 1; t < ksize; ++t ) {
    const cv::Point p0(0, ksize - 1 - t);
    const cv::Point p1(ksize - 1, t);
    cv::line(K[i++], p0, p1, 255, thickness, cv::LINE_8);
  }
  for ( int t = 1; t < ksize / 2; ++t ) {
    const cv::Point p0(t, 0);
    const cv::Point p1(ksize - 1 - t, ksize - 1);
    cv::line(K[i++], p0, p1, 255, thickness, cv::LINE_8);
  }

#if 0 // how to write to disk:
  for ( uint i = 0; i < K.size(); ++i ) {
    save_image(K[i], ssprintf("filters/K.%02u.png", i));
  }
#endif
}


void apply_morphological_filter_bank(const std::vector<cv::Mat> & K,
    cv::InputArray src, cv::OutputArray dst,
    int borderType, const cv::Scalar & borderValue)
{
  cv::Mat O, Max, Min;

  for ( uint i = 0, n = K.size(); i < n; ++i ) {
    cv::morphologyEx(src, O, cv::MORPH_OPEN, K[i], cv::Point(-1,-1), 1, borderType, borderValue);
    if ( i == 0 ) {
      O.copyTo(Max);
      O.copyTo(Min);
    }
    else {
      cv::max(O, Max, Max);
      cv::min(O, Min, Min);
    }
  }

  cv::subtract(Max, Min, dst);
}


/***/
void build_morph_gradient_pyramid(cv::InputArray image, cv::InputArray mask,
    std::vector<cv::Mat> & layers,
    int max_level)
{
  static const cv::Mat1b GSE(3, 3, 255);

  std::vector<cv::Mat> pyramid;
  cv::Mat tmp;

  cv::buildPyramid(image, pyramid, max_level,
      cv::BORDER_REPLICATE);

  layers.resize(pyramid.size());

  for( int l = 0, n = pyramid.size(); l < n; ++l ) {

    cv::morphologyEx(pyramid[l], layers[l],
        cv::MORPH_GRADIENT,
        GSE,
        cv::Point(-1, -1),
        1,
        cv::BORDER_REPLICATE);

    if ( l > 0 ) {

      cv::morphologyEx(layers[l-1], tmp,
          cv::MORPH_DILATE,
          GSE,
          cv::Point(1,1),
          1,
          cv::BORDER_REPLICATE);

       cv::pyrDown(tmp, tmp, pyramid[l].size(),
           cv::BORDER_REPLICATE);

      cv::add(layers[l], tmp,
          layers[l]);
    }
  }
}

/***/
void build_morph_laplacian_pyramid(cv::InputArray image, cv::InputArray mask,
    std::vector<cv::Mat> & layers,
    int max_level)
{
//
//  static constexpr float g[5] = {
//      1./16, 1./4, 3./8, 1./4, 1./16
//  };
//
//  static const cv::Mat1f Gx(1, 5, (float*)g);
//  static const cv::Mat1f Gy(5, 1, (float*)g);
//  static const cv::Mat1b GSE(3, 3, 255);
//
//  layers.clear();
//  layers.reserve(10);
//
//  cv::Mat tmp1;
//
//  while ( 42 ) {
//
//    cv::Mat tmp2;
//
//    if ( tmp1.empty() ) {
//      cv::sepFilter2D(image, tmp1, image.depth(), Gx, Gy, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
//    }
//    else {
//      cv::sepFilter2D(tmp1, tmp1, tmp1.depth(), Gx, Gy, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
//    }
//
//    cv::absdiff(tmp1, tmp2, tmp2);
//    layers.emplace_back(tmp2);
//
//    if ( layers.size() > 1 ) {
//      cv::Mat tmp3;
//      cv::dilate(layers[layers.size()-2], tmp3, GSE, cv::Point(-1,01), 1, cv::BORDER_REPLICATE);
//      cv::pyrDown(tmp3, tmp3, tmp1.size());
//      cv::max(layers.back(), tmp3, layers.back());
//    }
//
//
//    if( max_level > 0 && layers.size() >= max_level ) {
//      break;
//    }
//
//    const cv::Size current_size =
//        tmp1.size();
//
//    const cv::Size next_size =
//        cv::Size((current_size.width + 1) / 2, (current_size.height + 1) / 2);
//
//    if ( std::min( next_size.width, next_size.height) < 4 ) {
//      break;
//    }
//
//    downstrike_even(tmp1, tmp1, next_size);
//  }
//


}

