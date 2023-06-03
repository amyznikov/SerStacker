/*
 * laplacian_pyramid.cc
 *
 *  Created on: Feb 7, 2023
 *      Author: amyznikov
 */

#include "laplacian_pyramid.h"

void build_gaussian_pyramid(cv::InputArray input_image,
    std::vector<cv::Mat> & pyramid,
    int minimum_image_size,
    cv::BorderTypes borderType)
{
  pyramid.emplace_back(input_image.getMat().clone());

  while (42) {

    const cv::Size nextSize((pyramid.back().cols + 1) / 2,
        (pyramid.back().rows + 1) / 2);

    if( (std::min)(nextSize.width, nextSize.height) <= minimum_image_size ) {
      break;
    }

    cv::Mat filtered_image;

    cv::pyrDown(pyramid.back(), filtered_image,
        nextSize,
        borderType);

    pyramid.emplace_back(filtered_image);
  }
}


void build_laplacian_pyramid(cv::InputArray input_image,
    std::vector<cv::Mat> & pyramid,
    int minimum_image_size,
    cv::BorderTypes borderType)
{
  pyramid.emplace_back();

  input_image.getMat().convertTo(pyramid.back(),
      CV_32F);

  while (42) {

    const cv::Size nextSize((pyramid.back().cols + 1) / 2,
        (pyramid.back().rows + 1) / 2);

    if( (std::min)(nextSize.width, nextSize.height) <= minimum_image_size ) {
      break;
    }

    cv::Mat filtered_image;
    cv::Mat upscaled_image;

    cv::pyrDown(pyramid.back(), filtered_image,
        nextSize,
        borderType);

    cv::pyrUp(filtered_image, upscaled_image,
        pyramid.back().size(),
        borderType);

    cv::subtract(pyramid.back(), upscaled_image,
        pyramid.back());

    pyramid.emplace_back(filtered_image);
  }
}

void reconstruct_laplacian_pyramid(cv::OutputArray output_image,
    const std::vector<cv::Mat> & pyramid,
    cv::BorderTypes borderType)
{
  cv::Mat current_image;

  const int pyrsize =
      pyramid.size();

  if( pyrsize > 0 ) {

    pyramid.back().copyTo(current_image);

    for( int i = pyrsize - 2; i >= 0; --i ) {

      cv::pyrUp(current_image, current_image,
          pyramid[i].size(),
          borderType);

      cv::add(current_image, pyramid[i],
          current_image);
    }
  }

  if( !output_image.fixedType() || output_image.type() == current_image.type() ) {

    output_image.move(current_image);
  }
  else {

    current_image.convertTo(output_image,
        output_image.depth());
  }
}

namespace {

static void compute_m(const cv::Mat & gp, const cv::Mat & gn, cv::Mat & m)
{
  cv::Mat tmp;
  cv::pyrUp(gp, tmp);

  cv::pyrUp(gn, m, gp.size());
  cv::pyrUp(m, m, tmp.size());

  cv::absdiff(m, tmp, m);

  cv::pyrDown(m, m, gp.size());
  cv::pyrDown(m, m, gn.size());
}

static void build_melp_pyramid(const cv::Mat & G, c_melp_pyramid & p, int minimum_image_size)
{
  build_gaussian_pyramid(G, p.g, minimum_image_size);

  if ( p.g.size() > 1 ) {

    cv::Mat m;

    for( int s = 0; s < (int) (p.g.size()) - 1; ++s ) {

      compute_m(p.g[s], p.g[s + 1], m);

      p.p.emplace_back();

      build_melp_pyramid(m, p.p.back(), minimum_image_size);
    }
  }
}

} // namespace

void build_melp_pyramid(cv::InputArray input_image, c_melp_pyramid * p, int min_image_size)
{
  p->g.clear();
  p->p.clear();

  build_melp_pyramid(input_image.getMat(), *p, min_image_size);
}
