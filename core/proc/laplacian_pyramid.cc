/*
 * laplacian_pyramid.cc
 *
 *  Created on: Feb 7, 2023
 *      Author: amyznikov
 */

#include "laplacian_pyramid.h"
#include <core/debug.h>

void build_gaussian_pyramid(cv::InputArray input_image,
    std::vector<cv::Mat> & pyramid,
    int minimum_image_size,
    cv::BorderTypes borderType,
    double stretch_factor)
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

    if( stretch_factor != 0 && stretch_factor != 1 ) {
      pyramid.emplace_back(filtered_image * stretch_factor);
    }
    else {
      pyramid.emplace_back(filtered_image);
    }
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

static void compute_m(const cv::Mat & gp, const cv::Mat & gn, cv::Mat & m, double scale)
{
  cv::Mat tmp;
  cv::pyrUp(gp, tmp);

  cv::pyrUp(gn, m, gp.size());
  cv::pyrUp(m, m, tmp.size());

  cv::absdiff(m, tmp, m);

  cv::pyrDown(m, m, gp.size());
  cv::pyrDown(m, m, gn.size());
}

} // namespace


/**
 * Pure laplacian magnitude pyramid
 */
bool build_mpyramid(cv::InputArray input_image,
    std::vector<cv::Mat> & output_layers,
    int minimum_image_size)
{

  output_layers.clear();

  output_layers.emplace_back(input_image.getMat().clone());

  while ( 42 ) {

    const cv::Size current_size =
        output_layers.back().size();

    const cv::Size next_size((current_size.width + 1) / 2,
        (current_size.height + 1) / 2);

//    CF_DEBUG("scale: %dx%d -> %dx%d",
//        current_size.width, current_size.height,
//        next_size.width, next_size.height);

    if( (std::min)(next_size.width, next_size.height) < minimum_image_size ) {
      break;
    }

    cv::Mat l, m;

    cv::pyrDown(output_layers.back(), l, next_size);
    compute_m(output_layers.back(), l, m, 6);
    output_layers.emplace_back(m);
  }

  return true;
}


c_melp_pyramid::sptr build_melp_pyramid(cv::InputArray input_image, int minimum_image_size)
{
  c_melp_pyramid::sptr p(new c_melp_pyramid());

  input_image.getMat().copyTo(p->image);

  const cv::Size nextSize((p->image.cols + 1) / 2,
      (p->image.rows + 1) / 2);

  if( (std::min)(nextSize.width, nextSize.height) >= minimum_image_size ) {

    cv::Mat l, m;

    cv::pyrDown(p->image, l, nextSize);
    compute_m(p->image, l, m, 6);

    p->l = build_melp_pyramid(l, minimum_image_size);
    p->m = build_melp_pyramid(m * 6, minimum_image_size);
  }

  return p;
}
