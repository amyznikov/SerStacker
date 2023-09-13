/*
 * laplacian_pyramid.h
 *
 *  Created on: Feb 7, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __laplacian_pyramid_h__
#define __laplacian_pyramid_h__

#include <opencv2/opencv.hpp>
#include <memory>

void build_gaussian_pyramid(cv::InputArray input_image,
    std::vector<cv::Mat> & pyramid,
    int minimum_image_size = 4,
    cv::BorderTypes borderType = cv::BORDER_DEFAULT,
    double stretch_factor = 1.0);

void build_laplacian_pyramid(cv::InputArray input_image,
    std::vector<cv::Mat> & lp,
    int minimum_image_size = 4,
    cv::BorderTypes borderType = cv::BORDER_DEFAULT,
    std::vector<cv::Mat> * gp = nullptr);

void reconstruct_laplacian_pyramid(cv::OutputArray output_image,
    const std::vector<cv::Mat> & pyramid,
    cv::BorderTypes borderType = cv::BORDER_DEFAULT);

/**
 * Pure laplacian magnitude pyramid
 */
bool build_mpyramid(cv::InputArray input_image,
    std::vector<cv::Mat> & output_layers,
    int min_image_size = 8);


/**
 * Mikhail Sizintsev
 *  Hierarchical stereo with thin structures and transparency
 *    <https://www.researchgate.net/publication/4352693_Hierarchical_Stereo_with_Thin_Structures_and_Transparency>
 */

struct c_melp_pyramid
{
  typedef c_melp_pyramid this_class;
  typedef std::shared_ptr<this_class> sptr;

  cv::Mat image;
  sptr l, m;
};

c_melp_pyramid::sptr build_melp_pyramid(cv::InputArray input_image,
    int min_image_size = 8);




#endif /* __laplacian_pyramid_h__ */
