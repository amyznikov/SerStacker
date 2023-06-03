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

void build_gaussian_pyramid(cv::InputArray input_image,
    std::vector<cv::Mat> & pyramid,
    int minimum_image_size = 4,
    cv::BorderTypes borderType = cv::BORDER_DEFAULT);

void build_laplacian_pyramid(cv::InputArray input_image,
    std::vector<cv::Mat> & pyramid,
    int minimum_image_size = 4,
    cv::BorderTypes borderType = cv::BORDER_DEFAULT);

void reconstruct_laplacian_pyramid(cv::OutputArray output_image,
    const std::vector<cv::Mat> & pyramid,
    cv::BorderTypes borderType = cv::BORDER_DEFAULT);


/**
 * Mikhail Sizintsev
 *  Hierarchical stereo with thin structures and transparency
 *    <https://www.researchgate.net/publication/4352693_Hierarchical_Stereo_with_Thin_Structures_and_Transparency>
 */

struct c_melp_pyramid
{
  std::vector<cv::Mat> g;
  std::vector<c_melp_pyramid> p;
};

void build_melp_pyramid(cv::InputArray input_image,
    c_melp_pyramid * p,
    int min_image_size = 8);


#endif /* __laplacian_pyramid_h__ */
