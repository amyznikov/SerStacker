/*
 * gradient.h
 *
 *  Created on: May 1, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __gradient_h__
#define __gradient_h__

#include <opencv2/opencv.hpp>

bool compute_gradient(cv::InputArray src, cv::OutputArray dst,
    int dx, int dy,
    int kradius,
    int ddepth = -1,
    double delta = 0,
    double scale = 1);

bool compute_sobel_gradients(cv::InputArray src,
    cv::OutputArray gx,
    cv::OutputArray gy,
    int ddepth = -1,
    int borderType = cv::BORDER_DEFAULT);

bool compute_diagonal_gradients(cv::InputArray src,
    cv::OutputArray gx,
    cv::OutputArray gy,
    int ddepth = -1,
    int borderType = cv::BORDER_DEFAULT);


#endif /* __gradient_h__ */
