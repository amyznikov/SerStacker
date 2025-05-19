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

// To compute Determinant:
//  cv::subtract(gxx.mul(gyy), gxy.mul(gxy), det);
//  cv::absdiff(gx.mul(gy), gxy.mul(gxy), det_abs);
bool compute_second_sobel_derivatives(cv::InputArray src,
    cv::OutputArray gxx,
    cv::OutputArray gyy,
    cv::OutputArray gxy,
    int ddepth,
    int borderType = cv::BORDER_REPLICATE,
    double scale=1.0,
    double delta=0.0);

bool compute_hessian_eigenvalues(cv::InputArray gxx, cv::InputArray gxy, cv::InputArray gyy,
    cv::OutputArray mu1, cv::OutputArray mu2);

#endif /* __gradient_h__ */
