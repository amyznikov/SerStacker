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

// Compute first-order derivatives
void differentiate(cv::InputArray _src, cv::OutputArray gx,  cv::OutputArray gy,
    double scale = 1, double delta = 0);

bool compute_gradient(cv::InputArray src, cv::OutputArray dst,
    int dx, int dy,
    int kradius,
    double scale = 1,
    double delta = 0);

bool compute_sobel_gradients(cv::InputArray src,
    cv::OutputArray gx,
    cv::OutputArray gy,
    int borderType = cv::BORDER_DEFAULT,
    double scale = 1,
    double delta = 0);

// To compute Determinant:
//  cv::subtract(gxx.mul(gyy), gxy.mul(gxy), det);
//  cv::absdiff(gx.mul(gy), gxy.mul(gxy), det_abs);
bool compute_second_sobel_derivatives(cv::InputArray src,
    cv::OutputArray gxx,
    cv::OutputArray gyy,
    cv::OutputArray gxy,
    int borderType = cv::BORDER_REPLICATE,
    double scale=1.0,
    double delta=0.0);

bool compute_hessian_eigenvalues(cv::InputArray gxx, cv::InputArray gxy, cv::InputArray gyy,
    cv::OutputArray mu1, cv::OutputArray mu2);

/**
 * The output histogram is 45 deg phase shifted.
 * To RESTORE THE PHASE subtract the added 45 degrees back:
 *    double final_angle = angle - 45.0;
 *    if (final_angle < 0.0) {
 *      final_angle += 180.0;
 *    }
 */
bool compute_histogram_of_gradient_directions(cv::InputArray _gx, cv::InputArray _gy,
    cv::OutputArray outputHistogram);
bool compute_histogram_of_gradient_directions(cv::InputArray _image,
    cv::OutputArray outputHistogram);

/**
* @brief Spatial Radon via gradient tensor projection (Option 2)
* @param spatialCrop Cropped square ROI from the image
* @return double Dominant axis angle in degrees [0, 180)
*/
double gradientEstimateRadonOrientation(const cv::Mat1f & spatialCrop,
    cv::OutputArray outputDebugHistogram = cv::noArray());

#endif /* __gradient_h__ */
