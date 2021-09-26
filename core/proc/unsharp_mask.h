/*
 * unsharp_mask.h
 *
 *  Created on: Feb 20, 2021
 *      Author: amyznikov
 */

#ifndef __unsharp_mask_h__
#define __unsharp_mask_h__

#include <opencv2/opencv.hpp>

void unsharp_mask(cv::InputArray src,
    cv::OutputArray dst,
    double sigma,
    double alpha,
    double outmin = -1.,
    double outmax = -1.);

bool unsharp_mask(cv::InputArray src,
    cv::InputArray srcmask,
    cv::OutputArray dst,
    double sigma,
    double alpha,
    double outmin = -1,
    double outmax = -1);

double hpass_norm(cv::InputArray src, double sigma,
    cv::InputArray mask = cv::noArray(),
    enum cv::NormTypes normType = cv::NORM_L2);

#endif /* __unsharp_mask_h__ */
