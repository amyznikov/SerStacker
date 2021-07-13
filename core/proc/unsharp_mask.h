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


#endif /* __unsharp_mask_h__ */
